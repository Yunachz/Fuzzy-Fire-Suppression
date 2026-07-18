#include <DHT.h>

// PIN CONFIGURATION
#define FLAME_PIN      33
#define MQ2_PIN        35
#define DHT_PIN        4
#define BUZZER_PIN     23

// L298N — Fan
#define FAN_ENA        25
#define FAN_IN1        26
#define FAN_IN2        27

// L298N — Pump
#define PUMP_ENB       14
#define PUMP_IN3       12
#define PUMP_IN4       13

// DHT22
#define DHTTYPE DHT22
DHT dht(DHT_PIN, DHTTYPE);

// MQ-2: ADC rises as smoke concentration increases
#define MQ2_ADC_MIN     200      // clean air condition
#define MQ2_ADC_MAX     1000     // full saturation
#define MQ2_WARMUP_MS   1000UL // 30 SECONDS warm-up

// FLAME FILTER — MEDIAN + MOVING AVERAGE
// Chosen window sizes:
//   FLAME_MEDIAN_SIZE = 5  → rejects spikes, latency ~200ms
//   FLAME_MA_SIZE     = 5  → smoothing,      latency ~200ms
//   Total effective latency ≈ 400ms at delay(100) per loop.
//   Responsive enough for real flame detection.
#define FLAME_MEDIAN_SIZE  5   // must be odd so there's a single middle value
#define FLAME_MA_SIZE      5

// Median buffer — stores raw ADC samples
int   flameMedianBuf[FLAME_MEDIAN_SIZE] = {0};
int   flameMedianIdx  = 0;
bool  flameMedianFull = false;

// Moving average buffer — stores the median output
float flameMaBuf[FLAME_MA_SIZE] = {0.0f};
int   flameMaIdx  = 0;
bool  flameMaFull = false;
float flameMaSum  = 0.0f;

// FUZZY OUTPUT DOMAIN (0–100 %)
#define OUTPUT_DOMAIN   100
#define CENTROID_STEP   0.5f

struct FuzzyOutput {
    float fan;
    float pump;
};

// GLOBAL VARIABLES
int   flame_adc_raw = 0;
int   mq2_adc_raw   = 0;

float temperature   = 25.0f;
float humidity      = 50.0f;

float smoke_level   = 0.0f;
float env_level     = 0.0f;
float flame_input   = 0.0f;   // median+MA filter output, domain 0–4095

float flameMF[5];
float smokeMF[5];
float envMF[5];

// LPF MQ-2
float filtered_mq2 = 2000.0f;

bool mq2_ready = false;
unsigned long lastDHTRead = 0;

bool fanWasOff = true;
bool pumpWasOff = true;

enum State {
    VERY_LOW  = 0,
    LOW       = 1,
    MEDIUM    = 2,
    HIGH      = 3,
    VERY_HIGH = 4
};

#define VERY_CLOSE     VERY_LOW
#define CLOSE          LOW
#define FLAME_MEDIUM   MEDIUM
#define FAR            HIGH
#define NONE           VERY_HIGH

enum OutputState {
    OFF          = 0,
    SLOW         = 1,
    MEDIUM_OUT   = 2,
    FAST         = 3
};

// RULE STRUCT
struct Rule {
    int env;
    int smoke;
    int flame;
    int pump;
    int fan;
};

// UTILITY: FLOAT MAP
float fmap(float x,
           float in_min,  float in_max,
           float out_min, float out_max)
{
    return (x - in_min) *
           (out_max - out_min) /
           (in_max  - in_min) +
           out_min;
}

// LOW-PASS FILTER — used only for MQ-2
float lowPassFilter(float oldVal, float newVal, float alpha)
{
    return (alpha * oldVal) + ((1.0f - alpha) * newVal);
}

// FLAME FILTER — MEDIAN + MOVING AVERAGE
void flameMedianPush(int rawADC)
{
    flameMedianBuf[flameMedianIdx] = rawADC;
    flameMedianIdx = (flameMedianIdx + 1) % FLAME_MEDIAN_SIZE;
    if (!flameMedianFull && flameMedianIdx == 0)
        flameMedianFull = true;
}

float flameMedianGet()
{
    int count = flameMedianFull ? FLAME_MEDIAN_SIZE : flameMedianIdx;
    if (count == 0) return 0.0f;

    int temp[FLAME_MEDIAN_SIZE];
    for (int i = 0; i < count; i++) temp[i] = flameMedianBuf[i];

    // Insertion sort
    for (int i = 1; i < count; i++) {
        int key = temp[i];
        int j   = i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }

    return (float)temp[count / 2];
}

float flameMaUpdate(float medVal)
{
    if (flameMaFull)
        flameMaSum -= flameMaBuf[flameMaIdx];

    flameMaBuf[flameMaIdx] = medVal;
    flameMaSum += medVal;
    flameMaIdx = (flameMaIdx + 1) % FLAME_MA_SIZE;

    if (!flameMaFull && flameMaIdx == 0)
        flameMaFull = true;

    int count = flameMaFull ? FLAME_MA_SIZE : flameMaIdx;
    return (count > 0) ? (flameMaSum / (float)count) : medVal;
}

float processFlame(int rawADC)
{
    flameMedianPush(rawADC);
    float medVal = flameMedianGet();
    return flameMaUpdate(medVal);
}

// PREPROCESSING: SMOKE LEVEL (0–100 %)
float getSmokeLevel(float adc)
{
    if (adc < MQ2_ADC_MIN) adc = MQ2_ADC_MIN;
    if (adc > MQ2_ADC_MAX) adc = MQ2_ADC_MAX;
    return fmap(adc, MQ2_ADC_MIN, MQ2_ADC_MAX, 0.0f, 100.0f);
}

// PREPROCESSING: ENV LEVEL (0–100 %)
// Weighting: 70% temperature + 30% humidity (inverted)
float getEnvLevel(float temp, float hum)
{
    float temp_score = fmap(temp, 20.0f, 45.0f, 0.0f, 100.0f);
    temp_score = constrain(temp_score, 0.0f, 100.0f);

    float hum_score = fmap(hum, 100.0f, 20.0f, 0.0f, 100.0f);
    hum_score = constrain(hum_score, 0.0f, 100.0f);

    return (0.7f * temp_score) + (0.3f * hum_score);
}

// MEMBERSHIP FUNCTION — FLAME (domain: 0–4095)
float flameVeryClose(float x)
{
    if (x <= 410.0f)  return 1.0f;
    if (x >= 819.0f)  return 0.0f;
    return (819.0f - x) / (819.0f - 410.0f);
}

float flameClose(float x)
{
    if (x <= 410.0f || x >= 1638.0f) return 0.0f;
    if (x >= 819.0f && x <= 1228.0f) return 1.0f;
    if (x < 819.0f)  return (x  - 410.0f)  / (819.0f  - 410.0f);
    return                  (1638.0f - x)  / (1638.0f - 1228.0f);
}

float flameMedium(float x)
{
    if (x <= 1228.0f || x >= 2866.0f)  return 0.0f;
    if (x >= 1638.0f && x <= 2457.0f)  return 1.0f;
    if (x < 1638.0f) return (x    - 1228.0f) / (1638.0f - 1228.0f);
    return                  (2866.0f - x)    / (2866.0f - 2457.0f);
}

float flameFar(float x)
{
    if (x <= 2457.0f || x >= 3686.0f)  return 0.0f;
    if (x >= 2866.0f && x <= 3276.0f)  return 1.0f;
    if (x < 2866.0f) return (x    - 2457.0f) / (2866.0f - 2457.0f);
    return                  (3686.0f - x)    / (3686.0f - 3276.0f);
}

float flameNone(float x)
{
    if (x <= 3276.0f) return 0.0f;
    if (x >= 3686.0f) return 1.0f;
    return (x - 3276.0f) / (3686.0f - 3276.0f);
}

// MEMBERSHIP FUNCTION — SMOKE (domain: 0–100 %)
float smokeVeryThin(float x)
{
    if (x <= 10.0f) return 1.0f;
    if (x >= 25.0f) return 0.0f;
    return (25.0f - x) / 15.0f;
}

float smokeThin(float x)
{
    if (x <= 10.0f || x >= 40.0f) return 0.0f;
    if (x < 25.0f) return (x - 10.0f) / 15.0f;
    return (40.0f - x) / 15.0f;
}

float smokeMedium(float x)
{
    if (x <= 30.0f || x >= 70.0f) return 0.0f;
    if (x < 50.0f) return (x - 30.0f) / 20.0f;
    return (70.0f - x) / 20.0f;
}

float smokeThick(float x)
{
    if (x <= 60.0f || x >= 90.0f) return 0.0f;
    if (x < 75.0f) return (x - 60.0f) / 15.0f;
    return (90.0f - x) / 15.0f;
}

float smokeVeryThick(float x)
{
    if (x <= 80.0f) return 0.0f;
    if (x >= 90.0f) return 1.0f;
    return (x - 80.0f) / 10.0f;
}

// MEMBERSHIP FUNCTION — ENV (domain: 0–100 %)
float envVeryLow(float x)
{
    if (x <= 10.0f) return 1.0f;
    if (x >= 25.0f) return 0.0f;
    return (25.0f - x) / 15.0f;
}

float envLow(float x)
{
    if (x <= 10.0f || x >= 40.0f) return 0.0f;
    if (x < 25.0f) return (x - 10.0f) / 15.0f;
    return (40.0f - x) / 15.0f;
}

float envMedium(float x)
{
    if (x <= 30.0f || x >= 70.0f) return 0.0f;
    if (x < 50.0f) return (x - 30.0f) / 20.0f;
    return (70.0f - x) / 20.0f;
}

float envHigh(float x)
{
    if (x <= 60.0f || x >= 90.0f) return 0.0f;
    if (x < 75.0f) return (x - 60.0f) / 15.0f;
    return (90.0f - x) / 15.0f;
}

float envVeryHigh(float x)
{
    if (x <= 80.0f) return 0.0f;
    if (x >= 90.0f) return 1.0f;
    return (x - 80.0f) / 10.0f;
}

// MEMBERSHIP FUNCTION — OUTPUT (domain: 0–100 %)
float outOff(float x)
{
    if (x <= 10.0f) return 1.0f;
    if (x >= 25.0f) return 0.0f;
    return (25.0f - x) / 15.0f;
}

float outSlow(float x)
{
    if (x <= 15.0f || x >= 45.0f) return 0.0f;
    if (x == 30.0f) return 1.0f;
    if (x < 30.0f)  return (x - 15.0f) / 15.0f;
    return (45.0f - x) / 15.0f;
}

float outMedium(float x)
{
    if (x <= 35.0f || x >= 75.0f) return 0.0f;
    if (x == 55.0f) return 1.0f;
    if (x < 55.0f)  return (x - 35.0f) / 20.0f;
    return (75.0f - x) / 20.0f;
}

float outFast(float x)
{
    if (x <= 70.0f) return 0.0f;
    if (x >= 85.0f) return 1.0f;
    return (x - 70.0f) / 15.0f;
}

// RULE BASE (125 rules: 5 env × 5 smoke × 5 flame)
Rule rules[125];

void generateRules()
{
    // FLAME = VERY_CLOSE
    rules[0]  = {VERY_LOW, VERY_LOW, VERY_CLOSE, FAST, SLOW};
    rules[1]  = {VERY_LOW, LOW,      VERY_CLOSE, FAST, MEDIUM_OUT};
    rules[2]  = {VERY_LOW, MEDIUM,   VERY_CLOSE, FAST, MEDIUM_OUT};
    rules[3]  = {VERY_LOW, HIGH,     VERY_CLOSE, FAST, FAST};
    rules[4]  = {VERY_LOW, VERY_HIGH,VERY_CLOSE, FAST, FAST};
    rules[5]  = {LOW, VERY_LOW, VERY_CLOSE, FAST, SLOW};
    rules[6]  = {LOW, LOW,      VERY_CLOSE, FAST, MEDIUM_OUT};
    rules[7]  = {LOW, MEDIUM,   VERY_CLOSE, FAST, MEDIUM_OUT};
    rules[8]  = {LOW, HIGH,     VERY_CLOSE, FAST, FAST};
    rules[9]  = {LOW, VERY_HIGH,VERY_CLOSE, FAST, FAST};
    rules[10] = {MEDIUM, VERY_LOW, VERY_CLOSE, FAST, MEDIUM_OUT};
    rules[11] = {MEDIUM, LOW,      VERY_CLOSE, FAST, MEDIUM_OUT};
    rules[12] = {MEDIUM, MEDIUM,   VERY_CLOSE, FAST, FAST};
    rules[13] = {MEDIUM, HIGH,     VERY_CLOSE, FAST, FAST};
    rules[14] = {MEDIUM, VERY_HIGH,VERY_CLOSE, FAST, FAST};
    rules[15] = {HIGH, VERY_LOW, VERY_CLOSE, FAST, MEDIUM_OUT};
    rules[16] = {HIGH, LOW,      VERY_CLOSE, FAST, FAST};
    rules[17] = {HIGH, MEDIUM,   VERY_CLOSE, FAST, FAST};
    rules[18] = {HIGH, HIGH,     VERY_CLOSE, FAST, FAST};
    rules[19] = {HIGH, VERY_HIGH,VERY_CLOSE, FAST, FAST};
    rules[20] = {VERY_HIGH, VERY_LOW, VERY_CLOSE, FAST, MEDIUM_OUT};
    rules[21] = {VERY_HIGH, LOW,      VERY_CLOSE, FAST, FAST};
    rules[22] = {VERY_HIGH, MEDIUM,   VERY_CLOSE, FAST, FAST};
    rules[23] = {VERY_HIGH, HIGH,     VERY_CLOSE, FAST, FAST};
    rules[24] = {VERY_HIGH, VERY_HIGH,VERY_CLOSE, FAST, FAST};

    // FLAME = CLOSE
    rules[25] = {VERY_LOW, VERY_LOW, CLOSE, MEDIUM_OUT, SLOW};
    rules[26] = {VERY_LOW, LOW,      CLOSE, MEDIUM_OUT, MEDIUM_OUT};
    rules[27] = {VERY_LOW, MEDIUM,   CLOSE, MEDIUM_OUT, MEDIUM_OUT};
    rules[28] = {VERY_LOW, HIGH,     CLOSE, FAST,       FAST};
    rules[29] = {VERY_LOW, VERY_HIGH,CLOSE, FAST,       FAST};
    rules[30] = {LOW, VERY_LOW, CLOSE, MEDIUM_OUT, SLOW};
    rules[31] = {LOW, LOW,      CLOSE, MEDIUM_OUT, MEDIUM_OUT};
    rules[32] = {LOW, MEDIUM,   CLOSE, MEDIUM_OUT, MEDIUM_OUT};
    rules[33] = {LOW, HIGH,     CLOSE, FAST,       FAST};
    rules[34] = {LOW, VERY_HIGH,CLOSE, FAST,       FAST};
    rules[35] = {MEDIUM, VERY_LOW, CLOSE, MEDIUM_OUT, MEDIUM_OUT};
    rules[36] = {MEDIUM, LOW,      CLOSE, MEDIUM_OUT, MEDIUM_OUT};
    rules[37] = {MEDIUM, MEDIUM,   CLOSE, MEDIUM_OUT, FAST};
    rules[38] = {MEDIUM, HIGH,     CLOSE, FAST,       FAST};
    rules[39] = {MEDIUM, VERY_HIGH,CLOSE, FAST,       FAST};
    rules[40] = {HIGH, VERY_LOW, CLOSE, MEDIUM_OUT, MEDIUM_OUT};
    rules[41] = {HIGH, LOW,      CLOSE, MEDIUM_OUT, FAST};
    rules[42] = {HIGH, MEDIUM,   CLOSE, FAST,       FAST};
    rules[43] = {HIGH, HIGH,     CLOSE, FAST,       FAST};
    rules[44] = {HIGH, VERY_HIGH,CLOSE, FAST,       FAST};
    rules[45] = {VERY_HIGH, VERY_LOW, CLOSE, MEDIUM_OUT, MEDIUM_OUT};
    rules[46] = {VERY_HIGH, LOW,      CLOSE, MEDIUM_OUT, FAST};
    rules[47] = {VERY_HIGH, MEDIUM,   CLOSE, FAST,       FAST};
    rules[48] = {VERY_HIGH, HIGH,     CLOSE, FAST,       FAST};
    rules[49] = {VERY_HIGH, VERY_HIGH,CLOSE, FAST,       FAST};

    // FLAME = FLAME_MEDIUM
    rules[50] = {VERY_LOW, VERY_LOW, FLAME_MEDIUM, SLOW,       SLOW};
    rules[51] = {VERY_LOW, LOW,      FLAME_MEDIUM, SLOW,       MEDIUM_OUT};
    rules[52] = {VERY_LOW, MEDIUM,   FLAME_MEDIUM, MEDIUM_OUT, MEDIUM_OUT};
    rules[53] = {VERY_LOW, HIGH,     FLAME_MEDIUM, MEDIUM_OUT, FAST};
    rules[54] = {VERY_LOW, VERY_HIGH,FLAME_MEDIUM, MEDIUM_OUT, FAST};
    rules[55] = {LOW, VERY_LOW, FLAME_MEDIUM, SLOW,       SLOW};
    rules[56] = {LOW, LOW,      FLAME_MEDIUM, SLOW,       MEDIUM_OUT};
    rules[57] = {LOW, MEDIUM,   FLAME_MEDIUM, MEDIUM_OUT, MEDIUM_OUT};
    rules[58] = {LOW, HIGH,     FLAME_MEDIUM, MEDIUM_OUT, FAST};
    rules[59] = {LOW, VERY_HIGH,FLAME_MEDIUM, MEDIUM_OUT, FAST};
    rules[60] = {MEDIUM, VERY_LOW, FLAME_MEDIUM, SLOW,       SLOW};
    rules[61] = {MEDIUM, LOW,      FLAME_MEDIUM, MEDIUM_OUT, MEDIUM_OUT};
    rules[62] = {MEDIUM, MEDIUM,   FLAME_MEDIUM, MEDIUM_OUT, MEDIUM_OUT};
    rules[63] = {MEDIUM, HIGH,     FLAME_MEDIUM, MEDIUM_OUT, FAST};
    rules[64] = {MEDIUM, VERY_HIGH,FLAME_MEDIUM, MEDIUM_OUT, FAST};
    rules[65] = {HIGH, VERY_LOW, FLAME_MEDIUM, MEDIUM_OUT, MEDIUM_OUT};
    rules[66] = {HIGH, LOW,      FLAME_MEDIUM, MEDIUM_OUT, MEDIUM_OUT};
    rules[67] = {HIGH, MEDIUM,   FLAME_MEDIUM, MEDIUM_OUT, FAST};
    rules[68] = {HIGH, HIGH,     FLAME_MEDIUM, MEDIUM_OUT, FAST};
    rules[69] = {HIGH, VERY_HIGH,FLAME_MEDIUM, MEDIUM_OUT, FAST};
    rules[70] = {VERY_HIGH, VERY_LOW, FLAME_MEDIUM, MEDIUM_OUT, MEDIUM_OUT};
    rules[71] = {VERY_HIGH, LOW,      FLAME_MEDIUM, MEDIUM_OUT, MEDIUM_OUT};
    rules[72] = {VERY_HIGH, MEDIUM,   FLAME_MEDIUM, MEDIUM_OUT, FAST};
    rules[73] = {VERY_HIGH, HIGH,     FLAME_MEDIUM, MEDIUM_OUT, FAST};
    rules[74] = {VERY_HIGH, VERY_HIGH,FLAME_MEDIUM, MEDIUM_OUT, FAST};

    // FLAME = FAR
    rules[75] = {VERY_LOW, VERY_LOW, FAR, OFF,        OFF};
    rules[76] = {VERY_LOW, LOW,      FAR, SLOW,       SLOW};
    rules[77] = {VERY_LOW, MEDIUM,   FAR, SLOW,       MEDIUM_OUT};
    rules[78] = {VERY_LOW, HIGH,     FAR, MEDIUM_OUT, FAST};
    rules[79] = {VERY_LOW, VERY_HIGH,FAR, MEDIUM_OUT, FAST};
    rules[80] = {LOW, VERY_LOW, FAR, OFF,        SLOW};
    rules[81] = {LOW, LOW,      FAR, SLOW,       SLOW};
    rules[82] = {LOW, MEDIUM,   FAR, SLOW,       MEDIUM_OUT};
    rules[83] = {LOW, HIGH,     FAR, MEDIUM_OUT, FAST};
    rules[84] = {LOW, VERY_HIGH,FAR, MEDIUM_OUT, FAST};
    rules[85] = {MEDIUM, VERY_LOW, FAR, OFF,        SLOW};
    rules[86] = {MEDIUM, LOW,      FAR, SLOW,       SLOW};
    rules[87] = {MEDIUM, MEDIUM,   FAR, SLOW,       MEDIUM_OUT};
    rules[88] = {MEDIUM, HIGH,     FAR, MEDIUM_OUT, FAST};
    rules[89] = {MEDIUM, VERY_HIGH,FAR, MEDIUM_OUT, FAST};
    rules[90] = {HIGH, VERY_LOW, FAR, SLOW,       SLOW};
    rules[91] = {HIGH, LOW,      FAR, SLOW,       MEDIUM_OUT};
    rules[92] = {HIGH, MEDIUM,   FAR, MEDIUM_OUT, MEDIUM_OUT};
    rules[93] = {HIGH, HIGH,     FAR, MEDIUM_OUT, FAST};
    rules[94] = {HIGH, VERY_HIGH,FAR, MEDIUM_OUT, FAST};
    rules[95] = {VERY_HIGH, VERY_LOW, FAR, SLOW,       SLOW};
    rules[96] = {VERY_HIGH, LOW,      FAR, SLOW,       MEDIUM_OUT};
    rules[97] = {VERY_HIGH, MEDIUM,   FAR, MEDIUM_OUT, MEDIUM_OUT};
    rules[98] = {VERY_HIGH, HIGH,     FAR, MEDIUM_OUT, FAST};
    rules[99] = {VERY_HIGH, VERY_HIGH,FAR, MEDIUM_OUT, FAST};

    // FLAME = NONE
    rules[100] = {VERY_LOW, VERY_LOW, NONE, OFF, OFF};
    rules[101] = {VERY_LOW, LOW,      NONE, OFF, SLOW};
    rules[102] = {VERY_LOW, MEDIUM,   NONE, OFF, MEDIUM_OUT};
    rules[103] = {VERY_LOW, HIGH,     NONE, OFF, FAST};
    rules[104] = {VERY_LOW, VERY_HIGH,NONE, OFF, FAST};
    rules[105] = {LOW, VERY_LOW, NONE, OFF, OFF};
    rules[106] = {LOW, LOW,      NONE, OFF, SLOW};
    rules[107] = {LOW, MEDIUM,   NONE, OFF, MEDIUM_OUT};
    rules[108] = {LOW, HIGH,     NONE, OFF, FAST};
    rules[109] = {LOW, VERY_HIGH,NONE, OFF, FAST};
    rules[110] = {MEDIUM, VERY_LOW, NONE, OFF, OFF};
    rules[111] = {MEDIUM, LOW,      NONE, OFF, SLOW};
    rules[112] = {MEDIUM, MEDIUM,   NONE, OFF, MEDIUM_OUT};
    rules[113] = {MEDIUM, HIGH,     NONE, OFF, FAST};
    rules[114] = {MEDIUM, VERY_HIGH,NONE, OFF, FAST};
    rules[115] = {HIGH, VERY_LOW, NONE, OFF, SLOW};
    rules[116] = {HIGH, LOW,      NONE, OFF, MEDIUM_OUT};
    rules[117] = {HIGH, MEDIUM,   NONE, OFF, MEDIUM_OUT};
    rules[118] = {HIGH, HIGH,     NONE, OFF, FAST};
    rules[119] = {HIGH, VERY_HIGH,NONE, OFF, FAST};
    rules[120] = {VERY_HIGH, VERY_LOW, NONE, OFF, SLOW};
    rules[121] = {VERY_HIGH, LOW,      NONE, OFF, MEDIUM_OUT};
    rules[122] = {VERY_HIGH, MEDIUM,   NONE, OFF, MEDIUM_OUT};
    rules[123] = {VERY_HIGH, HIGH,     NONE, OFF, FAST};
    rules[124] = {VERY_HIGH, VERY_HIGH,NONE, OFF, FAST};
}

// FUZZIFICATION
void fuzzifyFlame(float x)
{
    flameMF[VERY_CLOSE]   = flameVeryClose(x);
    flameMF[CLOSE]        = flameClose(x);
    flameMF[FLAME_MEDIUM] = flameMedium(x);
    flameMF[FAR]          = flameFar(x);
    flameMF[NONE]         = flameNone(x);
}

void fuzzifySmoke(float x)
{
    smokeMF[VERY_LOW]  = smokeVeryThin(x);
    smokeMF[LOW]       = smokeThin(x);
    smokeMF[MEDIUM]    = smokeMedium(x);
    smokeMF[HIGH]      = smokeThick(x);
    smokeMF[VERY_HIGH] = smokeVeryThick(x);
}

void fuzzifyENV(float x)
{
    envMF[VERY_LOW]  = envVeryLow(x);
    envMF[LOW]       = envLow(x);
    envMF[MEDIUM]    = envMedium(x);
    envMF[HIGH]      = envHigh(x);
    envMF[VERY_HIGH] = envVeryHigh(x);
}

// MAMDANI INFERENCE + COG DEFUZZIFICATION (ON-THE-FLY)
static inline float getOutputMF(int state, float x)
{
    switch (state) {
        case OFF:        return outOff(x);
        case SLOW:       return outSlow(x);
        case MEDIUM_OUT: return outMedium(x);
        case FAST:       return outFast(x);
        default:         return 0.0f;
    }
}

FuzzyOutput computeOutputs()
{
    float num_fan  = 0.0f, den_fan  = 0.0f;
    float num_pump = 0.0f, den_pump = 0.0f;

    float alpha[125];
    for (int r = 0; r < 125; r++) {
        alpha[r] = min(envMF[rules[r].env],
                   min(smokeMF[rules[r].smoke],
                       flameMF[rules[r].flame]));
    }

    for (float x = 0.0f; x <= (float)OUTPUT_DOMAIN; x += CENTROID_STEP)
    {
        float agg_fan  = 0.0f;
        float agg_pump = 0.0f;

        for (int r = 0; r < 125; r++) {
            if (alpha[r] <= 0.0f) continue;

            float clipped_fan  = min(alpha[r], getOutputMF(rules[r].fan,  x));
            float clipped_pump = min(alpha[r], getOutputMF(rules[r].pump, x));

            if (clipped_fan  > agg_fan)  agg_fan  = clipped_fan;
            if (clipped_pump > agg_pump) agg_pump = clipped_pump;
        }

        num_fan  += x * agg_fan;   den_fan  += agg_fan;
        num_pump += x * agg_pump;  den_pump += agg_pump;
    }

    return {
        (den_fan  > 0.0f) ? (num_fan  / den_fan)  : 0.0f,
        (den_pump > 0.0f) ? (num_pump / den_pump) : 0.0f
    };
}

// BUZZER — percentage-based hysteresis
void updateBuzzer(float fanPct, float pumpPct)
{
    static bool active = false;

    if (!active && (fanPct > 30.0f || pumpPct > 30.0f))
        active = true;

    if (active && (fanPct < 20.0f && pumpPct < 20.0f))
        active = false;

    digitalWrite(BUZZER_PIN, active ? HIGH : LOW);
}

// REMAP PERCENTAGE → PWM (0–255) + DEADZONE
int percentToPWMFan(float pct)
{
    if (pct < 11.5f) return 0;

    return (int)fmap(
        pct,
        11.5f, 100.0f,
        84.0f, 255.0f
    );
}

int percentToPWMPump(float pct)
{
    if (pct < 11.5f) return 0;

    return (int)fmap(
        pct,
        11.5f, 100.0f,
        155.0f, 255.0f
    );
}

// ESP32 LEDC SETUP
void setupPWM()
{
    ledcSetup(0, 5000, 8);
    ledcAttachPin(FAN_ENA, 0);

    ledcSetup(1, 5000, 8);
    ledcAttachPin(PUMP_ENB, 1);
}

void setup()
{
    Serial.begin(115200);

    dht.begin();
    generateRules();
    setupPWM();

    pinMode(FAN_IN1,    OUTPUT);
    pinMode(FAN_IN2,    OUTPUT);
    pinMode(PUMP_IN3,   OUTPUT);
    pinMode(PUMP_IN4,   OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(FAN_IN1,  HIGH);
    digitalWrite(FAN_IN2,  LOW);
    digitalWrite(PUMP_IN3, HIGH);
    digitalWrite(PUMP_IN4, LOW);

    Serial.println("=== Fuzzy Fire Controller Ready ===");
    Serial.println("[INFO] Flame: Median(5)+MA(5), domain 0-4095, deadzone <20%");
}

void loop()
{
    // Read sensors
    flame_adc_raw = analogRead(FLAME_PIN);
    mq2_adc_raw   = analogRead(MQ2_PIN);

    // DHT22: read every 2 seconds
    if (millis() - lastDHTRead >= 2000UL) {
        float t = dht.readTemperature();
        float h = dht.readHumidity();

        if (!isnan(t) && !isnan(h)) {
            temperature = t;
            humidity    = h;
        } else {
            Serial.println("[WARN] DHT22 read error");
        }
        lastDHTRead = millis();
    }

    // Flame filter: Median + Moving Average
    // Raw ADC (0–4095) goes straight into the filter without conversion.
    flame_input = processFlame(flame_adc_raw);

    // LPF for MQ-2
    // filtered_mq2 = lowPassFilter(filtered_mq2, (float)mq2_adc_raw, 0.7f);
    filtered_mq2 = mq2_adc_raw;

    // Check MQ-2 warm-up
    if (!mq2_ready) {
        unsigned long elapsed = millis();
        if (elapsed < MQ2_WARMUP_MS) {
            unsigned long remaining = (MQ2_WARMUP_MS - elapsed) / 1000UL;
            Serial.print("[WARMUP] MQ-2 not stable yet, remaining: ");
            Serial.print(remaining);
            Serial.println(" s");
        } else {
            mq2_ready = true;
            Serial.println("[INFO] MQ-2 warm-up complete, smoke reading active");
        }
    }

    // Preprocess linguistic variables
    env_level   = getEnvLevel(temperature, humidity);
    smoke_level = mq2_ready ? getSmokeLevel(filtered_mq2) : 0.0f;

    // Fuzzification
    fuzzifyFlame(flame_input);
    fuzzifySmoke(smoke_level);
    fuzzifyENV(env_level);

    // Inference + Defuzzification (result: 0–100 %)
    FuzzyOutput result = computeOutputs();
    float fanPercent  = result.fan;
    float pumpPercent = result.pump;

    // Remap to PWM (deadzone <20% → 0)
    int fanPWM  = percentToPWMFan(fanPercent);
    int pumpPWM = percentToPWMPump(pumpPercent);

    // Send to actuators 
    // Kick start fan
    if (fanWasOff && fanPWM > 0)
    {
        ledcWrite(0, 170);
        delay(300);
    }

    // Kick start pump
    if (pumpWasOff && pumpPWM > 0)
    {
        ledcWrite(1, 170);
        delay(300);
    }

    // Normal PWM
    ledcWrite(0, fanPWM);
    ledcWrite(1, pumpPWM);

    // Update state
    fanWasOff  = (fanPWM == 0);
    pumpWasOff = (pumpPWM == 0);

    updateBuzzer(fanPercent, pumpPercent);

    // Debug serial output
    Serial.print(" Flame:"); Serial.print(flame_adc_raw);
    Serial.print(" | Smoke:");
    if (mq2_ready) { Serial.print(smoke_level, 1); Serial.print("%"); }
    else            { Serial.print("warmup"); }
    Serial.print(" | ENV:");   Serial.print(env_level,   1); Serial.print("%");
    Serial.print(" | T:");     Serial.print(temperature, 1); Serial.print("C");
    Serial.print(" H:");       Serial.print(humidity,    1); Serial.print("%");
    Serial.print(" || Fan:");  Serial.print(fanPercent,  1);
    Serial.print("%(PWM:");    Serial.print(fanPWM);
    Serial.print(") Pump:");   Serial.print(pumpPercent, 1);
    Serial.print("%(PWM:");    Serial.print(pumpPWM);
    Serial.println(")");

    delay(100);
}
