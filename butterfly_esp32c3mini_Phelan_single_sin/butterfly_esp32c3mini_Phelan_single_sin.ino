#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#if __has_include(<hal/brownout_ll.h>)
#include "hal/brownout_ll.h"
__attribute__((constructor)) static void disable_bod_early() {
  brownout_ll_bod_enable(false);
  brownout_ll_intr_enable(false);
}
#endif

#include <esp_task_wdt.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include "driver/uart.h"

enum class Protocol { DETECT, CRSF, SBUS };

namespace Config {
constexpr uint8_t RX_PIN = 3;
constexpr uint8_t SERVO_L_PIN = 5;
constexpr uint8_t SERVO_R_PIN = 6;
constexpr uint8_t LED_PIN = 8;

constexpr int SERVO_HZ = 300;
constexpr int SERVO_MIN_ANGLE = 20;
constexpr int SERVO_MAX_ANGLE = 160;
constexpr int SERVO_EMA_ALPHA_SHIFT = 1;
constexpr bool REVERSE_SERVO_L = false;
constexpr bool REVERSE_SERVO_R = false;

constexpr int SERVO_US_MIN = 500;
constexpr int SERVO_US_MAX = 2500;
constexpr int SERVO_US_CENTER = 1500;
constexpr int SERVO_US_DEADBAND = 3;
constexpr int SERVO_US_LIM_MIN = SERVO_US_MIN + (SERVO_MIN_ANGLE * (SERVO_US_MAX - SERVO_US_MIN) + 90) / 180;
constexpr int SERVO_US_LIM_MAX = SERVO_US_MIN + (SERVO_MAX_ANGLE * (SERVO_US_MAX - SERVO_US_MIN) + 90) / 180;

constexpr uint32_t CRSF_BAUD = 420000;
constexpr uint32_t SBUS_BAUD = 100000;
constexpr size_t RX_BUFFER_SIZE = 512;

constexpr unsigned long RX_TIMEOUT_MS = 200;
constexpr unsigned long RX_BYTE_TIMEOUT_MS = 5;
constexpr uint32_t WDT_TIMEOUT_SEC = 3;
constexpr unsigned long PROTO_LOCK_TIMEOUT_MS = 3000;
constexpr unsigned long PROTO_SWITCH_TIMEOUT_MS = 1500;
constexpr uint8_t CONN_LOST_THRESHOLD = 5;
constexpr unsigned long STICK_HOLD_MS = 2000;
constexpr unsigned long RX_HARD_FAILSAFE_MS = 5000;

constexpr int THROTTLE_IDLE_MAX = 1050;
constexpr bool USE_AUTO_DETECT = true;
constexpr Protocol FIXED_PROTOCOL = Protocol::CRSF;

constexpr unsigned long LOOP_DT_US = 3333;
constexpr float FLAP_AMP_IDLE = 55.0f;
constexpr float FLAP_AMP_MAX_THR = 35.0f;
constexpr uint32_t FLAP_SPEED_MIN = 157286;
constexpr uint32_t FLAP_SPEED_MAX = 262144;
constexpr int PERCH_MULT = 31;
constexpr int PERCH_BASE_UP_ANGLE = 30;
constexpr int FAILSAFE_DIHEDRAL = 15;
constexpr int GLIDE_SERVO_SPEED_DPS = 1570;

constexpr int16_t PERCH_ENTER_ELEV = 1300;
constexpr int16_t PERCH_EXIT_ELEV = 1700;
constexpr int16_t STICK_DEADBAND = 10;
constexpr int16_t ARM_YAW_HIGH = 1900;
constexpr int16_t ARM_YAW_LOW = 1100;

constexpr int16_t RX_MIN_MICROS = 800;
constexpr int16_t RX_MAX_MICROS = 2200;
constexpr int32_t SBUS_RAW_MIN = 172;
constexpr int32_t SBUS_RAW_MAX = 1811;
constexpr int32_t CRSF_RAW_MIN = 172;
constexpr int32_t CRSF_RAW_MAX = 1811;
constexpr uint8_t CRSF_MAX_PAYLOAD_LEN = 62;
constexpr uint8_t MAX_LOST_FRAMES = 255;

constexpr int16_t CH_CENTER = 1500;
constexpr int16_t CH_MIN = 1000;
constexpr int16_t CH_MAX = 2000;

constexpr int EMA_ALPHA_SHIFT = 3;

constexpr uint8_t CH_STEERING = 0;
constexpr uint8_t CH_ELEVATOR = 1;
constexpr uint8_t CH_THROTTLE = 2;
constexpr uint8_t CH_YAW = 3;
constexpr uint8_t CH_KILL_SWITCH = 4;
constexpr int16_t KILL_SWITCH_THRESHOLD = 1700;

constexpr int PERCH_HEADROOM_DEG = 15;

static_assert(CH_STEERING < 16, "CH_STEERING out of range");
static_assert(CH_ELEVATOR < 16, "CH_ELEVATOR out of range");
static_assert(CH_THROTTLE < 16, "CH_THROTTLE out of range");
static_assert(CH_YAW < 16, "CH_YAW out of range");
static_assert(CH_KILL_SWITCH < 16, "CH_KILL_SWITCH out of range");

inline float mapTrimFloat(int x) {
  int d = x - CH_CENTER;
  int mag = abs(d) - STICK_DEADBAND;
  if (mag <= 0)
    return 0.0f;
  float s = (d > 0) ? (float)mag : -(float)mag;
  constexpr float span = (float)(CH_MAX - CH_CENTER - STICK_DEADBAND);
  return constrain(s * 15.0f / span, -15.0f, 15.0f);
}

inline int angleToUs(float angleDeg) {
  return (int)lroundf(SERVO_US_MIN + angleDeg * (SERVO_US_MAX - SERVO_US_MIN) / 180.0f);
}

}

#define LOG_LEVEL_DEBUG 3
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_WARN 1
#define LOG_LEVEL_ERROR 0

#define CURRENT_LOG_LEVEL LOG_LEVEL_WARN

SemaphoreHandle_t logMutex = NULL;

#define SAFE_LOG(level, fmt, ...)                                              \
  do {                                                                         \
    if (logMutex)                                                              \
      xSemaphoreTakeRecursive(logMutex, portMAX_DELAY);                        \
    if (Serial.availableForWrite() >= 64) {                                    \
      Serial.print(level);                                                     \
      Serial.printf(fmt, ##__VA_ARGS__);                                       \
      Serial.println();                                                        \
    }                                                                          \
    if (logMutex)                                                              \
      xSemaphoreGiveRecursive(logMutex);                                       \
  } while (0)

#if CURRENT_LOG_LEVEL >= LOG_LEVEL_DEBUG
#define LOG_DEBUG(...) SAFE_LOG("[DEBUG] ", ##__VA_ARGS__)
#else
#define LOG_DEBUG(...)                                                         \
  do {                                                                         \
  } while (0)
#endif

#if CURRENT_LOG_LEVEL >= LOG_LEVEL_INFO
#define LOG_INFO(...) SAFE_LOG("[INFO] ", ##__VA_ARGS__)
#else
#define LOG_INFO(...)                                                          \
  do {                                                                         \
  } while (0)
#endif

#if CURRENT_LOG_LEVEL >= LOG_LEVEL_WARN
#define LOG_WARN(...) SAFE_LOG("[WARN] ", ##__VA_ARGS__)
#else
#define LOG_WARN(...)                                                          \
  do {                                                                         \
  } while (0)
#endif

#if CURRENT_LOG_LEVEL >= LOG_LEVEL_ERROR
#define LOG_ERROR(...) SAFE_LOG("[ERROR] ", ##__VA_ARGS__)
#else
#define LOG_ERROR(...)                                                         \
  do {                                                                         \
  } while (0)
#endif

namespace ServoSystem {

struct Control {
  Servo left;
  Servo right;

  static constexpr int centerAngle = 90;
  int lastUsL = Config::SERVO_US_CENTER;
  int lastUsR = Config::SERVO_US_CENTER;
  int32_t filteredL = Config::SERVO_US_CENTER << Config::SERVO_EMA_ALPHA_SHIFT;
  int32_t filteredR = Config::SERVO_US_CENTER << Config::SERVO_EMA_ALPHA_SHIFT;
  bool isAttached = false;
};

Control state;

void attachAll() {
  if (!state.isAttached) {
    state.left.setPeriodHertz(Config::SERVO_HZ);
    state.right.setPeriodHertz(Config::SERVO_HZ);
    state.left.attach(Config::SERVO_L_PIN, Config::SERVO_US_MIN, Config::SERVO_US_MAX);
    state.right.attach(Config::SERVO_R_PIN, Config::SERVO_US_MIN, Config::SERVO_US_MAX);
    state.left.writeMicroseconds(state.lastUsL);
    state.right.writeMicroseconds(state.lastUsR);
    state.filteredL = (int32_t)state.lastUsL << Config::SERVO_EMA_ALPHA_SHIFT;
    state.filteredR = (int32_t)state.lastUsR << Config::SERVO_EMA_ALPHA_SHIFT;
    state.isAttached = true;
  }
}

void detachAll() {
  if (state.isAttached) {
    state.left.detach();
    state.right.detach();
    state.isAttached = false;
  }
}

void update(float targetL, float targetR, bool bypassFilter = false) {
  if (!state.isAttached)
    return;
  targetL = constrain(targetL, (float)Config::SERVO_MIN_ANGLE, (float)Config::SERVO_MAX_ANGLE);
  targetR = constrain(targetR, (float)Config::SERVO_MIN_ANGLE, (float)Config::SERVO_MAX_ANGLE);

  float angleL = Config::REVERSE_SERVO_L ? (180.0f - targetL) : targetL;
  float angleR = Config::REVERSE_SERVO_R ? (180.0f - targetR) : targetR;

  int targetUsL = Config::angleToUs(angleL);
  int targetUsR = Config::angleToUs(angleR);

  if (bypassFilter) {
    state.filteredL = (int32_t)targetUsL << Config::SERVO_EMA_ALPHA_SHIFT;
    state.filteredR = (int32_t)targetUsR << Config::SERVO_EMA_ALPHA_SHIFT;
  } else {
    int32_t scaledTargetL = (int32_t)targetUsL << Config::SERVO_EMA_ALPHA_SHIFT;
    int32_t diffL = scaledTargetL - state.filteredL;
    if (diffL != 0) {
      int32_t stepL = diffL / (1 << Config::SERVO_EMA_ALPHA_SHIFT);
      if (stepL == 0) stepL = (diffL > 0) ? 1 : -1;
      state.filteredL += stepL;
    }
    int32_t scaledTargetR = (int32_t)targetUsR << Config::SERVO_EMA_ALPHA_SHIFT;
    int32_t diffR = scaledTargetR - state.filteredR;
    if (diffR != 0) {
      int32_t stepR = diffR / (1 << Config::SERVO_EMA_ALPHA_SHIFT);
      if (stepR == 0) stepR = (diffR > 0) ? 1 : -1;
      state.filteredR += stepR;
    }
  }

  int usL = state.filteredL >> Config::SERVO_EMA_ALPHA_SHIFT;
  int usR = state.filteredR >> Config::SERVO_EMA_ALPHA_SHIFT;

  usL = constrain(usL, Config::SERVO_US_LIM_MIN, Config::SERVO_US_LIM_MAX);
  usR = constrain(usR, Config::SERVO_US_LIM_MIN, Config::SERVO_US_LIM_MAX);

  if (abs(usL - state.lastUsL) >= Config::SERVO_US_DEADBAND) {
    state.left.writeMicroseconds(usL);
    state.lastUsL = usL;
  }
  if (abs(usR - state.lastUsR) >= Config::SERVO_US_DEADBAND) {
    state.right.writeMicroseconds(usR);
    state.lastUsR = usR;
  }
}

}

enum class FlightMode { INIT, CONNECTED, PERCHING, FAILSAFE };

namespace Flight {

struct State {
  FlightMode mode = FlightMode::INIT;

  uint16_t phase = 0;
  float currentOffset = 0.0f;

  bool isPerching = false;
  int perchAngleLevel = Config::CH_MIN;
  bool phaseNeedsResync = true;
  bool isFlapUp = true;
  unsigned long failsafeEnterMs = 0;

  bool wasFlapping = false;
  bool wasPerching = false;
  bool isFlapping = false;

  bool isArmed = false;
  unsigned long armStartTime = 0;
  unsigned long disarmStartTime = 0;
};

State state;

static const int16_t sin_LUT[256] = {
    0,     25,    49,   74,   98,   122,  147,  171,  195,  219,  243,  267,
    290,   314,   337,  360,  383,  405,  428,  450,  471,  493,  514,  535,
    556,   576,   596,  615,  634,  653,  672,  690,  707,  724,  741,  757,
    773,   788,   803,  818,  831,  845,  858,  870,  882,  893,  904,  914,
    924,   933,   942,  950,  957,  964,  970,  976,  981,  985,  989,  992,
    995,   997,   999,  1000, 1000, 1000, 999,  997,  995,  992,  989,  985,
    981,   976,   970,  964,  957,  950,  942,  933,  924,  914,  904,  893,
    882,   870,   858,  845,  831,  818,  803,  788,  773,  757,  741,  724,
    707,   690,   672,  653,  634,  615,  596,  576,  556,  535,  514,  493,
    471,   450,   428,  405,  383,  360,  337,  314,  290,  267,  243,  219,
    195,   171,   147,  122,  98,   74,   49,   25,   0,    -25,  -49,  -74,
    -98,   -122,  -147, -171, -195, -219, -243, -267, -290, -314, -337, -360,
    -383,  -405,  -428, -450, -471, -493, -514, -535, -556, -576, -596, -615,
    -634,  -653,  -672, -690, -707, -724, -741, -757, -773, -788, -803, -818,
    -831,  -845,  -858, -870, -882, -893, -904, -914, -924, -933, -942, -950,
    -957,  -964,  -970, -976, -981, -985, -989, -992, -995, -997, -999, -1000,
    -1000, -1000, -999, -997, -995, -992, -989, -985, -981, -976, -970, -964,
    -957,  -950,  -942, -933, -924, -914, -904, -893, -882, -870, -858, -845,
    -831,  -818,  -803, -788, -773, -757, -741, -724, -707, -690, -672, -653,
    -634,  -615,  -596, -576, -556, -535, -514, -493, -471, -450, -428, -405,
    -383,  -360,  -337, -314, -290, -267, -243, -219, -195, -171, -147, -122,
    -98,   -74,   -49,  -25};

static const uint16_t asin_LUT[256] = {
    0,     41,    82,    123,   164,   205,   245,   286,   327,   368,   409,
    450,   491,   532,   573,   614,   655,   696,   737,   778,   819,   860,
    901,   942,   983,   1024,  1065,  1106,  1148,  1189,  1230,  1271,  1312,
    1354,  1395,  1436,  1477,  1519,  1560,  1602,  1643,  1684,  1726,  1767,
    1809,  1850,  1892,  1934,  1975,  2017,  2059,  2100,  2142,  2184,  2226,
    2268,  2309,  2351,  2393,  2435,  2477,  2520,  2562,  2604,  2646,  2688,
    2731,  2773,  2816,  2858,  2900,  2943,  2986,  3028,  3071,  3114,  3157,
    3200,  3242,  3285,  3328,  3372,  3415,  3458,  3501,  3545,  3588,  3632,
    3675,  3719,  3762,  3806,  3850,  3894,  3938,  3982,  4026,  4070,  4114,
    4159,  4203,  4248,  4292,  4337,  4382,  4427,  4471,  4516,  4562,  4607,
    4652,  4697,  4743,  4789,  4834,  4880,  4926,  4972,  5018,  5064,  5110,
    5157,  5203,  5250,  5297,  5344,  5391,  5438,  5485,  5532,  5580,  5627,
    5675,  5723,  5771,  5819,  5867,  5916,  5964,  6013,  6062,  6111,  6160,
    6210,  6259,  6309,  6358,  6408,  6459,  6509,  6559,  6610,  6661,  6712,
    6763,  6815,  6866,  6918,  6970,  7022,  7075,  7127,  7180,  7233,  7287,
    7340,  7394,  7448,  7502,  7557,  7611,  7666,  7722,  7777,  7833,  7889,
    7945,  8002,  8059,  8116,  8174,  8232,  8290,  8349,  8408,  8467,  8526,
    8586,  8647,  8708,  8769,  8830,  8892,  8954,  9017,  9081,  9144,  9208,
    9273,  9338,  9404,  9470,  9537,  9604,  9672,  9741,  9810,  9879,  9950,
    10021, 10092, 10165, 10238, 10312, 10387, 10463, 10539, 10617, 10695, 10774,
    10855, 10936, 11019, 11102, 11187, 11274, 11361, 11450, 11541, 11633, 11727,
    11822, 11920, 12020, 12121, 12225, 12332, 12442, 12554, 12669, 12789, 12912,
    13039, 13171, 13309, 13453, 13605, 13764, 13934, 14117, 14315, 14534, 14782,
    15077, 15460, 16384};

inline bool isRisingPhase(uint16_t phase) {
  return (uint16_t)(phase + 16384) < 32768;
}

uint16_t sinPhaseFromOffset(float offset, float amp, bool flapUp) {
  if (amp <= 0.0f) return 0;
  int32_t ratioScaled = (int32_t)(offset * 1000.0f / amp);
  ratioScaled = constrain(ratioScaled, -1000, 1000);

  bool negate = (ratioScaled < 0);
  int32_t abs_ratio = negate ? -ratioScaled : ratioScaled;

  int index = (abs_ratio * 255) / 1000;
  uint16_t approxPhase = asin_LUT[index];

  int32_t signedPhase = negate ? -approxPhase : approxPhase;

  if (!flapUp) {
    signedPhase = 32768 - signedPhase;
  }

  return (uint16_t)signedPhase;
}

void slewOffsetTo(float target, unsigned long dt_us) {
  if (dt_us == 0) {
    if (target > state.currentOffset) state.isFlapUp = true;
    else if (target < state.currentOffset) state.isFlapUp = false;
    state.currentOffset = target;
    return;
  }
  float step = (Config::GLIDE_SERVO_SPEED_DPS * (float)dt_us) / 1000000.0f;
  if (state.currentOffset < target) {
    state.isFlapUp = true;
    state.currentOffset += step;
    if (state.currentOffset > target) {
      state.currentOffset = target;
    }
  } else if (state.currentOffset > target) {
    state.isFlapUp = false;
    state.currentOffset -= step;
    if (state.currentOffset < target) {
      state.currentOffset = target;
    }
  }
}

void updateCurrentOffsetSine(int throttle, unsigned long dt_us) {
  throttle = constrain(throttle, Config::THROTTLE_IDLE_MAX, Config::CH_MAX);

  float ratio = (float)(throttle - Config::THROTTLE_IDLE_MAX) /
                (float)(Config::CH_MAX - Config::THROTTLE_IDLE_MAX);
  float dynamicAmp = Config::FLAP_AMP_IDLE - ratio * (Config::FLAP_AMP_IDLE - Config::FLAP_AMP_MAX_THR);

  if (state.phaseNeedsResync) {
    state.phase = sinPhaseFromOffset(state.currentOffset, dynamicAmp, state.isFlapUp);
    state.phaseNeedsResync = false;
  }

  uint32_t speed = Config::FLAP_SPEED_MIN +
                   (uint32_t)(throttle - Config::THROTTLE_IDLE_MAX) *
                   (Config::FLAP_SPEED_MAX - Config::FLAP_SPEED_MIN) /
                   (Config::CH_MAX - Config::THROTTLE_IDLE_MAX);

  if (dt_us > 0) {
    uint32_t phaseDelta = ((uint64_t)speed * dt_us) / 1000000;
    state.phase += phaseDelta;
  }

  state.isFlapUp = isRisingPhase(state.phase);
  state.currentOffset =
      ((float)sin_LUT[state.phase >> 8] * dynamicAmp) / 1000.0f;
}

void updateCurrentOffsetPerch(int perchAngleLevel, unsigned long dt_us) {
  if (state.phaseNeedsResync) {
    state.phaseNeedsResync = false;
  }

  float maxUpAngle = Config::PERCH_BASE_UP_ANGLE +
                   ((perchAngleLevel - Config::CH_MIN) * Config::PERCH_MULT) / 1000.0f;
  constexpr float maxOffset = Config::SERVO_MAX_ANGLE - ServoSystem::Control::centerAngle - Config::PERCH_HEADROOM_DEG;
  float target = constrain(maxUpAngle, 0.0f, maxOffset);

  slewOffsetTo(target, dt_us);
}

void updateCurrentOffsetGlide(unsigned long dt_us) {
  slewOffsetTo(0.0f, dt_us);
  if (fabsf(state.currentOffset) < 0.01f) {
    state.isFlapUp = true;
  }
}

void processFailsafe(unsigned long dt_us) {
  state.isPerching = false;
  state.armStartTime = 0;
  state.disarmStartTime = 0;

  unsigned long lostMs = millis() - state.failsafeEnterMs;
  if (lostMs > Config::RX_HARD_FAILSAFE_MS) {
    state.isArmed = false;
  }

  if (!state.isArmed) {
    ServoSystem::detachAll();
    return;
  }

  slewOffsetTo(Config::FAILSAFE_DIHEDRAL, dt_us);

  if (ServoSystem::state.isAttached) {
    ServoSystem::update(
        ServoSystem::state.centerAngle + state.currentOffset,
        ServoSystem::state.centerAngle - state.currentOffset, false);
  }
  state.phaseNeedsResync = true;
}
}

namespace Receiver {

struct State {
  Protocol currentProto = Protocol::DETECT;
  Protocol trialProto = Protocol::CRSF;
  unsigned long lastProtoSwitchTime = 0;
  bool isLocked = false;
  uint8_t validFramesCount = 0;

  uint8_t crsfBuffer[65];
  uint8_t crsfIndex = 0;
  uint8_t crsfLength = 0;
  uint16_t crsfCrcErrors = 0;
  bool crsfFailSafe = false;

  uint8_t sbusBuffer[25];
  uint8_t sbusIndex = 0;
  bool sbusFailSafe = false;
  bool sbusFrameLost = false;
  uint8_t sbusLostFrames = 0;

  int16_t chMicros_cache[16] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
                                1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
  int16_t emaValues[16] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
                           1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

  bool hasFrame = false;
  unsigned long lastTime = 0;
  unsigned long lastByteTimeCrsf = 0;
  unsigned long lastByteTimeSbus = 0;
  HardwareSerial *serialPort = nullptr;
};

State state;
SemaphoreHandle_t rxMutex = NULL;
static TaskHandle_t rxNotifyTaskHandle = NULL;

void onUartData() {
  if (rxNotifyTaskHandle) {
    xTaskNotifyGive(rxNotifyTaskHandle);
  }
}

static const uint8_t crsf_crc8_table[256] PROGMEM = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83,
    0xD7, 0x02, 0xA8, 0x7D, 0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F, 0xA4, 0x71, 0xDB, 0x0E,
    0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75,
    0x21, 0xF4, 0x5E, 0x8B, 0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0, 0xCF, 0x1A, 0xB0, 0x65,
    0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA,
    0xEE, 0x3B, 0x91, 0x44, 0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16, 0xEF, 0x3A, 0x90, 0x45,
    0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E,
    0x6A, 0xBF, 0x15, 0xC0, 0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36, 0x19, 0xCC, 0x66, 0xB3,
    0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1,
    0xA5, 0x70, 0xDA, 0x0F, 0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D, 0xD6, 0x03, 0xA9, 0x7C,
    0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07,
    0x53, 0x86, 0x2C, 0xF9};

uint8_t crsf_crc8(uint8_t *ptr, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc = pgm_read_byte(&crsf_crc8_table[crc ^ ptr[i]]);
  }
  return crc;
}

bool isValidSbusFooter(uint8_t footer) {
  return (footer == 0x00) || ((footer & 0x0F) == 0x04);
}

uint16_t extract11BitChannel(const uint8_t *payload, uint8_t chIndex,
                             uint8_t payloadLen) {
  if (!payload || payloadLen == 0)
    return 0;
  uint16_t bitOffset = chIndex * 11;
  uint8_t byteOffset = bitOffset / 8;
  if (byteOffset >= payloadLen)
    return 0;
  uint8_t bitShift = bitOffset % 8;
  uint32_t val = payload[byteOffset];
  if (byteOffset + 1 < payloadLen) {
    val |= (uint32_t)payload[byteOffset + 1] << 8;
  }
  if (byteOffset + 2 < payloadLen) {
    val |= (uint32_t)payload[byteOffset + 2] << 16;
  }
  return (val >> bitShift) & 0x07FF;
}

void syncFilterCache();

void parseCrsfChannels(uint8_t *payload, uint8_t payload_len) {
  if (!payload || payload_len < 22)
    return;
  if (rxMutex)
    xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
  state.crsfFailSafe = false;
  bool firstFrame = !state.hasFrame;
  for (uint8_t i = 0; i < 16; i++) {
    uint8_t bitOffset = i * 11;
    uint8_t byteOffset = bitOffset / 8;
    if (byteOffset + 1 >= payload_len)
      break;
    uint16_t chRaw = extract11BitChannel(payload, i, payload_len);
    int32_t mappedRaw =
        Config::CH_MIN + ((int32_t)(chRaw - Config::CRSF_RAW_MIN) * 1000) /
                             (Config::CRSF_RAW_MAX - Config::CRSF_RAW_MIN);
    state.chMicros_cache[i] = (int16_t)constrain(
        mappedRaw, Config::RX_MIN_MICROS, Config::RX_MAX_MICROS);
  }
  if (firstFrame)
    syncFilterCache();
  if (rxMutex)
    xSemaphoreGiveRecursive(rxMutex);
}

void parseSbusChannels() {
  if (rxMutex)
    xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
  bool firstFrame = !state.hasFrame;
  for (uint8_t i = 0; i < 16; i++) {
    uint16_t chRaw = extract11BitChannel(&state.sbusBuffer[1], i, 22);
    int32_t mappedRaw =
        Config::CH_MIN + ((int32_t)(chRaw - Config::SBUS_RAW_MIN) * 1000) /
                             (Config::SBUS_RAW_MAX - Config::SBUS_RAW_MIN);
    state.chMicros_cache[i] = (int16_t)constrain(
        mappedRaw, Config::RX_MIN_MICROS, Config::RX_MAX_MICROS);
  }
  uint8_t flags = state.sbusBuffer[23];
  state.sbusFrameLost = (flags & 0x04) != 0;
  state.sbusFailSafe = (flags & 0x08) != 0;
  if (state.sbusFrameLost) {
    if (state.sbusLostFrames < Config::MAX_LOST_FRAMES)
      state.sbusLostFrames++;
  } else {
    state.sbusLostFrames = 0;
  }
  if (firstFrame)
    syncFilterCache();
  if (rxMutex)
    xSemaphoreGiveRecursive(rxMutex);
}

void resetParsers() {
  state.crsfIndex = 0;
  state.crsfLength = 0;
  state.sbusIndex = 0;
  state.lastByteTimeCrsf = 0;
  state.lastByteTimeSbus = 0;
  if (rxMutex)
    xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
  state.hasFrame = false;
  state.validFramesCount = 0;
  state.crsfFailSafe = false;
  state.sbusFailSafe = false;
  state.sbusFrameLost = false;
  state.sbusLostFrames = 0;
  if (rxMutex)
    xSemaphoreGiveRecursive(rxMutex);
}

void syncFilterCache() {
  for (uint8_t i = 0; i < 16; i++) {
    state.emaValues[i] = state.chMicros_cache[i];
  }
}

struct RcFrame {
  int16_t ch[4] = {1500, 1500, 1500, 1500};   // steering, elevator, throttle, yaw
  int16_t kill = 1500;
  bool connected = false;
  Protocol proto = Protocol::DETECT;
};

bool pollChannels(RcFrame &out) {
  if (!rxMutex || !xSemaphoreTakeRecursive(rxMutex, pdMS_TO_TICKS(1))) {
    return false;
  }
  unsigned long nowMs = millis();

  for (uint8_t ch = 0; ch < 16; ch++) {
    int16_t diff = state.chMicros_cache[ch] - state.emaValues[ch];
    if (diff != 0) {
      int16_t step = diff / (1 << Config::EMA_ALPHA_SHIFT);
      if (step == 0) {
        step = (diff > 0) ? 1 : -1;
      }
      state.emaValues[ch] += step;
    }
  }

  bool protocolReady = !Config::USE_AUTO_DETECT || state.isLocked;
  bool rawConnected = protocolReady && state.hasFrame &&
                      (nowMs - state.lastTime < Config::RX_TIMEOUT_MS);
  if (state.currentProto == Protocol::SBUS &&
      (state.sbusFailSafe || state.sbusLostFrames >= Config::CONN_LOST_THRESHOLD)) {
    rawConnected = false;
  }
  if (state.currentProto == Protocol::CRSF && state.crsfFailSafe) {
    rawConnected = false;
  }

  out.connected = rawConnected;
  out.ch[0] = state.emaValues[Config::CH_STEERING];
  out.ch[1] = state.emaValues[Config::CH_ELEVATOR];
  out.ch[2] = state.emaValues[Config::CH_THROTTLE];
  out.ch[3] = state.emaValues[Config::CH_YAW];
  out.kill = state.chMicros_cache[Config::CH_KILL_SWITCH];
  out.proto = state.currentProto;

  xSemaphoreGiveRecursive(rxMutex);
  return true;
}

bool readCrsf() {
  if (!state.serialPort)
    return false;
  bool gotFrame = false;
  unsigned long now = millis();
  if (state.crsfIndex > 0 &&
      (now - state.lastByteTimeCrsf > Config::RX_BYTE_TIMEOUT_MS))
    state.crsfIndex = 0;

  bool readAny = false;
  for (int i = 0; i < 256 && state.serialPort->available(); i++) {
    uint8_t c = state.serialPort->read();
    readAny = true;
    if (state.crsfIndex == 0 && c != 0xC8)
      continue;
    state.crsfBuffer[state.crsfIndex++] = c;
    if (state.crsfIndex >= sizeof(state.crsfBuffer)) {
      state.crsfIndex = 0;
      continue;
    }
    if (state.crsfIndex == 2) {
      state.crsfLength = c;
      if (state.crsfLength > Config::CRSF_MAX_PAYLOAD_LEN ||
          state.crsfLength < 2) {
        state.crsfIndex = 0;
        if (c == 0xC8) {
          state.crsfBuffer[0] = 0xC8;
          state.crsfIndex = 1;
        }
        continue;
      }
    }
    if (state.crsfIndex == 3) {
      uint8_t type = c;
      if (!((type == 0x16 && state.crsfLength == 24) ||
            (type == 0x14 && state.crsfLength == 12))) {
        state.crsfIndex = 0;
        if (c == 0xC8) {
          state.crsfBuffer[0] = c;
          state.crsfIndex = 1;
        }
        continue;
      }
    }
    if (state.crsfIndex > 2 && state.crsfIndex == state.crsfLength + 2) {
      uint8_t crc = crsf_crc8(&state.crsfBuffer[2], state.crsfLength - 1);
      if (crc == state.crsfBuffer[state.crsfIndex - 1]) {
        if (state.crsfBuffer[2] == 0x16 && state.crsfLength == 24) {
          parseCrsfChannels(&state.crsfBuffer[3], state.crsfLength - 2);
          gotFrame = true;
        } else if (state.crsfBuffer[2] == 0x14 && state.crsfLength >= 6) {
          uint8_t lq = state.crsfBuffer[5];
          bool failsafe = (lq == 0);
          if (rxMutex)
            xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
          state.crsfFailSafe = failsafe;
          if (rxMutex)
            xSemaphoreGiveRecursive(rxMutex);
        }
      } else {
        state.crsfCrcErrors++;
      }
      state.crsfIndex = 0;
    }
  }
  if (readAny)
    state.lastByteTimeCrsf = millis();
  return gotFrame;
}

bool readSbus() {
  if (!state.serialPort)
    return false;
  bool gotFrame = false;
  unsigned long now = millis();
  if (state.sbusIndex > 0 &&
      (now - state.lastByteTimeSbus > Config::RX_BYTE_TIMEOUT_MS))
    state.sbusIndex = 0;

  bool readAny = false;
  for (int i = 0; i < 256 && state.serialPort->available(); i++) {
    uint8_t c = state.serialPort->read();
    readAny = true;
    if (state.sbusIndex == 0 && c != 0x0F)
      continue;
    state.sbusBuffer[state.sbusIndex++] = c;
    if (state.sbusIndex >= 25) {
      state.sbusIndex = 0;
      if (!isValidSbusFooter(state.sbusBuffer[24]))
        continue;
      parseSbusChannels();
      gotFrame = true;
    }
  }
  if (readAny)
    state.lastByteTimeSbus = millis();
  return gotFrame;
}

void switchProtocol() {
  if (!state.serialPort)
    return;

  resetParsers();

  if (state.trialProto == Protocol::CRSF) {
    state.trialProto = Protocol::SBUS;
    uart_set_baudrate(UART_NUM_1, Config::SBUS_BAUD);
    uart_set_word_length(UART_NUM_1, UART_DATA_8_BITS);
    uart_set_parity(UART_NUM_1, UART_PARITY_EVEN);
    uart_set_stop_bits(UART_NUM_1, UART_STOP_BITS_2);
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);
  } else {
    state.trialProto = Protocol::CRSF;
    uart_set_baudrate(UART_NUM_1, Config::CRSF_BAUD);
    uart_set_word_length(UART_NUM_1, UART_DATA_8_BITS);
    uart_set_parity(UART_NUM_1, UART_PARITY_DISABLE);
    uart_set_stop_bits(UART_NUM_1, UART_STOP_BITS_1);
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_INV_DISABLE);
  }
  uart_flush_input(UART_NUM_1);
  while (state.serialPort->available()) {
    state.serialPort->read();
  }
  state.lastProtoSwitchTime = millis();
  LOG_INFO("Switched protocol: %s", state.trialProto == Protocol::CRSF ? "CRSF" : "SBUS");
}

void detectProtocol() {
  if (!state.serialPort)
    return;
  bool shouldUnlock = false;
  if (rxMutex)
    xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
  shouldUnlock = state.isLocked && state.hasFrame &&
                 (millis() - state.lastTime > Config::PROTO_LOCK_TIMEOUT_MS);
  if (shouldUnlock) {
    state.isLocked = false;
    state.currentProto = Protocol::DETECT;
    state.hasFrame = false;
  }
  if (rxMutex)
    xSemaphoreGiveRecursive(rxMutex);

  if (shouldUnlock) {
    resetParsers();
    state.lastProtoSwitchTime = millis();
    LOG_WARN("Protocol unlocked - no frames for 3s, re-detecting...");
  }

  if (!state.isLocked) {
    if (millis() - state.lastProtoSwitchTime >
        Config::PROTO_SWITCH_TIMEOUT_MS) {
      switchProtocol();
    }
  }
}

void lockReceiver() {
  if (rxMutex)
    xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
  state.validFramesCount++;
  if (state.validFramesCount >= 3) {
    state.isLocked = true;
    state.currentProto = state.trialProto;
  }
  if (rxMutex)
    xSemaphoreGiveRecursive(rxMutex);
  if (state.isLocked) {
    LOG_INFO("Locked protocol: %s",
             state.currentProto == Protocol::CRSF ? "CRSF" : "SBUS");
  }
}

void handleAutoDetect() {
  if (Config::USE_AUTO_DETECT)
    detectProtocol();

  bool frameReceived = false;
  if (state.trialProto == Protocol::CRSF)
    frameReceived = readCrsf();
  else
    frameReceived = readSbus();

  if (frameReceived) {
    if (rxMutex)
      xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
    state.hasFrame = true;
    state.lastTime = millis();
    if (rxMutex)
      xSemaphoreGiveRecursive(rxMutex);
    if (Config::USE_AUTO_DETECT && !state.isLocked) {
      lockReceiver();
    }
  }
}

void init(HardwareSerial *serial, TaskHandle_t taskToNotify) {
  if (rxMutex == NULL) {
    rxMutex = xSemaphoreCreateRecursiveMutex();
    configASSERT(rxMutex != NULL);
  }
  state.serialPort = serial;
  rxNotifyTaskHandle = taskToNotify;
  if (!state.serialPort)
    return;
  state.serialPort->setRxBufferSize(Config::RX_BUFFER_SIZE);

  if (Config::USE_AUTO_DETECT) {
    state.serialPort->begin(Config::CRSF_BAUD, SERIAL_8N1, Config::RX_PIN, -1,
                            false);
    state.lastProtoSwitchTime = millis();
  } else {
    state.trialProto = Config::FIXED_PROTOCOL;
    state.currentProto = Config::FIXED_PROTOCOL;
    state.isLocked = true;
    if (state.trialProto == Protocol::CRSF) {
      state.serialPort->begin(Config::CRSF_BAUD, SERIAL_8N1, Config::RX_PIN, -1,
                              false);
    } else {
      state.serialPort->begin(Config::SBUS_BAUD, SERIAL_8E2, Config::RX_PIN, -1,
                              true);
    }
  }
  state.serialPort->onReceive(onUartData);
}

}

HardwareSerial rxSerial(1);
TaskHandle_t rxTaskHandle = NULL;

void handleFailsafeMode(unsigned long dt_us, bool connected,
                        unsigned long currentMillis) {
  if (connected) {
    Flight::state.failsafeEnterMs = 0;
    Flight::state.mode =
        Flight::state.isArmed ? FlightMode::CONNECTED : FlightMode::INIT;
    return;
  }
  if (Flight::state.failsafeEnterMs == 0) {
    Flight::state.failsafeEnterMs = currentMillis;
    if (Flight::state.failsafeEnterMs == 0)
      Flight::state.failsafeEnterMs = 1;
  }
  Flight::processFailsafe(dt_us);
}

void handleInitMode(unsigned long dt_us, bool connected,
                    unsigned long currentMillis, int throttle, int yaw) {
  if (!connected) {
    Flight::state.mode = FlightMode::FAILSAFE;
    Flight::state.failsafeEnterMs = currentMillis;
    if (Flight::state.failsafeEnterMs == 0)
      Flight::state.failsafeEnterMs = 1;
    return;
  }

  if (throttle < Config::THROTTLE_IDLE_MAX && yaw > Config::ARM_YAW_HIGH) {
    if (Flight::state.armStartTime == 0)
      Flight::state.armStartTime = currentMillis;
    else if (currentMillis - Flight::state.armStartTime >=
             Config::STICK_HOLD_MS) {
      Flight::state.isArmed = true;
      Flight::state.armStartTime = 0;
      Flight::state.phaseNeedsResync = true;
      Flight::state.mode = FlightMode::CONNECTED;
      Flight::state.disarmStartTime = 0;
      Flight::state.isPerching = false;
      Flight::state.perchAngleLevel = Config::CH_MIN;
      Flight::state.currentOffset = 0.0f;
      Flight::state.wasFlapping = false;
      Flight::state.wasPerching = false;
      ServoSystem::state.lastUsL = Config::SERVO_US_CENTER;
      ServoSystem::state.lastUsR = Config::SERVO_US_CENTER;
      ServoSystem::state.filteredL = (int32_t)Config::SERVO_US_CENTER << Config::SERVO_EMA_ALPHA_SHIFT;
      ServoSystem::state.filteredR = (int32_t)Config::SERVO_US_CENTER << Config::SERVO_EMA_ALPHA_SHIFT;
      ServoSystem::attachAll();
    }
  } else {
    Flight::state.armStartTime = 0;
  }

  Flight::state.failsafeEnterMs = 0;
  Flight::updateCurrentOffsetGlide(dt_us);
  if (ServoSystem::state.isAttached) {
    ServoSystem::update(
        ServoSystem::state.centerAngle + Flight::state.currentOffset,
        ServoSystem::state.centerAngle - Flight::state.currentOffset,
        false);
  }
}

void handleDisarming(unsigned long currentMillis, int throttle, int yaw) {
  if (throttle < Config::THROTTLE_IDLE_MAX && yaw < Config::ARM_YAW_LOW) {
    if (Flight::state.disarmStartTime == 0)
      Flight::state.disarmStartTime = currentMillis;
    else if (currentMillis - Flight::state.disarmStartTime >=
             Config::STICK_HOLD_MS) {
      Flight::state.isArmed = false;
      Flight::state.currentOffset = 0.0f;
      Flight::state.isPerching = false;
      Flight::state.isFlapping = false;
      Flight::state.wasFlapping = false;
      Flight::state.wasPerching = false;
      Flight::state.phaseNeedsResync = true;
      ServoSystem::detachAll();
      Flight::state.mode = FlightMode::INIT;
      Flight::state.armStartTime = 0;
      Flight::state.disarmStartTime = 0;
    }
  } else {
    Flight::state.disarmStartTime = 0;
  }
}

void handleModeTransition(int elevator, int throttle) {
  Flight::state.isFlapping = (throttle >= Config::THROTTLE_IDLE_MAX);

  if (Flight::state.isFlapping && !Flight::state.wasFlapping) {
    Flight::state.phaseNeedsResync = true;
  }
  Flight::state.wasFlapping = Flight::state.isFlapping;

  if (throttle < Config::THROTTLE_IDLE_MAX) {
    if (elevator < Config::PERCH_ENTER_ELEV)
      Flight::state.isPerching = true;
    else if (elevator > Config::PERCH_EXIT_ELEV) {
      Flight::state.isPerching = false;
      Flight::state.perchAngleLevel = Config::CH_MIN;
    }
    Flight::state.mode =
        Flight::state.isPerching ? FlightMode::PERCHING : FlightMode::CONNECTED;
  } else {
    Flight::state.isPerching = false;
    Flight::state.perchAngleLevel = Config::CH_MIN;
    Flight::state.mode = FlightMode::CONNECTED;
  }

  if (Flight::state.mode == FlightMode::PERCHING &&
      !Flight::state.wasPerching) {
    Flight::state.phaseNeedsResync = true;
  }
  Flight::state.wasPerching = (Flight::state.mode == FlightMode::PERCHING);
}

void computeServoTargets(unsigned long dt_us, int steering, int elevator,
                         int throttle) {
  float steerValue = Config::mapTrimFloat(steering);
  float elevOffset = (Flight::state.mode != FlightMode::PERCHING)
                         ? Config::mapTrimFloat(elevator)
                         : 0.0f;

  if (Flight::state.mode == FlightMode::PERCHING) {
    int currentPull = Config::CH_MIN;
    if (elevator <= Config::PERCH_ENTER_ELEV) {
      currentPull = Config::CH_MIN + (int32_t)(Config::PERCH_ENTER_ELEV - elevator) *
                    (Config::CH_MAX - Config::CH_MIN) /
                    (Config::PERCH_ENTER_ELEV - Config::CH_MIN);
      currentPull = constrain(currentPull, Config::CH_MIN, Config::CH_MAX);
    }
    Flight::state.perchAngleLevel = currentPull;
    Flight::updateCurrentOffsetPerch(Flight::state.perchAngleLevel, dt_us);
  } else {
    if (throttle < Config::THROTTLE_IDLE_MAX)
      Flight::updateCurrentOffsetGlide(dt_us);
    else
      Flight::updateCurrentOffsetSine(throttle, dt_us);
  }

  ServoSystem::update(
      ServoSystem::state.centerAngle + Flight::state.currentOffset + elevOffset + steerValue,
      ServoSystem::state.centerAngle - Flight::state.currentOffset - elevOffset + steerValue,
      Flight::state.isFlapping);
}

void handleConnectedMode(unsigned long dt_us, bool connected,
                         unsigned long currentMillis, int steering,
                         int elevator, int throttle, int yaw) {
  if (!connected) {
    Flight::state.mode = FlightMode::FAILSAFE;
    Flight::state.failsafeEnterMs = currentMillis;
    if (Flight::state.failsafeEnterMs == 0)
      Flight::state.failsafeEnterMs = 1;
    return;
  }

  handleDisarming(currentMillis, throttle, yaw);
  if (Flight::state.mode == FlightMode::INIT)
    return;

  Flight::state.failsafeEnterMs = 0;
  handleModeTransition(elevator, throttle);
  computeServoTargets(dt_us, steering, elevator, throttle);
}

void processFlightLogic(unsigned long dt_us, int steering, int elevator,
                        int throttle, int yaw, bool connected) {
  unsigned long currentMillis = millis();

  switch (Flight::state.mode) {
  case FlightMode::FAILSAFE:
    handleFailsafeMode(dt_us, connected, currentMillis);
    break;
  case FlightMode::INIT:
    handleInitMode(dt_us, connected, currentMillis, throttle, yaw);
    break;
  case FlightMode::PERCHING:
  case FlightMode::CONNECTED:
    handleConnectedMode(dt_us, connected, currentMillis, steering, elevator,
                        throttle, yaw);
    break;
  }
}

void setupWatchdog() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdt_cfg = {};
  wdt_cfg.timeout_ms = Config::WDT_TIMEOUT_SEC * 1000;
  wdt_cfg.idle_core_mask = 0;
  wdt_cfg.trigger_panic = true;
  esp_err_t err = esp_task_wdt_init(&wdt_cfg);
  if (err == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_reconfigure(&wdt_cfg);
  } else if (err != ESP_OK) {
    LOG_ERROR("WDT Init failed");
  }
#else
  esp_task_wdt_init(Config::WDT_TIMEOUT_SEC, false);
#endif
  esp_task_wdt_add(NULL);
}

void vReceiverTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    esp_task_wdt_reset();
    Receiver::handleAutoDetect();
  }
}

#include <atomic>

struct TelemetrySnapshot {
  bool connected;
  bool armed;
  bool servoAttached;
  Protocol proto;
  int16_t throttle;
  int16_t offsetDeg;
};

static TelemetrySnapshot g_telemBuf[2];
static std::atomic<uint8_t> g_telemIdx{0};

inline void publishTelemetry(const TelemetrySnapshot &s) {
  uint8_t next = 1 - g_telemIdx.load(std::memory_order_relaxed);
  g_telemBuf[next] = s;
  g_telemIdx.store(next, std::memory_order_release);
}

inline TelemetrySnapshot readTelemetry() {
  uint8_t cur = g_telemIdx.load(std::memory_order_acquire);
  return g_telemBuf[cur];
}

void flightCoreTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(3);
  unsigned long lastUs = micros();
  static Receiver::RcFrame rc;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    esp_task_wdt_reset();

    unsigned long nowUs = micros();
    unsigned long dt_us = nowUs - lastUs;
    if (dt_us == 0 || dt_us > 100000)
      dt_us = Config::LOOP_DT_US;
    lastUs = nowUs;

    static uint8_t pollFailCount = 0;
    if (!Receiver::pollChannels(rc)) {
      if (++pollFailCount >= 3) {
        rc.connected = false;
      }
    } else {
      pollFailCount = 0;
    }

    bool killActive = rc.kill > Config::KILL_SWITCH_THRESHOLD;

    static uint8_t killDebounce = 0;
    if (killActive) {
      if (killDebounce < 10)
        killDebounce++;
    } else {
      killDebounce = 0;
    }

    if (killDebounce >= 5) {
      Flight::state.isArmed = false;
      Flight::state.armStartTime = 0;
      Flight::state.disarmStartTime = 0;
      Flight::state.isPerching = false;
      Flight::state.isFlapping = false;
      Flight::state.wasFlapping = false;
      Flight::state.wasPerching = false;
      Flight::state.phaseNeedsResync = true;
      Flight::state.currentOffset = 0.0f;
      Flight::state.mode = FlightMode::INIT;
      Flight::state.failsafeEnterMs = 0;
      ServoSystem::detachAll();
    } else {
      processFlightLogic(dt_us, rc.ch[0], rc.ch[1], rc.ch[2],
                         rc.ch[3], rc.connected);
    }

    publishTelemetry({rc.connected, Flight::state.isArmed,
                      ServoSystem::state.isAttached, rc.proto,
                      rc.ch[2], (int16_t)Flight::state.currentOffset});
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  logMutex = xSemaphoreCreateRecursiveMutex();
  Serial.printf("Reset reason: %d\n", esp_reset_reason());
  setupWatchdog();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  pinMode(Config::LED_PIN, OUTPUT);
  digitalWrite(Config::LED_PIN, HIGH);

  xTaskCreate(vReceiverTask, "RX_Task", 8192, NULL, configMAX_PRIORITIES - 2,
              &rxTaskHandle);

  Receiver::init(&rxSerial, rxTaskHandle);

  ServoSystem::state.lastUsL = Config::SERVO_US_CENTER;
  ServoSystem::state.lastUsR = Config::SERVO_US_CENTER;

  xTaskCreatePinnedToCore(flightCoreTask, "FlightCore", 4096, NULL, 5, NULL, 0);

  LOG_INFO("PHELAN BUTTERFLY 2IN1 SINE READY");
}

void loop() {
  esp_task_wdt_reset();

  unsigned long currentMillis = millis();

  TelemetrySnapshot t = readTelemetry();

  if (!t.connected) {
    digitalWrite(Config::LED_PIN, (currentMillis / 250) % 2 == 0 ? HIGH : LOW);
  } else {
    digitalWrite(Config::LED_PIN,
                 t.armed
                     ? HIGH
                     : ((currentMillis / 1000) % 2 == 0 ? HIGH : LOW));
  }

  static unsigned long lastLogTime = 0;
  if (currentMillis - lastLogTime >= 1000) {
    lastLogTime = currentMillis;
    if (logMutex)
      xSemaphoreTakeRecursive(logMutex, portMAX_DELAY);
    if (Serial.availableForWrite() >= 64) {
      if (t.connected) {
        Serial.print(t.armed ? F("[FLY] ") : F("[ARM?] "));
        if (t.proto == Protocol::CRSF)
          Serial.print(F("CRSF "));
        else if (t.proto == Protocol::SBUS)
          Serial.print(F("SBUS "));
        else
          Serial.print(F("DETC "));
        Serial.print(F("CH3:"));
        Serial.print(t.throttle);
        Serial.print(F(" Off:"));
        Serial.print(t.offsetDeg);
        Serial.print(F(" SVO:"));
        Serial.print(t.servoAttached ? F("OK") : F("DET"));
        Serial.print(F(" LED:"));
        Serial.println(digitalRead(Config::LED_PIN) ? F("ON") : F("OFF"));
      } else {
        Serial.println(F("[WARN] NO SIGNAL or DETECTING..."));
      }
    }
    if (logMutex)
      xSemaphoreGiveRecursive(logMutex);
  }

  delay(2);
}
