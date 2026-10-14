#include <Arduino.h>
#include <atomic>
#include <esp_task_wdt.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#if __has_include(<driver/temperature_sensor.h>)
#include <driver/temperature_sensor.h>
#define HAS_NEW_TEMP_SENSOR 1
#elif __has_include(<driver/temp_sensor.h>)
#include <driver/temp_sensor.h>
#define HAS_LEGACY_TEMP_SENSOR 1
#endif
#include "esp_wifi.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#if __has_include(<hal/brownout_ll.h>)
#include "hal/brownout_ll.h"
__attribute__((constructor)) static void disable_bod_early() {
  brownout_ll_bod_enable(false);
  brownout_ll_intr_enable(false);
}
#endif
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

enum class Protocol { DETECT, CRSF, SBUS, WIFI };

WiFiServer tcpBridgeServer(8880);
WiFiClient tcpBridgeClient;
bool isPassthroughMode = false;

WiFiUDP udpBridge;
IPAddress udpTargetIP(192, 168, 4, 2);
uint16_t udpTargetPort = 14550;
bool udpBridgeEnabled = false;

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

constexpr uint32_t CRSF_BAUD = 420000;
constexpr uint32_t SBUS_BAUD = 100000;
constexpr size_t RX_BUFFER_SIZE = 512;

constexpr unsigned long RX_TIMEOUT_MS = 200;
constexpr unsigned long RX_BYTE_TIMEOUT_MS = 5;
constexpr uint32_t WDT_TIMEOUT_SEC = 8;
constexpr unsigned long PROTO_LOCK_TIMEOUT_MS = 3000;
constexpr unsigned long PROTO_SWITCH_TIMEOUT_MS = 1500;
constexpr uint8_t CONN_LOST_THRESHOLD = 5;
constexpr unsigned long STICK_HOLD_MS = 2000;
constexpr unsigned long RX_HARD_FAILSAFE_MS = 3000;

constexpr int THROTTLE_IDLE_MAX = 1050;
constexpr bool USE_AUTO_DETECT = true;
constexpr Protocol FIXED_PROTOCOL = Protocol::CRSF;

constexpr unsigned long LOOP_DT_US = 3333;
constexpr int FLAP_AMP = 65;
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

inline int mapTrim15(int x) {
  int d = x - CH_CENTER;
  if (abs(d) <= STICK_DEADBAND)
    return 0;
  int s = (d > 0) ? (d - STICK_DEADBAND) : (d + STICK_DEADBAND);
  return constrain(s / 30, -15, 15);
}

}

#pragma pack(push, 1)
struct FlightPacket {
  uint8_t header;
  uint8_t flags;
  int8_t throttle;
  int8_t yaw;
  int8_t pitch;
  int8_t roll;
};

struct TelemetryPacket {
  uint8_t header;
  uint8_t flags;
  int8_t offset;
  uint8_t servoL;
  uint8_t servoR;
  uint8_t cpuTemp;
  uint8_t cpuMhz;
  uint16_t uptimeSec;
  uint8_t timeH;
  uint8_t timeM;
  uint8_t timeS;
};
#pragma pack(pop)

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

enum class FlightMode { INIT, CONNECTED, PERCHING, FAILSAFE };

namespace SysMonitor {
struct DateTime {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

DateTime bootTime;
unsigned long bootMillis = 0;
bool timeInitialized = false;

void parseCompileTime() {
  const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                          "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  char mStr[4];
  int d = 1, y = 2026, h = 0, min = 0, s = 0;

  if (sscanf(__DATE__, "%3s %d %d", mStr, &d, &y) == 3) {
    bootTime.day = d;
    bootTime.year = y;
    bootTime.month = 1;
    for (int i = 0; i < 12; i++) {
      if (strcmp(mStr, months[i]) == 0) {
        bootTime.month = i + 1;
        break;
      }
    }
  }
  if (sscanf(__TIME__, "%d:%d:%d", &h, &min, &s) == 3) {
    bootTime.hour = h;
    bootTime.minute = min;
    bootTime.second = s;
  }
  bootMillis = millis();
  timeInitialized = true;
}

void getFormattedTime(char *buffer, size_t len) {
  if (!buffer || len == 0) return;
  if (len < 20) {
    buffer[0] = '\0';
    return;
  }
  if (!timeInitialized) {
    snprintf(buffer, len, "0000-00-00 00:00:00");
    return;
  }
  unsigned long secondsSinceBoot = (millis() - bootMillis) / 1000;

  int s = bootTime.second + secondsSinceBoot;
  int m = bootTime.minute + s / 60;
  s %= 60;
  int h = bootTime.hour + m / 60;
  m %= 60;
  int d = bootTime.day + h / 24;
  h %= 24;

  int y = bootTime.year;
  int mon = bootTime.month;

  while (true) {
    int daysInMon = 30;
    if (mon == 1 || mon == 3 || mon == 5 || mon == 7 || mon == 8 || mon == 10 ||
        mon == 12) {
      daysInMon = 31;
    } else if (mon == 2) {
      bool isLeap = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
      daysInMon = isLeap ? 29 : 28;
    } else {
      daysInMon = 30;
    }

    if (d <= daysInMon)
      break;
    d -= daysInMon;
    mon++;
    if (mon > 12) {
      mon = 1;
      y++;
    }
  }

  snprintf(buffer, len, "%04d-%02d-%02d %02d:%02d:%02d", y, mon, d, h, m, s);
}

void setTime(int y, int mon, int d, int h, int m, int s) {
  bootTime.year = y;
  bootTime.month = mon;
  bootTime.day = d;
  bootTime.hour = h;
  bootTime.minute = m;
  bootTime.second = s;
  bootMillis = millis();
  timeInitialized = true;
}

void checkSerialTimeSync() {
  static char syncBuf[64];
  static uint8_t syncIdx = 0;
  static bool inOverflow = false;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (!inOverflow && syncIdx > 0) {
        syncBuf[syncIdx] = '\0';
        if (strncmp(syncBuf, "SETTIME=", 8) == 0) {
          int y, mon, d, h, m, s;
          if (sscanf(syncBuf + 8, "%d-%d-%d %d:%d:%d", &y, &mon, &d, &h, &m,
                     &s) == 6) {
            bootTime.year = y;
            bootTime.month = mon;
            bootTime.day = d;
            bootTime.hour = h;
            bootTime.minute = m;
            bootTime.second = s;
            bootMillis = millis();
            timeInitialized = true;
            Serial.println(
                F("[INFO] System time synchronized successfully to Hanoi time!"));
          }
        }
      }
      syncIdx = 0;
      inOverflow = false;
    } else if (!inOverflow) {
      if (syncIdx < sizeof(syncBuf) - 1) {
        syncBuf[syncIdx++] = c;
      } else {
        inOverflow = true;
        syncIdx = 0;
      }
    }
  }
}

float filteredTemp = 42.0f;
bool tempFirstSample = true;

float getCPUTemp() {
  float currentRaw = 42.0f;
#if defined(HAS_NEW_TEMP_SENSOR) && HAS_NEW_TEMP_SENSOR
  static temperature_sensor_handle_t temp_sensor = NULL;
  static bool sensor_inited = false;
  if (!sensor_inited) {
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    if (temperature_sensor_install(&temp_sensor_config, &temp_sensor) == ESP_OK) {
      if (temperature_sensor_enable(temp_sensor) == ESP_OK) {
        sensor_inited = true;
      }
    }
  }
  if (sensor_inited) {
    float ts_val = 0.0f;
    if (temperature_sensor_get_celsius(temp_sensor, &ts_val) == ESP_OK) {
      currentRaw = ts_val;
    } else {
      currentRaw = temperatureRead();
    }
  } else {
    currentRaw = temperatureRead();
  }
#else
  currentRaw = temperatureRead();
#endif

  if (isnan(currentRaw) || currentRaw < 10.0f || currentRaw > 105.0f) {
    currentRaw = 42.0f;
  }

  if (tempFirstSample) {
    filteredTemp = currentRaw;
    tempFirstSample = false;
  } else {
    filteredTemp = (filteredTemp * 0.90f) + (currentRaw * 0.10f);
  }
  return filteredTemp;
}

uint32_t getCPUMhz() { return getCpuFrequencyMhz(); }

void getAsciiWave(char *buffer, size_t len, float offset, float maxAmp) {
  if (!buffer || len == 0) return;
  if (len < 23) {
    buffer[0] = '\0';
    return;
  }
  int width = 20;
  if (maxAmp <= 0)
    maxAmp = 1.0f;
  int pos = (int)((offset + maxAmp) / (2.0f * maxAmp) * width);
  pos = constrain(pos, 0, width);

  buffer[0] = '[';
  for (int i = 0; i < width; i++) {
    if (i == width / 2) {
      buffer[i + 1] = '|';
    } else if (i == pos) {
      buffer[i + 1] = '*';
    } else {
      buffer[i + 1] = '-';
    }
  }
  buffer[width + 1] = ']';
  buffer[width + 2] = '\0';
}
}

namespace ServoSystem {

struct Control {
  Servo left;
  Servo right;

  static constexpr int centerAngle = 90;
  int lastPosL = centerAngle;
  int lastPosR = centerAngle;
  int32_t filteredL = 90 << Config::SERVO_EMA_ALPHA_SHIFT;
  int32_t filteredR = 90 << Config::SERVO_EMA_ALPHA_SHIFT;
  bool isAttached = false;
};

Control state;

void attachAll() {
  if (!state.isAttached) {
    state.left.setPeriodHertz(Config::SERVO_HZ);
    state.right.setPeriodHertz(Config::SERVO_HZ);
    state.left.attach(Config::SERVO_L_PIN, 500, 2500);
    state.right.attach(Config::SERVO_R_PIN, 500, 2500);
    int outL =
        Config::REVERSE_SERVO_L ? (180 - state.lastPosL) : state.lastPosL;
    int outR =
        Config::REVERSE_SERVO_R ? (180 - state.lastPosR) : state.lastPosR;
    state.left.write(outL);
    state.right.write(outR);
    state.filteredL = (int32_t)state.lastPosL << Config::SERVO_EMA_ALPHA_SHIFT;
    state.filteredR = (int32_t)state.lastPosR << Config::SERVO_EMA_ALPHA_SHIFT;
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

void update(int targetL, int targetR, bool bypassFilter = false) {
  if (!state.isAttached)
    return;
  targetL =
      constrain(targetL, Config::SERVO_MIN_ANGLE, Config::SERVO_MAX_ANGLE);
  targetR =
      constrain(targetR, Config::SERVO_MIN_ANGLE, Config::SERVO_MAX_ANGLE);

  if (bypassFilter) {
    state.filteredL = (int32_t)targetL << Config::SERVO_EMA_ALPHA_SHIFT;
    state.filteredR = (int32_t)targetR << Config::SERVO_EMA_ALPHA_SHIFT;
  } else {
    state.filteredL +=
        targetL - (state.filteredL >> Config::SERVO_EMA_ALPHA_SHIFT);
    state.filteredR +=
        targetR - (state.filteredR >> Config::SERVO_EMA_ALPHA_SHIFT);
  }

  targetL = state.filteredL >> Config::SERVO_EMA_ALPHA_SHIFT;
  targetR = state.filteredR >> Config::SERVO_EMA_ALPHA_SHIFT;
  targetL =
      constrain(targetL, Config::SERVO_MIN_ANGLE, Config::SERVO_MAX_ANGLE);
  targetR =
      constrain(targetR, Config::SERVO_MIN_ANGLE, Config::SERVO_MAX_ANGLE);

  if (abs(targetL - state.lastPosL) >= 2) {
    int outL = Config::REVERSE_SERVO_L ? (180 - targetL) : targetL;
    state.left.write(outL);
    state.lastPosL = targetL;
  }
  if (abs(targetR - state.lastPosR) >= 2) {
    int outR = Config::REVERSE_SERVO_R ? (180 - targetR) : targetR;
    state.right.write(outR);
    state.lastPosR = targetR;
  }
}

}

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

static const int16_t PROGMEM tri_LUT[256] = {
    0,     16,   31,   47,   63,   78,   94,   109,  125,  141,  156,  172,
    188,   203,  219,  234,  250,  266,  281,  297,  313,  328,  344,  359,
    375,   391,  406,  422,  438,  453,  469,  484,  500,  516,  531,  547,
    563,   578,  594,  609,  625,  641,  656,  672,  688,  703,  719,  734,
    750,   766,  781,  797,  813,  828,  844,  859,  875,  891,  906,  922,
    938,   953,  969,  984,  1000, 984,  969,  953,  938,  922,  906,  891,
    875,   859,  844,  828,  813,  797,  781,  766,  750,  734,  719,  703,
    688,   672,  656,  641,  625,  609,  594,  578,  563,  547,  531,  516,
    500,   484,  469,  453,  438,  422,  406,  391,  375,  359,  344,  328,
    313,   297,  281,  266,  250,  234,  219,  203,  188,  172,  156,  141,
    125,   109,  94,   78,   63,   47,   31,   16,   0,    -16,  -31,  -47,
    -63,   -78,  -94,  -109, -125, -141, -156, -172, -188, -203, -219, -234,
    -250,  -266, -281, -297, -313, -328, -344, -359, -375, -391, -406, -422,
    -438,  -453, -469, -484, -500, -516, -531, -547, -563, -578, -594, -609,
    -625,  -641, -656, -672, -688, -703, -719, -734, -750, -766, -781, -797,
    -813,  -828, -844, -859, -875, -891, -906, -922, -938, -953, -969, -984,
    -1000, -984, -969, -953, -938, -922, -906, -891, -875, -859, -844, -828,
    -813,  -797, -781, -766, -750, -734, -719, -703, -688, -672, -656, -641,
    -625,  -609, -594, -578, -563, -547, -531, -516, -500, -484, -469, -453,
    -438,  -422, -406, -391, -375, -359, -344, -328, -313, -297, -281, -266,
    -250,  -234, -219, -203, -188, -172, -156, -141, -125, -109, -94,  -78,
    -63,   -47,  -31,  -16};

static const uint16_t PROGMEM atri_LUT[256] = {
    0,     64,    128,   193,   257,   321,   386,   450,   514,   579,   643,
    707,   772,   836,   900,   965,   1029,  1093,  1158,  1222,  1286,  1351,
    1415,  1479,  1543,  1608,  1672,  1736,  1801,  1865,  1929,  1994,  2058,
    2122,  2187,  2251,  2315,  2380,  2444,  2508,  2572,  2637,  2701,  2765,
    2830,  2894,  2958,  3023,  3087,  3151,  3216,  3280,  3344,  3409,  3473,
    3537,  3601,  3666,  3730,  3794,  3859,  3923,  3987,  4052,  4116,  4180,
    4245,  4309,  4373,  4437,  4502,  4566,  4630,  4695,  4759,  4823,  4888,
    4952,  5016,  5081,  5145,  5209,  5273,  5338,  5402,  5466,  5531,  5595,
    5659,  5724,  5788,  5852,  5917,  5981,  6045,  6109,  6174,  6238,  6302,
    6367,  6431,  6495,  6560,  6624,  6688,  6753,  6817,  6881,  6945,  7010,
    7074,  7138,  7203,  7267,  7331,  7396,  7460,  7524,  7589,  7653,  7717,
    7781,  7846,  7910,  7974,  8039,  8103,  8167,  8232,  8296,  8360,  8425,
    8489,  8553,  8617,  8682,  8746,  8810,  8875,  8939,  9003,  9068,  9132,
    9196,  9261,  9325,  9389,  9453,  9518,  9582,  9646,  9711,  9775,  9839,
    9904,  9968,  10032, 10097, 10161, 10225, 10289, 10354, 10418, 10482, 10547,
    10611, 10675, 10740, 10804, 10868, 10933, 10997, 11061, 11125, 11190, 11254,
    11318, 11383, 11447, 11511, 11576, 11640, 11704, 11769, 11833, 11897, 11961,
    12026, 12090, 12154, 12219, 12283, 12347, 12412, 12476, 12540, 12605, 12669,
    12733, 12797, 12862, 12926, 12990, 13055, 13119, 13183, 13248, 13312, 13376,
    13441, 13505, 13569, 13633, 13698, 13762, 13826, 13891, 13955, 14019, 14084,
    14148, 14212, 14277, 14341, 14405, 14469, 14534, 14598, 14662, 14727, 14791,
    14855, 14920, 14984, 15048, 15113, 15177, 15241, 15305, 15370, 15434, 15498,
    15563, 15627, 15691, 15756, 15820, 15884, 15949, 16013, 16077, 16141, 16206,
    16270, 16334, 16384};

void updateCurrentOffsetTriangle(int throttle, unsigned long dt_us) {
  throttle = constrain(throttle, Config::THROTTLE_IDLE_MAX, Config::CH_MAX);

  float dynamicAmp = 55.0f - ((float)(throttle - Config::THROTTLE_IDLE_MAX) /
                              (float)(Config::CH_MAX - Config::THROTTLE_IDLE_MAX)) * 20.0f;

  if (state.phaseNeedsResync) {
    int32_t ratioScaled =
        (int32_t)(state.currentOffset * 1000.0f / dynamicAmp);
    ratioScaled = constrain(ratioScaled, -1000, 1000);

    bool negate = (ratioScaled < 0);
    int32_t abs_ratio = negate ? -ratioScaled : ratioScaled;

    int index = (abs_ratio * 255) / 1000;
    uint16_t approxPhase = atri_LUT[index];

    int32_t signedPhase = negate ? -approxPhase : approxPhase;

    if (!state.isFlapUp) {
      signedPhase = 32768 - signedPhase;
    }

    state.phase = (uint16_t)signedPhase;
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

  float oldOffset = state.currentOffset;
  state.currentOffset =
      ((float)tri_LUT[state.phase >> 8] * dynamicAmp) / 1000.0f;

  float delta = state.currentOffset - oldOffset;
  if (delta > 0.1f) {
    state.isFlapUp = true;
  } else if (delta < -0.1f) {
    state.isFlapUp = false;
  }
}

void updateCurrentOffsetPerch(int perchAngleLevel, unsigned long dt_us) {
  if (state.phaseNeedsResync) {
    state.phaseNeedsResync = false;
  }

  float maxUpAngle =
      Config::PERCH_BASE_UP_ANGLE +
      ((perchAngleLevel - Config::CH_MIN) * Config::PERCH_MULT) / 1000.0f;
  constexpr float maxOffset = Config::SERVO_MAX_ANGLE -
                              ServoSystem::Control::centerAngle -
                              Config::PERCH_HEADROOM_DEG;
  float target = constrain(maxUpAngle, 0.0f, maxOffset);

  if (dt_us > 0) {
    float step = (Config::GLIDE_SERVO_SPEED_DPS * (float)dt_us) / 1000000.0f;
    if (state.currentOffset < target) {
      state.currentOffset += step;
      if (state.currentOffset > target)
        state.currentOffset = target;
    } else if (state.currentOffset > target) {
      state.currentOffset -= step;
      if (state.currentOffset < target)
        state.currentOffset = target;
    }
  } else {
    state.currentOffset = target;
  }
}

void updateCurrentOffsetGlide(unsigned long dt_us) {
  if (state.currentOffset == 0)
    return;

  float step = (Config::GLIDE_SERVO_SPEED_DPS * (float)dt_us) / 1000000.0f;

  if (state.currentOffset > 0) {
    state.currentOffset -= step;
    if (state.currentOffset < 0)
      state.currentOffset = 0;
  } else {
    state.currentOffset += step;
    if (state.currentOffset > 0)
      state.currentOffset = 0;
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

  float step = (Config::GLIDE_SERVO_SPEED_DPS * (float)dt_us) / 1000000.0f;

  if (state.currentOffset < Config::FAILSAFE_DIHEDRAL) {
    state.currentOffset += step;
    if (state.currentOffset > Config::FAILSAFE_DIHEDRAL) {
      state.currentOffset = Config::FAILSAFE_DIHEDRAL;
    }
  } else if (state.currentOffset > Config::FAILSAFE_DIHEDRAL) {
    state.currentOffset -= step;
    if (state.currentOffset < Config::FAILSAFE_DIHEDRAL) {
      state.currentOffset = Config::FAILSAFE_DIHEDRAL;
    }
  }

  if (ServoSystem::state.isAttached) {
    ServoSystem::update(
        ServoSystem::state.centerAngle + (int)state.currentOffset,
        ServoSystem::state.centerAngle - (int)state.currentOffset, false);
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
  bool isConnected = false;
  unsigned long lastTime = 0;
  unsigned long lastByteTimeCrsf = 0;
  unsigned long lastByteTimeSbus = 0;
  unsigned long switchDelayStart = 0;
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

bool isValidSbusFooter(uint8_t footer) { return footer == 0x00; }

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
  state.hasFrame = false;
  state.validFramesCount = 0;
  if (rxMutex)
    xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
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

void updateFilters() {
  if (rxMutex)
    xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
  for (uint8_t ch = 0; ch < 16; ch++) {
    int16_t diff = state.chMicros_cache[ch] - state.emaValues[ch];
    if (diff != 0) {
      state.emaValues[ch] += (diff >> Config::EMA_ALPHA_SHIFT);
      if ((diff >> Config::EMA_ALPHA_SHIFT) == 0) {
        state.emaValues[ch] += (diff > 0) ? 1 : -1;
      }
    }
  }
  if (rxMutex)
    xSemaphoreGiveRecursive(rxMutex);
}

int16_t getFiltered(byte ch) {
  if (ch >= 16)
    return Config::CH_CENTER;
  int16_t val;
  if (rxMutex)
    xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
  val = state.emaValues[ch];
  if (rxMutex)
    xSemaphoreGiveRecursive(rxMutex);
  return val;
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
          state.crsfLength < 2)
        state.crsfIndex = 0;
    }
    if (state.crsfIndex == 3) {
      uint8_t type = c;
      if (!((type == 0x16 && state.crsfLength == 24) || type == 0x14)) {
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
  state.trialProto =
      (state.trialProto == Protocol::CRSF) ? Protocol::SBUS : Protocol::CRSF;

  while (state.serialPort->available())
    state.serialPort->read();

  state.serialPort->flush();
  state.serialPort->onReceive(NULL);
  state.serialPort->end();

  state.switchDelayStart = millis();
  if (state.switchDelayStart == 0)
    state.switchDelayStart = 1;
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

  if (state.switchDelayStart != 0) {
    if (millis() - state.switchDelayStart >= 5) {
      state.switchDelayStart = 0;
      resetParsers();
      state.validFramesCount = 0;

      state.serialPort->setRxBufferSize(Config::RX_BUFFER_SIZE);

      if (state.trialProto == Protocol::CRSF) {
        state.serialPort->begin(Config::CRSF_BAUD, SERIAL_8N1, Config::RX_PIN,
                                -1, false);
      } else {
        state.serialPort->begin(Config::SBUS_BAUD, SERIAL_8E2, Config::RX_PIN,
                                -1, true);
      }
      state.serialPort->onReceive(onUartData);
      state.lastProtoSwitchTime = millis();
      LOG_INFO("Switched protocol.");
    }
    return;
  }

  if (!state.isLocked) {
    if (millis() - state.lastProtoSwitchTime >
        Config::PROTO_SWITCH_TIMEOUT_MS) {
      switchProtocol();
    }
  }
}

void lockReceiver() {
  state.validFramesCount++;
  if (state.validFramesCount >= 3) {
    if (rxMutex)
      xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
    state.isLocked = true;
    state.currentProto = state.trialProto;
    if (rxMutex)
      xSemaphoreGiveRecursive(rxMutex);
    LOG_INFO("Locked protocol: %s",
             state.currentProto == Protocol::CRSF ? "CRSF" : "SBUS");
  }
}

void handleAutoDetect() {
  if (Config::USE_AUTO_DETECT)
    detectProtocol();

  if (state.switchDelayStart != 0)
    return;

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

void setNotifyTask(TaskHandle_t taskToNotify) {
  rxNotifyTaskHandle = taskToNotify;
}

}

HardwareSerial rxSerial(1);
TaskHandle_t rxTaskHandle = NULL;

void enablePassthrough() {
  if (isPassthroughMode)
    return;
  isPassthroughMode = true;

  rxSerial.onReceive(NULL);
  Receiver::resetParsers();

  tcpBridgeServer.begin();
  LOG_WARN("WiFi UART Bridge ENABLED on port 8880");
}

void disablePassthrough() {
  if (!isPassthroughMode)
    return;
  isPassthroughMode = false;

  if (tcpBridgeClient.connected())
    tcpBridgeClient.stop();
  tcpBridgeServer.end();

  rxSerial.onReceive(Receiver::onUartData);
  LOG_INFO("WiFi UART Bridge DISABLED. Resuming flight logic.");
}

void handleSerialBridge() {
  if (!isPassthroughMode)
    return;

  if (!tcpBridgeClient || !tcpBridgeClient.connected()) {
    tcpBridgeClient = tcpBridgeServer.available();
    if (tcpBridgeClient) {
      tcpBridgeClient.setNoDelay(true);
      LOG_INFO("Bridge Client Connected!");
    }
  }

  if (tcpBridgeClient && tcpBridgeClient.connected()) {
    while (tcpBridgeClient.available()) {
      rxSerial.write(tcpBridgeClient.read());
    }
    while (rxSerial.available()) {
      tcpBridgeClient.write(rxSerial.read());
    }
  }
}

void sendChannelsUDP(int16_t *channels) {
  if (!udpBridgeEnabled)
    return;

  uint8_t buffer[32];
  for (int i = 0; i < 16; i++) {
    int16_t val = channels[i];
    buffer[i * 2] = val & 0xFF;
    buffer[i * 2 + 1] = (val >> 8) & 0xFF;
  }

  udpBridge.beginPacket(udpTargetIP, udpTargetPort);
  udpBridge.write(buffer, 32);
  udpBridge.endPacket();
}

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
      ServoSystem::state.lastPosL = ServoSystem::Control::centerAngle;
      ServoSystem::state.lastPosR = ServoSystem::Control::centerAngle;
      ServoSystem::attachAll();
    }
  } else {
    Flight::state.armStartTime = 0;
  }

  Flight::state.failsafeEnterMs = 0;
  Flight::updateCurrentOffsetGlide(dt_us);
  if (ServoSystem::state.isAttached) {
    ServoSystem::update(
        ServoSystem::state.centerAngle + (int)Flight::state.currentOffset,
        ServoSystem::state.centerAngle - (int)Flight::state.currentOffset,
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
  int steerValue = (abs(steering - Config::CH_CENTER) > Config::STICK_DEADBAND)
                       ? Config::mapTrim15(steering)
                       : 0;
  int elevOffset = 0;

  if (Flight::state.mode != FlightMode::PERCHING) {
    elevOffset = (abs(elevator - Config::CH_CENTER) > Config::STICK_DEADBAND)
                     ? Config::mapTrim15(elevator)
                     : 0;
  }

  if (Flight::state.mode == FlightMode::PERCHING) {
    int currentPull =
        constrain(Config::CH_MIN + (Config::CH_CENTER - elevator) * 2,
                  Config::CH_MIN, Config::CH_MAX);
    Flight::state.perchAngleLevel = currentPull;
    Flight::updateCurrentOffsetPerch(Flight::state.perchAngleLevel, dt_us);
  } else {
    if (throttle < Config::THROTTLE_IDLE_MAX)
      Flight::updateCurrentOffsetGlide(dt_us);
    else
      Flight::updateCurrentOffsetTriangle(throttle, dt_us);
  }

  int offsetInt = (int)Flight::state.currentOffset;
  ServoSystem::update(
      ServoSystem::state.centerAngle + offsetInt + elevOffset + steerValue,
      ServoSystem::state.centerAngle - offsetInt - elevOffset + steerValue,
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

struct alignas(4) ControlSnapshot {
  int16_t ch[4];
  uint8_t flags;
};

static ControlSnapshot g_cmd_buffer[2];
static std::atomic<uint8_t> g_cmd_index{0};

inline void publishControlSnapshot(int16_t roll, int16_t pitch, int16_t throttle,
                                  int16_t yaw, uint8_t flags) {
  uint8_t next = 1 - g_cmd_index.load(std::memory_order_relaxed);
  g_cmd_buffer[next].ch[0] = roll;
  g_cmd_buffer[next].ch[1] = pitch;
  g_cmd_buffer[next].ch[2] = throttle;
  g_cmd_buffer[next].ch[3] = yaw;
  g_cmd_buffer[next].flags = flags;
  g_cmd_index.store(next, std::memory_order_release);
}

inline ControlSnapshot readControlSnapshot() {
  uint8_t idx = g_cmd_index.load(std::memory_order_acquire);
  return g_cmd_buffer[idx];
}

void flightCoreTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(3);
  unsigned long lastUs = micros();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    esp_task_wdt_reset();

    unsigned long nowUs = micros();
    unsigned long dt_us = nowUs - lastUs;
    if (dt_us == 0 || dt_us > 100000)
      dt_us = Config::LOOP_DT_US;
    lastUs = nowUs;

    ControlSnapshot cmd = readControlSnapshot();
    bool snapshotConnected = (cmd.flags & 0x01) != 0;
    bool killActive = (cmd.flags & 0x02) != 0;
    bool reqArm = (cmd.flags & 0x04) != 0;
    bool isWebControl = (cmd.flags & 0x08) != 0;

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
      Flight::state.mode = FlightMode::INIT;
      Flight::state.failsafeEnterMs = 0;
      ServoSystem::detachAll();
    } else {
      bool throttleIdle = cmd.ch[Config::CH_THROTTLE] < Config::THROTTLE_IDLE_MAX;

      if (isWebControl) {
        if (reqArm && !Flight::state.isArmed && snapshotConnected) {
          if (throttleIdle) {
            Flight::state.isArmed = true;
            Flight::state.armStartTime = 0;
            Flight::state.phaseNeedsResync = true;
            Flight::state.mode = FlightMode::CONNECTED;
            Flight::state.disarmStartTime = 0;
            Flight::state.isPerching = false;
            Flight::state.perchAngleLevel = Config::CH_MIN;
            ServoSystem::state.lastPosL = ServoSystem::Control::centerAngle;
            ServoSystem::state.lastPosR = ServoSystem::Control::centerAngle;
            ServoSystem::attachAll();
          }
        } else if (!reqArm && Flight::state.isArmed) {
          if (throttleIdle) {
            Flight::state.isArmed = false;
            ServoSystem::detachAll();
            Flight::state.mode = FlightMode::INIT;
            Flight::state.armStartTime = 0;
            Flight::state.disarmStartTime = 0;
          }
        }
      }

      processFlightLogic(dt_us, cmd.ch[0], cmd.ch[1], cmd.ch[2], cmd.ch[3],
                         snapshotConnected);
    }
  }
}

namespace WebControl {

constexpr char ROUTER_SSID[] = "Phonglhach";
constexpr char ROUTER_PASS[] = "123456789";
constexpr char BUTTERFLY_ID[] = "PHELAN-102";
constexpr unsigned long STA_CONNECT_TIMEOUT_MS = 10000;

enum class LinkState { GOOD, WEAK, LOST };

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool wifiActive = false;
bool webControlEnabled = false;
volatile unsigned long lastWsPacketTime = 0;
unsigned long lastTelemSendTime = 0;
volatile int8_t wsArmRequest = 0;
volatile LinkState currentLinkState = LinkState::LOST;
constexpr unsigned long WS_FAILSAFE_MS = 500;
constexpr unsigned long TELEM_INTERVAL_MS = 300;

volatile bool idVerified = true;
char deviceIP[20] = "0.0.0.0";

int16_t webChannels[16] = {1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500,
                           1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

const char PAGE_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<title>Phelan GCS - Tactical Flight Deck</title>
<style>
:root {
  --bg-black: #000000;
  --bg-panel: #0a0a0a;
  --bg-touch: #0f0f0f;
  --bg-card: #111111;

  --border-dim: #222222;
  --border-focus: #444444;
  --border-active: #555555;

  --text-white: #ffffff;
  --text-gray: #888888;
  --text-dark: #444444;

  --accent-green: #10b981;
  --accent-green-glow: rgba(16, 185, 129, 0.35);
  --accent-red: #ef4444;
  --accent-amber: #f59e0b;
  --accent-cyan: #00f0ff;

  --font-mono: ui-monospace, SFMono-Regular, "JetBrains Mono", Menlo, Consolas, monospace;
  --font-sans: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
}

* {
  box-sizing: border-box;
  margin: 0;
  padding: 0;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  user-select: none;
  touch-action: none;
}

html, body {
  width: 100vw;
  height: 100vh;
  background: var(--bg-black);
  color: var(--text-white);
  font-family: var(--font-sans);
  overflow: hidden;
  position: fixed;
  inset: 0;
}

.header-bar {
  height: 48px;
  background: var(--bg-panel);
  border-bottom: 1px solid var(--border-dim);
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 16px;
  position: relative;
  z-index: 100;
}

.target-badge {
  display: flex;
  align-items: center;
  gap: 8px;
  background: var(--bg-card);
  border: 1px solid var(--border-dim);
  padding: 4px 12px;
  border-radius: 6px;
  z-index: 2;
  font-family: var(--font-mono);
}
.target-label {
  color: var(--text-gray);
  font-size: 11px;
}
.target-id-val {
  color: var(--text-white);
  font-weight: 700;
  letter-spacing: 0.5px;
}
.badge-linked {
  color: var(--accent-green);
  font-size: 10px;
  font-weight: 700;
}

.brand-center-group {
  position: absolute;
  left: 50%;
  top: 50%;
  transform: translate(-50%, -50%);
  pointer-events: none;
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1;
}
.brand-img {
  height: 32px;
  width: auto;
  object-fit: contain;
  display: block;
}

.header-right {
  display: flex;
  align-items: center;
  gap: 12px;
  z-index: 2;
}
.cpu-temp {
  font-family: var(--font-mono);
  font-size: 12px;
  color: var(--text-white);
}
.cpu-temp span {
  color: var(--text-gray);
}
.cpu-temp.warn {
  color: var(--accent-amber);
}
.cpu-temp.alert {
  color: var(--accent-red);
}
.btn-fs {
  background: transparent;
  border: 1px solid var(--border-dim);
  color: var(--text-gray);
  border-radius: 6px;
  padding: 4px 8px;
  font-size: 11px;
  font-family: var(--font-mono);
  cursor: pointer;
}
.btn-fs:hover {
  color: var(--text-white);
  border-color: var(--border-focus);
}

.main-layout {
  display: grid;
  grid-template-columns: 1fr 280px 1fr;
  height: calc(100vh - 48px);
  width: 100vw;
  position: relative;
}

.joystick-col {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 12px;
  position: relative;
}

.joystick-touch-zone {
  width: min(290px, 80vw, 75vh);
  height: min(290px, 80vw, 75vh);
  background: radial-gradient(circle, #141414 0%, #0d0d0d 70%, #080808 100%);
  border: 1.5px solid var(--border-dim);
  border-radius: 50%;
  position: relative;
  touch-action: none;
  cursor: crosshair;
  box-shadow: inset 0 0 20px rgba(0, 0, 0, 0.9), 0 0 15px rgba(0, 0, 0, 0.5);
}

.joystick-touch-zone::before,
.joystick-touch-zone::after {
  content: "";
  position: absolute;
  background: rgba(255, 255, 255, 0.05);
  pointer-events: none;
}
.joystick-touch-zone::before {
  left: 50%;
  top: 10%;
  bottom: 10%;
  width: 1px;
  transform: translateX(-50%);
}
.joystick-touch-zone::after {
  top: 50%;
  left: 10%;
  right: 10%;
  height: 1px;
  transform: translateY(-50%);
}

.throttle-graduations {
  position: absolute;
  inset: 0;
  pointer-events: none;
}
.throttle-baseline {
  position: absolute;
  left: 20%;
  right: 20%;
  top: calc(50% + min(100px, 28vw, 26vh));
  height: 1.5px;
  background: rgba(16, 185, 129, 0.35);
  box-shadow: 0 0 8px rgba(16, 185, 129, 0.2);
}
.throttle-mark {
  position: absolute;
  left: calc(50% + 35px);
  transform: translateY(-50%);
  font-family: var(--font-mono);
  font-size: 10px;
  color: var(--text-gray);
  pointer-events: none;
}
.throttle-mark.mark-100 {
  top: calc(50% - min(100px, 28vw, 26vh));
}
.throttle-mark.mark-50 {
  top: 50%;
}
.throttle-mark.mark-0 {
  top: calc(50% + min(100px, 28vw, 26vh));
  color: var(--accent-green);
  font-weight: 700;
}

.joystick-knob {
  width: 44px;
  height: 44px;
  border-radius: 50%;
  background: radial-gradient(circle, #252525 0%, #151515 100%);
  border: 2px solid var(--text-white);
  position: absolute;
  left: 50%;
  top: 50%;
  transform: translate(-50%, -50%);
  pointer-events: none;
  box-shadow: 0 0 14px rgba(255, 255, 255, 0.25), inset 0 1px 2px rgba(255, 255, 255, 0.4);
  transition: transform 0.05s ease-out;
}
.joystick-knob-dot {
  width: 6px;
  height: 6px;
  background: var(--text-white);
  border-radius: 50%;
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
}

.joystick-readout {
  margin-top: 14px;
  font-family: var(--font-mono);
  font-size: 13px;
  color: var(--text-gray);
  letter-spacing: 0.5px;
}
.joystick-readout b {
  color: var(--text-white);
}

.center-col {
  background: var(--bg-panel);
  border-left: 1px solid var(--border-dim);
  border-right: 1px solid var(--border-dim);
  display: flex;
  flex-direction: column;
  justify-content: center;
  padding: 16px 20px;
  gap: 16px;
}

.timer-block {
  display: flex;
  flex-direction: column;
  gap: 8px;
  font-family: var(--font-mono);
}
.timer-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
}
.timer-label {
  color: var(--text-gray);
  font-size: 12px;
  letter-spacing: 0.5px;
}
.timer-val {
  color: var(--text-white);
  font-weight: 700;
  letter-spacing: 1px;
}
.timer-val.warn { color: var(--accent-amber); }
.timer-val.alert { color: var(--accent-red); }
.timer-val.ping-good { color: var(--accent-green); }
.timer-val.ping-warn { color: var(--accent-amber); }
.timer-val.ping-bad  { color: var(--accent-red); }

.col-divider {
  height: 1px;
  background: var(--border-dim);
  width: 100%;
}

.action-buttons {
  display: flex;
  flex-direction: column;
  gap: 10px;
}

.btn-action {
  width: 100%;
  height: 48px;
  border-radius: 8px;
  font-family: var(--font-mono);
  cursor: pointer;
  background: linear-gradient(180deg, #181818 0%, #0d0d0d 100%);
  position: relative;
  overflow: hidden;
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 0 16px;
  gap: 12px;
  transition: all 0.18s cubic-bezier(0.16, 1, 0.3, 1);
  box-shadow: inset 0 1px 1px rgba(255, 255, 255, 0.08), 0 4px 12px rgba(0, 0, 0, 0.6);
  user-select: none;
}
.btn-action:active {
  transform: scale(0.98);
}

.btn-tactical-led {
  width: 10px;
  height: 10px;
  border-radius: 50%;
  flex-shrink: 0;
  transition: all 0.2s ease;
}
.led-green {
  background: var(--accent-green);
  box-shadow: 0 0 10px var(--accent-green);
}
.led-red {
  background: var(--accent-red);
  box-shadow: 0 0 10px var(--accent-red);
}
.led-amber {
  background: var(--accent-amber);
  box-shadow: 0 0 10px var(--accent-amber);
}

.btn-main-txt {
  font-size: 16px;
  font-weight: 800;
  letter-spacing: 2.5px;
  line-height: 1;
  transition: color 0.15s;
}

.btn-arm {
  border: 1.5px solid rgba(16, 185, 129, 0.55);
  color: var(--accent-green);
}
.btn-arm:hover {
  border-color: var(--accent-green);
  box-shadow: inset 0 1px 1px rgba(255, 255, 255, 0.12), 0 0 16px rgba(16, 185, 129, 0.25);
}
.btn-arm.armed {
  background: linear-gradient(180deg, #10b981 0%, #059669 100%);
  color: #000000;
  border-color: #34d399;
  box-shadow: 0 0 24px rgba(16, 185, 129, 0.55), inset 0 1px 2px rgba(255, 255, 255, 0.5);
}
.btn-arm.armed .btn-tactical-led {
  background: #000000;
  box-shadow: 0 0 4px #000000;
}

.btn-stop {
  border: 1.5px solid rgba(239, 68, 68, 0.6);
  color: var(--accent-red);
}
.btn-stop:hover {
  border-color: var(--accent-red);
  box-shadow: inset 0 1px 1px rgba(255, 255, 255, 0.12), 0 0 16px rgba(239, 68, 68, 0.25);
}
.btn-stop:active {
  background: var(--accent-red);
  color: #ffffff;
}

.btn-perch {
  border: 1.5px solid rgba(245, 158, 11, 0.55);
  color: var(--accent-amber);
}
.btn-perch:hover {
  border-color: var(--accent-amber);
  box-shadow: inset 0 1px 1px rgba(255, 255, 255, 0.12), 0 0 16px rgba(245, 158, 11, 0.25);
}
.btn-perch.active {
  background: linear-gradient(180deg, #f59e0b 0%, #d97706 100%);
  color: #000000;
  border-color: #fbbf24;
  box-shadow: 0 0 24px rgba(245, 158, 11, 0.5), inset 0 1px 2px rgba(255, 255, 255, 0.5);
}
.btn-perch.active .btn-tactical-led {
  background: #000000;
  box-shadow: 0 0 4px #000000;
}

#interlock-toast {
  display: none;
  position: fixed;
  bottom: 24px;
  left: 50%;
  transform: translateX(-50%);
  background: rgba(239, 68, 68, 0.95);
  color: #ffffff;
  padding: 10px 24px;
  border-radius: 20px;
  font-family: var(--font-mono);
  font-size: 13px;
  font-weight: 700;
  z-index: 3000;
  box-shadow: 0 4px 20px rgba(239, 68, 68, 0.5);
  letter-spacing: 0.5px;
}
#interlock-toast.show {
  display: block;
  animation: toastFade 2.2s forwards;
}
@keyframes toastFade {
  0% { opacity: 0; transform: translate(-50%, 10px); }
  15% { opacity: 1; transform: translate(-50%, 0); }
  85% { opacity: 1; transform: translate(-50%, 0); }
  100% { opacity: 0; transform: translate(-50%, -10px); }
}

#failsafe-banner {
  display: none;
  position: fixed;
  top: 56px;
  left: 50%;
  transform: translateX(-50%);
  background: var(--accent-red);
  color: #fff;
  padding: 8px 24px;
  border-radius: 6px;
  font-family: var(--font-mono);
  font-size: 13px;
  font-weight: 800;
  letter-spacing: 1px;
  z-index: 3000;
  box-shadow: 0 0 25px rgba(239, 68, 68, 0.8);
  animation: pulseFail 1s infinite alternate;
}
@keyframes pulseFail {
  0% { opacity: 0.8; transform: translate(-50%, 0) scale(0.98); }
  100% { opacity: 1; transform: translate(-50%, 0) scale(1.02); }
}

#portrait-guard {
  display: none;
  position: fixed;
  inset: 0;
  background: #000000;
  z-index: 99999;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 16px;
}
.rot-svg {
  width: 60px;
  height: 60px;
  stroke: var(--accent-green);
  animation: rotAnim 2.5s infinite ease-in-out;
}
@keyframes rotAnim {
  0% { transform: rotate(0deg); }
  35% { transform: rotate(-90deg); }
  65% { transform: rotate(-90deg); }
  100% { transform: rotate(0deg); }
}
.rot-msg {
  font-family: var(--font-mono);
  font-size: 14px;
  color: var(--text-white);
  letter-spacing: 1px;
}

@media (orientation: portrait) {
  #portrait-guard {
    display: flex !important;
  }
}
</style>
</head>

<body>

<div id="portrait-guard">
  <svg class="rot-svg" viewBox="0 0 24 24" fill="none" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round">
    <rect x="5" y="2" width="14" height="20" rx="3"></rect>
    <line x1="12" y1="18" x2="12.01" y2="18"></line>
  </svg>
  <div class="rot-msg">XOAY NGANG ĐỂ ĐIỀU KHIỂN</div>
</div>

<div id="interlock-toast">CẢNH BÁO: HẠ TAY GA VỀ 0% ĐỂ ARM!</div>

<div id="failsafe-banner">FAILSAFE ACTIVATED - ĐÃ CẮT TẢI SERVO</div>

<header class="header-bar">
  <div class="target-badge">
    <span class="target-label">ID:</span>
    <b class="target-id-val" id="hdr-target-id">PHELAN BUTTERFLY</b>
    <span class="badge-linked" id="hdr-link-state">DISCONNECTED</span>
  </div>

  <div class="brand-center-group">
    <img src="data:image/png;base64,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" alt="Phelan" class="brand-img">
  </div>

  <div class="header-right">
    <div class="cpu-temp" id="wrap-cpu-temp">
      <span>CPU:</span> <b id="val-cpu-temp">42°C</b>
    </div>
    <button class="btn-fs" onclick="toggleFullScreen()" title="Toàn màn hình">FULL</button>
  </div>
</header>

<main class="main-layout" id="cockpit-layout">

  <section class="joystick-col">
    <div class="joystick-touch-zone" id="zone-l">
      <div class="throttle-graduations">
        <div class="throttle-baseline"></div>
        <span class="throttle-mark mark-100">100%</span>
        <span class="throttle-mark mark-50">50%</span>
        <span class="throttle-mark mark-0">0%</span>
      </div>
      <div class="joystick-knob" id="knob-l">
        <div class="joystick-knob-dot"></div>
      </div>
    </div>
    <div class="joystick-readout">
      <span>THR:</span> <b id="txt-thr">0%</b> | <span>YAW:</span> <b id="txt-yaw">0%</b>
    </div>
  </section>

  <section class="center-col">

    <div class="timer-block">
      <div class="timer-row">
        <span class="timer-label">UPTIME</span>
        <span class="timer-val" id="time-uptime">00:00</span>
      </div>
      <div class="timer-row">
        <span class="timer-label">ARM TIME</span>
        <span class="timer-val" id="time-armed">00:00</span>
      </div>
      <div class="timer-row">
        <span class="timer-label">FLIGHT TIME</span>
        <span class="timer-val" id="time-flight">00:00</span>
      </div>
      <div class="timer-row">
        <span class="timer-label">PING</span>
        <span class="timer-val ping-good" id="val-ping">-- ms</span>
      </div>
    </div>

    <div class="col-divider"></div>

    <div class="action-buttons">

      <button class="btn-action btn-arm ready" id="btn-arm" onclick="handleArmClick()">
        <div class="btn-tactical-led led-green" id="led-arm"></div>
        <span class="btn-main-txt" id="txt-arm-main">ARM</span>
      </button>

      <button class="btn-action btn-stop" id="btn-stop" onclick="handleStopClick()">
        <div class="btn-tactical-led led-red"></div>
        <span class="btn-main-txt">STOP</span>
      </button>

      <button class="btn-action btn-perch" id="btn-perch" onclick="togglePerchClick()">
        <div class="btn-tactical-led led-amber" id="led-perch"></div>
        <span class="btn-main-txt" id="txt-perch-main">PERCH</span>
      </button>

    </div>

  </section>

  <section class="joystick-col">
    <div class="joystick-touch-zone" id="zone-r">
      <div class="joystick-knob" id="knob-r">
        <div class="joystick-knob-dot"></div>
      </div>
    </div>
    <div class="joystick-readout">
      <span>PIT:</span> <b id="txt-pit">0%</b> | <span>ROL:</span> <b id="txt-rol">0%</b>
    </div>
  </section>

</main>

<script>

const App = {
  isArmed: false,
  isPerching: false,
  isFailsafe: false,

  thr: 0,
  yaw: 0,
  pit: 0,
  rol: 0,

  ws: null,
  wsConnected: false,
  isSimulation: false,

  startTime: performance.now(),
  armedStartTime: 0,
  flightStartTime: 0,
  lastPingTime: 0,
  lastHeartbeatTime: 0,
  cpuTemp: 42.0
};

function toggleFullScreen() {
  if (!document.fullscreenElement) {
    document.documentElement.requestFullscreen().catch(() => {});
  } else {
    document.exitFullscreen().catch(() => {});
  }
}

function setKnobPosition(knob, x, y) {
  if (!knob) return;
  knob.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
}

function resetJoysticksToIdle() {
  App.pit = 0;
  App.rol = 0;
  App.thr = 0;
  App.yaw = 0;

  const knobR = document.getElementById('knob-r');
  const knobL = document.getElementById('knob-l');
  const zoneL = document.getElementById('zone-l');

  if (knobR) setKnobPosition(knobR, 0, 0);

  if (zoneL && knobL) {
    const rect = zoneL.getBoundingClientRect();
    const maxDist = (rect.width / 2) * 0.72;
    setKnobPosition(knobL, 0, maxDist);
  }

  const elThr = document.getElementById('txt-thr');
  const elYaw = document.getElementById('txt-yaw');
  const elPit = document.getElementById('txt-pit');
  const elRol = document.getElementById('txt-rol');

  if (elThr) elThr.textContent = '0%';
  if (elYaw) elYaw.textContent = '0%';
  if (elPit) elPit.textContent = '0%';
  if (elRol) elRol.textContent = '0%';
}

function initJoysticks() {
  const zoneL = document.getElementById('zone-l');
  const zoneR = document.getElementById('zone-r');
  const knobL = document.getElementById('knob-l');
  const knobR = document.getElementById('knob-r');

  const activeTouches = {};

  function getZoneMetrics(zone) {
    const rect = zone.getBoundingClientRect();
    const radius = rect.width / 2;
    const maxDist = radius * 0.72;
    return { rect, radius, maxDist };
  }

  function updateLeftKnobPosition() {
    const { maxDist } = getZoneMetrics(zoneL);
    const pixelX = (App.yaw / 100) * maxDist;
    const pixelY = maxDist - (App.thr / 100) * (2 * maxDist);
    setKnobPosition(knobL, pixelX, pixelY);
  }

  updateLeftKnobPosition();

  function handleMove(zone, clientX, clientY, isLeft) {
    const { rect, radius, maxDist } = getZoneMetrics(zone);
    const centerX = rect.left + radius;
    const centerY = rect.top + radius;

    let dx = clientX - centerX;
    let dy = clientY - centerY;

    if (isLeft) {

      dx = Math.max(-maxDist, Math.min(maxDist, dx));
      dy = Math.max(-maxDist, Math.min(maxDist, dy));

      const rawThr = Math.round(((maxDist - dy) / (2 * maxDist)) * 100);
      App.thr = Math.max(0, Math.min(100, rawThr));
      App.yaw = Math.round((dx / maxDist) * 100);

      setKnobPosition(knobL, dx, dy);
      document.getElementById('txt-thr').textContent = App.thr + '%';
      document.getElementById('txt-yaw').textContent = App.yaw + '%';

      if (App.isArmed && App.thr > 0 && App.flightStartTime === 0) {
        App.flightStartTime = performance.now();
      }
    } else {

      const dist = Math.sqrt(dx * dx + dy * dy);
      if (dist > maxDist) {
        dx = (dx / dist) * maxDist;
        dy = (dy / dist) * maxDist;
      }

      App.rol = Math.round((dx / maxDist) * 100);
      App.pit = Math.round((-dy / maxDist) * 100);

      setKnobPosition(knobR, dx, dy);
      document.getElementById('txt-rol').textContent = App.rol + '%';
      document.getElementById('txt-pit').textContent = App.pit + '%';
    }

    sendFlightPacket();
  }

  function handleRelease(isLeft) {
    if (isLeft) {

      App.yaw = 0;
      updateLeftKnobPosition();
      document.getElementById('txt-yaw').textContent = '0%';
    } else {

      App.pit = 0;
      App.rol = 0;
      setKnobPosition(knobR, 0, 0);
      document.getElementById('txt-pit').textContent = '0%';
      document.getElementById('txt-rol').textContent = '0%';
    }
    sendFlightPacket();
  }

  [ { zone: zoneL, isLeft: true }, { zone: zoneR, isLeft: false } ].forEach(({ zone, isLeft }) => {
    let isMouseDown = false;

    zone.addEventListener('mousedown', (e) => {
      isMouseDown = true;
      handleMove(zone, e.clientX, e.clientY, isLeft);
    });
    window.addEventListener('mousemove', (e) => {
      if (isMouseDown) handleMove(zone, e.clientX, e.clientY, isLeft);
    });
    window.addEventListener('mouseup', () => {
      if (isMouseDown) {
        isMouseDown = false;
        handleRelease(isLeft);
      }
    });

    zone.addEventListener('touchstart', (e) => {
      for (let i = 0; i < e.changedTouches.length; i++) {
        const t = e.changedTouches[i];
        activeTouches[t.identifier] = { isLeft, zone };
        handleMove(zone, t.clientX, t.clientY, isLeft);
      }
    }, { passive: false });

    zone.addEventListener('touchmove', (e) => {
      for (let i = 0; i < e.changedTouches.length; i++) {
        const t = e.changedTouches[i];
        if (activeTouches[t.identifier] && activeTouches[t.identifier].zone === zone) {
          handleMove(zone, t.clientX, t.clientY, isLeft);
        }
      }
    }, { passive: false });

    const onTouchEnd = (e) => {
      for (let i = 0; i < e.changedTouches.length; i++) {
        const t = e.changedTouches[i];
        if (activeTouches[t.identifier]) {
          const wasLeft = activeTouches[t.identifier].isLeft;
          delete activeTouches[t.identifier];
          handleRelease(wasLeft);
        }
      }
    };
    zone.addEventListener('touchend', onTouchEnd);
    zone.addEventListener('touchcancel', onTouchEnd);
  });

  window.addEventListener('resize', () => {
    updateLeftKnobPosition();
    setKnobPosition(knobR, 0, 0);
  });
}

function showInterlockToast(msg) {
  const toast = document.getElementById('interlock-toast');
  toast.textContent = msg;
  toast.classList.add('show');
  setTimeout(() => {
    toast.classList.remove('show');
  }, 2200);
}

function updateArmButtonUI(isArmed) {
  const btn = document.getElementById('btn-arm');
  const txtMain = document.getElementById('txt-arm-main');
  if (!btn) return;

  if (isArmed) {
    btn.classList.add('armed');
    if (txtMain) txtMain.textContent = 'DISARM';
  } else {
    btn.classList.remove('armed');
    if (txtMain) txtMain.textContent = 'ARM';
  }
}

function updatePerchButtonUI(isPerching) {
  const btn = document.getElementById('btn-perch');
  const txtMain = document.getElementById('txt-perch-main');
  if (!btn) return;

  if (isPerching) {
    btn.classList.add('active');
    if (txtMain) txtMain.textContent = 'PERCHING';
  } else {
    btn.classList.remove('active');
    if (txtMain) txtMain.textContent = 'PERCH';
  }
}

function handleArmClick() {
  if (!App.isArmed) {

    if (App.thr > 0) {
      showInterlockToast("CẢNH BÁO: HẠ TAY GA VỀ 0% ĐỂ ARM!");
      if (navigator.vibrate) navigator.vibrate([120, 60, 120]);
      return;
    }

    App.isArmed = true;
    App.armedStartTime = performance.now();
    updateArmButtonUI(true);

    if (navigator.vibrate) navigator.vibrate(60);
  } else {

    App.isArmed = false;
    App.flightStartTime = 0;
    updateArmButtonUI(false);

    if (navigator.vibrate) navigator.vibrate(40);
  }

  sendFlightPacket();
}

function handleStopClick() {
  App.isArmed = false;
  App.isPerching = false;
  App.flightStartTime = 0;

  if (navigator.vibrate) navigator.vibrate([100, 50, 150]);

  updateArmButtonUI(false);
  updatePerchButtonUI(false);

  resetJoysticksToIdle();

  sendFlightPacket(true);
}

function togglePerchClick() {
  App.isPerching = !App.isPerching;
  updatePerchButtonUI(App.isPerching);
  if (navigator.vibrate) navigator.vibrate(50);
  sendFlightPacket();
}

function formatTime(totalSec) {
  const m = String(Math.floor(totalSec / 60)).padStart(2, '0');
  const s = String(totalSec % 60).padStart(2, '0');
  return `${m}:${s}`;
}

setInterval(() => {
  const now = performance.now();

  const upSec = Math.floor((now - App.startTime) / 1000);
  document.getElementById('time-uptime').textContent = formatTime(upSec);

  if (App.isArmed && App.armedStartTime > 0) {
    const armedSec = Math.floor((now - App.armedStartTime) / 1000);
    const armedEl = document.getElementById('time-armed');
    armedEl.textContent = formatTime(armedSec);

    if (armedSec >= 480) {
      armedEl.className = 'timer-val alert';
    } else if (armedSec >= 360) {
      armedEl.className = 'timer-val warn';
    } else {
      armedEl.className = 'timer-val';
    }
  }

  if (App.isArmed && App.thr > 0 && App.flightStartTime > 0) {
    const flightSec = Math.floor((now - App.flightStartTime) / 1000);
    document.getElementById('time-flight').textContent = formatTime(flightSec);
  }

  if (App.isSimulation) {
    App.cpuTemp = 42.0 + (App.thr / 100) * 8.0 + (Math.random() - 0.5) * 0.4;
    const cpuEl = document.getElementById('val-cpu-temp');
    const cpuWrap = document.getElementById('wrap-cpu-temp');
    cpuEl.textContent = Math.round(App.cpuTemp) + '°C';
    if (App.cpuTemp >= 65) {
      cpuWrap.className = 'cpu-temp alert';
    } else if (App.cpuTemp >= 55) {
      cpuWrap.className = 'cpu-temp warn';
    } else {
      cpuWrap.className = 'cpu-temp';
    }

    const pingEl = document.getElementById('val-ping');
    if (pingEl) {
      const p = Math.round(16 + Math.random() * 4);
      pingEl.textContent = p + ' ms';
      pingEl.className = 'timer-val ping-good';
    }
  }
}, 500);

function initWebSocket() {
  if (location.protocol === 'file:') {
    App.isSimulation = true;
    document.getElementById('hdr-link-state').textContent = "SIMULATION";
    document.getElementById('hdr-link-state').style.color = "var(--accent-cyan)";
    const pingEl = document.getElementById('val-ping');
    if (pingEl) {
      pingEl.textContent = "18 ms";
      pingEl.className = "timer-val ping-good";
    }
    return;
  }

  const wsUrl = `ws://${location.host}/ws`;
  App.ws = new WebSocket(wsUrl);
  App.ws.binaryType = 'arraybuffer';

  App.ws.onopen = () => {
    App.wsConnected = true;
    App.isSimulation = false;
    App.lastHeartbeatTime = performance.now();
    document.getElementById('hdr-link-state').textContent = "LINKED";
    document.getElementById('hdr-link-state').style.color = "var(--accent-green)";

    const d = new Date();
    App.ws.send('SYNC:' + d.getFullYear() + '-' + (d.getMonth() + 1) + '-' + d.getDate() + '_' + d.getHours() + ':' + d.getMinutes() + ':' + d.getSeconds());
  };

  App.ws.onclose = () => {
    App.wsConnected = false;
    document.getElementById('hdr-link-state').textContent = "DISCONNECTED";
    document.getElementById('hdr-link-state').style.color = "var(--accent-red)";
    const pingEl = document.getElementById('val-ping');
    if (pingEl) {
      pingEl.textContent = "-- ms";
      pingEl.className = "timer-val ping-bad";
    }

    if (App.isArmed) {
      triggerFailsafeAlert();
    }
    setTimeout(initWebSocket, 2000);
  };

  App.ws.onmessage = (event) => {
    App.lastHeartbeatTime = performance.now();

    if (typeof event.data === 'string') {
      if (event.data === "PONG") {
        const rtt = Math.round(performance.now() - App.lastPingTime);
        const pingEl = document.getElementById('val-ping');
        if (pingEl) {
          pingEl.textContent = rtt + ' ms';
          pingEl.className = 'timer-val ' + (rtt < 40 ? 'ping-good' : (rtt < 100 ? 'ping-warn' : 'ping-bad'));
        }
      }
    } else if (event.data instanceof ArrayBuffer && event.data.byteLength === 12) {
      const view = new DataView(event.data);
      if (view.getUint8(0) === 0xBB) {
        const flags = view.getUint8(1);
        const armedFromHw = (flags & 0x01) !== 0;
        if (armedFromHw !== App.isArmed) {
          App.isArmed = armedFromHw;
          updateArmButtonUI(App.isArmed);
        }

        const cpuTemp = view.getUint8(5) / 2.0;
        document.getElementById('val-cpu-temp').textContent = Math.round(cpuTemp) + '°C';

        const uptime = view.getUint16(7, true);
        document.getElementById('time-uptime').textContent = formatTime(uptime);
      }
    }
  };
}

function triggerFailsafeAlert() {
  App.isArmed = false;
  App.isFailsafe = true;
  document.getElementById('failsafe-banner').style.display = 'block';
  updateArmButtonUI(false);
  setTimeout(() => {
    document.getElementById('failsafe-banner').style.display = 'none';
  }, 4000);
}

function sendFlightPacket(isKill = false) {
  if (!App.ws || App.ws.readyState !== WebSocket.OPEN) return;
  if (App.ws.bufferedAmount > 0 && !isKill) return;

  const buf = new ArrayBuffer(6);
  const view = new DataView(buf);

  view.setUint8(0, 0xAA);

  let flags = 0;
  if (App.isArmed && !isKill) flags |= 0x01;
  if (isKill) flags |= 0x02;
  if (App.isPerching) flags |= 0x04;
  view.setUint8(1, flags);

  view.setInt8(2, App.thr);
  view.setInt8(3, App.yaw);
  view.setInt8(4, App.pit);
  view.setInt8(5, App.rol);

  App.ws.send(buf);
}

setInterval(() => {
  if (App.wsConnected) {
    sendFlightPacket();
  }
}, 20);

setInterval(() => {
  if (App.wsConnected) {
    App.lastPingTime = performance.now();
    App.ws.send("PING");

    if (performance.now() - App.lastHeartbeatTime > 600 && App.isArmed) {
      triggerFailsafeAlert();
    }
  }
}, 250);

window.addEventListener('DOMContentLoaded', () => {
  initJoysticks();
  initWebSocket();
});
</script>

</body>
</html>
)rawhtml";

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    LOG_INFO("WebSocket client #%u connected from %s", client->id(),
             client->remoteIP().toString().c_str());

    udpTargetIP = client->remoteIP();
    LOG_INFO("UDP target IP updated to %s", udpTargetIP.toString().c_str());

    if (client->client()) {
      client->client()->setNoDelay(true);
    }
  } else if (type == WS_EVT_DISCONNECT) {
    LOG_INFO("WebSocket client #%u disconnected", client->id());
    if (ws.count() == 0) {
      idVerified = true;
      webControlEnabled = false;
    }
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->opcode == WS_BINARY && len == sizeof(FlightPacket) &&
        data[0] == 0xAA) {
      if (!idVerified)
        return;

      FlightPacket *pkt = (FlightPacket *)data;
      lastWsPacketTime = millis();

      webChannels[Config::CH_THROTTLE] =
          constrain(1000 + (int16_t)pkt->throttle * 10, 1000, 2000);
      webChannels[Config::CH_YAW] =
          constrain(1500 + (int16_t)pkt->yaw * 5, 1000, 2000);

      bool webPerchActive = (pkt->flags & 0x04) != 0;
      if (webPerchActive && webChannels[Config::CH_THROTTLE] < Config::THROTTLE_IDLE_MAX) {
        webChannels[Config::CH_ELEVATOR] = Config::CH_MIN;
      } else {
        webChannels[Config::CH_ELEVATOR] =
            constrain(1500 + (int16_t)pkt->pitch * 5, 1000, 2000);
      }

      webChannels[Config::CH_STEERING] =
          constrain(1500 + (int16_t)pkt->roll * 5, 1000, 2000);

      wsArmRequest = (pkt->flags & 0x01) ? 1 : 0;
      webChannels[Config::CH_KILL_SWITCH] = (pkt->flags & 0x02) ? 1900 : 1000;

      webControlEnabled = true;
    } else if (info->opcode == WS_TEXT && len > 3) {

      char buf[48];
      size_t copyLen = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
      memcpy(buf, data, copyLen);
      buf[copyLen] = '\0';

      if (strcmp(buf, "PING") == 0) {
        client->text("PONG");
      } else if (strncmp(buf, "SYNC:", 5) == 0) {
        int y, mon, d, h, m, s;
        if (sscanf(buf + 5, "%d-%d-%d_%d:%d:%d", &y, &mon, &d, &h, &m, &s) ==
            6) {
          SysMonitor::setTime(y, mon, d, h, m, s);
        }
      } else if (strcmp(buf, "BRIDGE:ON") == 0) {
        enablePassthrough();
        client->text("BRIDGE:ACK:ON");
        LOG_WARN("BRIDGE mode activated via WebSocket");
      } else if (strcmp(buf, "BRIDGE:OFF") == 0) {
        disablePassthrough();
        client->text("BRIDGE:ACK:OFF");
        LOG_INFO("BRIDGE mode deactivated via WebSocket");
      } else if (strcmp(buf, "UDP:ON") == 0) {
        udpBridgeEnabled = true;
        client->text("UDP:ACK:ON");
        LOG_INFO("UDP Channel Bridge ENABLED (port %u)", udpTargetPort);
      } else if (strcmp(buf, "UDP:OFF") == 0) {
        udpBridgeEnabled = false;
        client->text("UDP:ACK:OFF");
        LOG_INFO("UDP Channel Bridge DISABLED");
      } else if (strncmp(buf, "ID:", 3) == 0) {
        if (strcmp(buf + 3, BUTTERFLY_ID) == 0) {
          idVerified = true;
          client->text("ID:OK");
          LOG_INFO("Butterfly ID verified by client #%u", client->id());
        } else {
          client->text("ID:FAIL");
          LOG_WARN("Invalid Butterfly ID: %s", buf + 3);
        }
      }
    }
  }
}

void sendTelemetry() {
  if (ws.count() == 0)
    return;

  TelemetryPacket tpkt;
  tpkt.header = 0xBB;

  uint8_t modeId = 0;
  switch (Flight::state.mode) {
  case FlightMode::CONNECTED:
    modeId = 1;
    break;
  case FlightMode::PERCHING:
    modeId = 2;
    break;
  case FlightMode::FAILSAFE:
    modeId = 3;
    break;
  default:
    modeId = 0;
    break;
  }
  tpkt.flags = (Flight::state.isArmed ? 1 : 0) | (modeId << 1);
  tpkt.offset = (int8_t)constrain((int)Flight::state.currentOffset, -127, 127);
  tpkt.servoL = (uint8_t)constrain(ServoSystem::state.lastPosL, 0, 255);
  tpkt.servoR = (uint8_t)constrain(ServoSystem::state.lastPosR, 0, 255);
  tpkt.cpuTemp = (uint8_t)(SysMonitor::getCPUTemp() * 2.0f);
  tpkt.cpuMhz = (uint8_t)SysMonitor::getCPUMhz();
  tpkt.uptimeSec = (uint16_t)(millis() / 1000);

  char timeBuf[32];
  SysMonitor::getFormattedTime(timeBuf, sizeof(timeBuf));
  if (strlen(timeBuf) >= 19) {
    tpkt.timeH = (uint8_t)((timeBuf[11] - '0') * 10 + (timeBuf[12] - '0'));
    tpkt.timeM = (uint8_t)((timeBuf[14] - '0') * 10 + (timeBuf[15] - '0'));
    tpkt.timeS = (uint8_t)((timeBuf[17] - '0') * 10 + (timeBuf[18] - '0'));
  } else {
    tpkt.timeH = 0;
    tpkt.timeM = 0;
    tpkt.timeS = 0;
  }

  ws.binaryAll((uint8_t *)&tpkt, sizeof(tpkt));
}

void stopWiFi() {
  if (wifiActive) {
    ws.closeAll();
    server.end();
    MDNS.end();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiActive = false;
    webControlEnabled = false;
    idVerified = false;
    Serial.println(F("[WIFI] WiFi turned OFF."));
  }
}

void init() {
  Serial.printf("[WIFI] Connecting STA -> %s ...\n", ROUTER_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.begin(ROUTER_SSID, ROUTER_PASS);

  unsigned long staStart = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - staStart < 4000) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    snprintf(deviceIP, sizeof(deviceIP), "%d.%d.%d.%d", ip[0], ip[1], ip[2],
             ip[3]);
    esp_wifi_set_ps(WIFI_PS_NONE);

    String mdnsName = BUTTERFLY_ID;
    mdnsName.toLowerCase();
    if (MDNS.begin(mdnsName.c_str())) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("[WIFI] mDNS: %s.local\n", mdnsName.c_str());
    }
    Serial.printf("[WIFI] STA OK! SSID:%s IP:%s ID:%s\n", ROUTER_SSID, deviceIP,
                  BUTTERFLY_ID);
  } else {
    Serial.println(
        F("[WIFI] STA not connected yet. Background auto-reconnect active."));
  }

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", PAGE_HTML);
  });
  server.begin();

  wifiActive = true;
}

void handle() {
  if (!wifiActive)
    return;

  unsigned long now = millis();

  static unsigned long lastStaCheck = 0;
  if (now - lastStaCheck >= 3000) {
    lastStaCheck = now;
    if (WiFi.status() == WL_CONNECTED) {
      if (strcmp(deviceIP, "0.0.0.0") == 0) {
        IPAddress ip = WiFi.localIP();
        snprintf(deviceIP, sizeof(deviceIP), "%d.%d.%d.%d", ip[0], ip[1], ip[2],
                 ip[3]);
        esp_wifi_set_ps(WIFI_PS_NONE);
        String mdnsName = BUTTERFLY_ID;
        mdnsName.toLowerCase();
        if (MDNS.begin(mdnsName.c_str())) {
          MDNS.addService("http", "tcp", 80);
        }
        Serial.printf("[WIFI] STA Connected! IP: %s\n", deviceIP);
      }
    } else {
      if (strcmp(deviceIP, "0.0.0.0") != 0) {
        strcpy(deviceIP, "0.0.0.0");
        Serial.println(F("[WIFI] STA lost. Reconnecting..."));
      }
      WiFi.reconnect();
    }
  }
  unsigned long dtLink = now - lastWsPacketTime;

  if (dtLink < 200) {
    currentLinkState = LinkState::GOOD;
  } else if (dtLink <= WS_FAILSAFE_MS) {
    currentLinkState = LinkState::WEAK;

    if (webChannels[Config::CH_THROTTLE] > 1400) {
      webChannels[Config::CH_THROTTLE] = 1400;
    }
  } else {
    currentLinkState = LinkState::LOST;
  }

  if (webControlEnabled && (currentLinkState == LinkState::LOST)) {
    webControlEnabled = false;
    wsArmRequest = 0;
  }

  if (isPassthroughMode && (currentLinkState == LinkState::LOST)) {
    disablePassthrough();
    LOG_WARN("Web lost during Bridge mode. Auto-recovering flight logic.");
  }

  if (udpBridgeEnabled && (currentLinkState == LinkState::LOST)) {
    udpBridgeEnabled = false;
    LOG_WARN("Web lost. UDP Bridge auto-disabled.");
  }

  if (now - lastTelemSendTime >= TELEM_INTERVAL_MS) {
    lastTelemSendTime = now;
    sendTelemetry();
    ws.cleanupClients();
  }
}

}

void setupWatchdog() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdt_cfg = {};
  wdt_cfg.timeout_ms = Config::WDT_TIMEOUT_SEC * 1000;
  wdt_cfg.idle_core_mask = 0;
  wdt_cfg.trigger_panic = true;
  if (esp_task_wdt_reconfigure(&wdt_cfg) != ESP_OK) {
    esp_task_wdt_init(&wdt_cfg);
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

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  SysMonitor::parseCompileTime();
  logMutex = xSemaphoreCreateRecursiveMutex();

  esp_reset_reason_t rstReason = esp_reset_reason();
  Serial.printf("[BOOT] Reset reason: %d\n", rstReason);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  pinMode(Config::LED_PIN, OUTPUT);
  digitalWrite(Config::LED_PIN, HIGH);

Receiver::init(&rxSerial, NULL);
  xTaskCreate(vReceiverTask, "RX_Task", 8192, NULL, configMAX_PRIORITIES - 2,
              &rxTaskHandle);
  Receiver::setNotifyTask(rxTaskHandle);

  ServoSystem::state.lastPosL = ServoSystem::state.centerAngle;
  ServoSystem::state.lastPosR = ServoSystem::state.centerAngle;

  WebControl::init();

  udpBridge.begin(14551);
  LOG_INFO("UDP Bridge initialized on local port 14551, target %s:%u",
           udpTargetIP.toString().c_str(), udpTargetPort);

  LOG_INFO("PHELAN BUTTERFLY 3IN1 WiFi Bridge READY");

  xTaskCreatePinnedToCore(flightCoreTask, "flightCore", 4096, NULL, 5, NULL, 0);

  setupWatchdog();
  esp_task_wdt_reset();
}

void loop() {
  static bool snapshotConnected = false;
  static Protocol snapProto = Protocol::DETECT;
  static int16_t emaSnapshot[16] = {1500, 1500, 1500, 1500, 1500, 1500,
                                    1500, 1500, 1500, 1500, 1500, 1500,
                                    1500, 1500, 1500, 1500};

  esp_task_wdt_reset();

  WebControl::handle();

  if (isPassthroughMode) {
    publishControlSnapshot(Config::CH_CENTER, Config::CH_CENTER,
                           Config::CH_MIN, Config::CH_CENTER, 0x02);
    handleSerialBridge();
    delay(1);
    return;
  }

  unsigned long currentMillis = millis();

  Receiver::updateFilters();

  if (Receiver::rxMutex) {
    if (xSemaphoreTakeRecursive(Receiver::rxMutex, pdMS_TO_TICKS(1))) {
      bool protocolReady =
          !Config::USE_AUTO_DETECT || Receiver::state.isLocked;

      bool rawConnected =
          protocolReady && Receiver::state.hasFrame &&
          (currentMillis - Receiver::state.lastTime < Config::RX_TIMEOUT_MS);
      if (Receiver::state.currentProto == Protocol::SBUS &&
          (Receiver::state.sbusFailSafe ||
           Receiver::state.sbusLostFrames >= Config::CONN_LOST_THRESHOLD)) {
        rawConnected = false;
      }
      if (Receiver::state.currentProto == Protocol::CRSF &&
          Receiver::state.crsfFailSafe) {
        rawConnected = false;
      }
      Receiver::state.isConnected = rawConnected;
      snapshotConnected = rawConnected;
      snapProto = Receiver::state.currentProto;
      memcpy(emaSnapshot, Receiver::state.emaValues, sizeof(emaSnapshot));
      xSemaphoreGiveRecursive(Receiver::rxMutex);
    }
  }

  if (WebControl::webControlEnabled) {
    memcpy(emaSnapshot, WebControl::webChannels, sizeof(emaSnapshot));
    snapshotConnected = true;
    snapProto = Protocol::WIFI;
  }

  bool killActive =
      emaSnapshot[Config::CH_KILL_SWITCH] > Config::KILL_SWITCH_THRESHOLD;
  bool armRequested = WebControl::webControlEnabled && (WebControl::wsArmRequest == 1);
  bool isWebControl = WebControl::webControlEnabled;

  uint8_t flags = 0;
  if (snapshotConnected) flags |= 0x01;
  if (killActive) flags |= 0x02;
  if (armRequested) flags |= 0x04;
  if (isWebControl) flags |= 0x08;

  publishControlSnapshot(
      emaSnapshot[Config::CH_STEERING],
      emaSnapshot[Config::CH_ELEVATOR],
      emaSnapshot[Config::CH_THROTTLE],
      emaSnapshot[Config::CH_YAW],
      flags);

if (!snapshotConnected) {
    digitalWrite(Config::LED_PIN, (currentMillis / 250) % 2 == 0 ? HIGH : LOW);
  } else {
    digitalWrite(Config::LED_PIN,
                 Flight::state.isArmed
                     ? HIGH
                     : ((currentMillis / 1000) % 2 == 0 ? HIGH : LOW));
  }

  static unsigned long lastUdpSendTime = 0;
  if (currentMillis - lastUdpSendTime >= 20) {
    lastUdpSendTime = currentMillis;
    sendChannelsUDP(emaSnapshot);
  }

  SysMonitor::checkSerialTimeSync();

  static unsigned long lastLogTime = 0;
  constexpr unsigned long LOG_INTERVAL_MS = 1000;
  if (currentMillis - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = currentMillis;
    if (logMutex)
      xSemaphoreTakeRecursive(logMutex, portMAX_DELAY);
    if (Serial.availableForWrite() >= 128) {
      char timeBuf[32];
      SysMonitor::getFormattedTime(timeBuf, sizeof(timeBuf));
      float cpuTemp = SysMonitor::getCPUTemp();
      uint32_t cpuFreq = SysMonitor::getCPUMhz();

      char waveBuf[32];
      float currentOff = Flight::state.currentOffset;
      SysMonitor::getAsciiWave(waveBuf, sizeof(waveBuf), currentOff,
                               (float)Config::FLAP_AMP);

      if (snapshotConnected) {

        Serial.print(F("["));
        Serial.print(timeBuf);
        Serial.print(F("] [CPU:"));
        Serial.print(cpuFreq);
        Serial.print(F("MHz "));
        Serial.print(cpuTemp, 1);
        Serial.print(F("°C] "));

        Serial.print(Flight::state.isArmed ? F("[FLY] ") : F("[ARM?] "));
        if (snapProto == Protocol::WIFI)
          Serial.print(F("WIFI "));
        else if (snapProto == Protocol::CRSF)
          Serial.print(F("CRSF "));
        else if (snapProto == Protocol::SBUS)
          Serial.print(F("SBUS "));
        else
          Serial.print(F("DETC "));

        Serial.print(F("CH3:"));
        Serial.print(emaSnapshot[2]);
        Serial.print(F(" SVO:"));
        Serial.print(ServoSystem::state.isAttached ? F("OK ") : F("DET "));

        Serial.print(F("WAVE: "));
        Serial.print(waveBuf);
        Serial.print(F(" Off:"));
        Serial.print((int)currentOff);

        Serial.print(F(" | PLOT:L_Servo:"));
        Serial.print(ServoSystem::state.centerAngle + (int)currentOff);
        Serial.print(F(" R_Servo:"));
        Serial.print(ServoSystem::state.centerAngle - (int)currentOff);
        Serial.print(F(" Offset:"));
        Serial.println((int)currentOff);
      } else {

        Serial.print(F("["));
        Serial.print(timeBuf);
        Serial.print(F("] [CPU:"));
        Serial.print(cpuFreq);
        Serial.print(F("MHz "));
        Serial.print(cpuTemp, 1);
        Serial.print(F("°C] "));
        Serial.print(F("[WARN] NO SIGNAL or DETECTING... WAVE: "));
        Serial.print(waveBuf);
        Serial.print(F(" | PLOT:L_Servo:90 R_Servo:90 Offset:0"));
        Serial.println();
      }
    }
    if (logMutex)
      xSemaphoreGiveRecursive(logMutex);
  }

  delay(2);
}