#ifndef CONFIG_H
#define CONFIG_H

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

extern SemaphoreHandle_t logMutex;

#define SAFE_LOG(level, fmt, ...)                                              \
  do {                                                                         \
    if (logMutex) xSemaphoreTakeRecursive(logMutex, portMAX_DELAY);            \
    if (Serial.availableForWrite() >= 64) {                                    \
      Serial.print(level);                                                     \
      Serial.printf(fmt, ##__VA_ARGS__);                                       \
      Serial.println();                                                        \
    }                                                                          \
    if (logMutex) xSemaphoreGiveRecursive(logMutex);                           \
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

#endif
