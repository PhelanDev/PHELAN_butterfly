#pragma once

#include <Arduino.h>
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

#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
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

#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

enum class Protocol { DETECT, CRSF, SBUS, WIFI };

namespace Config {
constexpr uint8_t RX_PIN = 3;
constexpr uint8_t SERVO_L_PIN = 5;
constexpr uint8_t SERVO_R_PIN = 6;
constexpr uint8_t LED_PIN = 14;

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
constexpr uint32_t WDT_TIMEOUT_SEC = 3;
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
  int8_t  throttle;
  int8_t  yaw;
  int8_t  pitch;
  int8_t  roll;
};

struct TelemetryPacket {
  uint8_t  header;
  uint8_t  flags;
  int8_t   offset;
  uint8_t  servoL;
  uint8_t  servoR;
  uint8_t  cpuTemp;
  uint8_t  cpuMhz;
  uint16_t uptimeSec;
  uint8_t  timeH;
  uint8_t  timeM;
  uint8_t  timeS;
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

