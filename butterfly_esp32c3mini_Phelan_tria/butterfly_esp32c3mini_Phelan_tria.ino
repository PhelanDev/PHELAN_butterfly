#include <Arduino.h>
#include <esp_task_wdt.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#include "config.h"
#include "flight.h"
#include "receiver.h"
#include "servo_sys.h"

HardwareSerial rxSerial(1);
TaskHandle_t rxTaskHandle = NULL;
SemaphoreHandle_t logMutex = NULL;

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
      ServoSystem::state.lastUsL = Config::SERVO_US_CENTER;
      ServoSystem::state.lastUsR = Config::SERVO_US_CENTER;
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
      Flight::updateCurrentOffsetTriangle(throttle, dt_us);
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
    if (!Receiver::pollChannels(millis(), rc)) {
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

  LOG_INFO("PHELAN BUTTERFLY 2IN1 TRIANGLE READY");
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
