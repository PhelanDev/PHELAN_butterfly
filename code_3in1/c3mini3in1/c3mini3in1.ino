#include "config.h"
#include "web_html.h"
#include "display_oled.h"
#include "servo_system.h"
#include "flight_control.h"
#include "receiver.h"
#include "web_control.h"

void setupWatchdog() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdt_cfg = {};
  wdt_cfg.timeout_ms = Config::WDT_TIMEOUT_SEC * 1000;
  wdt_cfg.idle_core_mask = 0;
  wdt_cfg.trigger_panic = true;
  esp_err_t err = esp_task_wdt_init(&wdt_cfg);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    LOG_ERROR("WDT Init failed");
  }
#else
  esp_task_wdt_init(Config::WDT_TIMEOUT_SEC, false);
#endif
  esp_task_wdt_add(NULL);
}

#include <atomic>

struct alignas(4) ControlSnapshot {
  int16_t ch[4];
  uint8_t flags;
};

static ControlSnapshot g_cmd_buffer[2];
static std::atomic<uint8_t> g_cmd_index{0};

inline void publishControlSnapshot(int16_t roll, int16_t pitch, int16_t throttle, int16_t yaw, uint8_t flags) {
  uint8_t next = 1 - g_cmd_index.load(std::memory_order_relaxed);
  g_cmd_buffer[next].ch[0] = roll;
  g_cmd_buffer[next].ch[1] = pitch;
  g_cmd_buffer[next].ch[2] = throttle;
  g_cmd_buffer[next].ch[3] = yaw;
  g_cmd_buffer[next].flags = flags;
  g_cmd_index.store(next, std::memory_order_release);
}

inline ControlSnapshot readControlSnapshot() {
  return g_cmd_buffer[g_cmd_index.load(std::memory_order_acquire)];
}

void flightCoreTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(3);

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    ControlSnapshot cmd = readControlSnapshot();
    bool isConnected = (cmd.flags & 0x01) != 0;
    bool killActive = (cmd.flags & 0x02) != 0;

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
      processFlightLogic(Config::LOOP_DT_US, cmd.ch[0], cmd.ch[1], cmd.ch[2],
                         cmd.ch[3], isConnected);
    }
  }
}

void setup() {
  Serial.begin(115200);
  SysMonitor::parseCompileTime();
  logMutex = xSemaphoreCreateRecursiveMutex();
  Serial.printf("Reset reason: %d\n", esp_reset_reason());
  setupWatchdog();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  pinMode(Config::LED_PIN, OUTPUT);
  digitalWrite(Config::LED_PIN, HIGH);

#if USE_OLED
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("[ERROR] OLED Init Failed! Running in headless mode."));
  } else {

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(16, 15);
    display.println(F("PHELAN BUTTERFLY"));
    display.drawFastHLine(16, 27, 96, SSD1306_WHITE);
    display.setCursor(24, 33);
    display.println(F("TELEMETRY SYSTEM"));
    display.setCursor(20, 48);
    display.println(F("INITIALIZING..."));
    display.display();
    delay(1000);

    telemMutex = xSemaphoreCreateMutex();
    if (telemMutex != NULL) {
      xTaskCreate(vDisplayTask, "OLED_Task", 4096, NULL, 1, &displayTaskHandle);
    }
  }
#endif

  xTaskCreate(vReceiverTask, "RX_Task", 8192, NULL, configMAX_PRIORITIES - 2,
              &rxTaskHandle);

  Receiver::init(&rxSerial, rxTaskHandle);

  ServoSystem::state.lastPosL = ServoSystem::state.centerAngle;
  ServoSystem::state.lastPosR = ServoSystem::state.centerAngle;

  WebControl::init();

  xTaskCreatePinnedToCore(flightCoreTask, "FlightCore", 4096, NULL, 5, NULL, 0);

  LOG_INFO("PHELAN BUTTERFLY 2IN1 TRIANGLE READY");
}

void loop() {
  static bool snapshotConnected = false;
  static Protocol snapProto = Protocol::DETECT;
  static int16_t emaSnapshot[16] = {1500, 1500, 1500, 1500, 1500, 1500,
                                    1500, 1500, 1500, 1500, 1500, 1500,
                                    1500, 1500, 1500, 1500};

  esp_task_wdt_reset();

  WebControl::handle();

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

  uint8_t flags = (snapshotConnected ? 0x01 : 0x00) | (killActive ? 0x02 : 0x00);
  publishControlSnapshot(emaSnapshot[Config::CH_STEERING],
                         emaSnapshot[Config::CH_ELEVATOR],
                         emaSnapshot[Config::CH_THROTTLE],
                         emaSnapshot[Config::CH_YAW],
                         flags);

#if USE_OLED
  if (telemMutex) {
    if (xSemaphoreTake(telemMutex, pdMS_TO_TICKS(2))) {
      char timeBuf[32];
      SysMonitor::getFormattedTime(timeBuf, sizeof(timeBuf));
      strcpy(sharedTelem.timeStr, timeBuf);
      sharedTelem.uptimeSec = millis() / 1000;
      sharedTelem.cpuTemp = SysMonitor::getCPUTemp();
      sharedTelem.cpuFreq = SysMonitor::getCPUMhz();
      sharedTelem.mode = Flight::state.mode;
      sharedTelem.proto = snapProto;
      sharedTelem.isConnected = snapshotConnected;
      sharedTelem.isArmed = Flight::state.isArmed;
      sharedTelem.leftServo = ServoSystem::state.lastPosL;
      sharedTelem.rightServo = ServoSystem::state.lastPosR;
      sharedTelem.offset = Flight::state.currentOffset;
      xSemaphoreGive(telemMutex);
    }
  }
#endif

  if (!snapshotConnected) {
    digitalWrite(Config::LED_PIN, (currentMillis / 250) % 2 == 0 ? HIGH : LOW);
  } else {
    digitalWrite(Config::LED_PIN,
                 Flight::state.isArmed
                     ? HIGH
                     : ((currentMillis / 1000) % 2 == 0 ? HIGH : LOW));
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