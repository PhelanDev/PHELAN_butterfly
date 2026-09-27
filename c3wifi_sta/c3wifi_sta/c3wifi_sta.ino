#include <atomic>
#include "config.h"
#include "web_html.h"
#include "display_oled.h"
#include "servo_system.h"
#include "flight_control.h"
#include "web_control.h"

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
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(3);
  unsigned long lastUs = micros();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    unsigned long nowUs = micros();
    unsigned long dt_us = nowUs - lastUs;
    if (dt_us == 0 || dt_us > 100000)
      dt_us = Config::LOOP_DT_US;
    lastUs = nowUs;

    ControlSnapshot cmd = readControlSnapshot();
    bool snapshotConnected = (cmd.flags & 0x01) != 0;
    bool killActive = (cmd.flags & 0x02) != 0;
    bool reqArm = (cmd.flags & 0x04) != 0;

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

      processFlightLogic(dt_us, cmd.ch[0], cmd.ch[1], cmd.ch[2], cmd.ch[3],
                         snapshotConnected);
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

#if USE_OLED
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("[ERROR] OLED Init Failed! Running in headless mode."));
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(16, 10);
    display.println(F("PHELAN BUTTERFLY"));
    display.drawFastHLine(16, 22, 96, SSD1306_WHITE);
    display.setCursor(24, 28);
    display.println(F("STA WIFI SYSTEM"));
    display.setCursor(20, 42);
    display.println(F("INITIALIZING..."));
    display.setCursor(24, 54);
    display.printf("RST REASON: %d", (int)rstReason);
    display.display();
    delay(1000);

    telemMutex = xSemaphoreCreateMutex();
    if (telemMutex != NULL) {
      sharedTelem.cpuTemp = SysMonitor::getCPUTemp();
      sharedTelem.cpuFreq = SysMonitor::getCPUMhz();
      xTaskCreate(vDisplayTask, "OLED_Task", 4096, NULL, 1, &displayTaskHandle);
    }
  }
#endif

  ServoSystem::state.lastPosL = ServoSystem::state.centerAngle;
  ServoSystem::state.lastPosR = ServoSystem::state.centerAngle;

  WebControl::init();

  udpBridge.begin(14551);
  LOG_INFO("UDP Bridge initialized on local port 14551, target %s:%u",
           udpTargetIP.toString().c_str(), udpTargetPort);

  LOG_INFO("PHELAN BUTTERFLY C3 WIFI-STA READY");

  xTaskCreatePinnedToCore(flightCoreTask, "flightCore", 4096, NULL, 5, NULL, 0);

  setupWatchdog();
  esp_task_wdt_reset();
}

void loop() {
  esp_task_wdt_reset();

  WebControl::handle();

  unsigned long currentMillis = millis();

  bool snapshotConnected =
      (WebControl::currentLinkState != WebControl::LinkState::LOST) &&
      WebControl::webControlEnabled;
  bool killActive =
      WebControl::webChannels[Config::CH_KILL_SWITCH] > Config::KILL_SWITCH_THRESHOLD;
  bool armRequested = (WebControl::wsArmRequest == 1);

  uint8_t flags = 0;
  if (snapshotConnected) flags |= 0x01;
  if (killActive) flags |= 0x02;
  if (armRequested) flags |= 0x04;

  publishControlSnapshot(
      WebControl::webChannels[Config::CH_STEERING],
      WebControl::webChannels[Config::CH_ELEVATOR],
      WebControl::webChannels[Config::CH_THROTTLE],
      WebControl::webChannels[Config::CH_YAW],
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
      sharedTelem.proto = Protocol::WIFI;
      sharedTelem.isConnected = snapshotConnected;
      sharedTelem.isArmed = Flight::state.isArmed;
      sharedTelem.leftServo = ServoSystem::state.lastPosL;
      sharedTelem.rightServo = ServoSystem::state.lastPosR;
      sharedTelem.offset = Flight::state.currentOffset;
      strncpy(sharedTelem.wifiIP, WebControl::deviceIP,
              sizeof(sharedTelem.wifiIP) - 1);
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

  static unsigned long lastUdpSendTime = 0;
  if (currentMillis - lastUdpSendTime >= 20) {
    lastUdpSendTime = currentMillis;
    sendChannelsUDP(WebControl::webChannels);
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
        Serial.print(F("WIFI_STA "));

        Serial.print(F("THR:"));
        Serial.print(WebControl::webChannels[Config::CH_THROTTLE]);
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
        Serial.print(F("[WAITING WIFI STA CLIENT...] IP: "));
        Serial.print(WebControl::deviceIP);
        Serial.print(F(" | WAVE: "));
        Serial.print(waveBuf);
        Serial.println();
      }
    }
    if (logMutex)
      xSemaphoreGiveRecursive(logMutex);
  }

  delay(2);
}
