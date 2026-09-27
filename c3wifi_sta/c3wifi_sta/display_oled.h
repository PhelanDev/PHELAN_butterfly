#pragma once
#include "config.h"

#if USE_OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SDA_PIN 20
#define SCL_PIN 21

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TaskHandle_t displayTaskHandle = NULL;

DisplayTelemetry sharedTelem = {"2026-09-27 00:00:00",
                                0,
                                0.0f,
                                160,
                                FlightMode::INIT,
                                Protocol::WIFI,
                                false,
                                false,
                                90,
                                90,
                                0.0f,
                                "0.0.0.0"};

SemaphoreHandle_t telemMutex = NULL;
#endif

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
  if (Serial.available()) {
    String str = Serial.readStringUntil('\n');
    str.trim();
    if (str.startsWith("SYNC:")) {
      int y, mon, d, h, m, s;
      if (sscanf(str.c_str() + 5, "%d-%d-%d_%d:%d:%d", &y, &mon, &d, &h, &m,
                 &s) == 6) {
        setTime(y, mon, d, h, m, s);
        Serial.println(F("[TIME] Synchronized with host successfully."));
      }
    }
  }
}

float getCPUTemp() {
  static float filteredTemp = 42.0f;
  static unsigned long lastReadMs = 0;
  unsigned long now = millis();

  if (now - lastReadMs >= 200 || lastReadMs == 0) {
    lastReadMs = now;
    float currentRaw = 42.0f;
#if defined(HAS_NEW_TEMP_SENSOR)
    static temperature_sensor_handle_t temp_handle = NULL;
    if (temp_handle == NULL) {
      temperature_sensor_config_t temp_sensor_config =
          TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
      if (temperature_sensor_install(&temp_sensor_config, &temp_handle) ==
          ESP_OK) {
        temperature_sensor_enable(temp_handle);
      }
    }
    if (temp_handle != NULL) {
      temperature_sensor_get_celsius(temp_handle, &currentRaw);
    }
#elif defined(HAS_LEGACY_TEMP_SENSOR)
    static bool legacy_init = false;
    if (!legacy_init) {
      temp_sensor_config_t tsens = TSENS_CONFIG_DEFAULT();
      temp_sensor_set_config(tsens);
      temp_sensor_start();
      legacy_init = true;
    }
    temp_sensor_read_celsius(&currentRaw);
#endif
    if (isnan(currentRaw) || currentRaw < 10.0f || currentRaw > 105.0f) {
      currentRaw = 42.0f;
    }
    filteredTemp = (filteredTemp * 0.90f) + (currentRaw * 0.10f);
  }
  return filteredTemp;
}

uint32_t getCPUMhz() { return getCpuFrequencyMhz(); }

void getAsciiWave(char *buf, size_t len, float offset, float amp) {
  if (len < 10)
    return;
  const char chars[] = {'_', '.', '-', '~', '=', '^', '"', '*'};
  int numChars = sizeof(chars);
  float norm = (offset + amp) / (2.0f * amp);
  norm = constrain(norm, 0.0f, 1.0f);
  int idx = (int)(norm * (numChars - 1));
  snprintf(buf, len, "[%c%c%c%c%c]", chars[idx], chars[idx], chars[idx],
           chars[idx], chars[idx]);
}

}

#if USE_OLED
void vDisplayTask(void *pvParameters) {
  unsigned long lastDraw = 0;
  for (;;) {
    unsigned long now = millis();
    if (now - lastDraw >= 100) {
      lastDraw = now;
      DisplayTelemetry localTelem = {};
      bool dataOk = false;
      if (telemMutex) {
        if (xSemaphoreTake(telemMutex, pdMS_TO_TICKS(10))) {
          memcpy(&localTelem, &sharedTelem, sizeof(DisplayTelemetry));
          dataOk = true;
          xSemaphoreGive(telemMutex);
        }
      }

      if (dataOk) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);

        display.setCursor(0, 0);
        display.print(localTelem.timeStr + 11);
        display.setCursor(64, 0);
        display.printf("%3.0fC %3uM", localTelem.cpuTemp,
                       (unsigned int)localTelem.cpuFreq);
        display.drawFastHLine(0, 9, 128, SSD1306_WHITE);

        display.setCursor(0, 12);
        display.print(F("MODE: "));
        switch (localTelem.mode) {
        case FlightMode::INIT:
          display.print(F("STANDBY"));
          break;
        case FlightMode::CONNECTED:
          display.print(localTelem.isArmed ? F("ARMED (FLY)") : F("DISARMED"));
          break;
        case FlightMode::PERCHING:
          display.print(F("PERCHING"));
          break;
        case FlightMode::FAILSAFE:
          display.print(F("FAILSAFE"));
          break;
        }

        display.setCursor(0, 22);
        display.print(F("NET:  "));
        display.print(localTelem.isConnected ? F("WIFI STA OK") : F("SEARCHING..."));

        display.setCursor(0, 32);
        display.printf("IP: %s", localTelem.wifiIP);

        display.setCursor(0, 42);
        display.printf("SVO: L%3d R%3d", localTelem.leftServo,
                       localTelem.rightServo);

        display.setCursor(0, 52);
        display.printf("OFFSET: %+3.1f DEG", localTelem.offset);

        display.display();
      }
      vTaskDelay(localTelem.isArmed ? pdMS_TO_TICKS(500) : pdMS_TO_TICKS(100));
    } else {
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}
#endif
