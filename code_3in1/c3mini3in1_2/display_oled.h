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

DisplayTelemetry sharedTelem = {"2026-08-30 00:00:00",
                                0,
                                0.0f,
                                160,
                                FlightMode::INIT,
                                Protocol::DETECT,
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
  while (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("SETTIME=")) {
      int y, mon, d, h, m, s;
      if (sscanf(input.substring(8).c_str(), "%d-%d-%d %d:%d:%d", &y, &mon, &d,
                 &h, &m, &s) == 6) {
        bootTime.year = y;
        bootTime.month = mon;
        bootTime.day = d;
        bootTime.hour = h;
        bootTime.minute = m;
        bootTime.second = s;
        bootMillis = millis();
        Serial.println(
            F("[INFO] System time synchronized successfully to Hanoi time!"));
      }
    }
  }
}

float filteredTemp = 42.0f;
bool tempFirstSample = true;

float getCPUTemp() {
  float currentRaw = temperatureRead();
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

#if USE_OLED
void vDisplayTask(void *pvParameters) {
  float waveHistory[60] = {0};

  for (;;) {
    DisplayTelemetry localTelem = {};
    bool dataOk = false;

    if (telemMutex && xSemaphoreTake(telemMutex, pdMS_TO_TICKS(10))) {
      localTelem = sharedTelem;
      dataOk = true;
      xSemaphoreGive(telemMutex);
    }

    if (dataOk) {

      for (int i = 0; i < 59; i++) {
        waveHistory[i] = waveHistory[i + 1];
      }
      waveHistory[59] = localTelem.offset;

      display.clearDisplay();
      display.setTextWrap(false);
      display.setTextColor(SSD1306_WHITE);

      display.setTextSize(1);

      char shortTime[10] = "00:00:00";
      if (strlen(localTelem.timeStr) >= 19) {
        strncpy(shortTime, localTelem.timeStr + 11, 8);
        shortTime[8] = '\0';
      }
      display.setCursor(0, 2);
      display.print(shortTime);

      display.setCursor(54, 2);
      if (localTelem.proto == Protocol::WIFI) {
        display.print(F("STA"));
      } else if (localTelem.isConnected) {
        if (localTelem.proto == Protocol::CRSF)
          display.print(F("CRSF"));
        else if (localTelem.proto == Protocol::SBUS)
          display.print(F("SBUS"));
        else
          display.print(F("CONN"));
      } else {
        display.print(F("LOST"));
      }

      if (localTelem.isArmed) {
        display.fillRect(96, 0, 32, 11, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        display.setCursor(102, 2);
        display.print(F("FLY"));
        display.setTextColor(SSD1306_WHITE);
      } else {
        display.drawRect(96, 0, 32, 11, SSD1306_WHITE);
        display.setCursor(100, 2);
        display.print(F("ARM?"));
      }

      display.drawFastHLine(0, 12, 128, SSD1306_WHITE);

      char upBuf[12];
      uint32_t s = localTelem.uptimeSec;
      uint32_t h = s / 3600;
      uint32_t m = (s % 3600) / 60;
      uint32_t sec = s % 60;
      if (h > 0) {
        snprintf(upBuf, sizeof(upBuf), "%lu:%02lu:%02lu", h, m, sec);
      } else {
        snprintf(upBuf, sizeof(upBuf), "%02lu:%02lu", m, sec);
      }
      display.setCursor(0, 15);
      display.print(F("UP:"));
      display.print(upBuf);

      display.setCursor(0, 25);
      display.print(F("CPU:"));
      display.print(localTelem.cpuFreq);
      display.print(F("M"));

      display.setCursor(0, 35);
      display.print(F("TMP:"));
      display.print(localTelem.cpuTemp, 1);
      display.print(F("C"));

      display.setCursor(0, 45);
      display.print(F("S_L:"));
      display.print(localTelem.leftServo);

      display.setCursor(0, 55);
      if (strlen(localTelem.wifiIP) > 1 &&
          strcmp(localTelem.wifiIP, "0.0.0.0") != 0) {
        display.print(localTelem.wifiIP);
      } else {
        display.print(F("S_R:"));
        display.print(localTelem.rightServo);
      }

      display.drawFastVLine(62, 12, 52, SSD1306_WHITE);

      int boxX = 66;
      int boxY = 15;
      int boxW = 60;
      int boxH = 32;
      display.drawRect(boxX, boxY, boxW, boxH, SSD1306_WHITE);

      display.drawFastHLine(boxX + 1, boxY + boxH / 2, boxW - 2, SSD1306_WHITE);

      int centerY = boxY + boxH / 2;
      int prevX = -1;
      int prevY = -1;
      for (int i = 0; i < boxW; i++) {
        int wx = boxX + i;
        float val = waveHistory[i];

        float scaled = (val / (float)Config::FLAP_AMP) * 13.0f;
        int wy = centerY - (int)scaled;
        wy = constrain(wy, boxY + 1, boxY + boxH - 2);

        if (prevX != -1) {
          display.drawLine(prevX, prevY, wx, wy, SSD1306_WHITE);
        }
        prevX = wx;
        prevY = wy;
      }

      display.setCursor(66, 49);
      if (localTelem.mode == FlightMode::FAILSAFE) {
        display.print(F("FAILSAFE"));
      } else if (localTelem.mode == FlightMode::PERCHING) {
        display.print(F("PERCHING"));
      } else {
        display.print(F("OFF:"));
        display.print((int)localTelem.offset);
        if (localTelem.offset == 0 ||
            localTelem.offset == Config::FAILSAFE_DIHEDRAL) {
          display.print(F(" (GLD)"));
        }
      }

      display.display();
    }

    vTaskDelay(localTelem.isArmed ? pdMS_TO_TICKS(500) : pdMS_TO_TICKS(100));
  }
}
#endif
