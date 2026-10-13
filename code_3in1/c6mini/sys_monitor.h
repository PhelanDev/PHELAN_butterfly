#pragma once
#include "config.h"

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
