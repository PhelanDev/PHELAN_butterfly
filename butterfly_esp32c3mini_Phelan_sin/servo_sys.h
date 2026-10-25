#ifndef SERVO_SYS_H
#define SERVO_SYS_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"

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

extern Control state;

void attachAll();
void detachAll();
void update(float targetL, float targetR, bool bypassFilter = false);
}
#endif