#include "servo_sys.h"
#include "config.h"

namespace ServoSystem {

Control state;

void attachAll() {
  if (!state.isAttached) {
    state.left.setPeriodHertz(Config::SERVO_HZ);
    state.right.setPeriodHertz(Config::SERVO_HZ);
    state.left.attach(Config::SERVO_L_PIN, Config::SERVO_US_MIN, Config::SERVO_US_MAX);
    state.right.attach(Config::SERVO_R_PIN, Config::SERVO_US_MIN, Config::SERVO_US_MAX);
    state.left.writeMicroseconds(state.lastUsL);
    state.right.writeMicroseconds(state.lastUsR);
    state.filteredL = (int32_t)state.lastUsL << Config::SERVO_EMA_ALPHA_SHIFT;
    state.filteredR = (int32_t)state.lastUsR << Config::SERVO_EMA_ALPHA_SHIFT;
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

void update(float targetL, float targetR, bool bypassFilter) {
  if (!state.isAttached)
    return;
  targetL = constrain(targetL, (float)Config::SERVO_MIN_ANGLE, (float)Config::SERVO_MAX_ANGLE);
  targetR = constrain(targetR, (float)Config::SERVO_MIN_ANGLE, (float)Config::SERVO_MAX_ANGLE);

  float angleL = Config::REVERSE_SERVO_L ? (180.0f - targetL) : targetL;
  float angleR = Config::REVERSE_SERVO_R ? (180.0f - targetR) : targetR;

  int targetUsL = Config::angleToUs(angleL);
  int targetUsR = Config::angleToUs(angleR);

  if (bypassFilter) {
    state.filteredL = (int32_t)targetUsL << Config::SERVO_EMA_ALPHA_SHIFT;
    state.filteredR = (int32_t)targetUsR << Config::SERVO_EMA_ALPHA_SHIFT;
  } else {
    int32_t scaledTargetL = (int32_t)targetUsL << Config::SERVO_EMA_ALPHA_SHIFT;
    int32_t diffL = scaledTargetL - state.filteredL;
    if (diffL != 0) {
      int32_t stepL = diffL / (1 << Config::SERVO_EMA_ALPHA_SHIFT);
      if (stepL == 0) stepL = (diffL > 0) ? 1 : -1;
      state.filteredL += stepL;
    }
    int32_t scaledTargetR = (int32_t)targetUsR << Config::SERVO_EMA_ALPHA_SHIFT;
    int32_t diffR = scaledTargetR - state.filteredR;
    if (diffR != 0) {
      int32_t stepR = diffR / (1 << Config::SERVO_EMA_ALPHA_SHIFT);
      if (stepR == 0) stepR = (diffR > 0) ? 1 : -1;
      state.filteredR += stepR;
    }
  }

  int usL = state.filteredL >> Config::SERVO_EMA_ALPHA_SHIFT;
  int usR = state.filteredR >> Config::SERVO_EMA_ALPHA_SHIFT;

  usL = constrain(usL, Config::SERVO_US_LIM_MIN, Config::SERVO_US_LIM_MAX);
  usR = constrain(usR, Config::SERVO_US_LIM_MIN, Config::SERVO_US_LIM_MAX);

  if (abs(usL - state.lastUsL) >= Config::SERVO_US_DEADBAND) {
    state.left.writeMicroseconds(usL);
    state.lastUsL = usL;
  }
  if (abs(usR - state.lastUsR) >= Config::SERVO_US_DEADBAND) {
    state.right.writeMicroseconds(usR);
    state.lastUsR = usR;
  }
}

}