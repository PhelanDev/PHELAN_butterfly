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
    state.filteredL += targetUsL - (state.filteredL >> Config::SERVO_EMA_ALPHA_SHIFT);
    state.filteredR += targetUsR - (state.filteredR >> Config::SERVO_EMA_ALPHA_SHIFT);
  }

  int usL = state.filteredL >> Config::SERVO_EMA_ALPHA_SHIFT;
  int usR = state.filteredR >> Config::SERVO_EMA_ALPHA_SHIFT;

  const int usMin = Config::angleToUs((float)Config::SERVO_MIN_ANGLE);
  const int usMax = Config::angleToUs((float)Config::SERVO_MAX_ANGLE);
  usL = constrain(usL, usMin, usMax);
  usR = constrain(usR, usMin, usMax);

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
