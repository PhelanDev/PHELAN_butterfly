#include "servo_sys.h"
#include "config.h"

namespace ServoSystem {

Control state;

void attachAll() {
  if (!state.isAttached) {
    state.left.setPeriodHertz(Config::SERVO_HZ);
    state.right.setPeriodHertz(Config::SERVO_HZ);
    state.left.attach(Config::SERVO_L_PIN, 500, 2500);
    state.right.attach(Config::SERVO_R_PIN, 500, 2500);
    state.left.write(state.lastPosL);
    state.right.write(state.lastPosR);
    state.filteredL = (int32_t)state.lastPosL << Config::SERVO_EMA_ALPHA_SHIFT;
    state.filteredR = (int32_t)state.lastPosR << Config::SERVO_EMA_ALPHA_SHIFT;
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

void update(int targetL, int targetR, bool bypassFilter) {
  if (!state.isAttached)
    return;
  targetL =
      constrain(targetL, Config::SERVO_MIN_ANGLE, Config::SERVO_MAX_ANGLE);
  targetR =
      constrain(targetR, Config::SERVO_MIN_ANGLE, Config::SERVO_MAX_ANGLE);

  if (bypassFilter) {
    state.filteredL = (int32_t)targetL << Config::SERVO_EMA_ALPHA_SHIFT;
    state.filteredR = (int32_t)targetR << Config::SERVO_EMA_ALPHA_SHIFT;
  } else {
    state.filteredL += targetL - (state.filteredL >> Config::SERVO_EMA_ALPHA_SHIFT);
    state.filteredR += targetR - (state.filteredR >> Config::SERVO_EMA_ALPHA_SHIFT);
  }

  targetL = state.filteredL >> Config::SERVO_EMA_ALPHA_SHIFT;
  targetR = state.filteredR >> Config::SERVO_EMA_ALPHA_SHIFT;

  if (abs(targetL - state.lastPosL) >= 1) {
    state.left.write(targetL);
    state.lastPosL = targetL;
  }
  if (abs(targetR - state.lastPosR) >= 1) {
    state.right.write(targetR);
    state.lastPosR = targetR;
  }
}

}