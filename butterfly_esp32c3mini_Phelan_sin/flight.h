#ifndef FLIGHT_H
#define FLIGHT_H

#include "config.h"
#include <Arduino.h>

enum class FlightMode { INIT, CONNECTED, PERCHING, FAILSAFE };

namespace Flight {

struct State {
  FlightMode mode = FlightMode::INIT;

  uint16_t phase = 0;
  float currentOffset = 0.0f;

  bool isPerching = false;
  int perchAngleLevel = Config::CH_MIN;
  bool phaseNeedsResync = true;
  bool isFlapUp = true;
  unsigned long failsafeEnterMs = 0;

  bool wasFlapping = false;
  bool wasPerching = false;
  bool isFlapping = false;

  bool isArmed = false;
  unsigned long armStartTime = 0;
  unsigned long disarmStartTime = 0;
};

extern State state;

inline bool isRisingPhase(uint16_t phase) {
  return (uint16_t)(phase + 16384) < 32768;
}

uint16_t sinPhaseFromOffset(float offset, float amp, bool flapUp);
void slewOffsetTo(float target, unsigned long dt_us);
void updateCurrentOffsetSine(int throttle, unsigned long dt_us);
void updateCurrentOffsetPerch(int perchAngleLevel, unsigned long dt_us);
void updateCurrentOffsetGlide(unsigned long dt_us);
void processFailsafe(unsigned long dt_us);

}
#endif