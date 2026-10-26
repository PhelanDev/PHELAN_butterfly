#include "flight.h"
#include "config.h"
#include "servo_sys.h"

namespace Flight {

State state;

static const int16_t tri_LUT[256] = {
    0,     16,   31,   47,   63,   78,   94,   109,  125,  141,  156,  172,
    188,   203,  219,  234,  250,  266,  281,  297,  313,  328,  344,  359,
    375,   391,  406,  422,  438,  453,  469,  484,  500,  516,  531,  547,
    563,   578,  594,  609,  625,  641,  656,  672,  688,  703,  719,  734,
    750,   766,  781,  797,  813,  828,  844,  859,  875,  891,  906,  922,
    938,   953,  969,  984,  1000, 984,  969,  953,  938,  922,  906,  891,
    875,   859,  844,  828,  813,  797,  781,  766,  750,  734,  719,  703,
    688,   672,  656,  641,  625,  609,  594,  578,  563,  547,  531,  516,
    500,   484,  469,  453,  438,  422,  406,  391,  375,  359,  344,  328,
    313,   297,  281,  266,  250,  234,  219,  203,  188,  172,  156,  141,
    125,   109,  94,   78,   63,   47,   31,   16,   0,    -16,  -31,  -47,
    -63,   -78,  -94,  -109, -125, -141, -156, -172, -188, -203, -219, -234,
    -250,  -266, -281, -297, -313, -328, -344, -359, -375, -391, -406, -422,
    -438,  -453, -469, -484, -500, -516, -531, -547, -563, -578, -594, -609,
    -625,  -641, -656, -672, -688, -703, -719, -734, -750, -766, -781, -797,
    -813,  -828, -844, -859, -875, -891, -906, -922, -938, -953, -969, -984,
    -1000, -984, -969, -953, -938, -922, -906, -891, -875, -859, -844, -828,
    -813,  -797, -781, -766, -750, -734, -719, -703, -688, -672, -656, -641,
    -625,  -609, -594, -578, -563, -547, -531, -516, -500, -484, -469, -453,
    -438,  -422, -406, -391, -375, -359, -344, -328, -313, -297, -281, -266,
    -250,  -234, -219, -203, -188, -172, -156, -141, -125, -109, -94,  -78,
    -63,   -47,  -31,  -16};

uint16_t triPhaseFromOffset(float offset, float amp, bool flapUp) {
  if (amp <= 0.0f) return 0;
  float r = constrain(offset / amp, -1.0f, 1.0f);
  int32_t p = (int32_t)lroundf(r * 16384.0f);
  if (!flapUp) {
    p = 32768 - p;
  }
  return (uint16_t)p;
}

void slewOffsetTo(float target, unsigned long dt_us) {
  if (dt_us == 0) {
    if (target > state.currentOffset) state.isFlapUp = true;
    else if (target < state.currentOffset) state.isFlapUp = false;
    state.currentOffset = target;
    return;
  }
  float step = (Config::GLIDE_SERVO_SPEED_DPS * (float)dt_us) / 1000000.0f;
  if (state.currentOffset < target) {
    state.isFlapUp = true;
    state.currentOffset += step;
    if (state.currentOffset > target) {
      state.currentOffset = target;
    }
  } else if (state.currentOffset > target) {
    state.isFlapUp = false;
    state.currentOffset -= step;
    if (state.currentOffset < target) {
      state.currentOffset = target;
    }
  }
}

void updateCurrentOffsetTriangle(int throttle, unsigned long dt_us) {
  throttle = constrain(throttle, Config::THROTTLE_IDLE_MAX, Config::CH_MAX);

  float ratio = (float)(throttle - Config::THROTTLE_IDLE_MAX) /
                (float)(Config::CH_MAX - Config::THROTTLE_IDLE_MAX);
  float dynamicAmp = Config::FLAP_AMP_IDLE - ratio * (Config::FLAP_AMP_IDLE - Config::FLAP_AMP_MAX_THR);

  if (state.phaseNeedsResync) {
    state.phase = triPhaseFromOffset(state.currentOffset, dynamicAmp, state.isFlapUp);
    state.phaseNeedsResync = false;
  }

  uint32_t speed = Config::FLAP_SPEED_MIN +
                   (uint32_t)(throttle - Config::THROTTLE_IDLE_MAX) *
                       (Config::FLAP_SPEED_MAX - Config::FLAP_SPEED_MIN) /
                       (Config::CH_MAX - Config::THROTTLE_IDLE_MAX);

  if (dt_us > 0) {
    uint32_t phaseDelta = ((uint64_t)speed * dt_us) / 1000000;
    state.phase += phaseDelta;
  }

  state.currentOffset =
      ((float)tri_LUT[state.phase >> 8] * dynamicAmp) / 1000.0f;
  state.isFlapUp = isRisingPhase(state.phase);
}

void updateCurrentOffsetPerch(int perchAngleLevel, unsigned long dt_us) {
  if (state.phaseNeedsResync) {
    state.phaseNeedsResync = false;
  }

  float maxUpAngle =
      Config::PERCH_BASE_UP_ANGLE +
      ((perchAngleLevel - Config::CH_MIN) * Config::PERCH_MULT) / 1000.0f;
  constexpr float maxOffset = Config::SERVO_MAX_ANGLE -
                              ServoSystem::Control::centerAngle -
                              Config::PERCH_HEADROOM_DEG;
  float target = constrain(maxUpAngle, 0.0f, maxOffset);

  slewOffsetTo(target, dt_us);
}

void updateCurrentOffsetGlide(unsigned long dt_us) {
  slewOffsetTo(0.0f, dt_us);
  if (fabsf(state.currentOffset) < 0.01f) {
    state.isFlapUp = true;
  }
}

void processFailsafe(unsigned long dt_us) {
  state.isPerching = false;
  state.armStartTime = 0;
  state.disarmStartTime = 0;

  unsigned long lostMs = millis() - state.failsafeEnterMs;
  if (lostMs > Config::RX_HARD_FAILSAFE_MS) {
    state.isArmed = false;
  }

  if (!state.isArmed) {
    ServoSystem::detachAll();
    return;
  }

  slewOffsetTo(Config::FAILSAFE_DIHEDRAL, dt_us);

  if (ServoSystem::state.isAttached) {
    ServoSystem::update(
        ServoSystem::state.centerAngle + state.currentOffset,
        ServoSystem::state.centerAngle - state.currentOffset, false);
  }
  state.phaseNeedsResync = true;
}
}
