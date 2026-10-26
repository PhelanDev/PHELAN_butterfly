#include "flight.h"
#include "config.h"
#include "servo_sys.h"

namespace Flight {

State state;

static const int16_t sin_LUT[256] = {
    0,     25,    49,   74,   98,   122,  147,  171,  195,  219,  243,  267,
    290,   314,   337,  360,  383,  405,  428,  450,  471,  493,  514,  535,
    556,   576,   596,  615,  634,  653,  672,  690,  707,  724,  741,  757,
    773,   788,   803,  818,  831,  845,  858,  870,  882,  893,  904,  914,
    924,   933,   942,  950,  957,  964,  970,  976,  981,  985,  989,  992,
    995,   997,   999,  1000, 1000, 1000, 999,  997,  995,  992,  989,  985,
    981,   976,   970,  964,  957,  950,  942,  933,  924,  914,  904,  893,
    882,   870,   858,  845,  831,  818,  803,  788,  773,  757,  741,  724,
    707,   690,   672,  653,  634,  615,  596,  576,  556,  535,  514,  493,
    471,   450,   428,  405,  383,  360,  337,  314,  290,  267,  243,  219,
    195,   171,   147,  122,  98,   74,   49,   25,   0,    -25,  -49,  -74,
    -98,   -122,  -147, -171, -195, -219, -243, -267, -290, -314, -337, -360,
    -383,  -405,  -428, -450, -471, -493, -514, -535, -556, -576, -596, -615,
    -634,  -653,  -672, -690, -707, -724, -741, -757, -773, -788, -803, -818,
    -831,  -845,  -858, -870, -882, -893, -904, -914, -924, -933, -942, -950,
    -957,  -964,  -970, -976, -981, -985, -989, -992, -995, -997, -999, -1000,
    -1000, -1000, -999, -997, -995, -992, -989, -985, -981, -976, -970, -964,
    -957,  -950,  -942, -933, -924, -914, -904, -893, -882, -870, -858, -845,
    -831,  -818,  -803, -788, -773, -757, -741, -724, -707, -690, -672, -653,
    -634,  -615,  -596, -576, -556, -535, -514, -493, -471, -450, -428, -405,
    -383,  -360,  -337, -314, -290, -267, -243, -219, -195, -171, -147, -122,
    -98,   -74,   -49,  -25};

static const uint16_t asin_LUT[256] = {
    0,     41,    82,    123,   164,   205,   245,   286,   327,   368,   409,
    450,   491,   532,   573,   614,   655,   696,   737,   778,   819,   860,
    901,   942,   983,   1024,  1065,  1106,  1148,  1189,  1230,  1271,  1312,
    1354,  1395,  1436,  1477,  1519,  1560,  1602,  1643,  1684,  1726,  1767,
    1809,  1850,  1892,  1934,  1975,  2017,  2059,  2100,  2142,  2184,  2226,
    2268,  2309,  2351,  2393,  2435,  2477,  2520,  2562,  2604,  2646,  2688,
    2731,  2773,  2816,  2858,  2900,  2943,  2986,  3028,  3071,  3114,  3157,
    3200,  3242,  3285,  3328,  3372,  3415,  3458,  3501,  3545,  3588,  3632,
    3675,  3719,  3762,  3806,  3850,  3894,  3938,  3982,  4026,  4070,  4114,
    4159,  4203,  4248,  4292,  4337,  4382,  4427,  4471,  4516,  4562,  4607,
    4652,  4697,  4743,  4789,  4834,  4880,  4926,  4972,  5018,  5064,  5110,
    5157,  5203,  5250,  5297,  5344,  5391,  5438,  5485,  5532,  5580,  5627,
    5675,  5723,  5771,  5819,  5867,  5916,  5964,  6013,  6062,  6111,  6160,
    6210,  6259,  6309,  6358,  6408,  6459,  6509,  6559,  6610,  6661,  6712,
    6763,  6815,  6866,  6918,  6970,  7022,  7075,  7127,  7180,  7233,  7287,
    7340,  7394,  7448,  7502,  7557,  7611,  7666,  7722,  7777,  7833,  7889,
    7945,  8002,  8059,  8116,  8174,  8232,  8290,  8349,  8408,  8467,  8526,
    8586,  8647,  8708,  8769,  8830,  8892,  8954,  9017,  9081,  9144,  9208,
    9273,  9338,  9404,  9470,  9537,  9604,  9672,  9741,  9810,  9879,  9950,
    10021, 10092, 10165, 10238, 10312, 10387, 10463, 10539, 10617, 10695, 10774,
    10855, 10936, 11019, 11102, 11187, 11274, 11361, 11450, 11541, 11633, 11727,
    11822, 11920, 12020, 12121, 12225, 12332, 12442, 12554, 12669, 12789, 12912,
    13039, 13171, 13309, 13453, 13605, 13764, 13934, 14117, 14315, 14534, 14782,
    15077, 15460, 16384};

uint16_t sinPhaseFromOffset(float offset, float amp, bool flapUp) {
  if (amp <= 0.0f) return 0;
  int32_t ratioScaled = (int32_t)(offset * 1000.0f / amp);
  ratioScaled = constrain(ratioScaled, -1000, 1000);

  bool negate = (ratioScaled < 0);
  int32_t abs_ratio = negate ? -ratioScaled : ratioScaled;

  int index = (abs_ratio * 255) / 1000;
  uint16_t approxPhase = asin_LUT[index];

  int32_t signedPhase = negate ? -approxPhase : approxPhase;

  if (!flapUp) {
    signedPhase = 32768 - signedPhase;
  }

  return (uint16_t)signedPhase;
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

void updateCurrentOffsetSine(int throttle, unsigned long dt_us) {
  throttle = constrain(throttle, Config::THROTTLE_IDLE_MAX, Config::CH_MAX);

  float ratio = (float)(throttle - Config::THROTTLE_IDLE_MAX) /
                (float)(Config::CH_MAX - Config::THROTTLE_IDLE_MAX);
  float dynamicAmp = Config::FLAP_AMP_IDLE - ratio * (Config::FLAP_AMP_IDLE - Config::FLAP_AMP_MAX_THR);

  if (state.phaseNeedsResync) {
    state.phase = sinPhaseFromOffset(state.currentOffset, dynamicAmp, state.isFlapUp);
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

  state.isFlapUp = isRisingPhase(state.phase);
  state.currentOffset =
      ((float)sin_LUT[state.phase >> 8] * dynamicAmp) / 1000.0f;
}

void updateCurrentOffsetPerch(int perchAngleLevel, unsigned long dt_us) {
  if (state.phaseNeedsResync) {
    state.phaseNeedsResync = false;
  }

  float maxUpAngle = Config::PERCH_BASE_UP_ANGLE +
                   ((perchAngleLevel - Config::CH_MIN) * Config::PERCH_MULT) / 1000.0f;
  constexpr float maxOffset = Config::SERVO_MAX_ANGLE - ServoSystem::Control::centerAngle - Config::PERCH_HEADROOM_DEG;
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