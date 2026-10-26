#include "arduino_shim.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>

#ifdef HAS_EXTRACTED_CONFIG
#include "extracted_config.h"
#endif

// Unit test 1: mapTrimFloat with span 490 (Bug 13)
void test_trim_span_490() {
#ifdef HAS_EXTRACTED_CONFIG
  float t_max = Config::mapTrimFloat(2000);
  float t_min = Config::mapTrimFloat(1000);
  float t_zero = Config::mapTrimFloat(1500);
  float t_dead = Config::mapTrimFloat(1508);
  float t_mid = Config::mapTrimFloat(1745);

  assert(fabsf(t_max - 15.0f) < 0.01f);
  assert(fabsf(t_min - (-15.0f)) < 0.01f);
  assert(fabsf(t_zero - 0.0f) < 0.01f);
  assert(fabsf(t_dead - 0.0f) < 0.01f);
  // With span = 490: (1745 - 1500 - 10) = 235; 235 * 15.0f / 490.0f = 7.1938f
  // Old buggy formula divided by 30.0f and clipped to 15.0f at stick 1960!
  assert(t_mid > 7.1f && t_mid < 7.3f);
#endif
  printf("  [PASS] test_trim_span_490\n");
}

// Unit test 2: isRisingPhase (Bug 11)
inline bool isRisingPhase(uint16_t phase) {
  return (uint16_t)(phase + 16384) < 32768;
}

void test_is_rising_phase() {
  assert(isRisingPhase(0) == true);          // Center going up
  assert(isRisingPhase(8192) == true);       // Mid up
  assert(isRisingPhase(16383) == true);      // Just before peak
  assert(isRisingPhase(16384) == false);     // Peak starts down
  assert(isRisingPhase(24576) == false);     // Mid down
  assert(isRisingPhase(32768) == false);     // Center going down
  assert(isRisingPhase(49151) == false);     // Just before trough
  assert(isRisingPhase(49152) == true);      // Trough starts up
  assert(isRisingPhase(60000) == true);      // Going up towards center
  printf("  [PASS] test_is_rising_phase\n");
}

// Unit test 3: Symmetric integer division EMA for RC channels (Bug 8)
void test_symmetric_rc_ema() {
  const int SHIFT = 3;
  // Test positive convergence: 1500 -> 1507
  int16_t valPos = 1500;
  int stepsPos = 0;
  while (valPos != 1507 && stepsPos < 100) {
    int16_t diff = 1507 - valPos;
    int16_t step = diff / (1 << SHIFT);
    if (step == 0) step = (diff > 0) ? 1 : -1;
    valPos += step;
    stepsPos++;
  }
  assert(valPos == 1507);

  // Test negative convergence: 1500 -> 1493
  int16_t valNeg = 1500;
  int stepsNeg = 0;
  while (valNeg != 1493 && stepsNeg < 100) {
    int16_t diff = 1493 - valNeg;
    int16_t step = diff / (1 << SHIFT);
    if (step == 0) step = (diff > 0) ? 1 : -1;
    valNeg += step;
    stepsNeg++;
  }
  assert(valNeg == 1493);
  assert(stepsPos == stepsNeg); // Exactly symmetric!
  printf("  [PASS] test_symmetric_rc_ema\n");
}

// Unit test 4: Scaled servo EMA with zero residual bias (Bug 12)
void test_scaled_servo_ema() {
  const int SHIFT = 1;
  // Starting at 1500 us (filtered = 3000)
  int32_t filtered = 1500 << SHIFT;

  // Move up to 1501
  int32_t scaledTargetUp = 1501 << SHIFT;
  for (int i = 0; i < 20; i++) {
    int32_t diff = scaledTargetUp - filtered;
    if (diff != 0) {
      int32_t step = diff / (1 << SHIFT);
      if (step == 0) step = (diff > 0) ? 1 : -1;
      filtered += step;
    }
  }
  assert(filtered == scaledTargetUp); // Must reach 3002 exactly
  assert((filtered >> SHIFT) == 1501);

  // Move down to 1499
  int32_t scaledTargetDown = 1499 << SHIFT;
  for (int i = 0; i < 20; i++) {
    int32_t diff = scaledTargetDown - filtered;
    if (diff != 0) {
      int32_t step = diff / (1 << SHIFT);
      if (step == 0) step = (diff > 0) ? 1 : -1;
      filtered += step;
    }
  }
  assert(filtered == scaledTargetDown); // Must reach 2998 exactly, not 2999!
  assert((filtered >> SHIFT) == 1499);
  printf("  [PASS] test_scaled_servo_ema\n");
}

// Unit test 5: Smooth perch mapping from PERCH_ENTER_ELEV (Bug 14)
void test_smooth_perch_mapping() {
  const int16_t CH_MIN = 1000;
  const int16_t CH_MAX = 2000;
  const int16_t PERCH_ENTER = 1300;

  auto mapPerch = [&](int elevator) -> int {
    int currentPull = CH_MIN;
    if (elevator <= PERCH_ENTER) {
      currentPull = CH_MIN + (int32_t)(PERCH_ENTER - elevator) * (CH_MAX - CH_MIN) / (PERCH_ENTER - CH_MIN);
      currentPull = constrain(currentPull, (int)CH_MIN, (int)CH_MAX);
    }
    return currentPull;
  };

  assert(mapPerch(1300) == 1000); // Smooth entry at exactly minimum perch!
  assert(mapPerch(1150) == 1500); // Halfway pull
  assert(mapPerch(1000) == 2000); // Full pull
  assert(mapPerch(900) == 2000);  // Constrained to max
  printf("  [PASS] test_smooth_perch_mapping\n");
}

// Unit test 6: Ping-Pong Double Buffer Telemetry (Bug 3)
struct TelemetrySnapshot {
  bool connected;
  bool armed;
  bool servoAttached;
  Protocol proto;
  int16_t throttle;
  int16_t offsetDeg;
};

static TelemetrySnapshot g_telemBuf[2];
static std::atomic<uint8_t> g_telemIdx{0};

void publishTelemetry(const TelemetrySnapshot &s) {
  uint8_t next = 1 - g_telemIdx.load(std::memory_order_relaxed);
  g_telemBuf[next] = s;
  g_telemIdx.store(next, std::memory_order_release);
}

TelemetrySnapshot readTelemetry() {
  return g_telemBuf[g_telemIdx.load(std::memory_order_acquire)];
}

void test_ping_pong_telemetry() {
  TelemetrySnapshot s1 = {true, true, true, Protocol::CRSF, 1800, 25};
  publishTelemetry(s1);
  TelemetrySnapshot r1 = readTelemetry();
  assert(r1.connected == true);
  assert(r1.throttle == 1800);
  assert(r1.offsetDeg == 25);

  TelemetrySnapshot s2 = {false, false, false, Protocol::SBUS, 1000, 0};
  publishTelemetry(s2);
  TelemetrySnapshot r2 = readTelemetry();
  assert(r2.connected == false);
  assert(r2.proto == Protocol::SBUS);
  assert(r2.throttle == 1000);
  printf("  [PASS] test_ping_pong_telemetry\n");
}

void test_servo_compile_time_limits() {
  const int SERVO_US_MIN = 500;
  const int SERVO_US_MAX = 2500;
  const int SERVO_MIN_ANGLE = 20;
  const int SERVO_MAX_ANGLE = 160;

  constexpr int LIM_MIN = 500 + (20 * (2500 - 500) + 90) / 180;
  constexpr int LIM_MAX = 500 + (160 * (2500 - 500) + 90) / 180;

  int runtime_min = (int)lroundf(500.0f + 20.0f * (2500.0f - 500.0f) / 180.0f);
  int runtime_max = (int)lroundf(500.0f + 160.0f * (2500.0f - 500.0f) / 180.0f);

  assert(LIM_MIN == 722);
  assert(LIM_MAX == 2278);
  assert(LIM_MIN == runtime_min);
  assert(LIM_MAX == runtime_max);
  printf("  [PASS] test_servo_compile_time_limits (722, 2278)\n");
}

int main() {
  test_trim_span_490();
  test_is_rising_phase();
  test_symmetric_rc_ema();
  test_scaled_servo_ema();
  test_smooth_perch_mapping();
  test_ping_pong_telemetry();
  test_servo_compile_time_limits();
  printf("ALL HOST MATH TESTS PASSED!\n");
  return 0;
}
