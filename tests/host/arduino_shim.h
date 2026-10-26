#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <string.h>
#include <atomic>

template <typename T>
inline T constrain(T amt, T low, T high) {
  return (amt < low) ? low : ((amt > high) ? high : amt);
}

enum class Protocol { DETECT, CRSF, SBUS };
