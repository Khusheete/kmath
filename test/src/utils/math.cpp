#include "math.hpp"


#include <cmath>


float ping_pong(const double t) {
  return std::abs(std::fmod(t + 1.0, 2.0) - 1.0);
}

