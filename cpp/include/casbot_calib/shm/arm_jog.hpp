#pragma once
#include <casbot_calib/shm/arm_codec.hpp>

namespace casbot::shm {
inline bool jog_within_limits(double q, double lower, double upper, double amplitude) {
  return std::isfinite(q) && std::isfinite(lower) && std::isfinite(upper) && std::isfinite(amplitude) &&
    lower<upper && amplitude>0 && q-amplitude>=lower && q+amplitude<=upper;
}
struct JogValue { double position, velocity; };
// q0 -> q0-a -> q0+a -> q0. Each leg has C3 joins and equal duration.
inline JogValue jog_at(double seconds, double segment_s, double amplitude) {
  if (!std::isfinite(segment_s) || segment_s<=0 || !std::isfinite(amplitude) || amplitude<=0 ||
      !std::isfinite(seconds)) throw std::invalid_argument("Invalid jog waveform");
  if (seconds<=0 || seconds>=3*segment_s) return {0,0};
  const int leg=static_cast<int>(seconds/segment_s);
  const double u=(seconds-leg*segment_s)/segment_s;
  const double f=u*u*u*u*(35+u*(-84+u*(70-20*u)));
  const double df=140*u*u*u*std::pow(1-u,3)/segment_s;
  const std::array<double,4> x{0,-amplitude,amplitude,0};
  return {x[leg]+(x[leg+1]-x[leg])*f,(x[leg+1]-x[leg])*df};
}
}  // namespace casbot::shm
