#pragma once
#include <casbot_calib/shm/wire35.hpp>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace casbot::shm {
inline constexpr std::array<int, 14> arm_direction{1,-1,-1,-1,-1,1,1,-1,-1,-1,1,-1,-1,1};
inline constexpr std::array<double, 14> arm_offset{
  -.019698,.00643665,.005416,.007078,.026015,-.025037,-.007323,
  -.013976,-.00224065,-.01494833,.007841,-.01723733,-.01549655,.00541233};
inline constexpr std::array<const char*,14> arm_names{
  "left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint",
  "left_elbow_pitch_joint","left_wrist_yaw_joint","left_wrist_pitch_joint","left_wrist_roll_joint",
  "right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint",
  "right_elbow_pitch_joint","right_wrist_yaw_joint","right_wrist_pitch_joint","right_wrist_roll_joint"};
inline void check_arm(std::size_t j) { if (j>=14) throw std::out_of_range("Arm ID must be 1..14"); }
inline double arm_position(const StatusFrame& s, std::size_t j) {
  check_arm(j); return arm_direction[j]*static_cast<double>(s.joints[j+12].position)*1e-6+arm_offset[j];
}
template<class Integer> Integer checked_truncate(double value) {
  const double truncated = std::trunc(value);
  if (!std::isfinite(truncated) || truncated < std::numeric_limits<Integer>::min() || truncated > std::numeric_limits<Integer>::max())
    throw std::invalid_argument("SHM fixed-point value is not representable");
  return static_cast<Integer>(truncated);
}
inline void arm_target(CommandFrame& c, std::size_t j, double q, double v, double kp, double kd) {
  check_arm(j);
  if (!std::isfinite(kp) || !std::isfinite(kd) || kp<0 || kd<0) throw std::invalid_argument("Invalid PD gains");
  auto& a=c.joints[j+12];
  a.position=checked_truncate<std::int32_t>(arm_direction[j]*(q-arm_offset[j])*1e6);
  a.velocity=checked_truncate<std::int32_t>(arm_direction[j]*v*1e6);
  a.torque=0;
  a.kp=checked_truncate<std::int16_t>(kp*10); a.kd=checked_truncate<std::int16_t>(kd*10);
}
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
