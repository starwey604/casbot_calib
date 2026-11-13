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
// CiA402 PDS state excludes voltage-present bit 4 and non-state flags.
inline bool drive_state_is(std::uint16_t status, std::uint16_t expected) {
  return (status & 0x006f)==expected;
}
inline void check_arm(std::size_t j) { if (j>=14) throw std::out_of_range("Internal arm index must be 0..13"); }
inline std::size_t arm_index(int id) {
  if (id<12 || id>25) throw std::out_of_range("Joint ID must be 12..25 (arms only)");
  return static_cast<std::size_t>(id-12);
}
inline double arm_position(const StatusFrame& s, std::size_t j) {
  check_arm(j); return arm_direction[j]*static_cast<double>(s.joints[j+12].position)*1e-6+arm_offset[j];
}
template<class Integer> Integer checked_truncate(double value) {
  const double truncated = std::trunc(value);
  if (!std::isfinite(truncated) || truncated < std::numeric_limits<Integer>::min() || truncated > std::numeric_limits<Integer>::max())
    throw std::invalid_argument("SHM fixed-point value is not representable");
  return static_cast<Integer>(truncated);
}
inline void arm_target(CommandFrame& c, std::size_t j, double q, double v, double kp, double kd, double effort=0) {
  check_arm(j);
  if (!std::isfinite(kp) || !std::isfinite(kd) || kp<0 || kd<0) throw std::invalid_argument("Invalid PD gains");
  auto& a=c.joints[j+12];
  a.position=checked_truncate<std::int32_t>(arm_direction[j]*(q-arm_offset[j])*1e6);
  a.velocity=checked_truncate<std::int32_t>(arm_direction[j]*v*1e6);
  a.torque=checked_truncate<std::int16_t>(arm_direction[j]*effort*100);
  a.kp=checked_truncate<std::int16_t>(kp*10); a.kd=checked_truncate<std::int16_t>(kd*10);
}
struct ArmFields {double position,velocity,effort,kp,kd;};
inline ArmFields arm_command(const CommandFrame& c,std::size_t j) {
  check_arm(j);const auto& a=c.joints[j+12];
  return {arm_direction[j]*double(a.position)*1e-6+arm_offset[j],arm_direction[j]*double(a.velocity)*1e-6,
    arm_direction[j]*double(a.torque)*.01,a.kp*.1,a.kd*.1};
}
struct ArmFeedback {double position,velocity,effort;};
inline ArmFeedback arm_feedback(const StatusFrame& s,std::size_t j) {
  check_arm(j);const auto& a=s.joints[j+12];
  return {arm_position(s,j),arm_direction[j]*double(a.velocity)*1e-6,arm_direction[j]*double(a.torque)*.01};
}
inline void arm_status(StatusFrame& s,std::size_t j,double q,double v,double effort) {
  CommandFrame c{};arm_target(c,j,q,v,0,0,effort);
  auto& a=s.joints[j+12];a.position=c.joints[j+12].position;a.velocity=c.joints[j+12].velocity;a.torque=c.joints[j+12].torque;
}
}  // namespace casbot::shm
