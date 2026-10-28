#pragma once
#include "casbot_calib/arm_dynamics.hpp"

namespace casbot {
// Ideal mastered flat-tip indicator geometry; no contact or force simulation.
// readings = axes * (R_WS^T * (ball_W - position_WS)) + zero.
// axes: unit row normals in sensor frame; zero: centered readings (metres).
// Ball/master radii and mechanical mastering are incorporated in zero beforehand.
struct TricalCalibration {
  Eigen::Matrix3d axes = Eigen::Matrix3d::Identity();
  Eigen::Vector3d zero = Eigen::Vector3d::Zero();
};
Eigen::Vector3d trical_readings(const ArmPose& sensor_world,const Eigen::Vector3d& ball_world,
                              const TricalCalibration& calibration);
Eigen::Vector3d trical_offset(const Eigen::Vector3d& readings,const TricalCalibration& calibration);
}  // namespace casbot
