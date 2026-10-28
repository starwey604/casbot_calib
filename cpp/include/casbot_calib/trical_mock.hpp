#pragma once
#include "casbot_calib/arm_simulator.hpp"
#include "casbot_calib/trical.hpp"
#include <array>
namespace casbot {
struct TricalMockConfig {
  TricalCalibration calibration;
  Eigen::Vector3d minimum=Eigen::Vector3d::Constant(-.005),maximum=Eigen::Vector3d::Constant(.005);
  Eigen::Vector3d bias=Eigen::Vector3d::Zero();
  Eigen::VectorXd encoder_offset; // physical q = observed q + offset; hidden in Mock only.
  double face_radius=.025,noise_std=.000001,quantum=.000001,drop_probability=0.;
  Nanoseconds period_ns=20000000,delay_ns=3000000,jitter_ns=500000;
  std::array<Nanoseconds,3> phase_ns{{0,1000000,2000000}};
  bool expose_sample_time=true;
  std::uint32_t seed=28;
};
struct IndicatorPacket {
  int channel=0,status=1;
  Nanoseconds sequence=0,sample_ns=-1,receive_ns=0;
  bool sample_known=false;
  double reading=0.;
  std::string target_id,tool_frame;
};
struct TricalMockResult {ArmSimulationResult simulation;std::vector<IndicatorPacket> indicators;};
// A sensor wrapper around the SAME persistent arm plant, not another dynamics engine.
// Physical state is used internally for readout; only observations reach the policy.
class TricalMockSession {
 public:
  TricalMockSession(ArmDynamics plant,ArmSimulationConfig arm,TricalMockConfig sensor,
                   const Eigen::VectorXd& q0,const Eigen::VectorXd& v0,bool retain_truth=false);
  ~TricalMockSession();
  void submit(const ArmCommand& command);
  void select_target(const std::string& frame,const std::string& target,const Eigen::Vector3d& ball_world);
  // none, disconnected, frozen, out_of_range, no_contact. Frozen retains old sequence/time.
  void set_fault(int channel,const std::string& fault);
  TricalMockResult advance_to(Nanoseconds end_ns);
  Nanoseconds time_ns() const;
  int joint_count() const;
 private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
}  // namespace casbot
