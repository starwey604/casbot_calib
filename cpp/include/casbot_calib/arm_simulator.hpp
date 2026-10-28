#pragma once
#include "casbot_calib/arm_dynamics.hpp"
#include "casbot_calib/simulator.hpp"
namespace casbot {
struct ArmSimulationConfig {
  Nanoseconds integration_ns=50000,sample_period_ns=1000000,command_delay_ns=1350000,feedback_delay_ns=700000;
  double current_tau=.001,torque_limit=100.,position_noise=1e-6,current_noise=.0002;
  double drop_probability=0.,repeat_probability=0.;
  std::uint32_t seed=27;
  // Opt-in set-valued dry friction. Disabled for frozen historical regressions.
  bool stiction_enabled=false;
  double stick_velocity_tolerance=1e-7;
  Eigen::VectorXd gain;
};
struct ArmCommand {
  Nanoseconds send_ns;
  Eigen::VectorXd q,dq,torque,kp,kd;
};
struct ArmSample {
  Nanoseconds sequence,sample_ns,receive_ns;
  Eigen::VectorXd q,dq,current;
  Eigen::VectorXi status;
};
struct ArmTruth {
  Nanoseconds sample_ns;
  Eigen::VectorXd q,dq,ddq,current,request;
};
struct ArmSimulationResult {std::vector<ArmSample> observations;std::vector<ArmTruth> truth;};
// Persistent virtual-time plant for ROS/SDK Mock adapters. No wall clock or ROS
// dependency. submit() never rewinds time; advance_to() returns newly sampled
// observations (their delivery timestamps may still lie in the future).
class ArmSimulationSession {
 public:
  ArmSimulationSession(ArmDynamics plant,ArmSimulationConfig config,
                       const Eigen::VectorXd& initial_q,const Eigen::VectorXd& initial_v,
                       bool retain_truth=false);
  ~ArmSimulationSession();
  ArmSimulationSession(ArmSimulationSession&&) noexcept;
  ArmSimulationSession& operator=(ArmSimulationSession&&) noexcept;
  void submit(const ArmCommand& command);
  ArmSimulationResult advance_to(Nanoseconds end_ns);
  Nanoseconds time_ns() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
class ArmSimulationRunner {
 public:
  ArmSimulationRunner(ArmDynamics plant,ArmSimulationConfig config);
  ArmSimulationResult run(const std::vector<ArmCommand>& commands,Nanoseconds duration_ns,
                          const Eigen::VectorXd& initial_q,const Eigen::VectorXd& initial_v) const;
 private:
  ArmDynamics plant_;
  ArmSimulationConfig config_;
};
}
