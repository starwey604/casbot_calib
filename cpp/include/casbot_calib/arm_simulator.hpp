#pragma once
#include "casbot_calib/arm_dynamics.hpp"
#include "casbot_calib/simulator.hpp"
namespace casbot {
struct ArmSimulationConfig {
  Nanoseconds integration_ns=50000,sample_period_ns=1000000,command_delay_ns=1350000,feedback_delay_ns=700000;
  double current_tau=.001,torque_limit=100.,position_noise=1e-6,current_noise=.0002;
  double drop_probability=0.,repeat_probability=0.;
  std::uint32_t seed=27;
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
