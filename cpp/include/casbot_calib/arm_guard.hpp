#pragma once
#include "casbot_calib/arm_simulator.hpp"
#include <string>
namespace casbot {
struct ArmGuardConfig {
  int joint=0;
  Nanoseconds begin_ns=0,end_ns=0,sustain_ns=3'000'000,max_sample_age_ns=6'000'000;
  double displacement_threshold=.0002,speed_threshold=.005;
  double maximum_displacement=.03,maximum_speed=.4;
  double restore_kp=20.,restore_kd=2.,baseline_torque=0.;
};
struct ArmGuardEvent {
  std::string reason;
  Nanoseconds first_sample_ns=-1,confirmed_sample_ns=-1,decision_ns=-1;
  double initial_q=0.;
  bool fault=false;
};
// A control-side policy. Uses delivered samples only, never plant truth/FK.
// A hardware adapter may reuse it at its own command-adoption loop.
class ArmFeedbackGuard {
 public:
  explicit ArmFeedbackGuard(ArmGuardConfig config,int joints);
  void observe(const ArmSample& sample,Nanoseconds now);
  void apply(ArmCommand& command,Nanoseconds now);
  const std::vector<ArmGuardEvent>& events() const {return events_;}
 private:
  ArmGuardConfig c_;
  int joints_;
  bool anchored_=false,latched_=false,finished_=false,have_sample_=false;
  double initial_q_=0.;
  Nanoseconds candidate_=-1,last_sequence_=-1,last_sample_=-1;
  ArmSample latest_{};
  std::vector<ArmGuardEvent> events_;
  void latch(const char* reason,Nanoseconds now,Nanoseconds sample,bool fault);
};
struct ArmGuardedResult {
  ArmSimulationResult simulation;
  std::vector<ArmCommand> executed_commands;
  std::vector<ArmGuardEvent> events;
};
ArmGuardedResult run_guarded_arm(const ArmDynamics& plant,const ArmSimulationConfig& config,
  const std::vector<ArmCommand>& commands,Nanoseconds duration,const Eigen::VectorXd& initial_q,
  const Eigen::VectorXd& initial_v,const ArmGuardConfig& guard);
}
