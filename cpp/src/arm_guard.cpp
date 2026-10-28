#include "casbot_calib/arm_guard.hpp"
#include <cmath>
#include <deque>
#include <iterator>
#include <stdexcept>
namespace casbot {
ArmFeedbackGuard::ArmFeedbackGuard(ArmGuardConfig c,int n):c_(c),joints_(n) {
  if(n<=0 || c.joint<0 || c.joint>=n || c.begin_ns<0 || c.end_ns<=c.begin_ns || c.sustain_ns<0 || c.max_sample_age_ns<=0 ||
     !std::isfinite(c.displacement_threshold) || c.displacement_threshold<=0 || !std::isfinite(c.speed_threshold) || c.speed_threshold<=0 ||
     !std::isfinite(c.maximum_displacement) || c.maximum_displacement<=c.displacement_threshold ||
     !std::isfinite(c.maximum_speed) || c.maximum_speed<=c.speed_threshold || !std::isfinite(c.baseline_torque) ||
     !std::isfinite(c.restore_kp) || c.restore_kp<=0 || !std::isfinite(c.restore_kd) || c.restore_kd<=0)
    throw std::invalid_argument("Invalid feedback guard configuration");
}
void ArmFeedbackGuard::latch(const char* reason,Nanoseconds now,Nanoseconds sample,bool fault) {
  if(latched_ || finished_)return;
  latched_=true;events_.push_back({reason,candidate_,sample,now,initial_q_,fault});
}
void ArmFeedbackGuard::observe(const ArmSample& sample,Nanoseconds now) {
  if(sample.receive_ns>now || sample.sample_ns>sample.receive_ns || sample.q.size()!=joints_ || sample.dq.size()!=joints_ ||
     sample.status.size()!=joints_ || !sample.q.allFinite() || !sample.dq.allFinite())
    throw std::invalid_argument("Guard requires valid delivered samples in one mapped clock");
  if(sample.sequence<=last_sequence_ || sample.sample_ns<=last_sample_) {candidate_=-1;return;}
  if(last_sequence_>=0 && sample.sequence!=last_sequence_+1)candidate_=-1;
  last_sequence_=sample.sequence;last_sample_=sample.sample_ns;latest_=sample;have_sample_=true;
  if(!anchored_ || finished_ || sample.sample_ns<c_.begin_ns || sample.sample_ns>=c_.end_ns)return;
  if(latched_) {
    if(!events_.back().fault && (std::abs(sample.q[c_.joint]-initial_q_)>=c_.maximum_displacement || std::abs(sample.dq[c_.joint])>=c_.maximum_speed)) {
      events_.back().fault=true;events_.back().reason="recovery_motion_limit";
    }
    return;
  }
  if(sample.status[c_.joint]!=0) {latch("invalid_feedback",now,sample.sample_ns,true);return;}
  const double distance=std::abs(sample.q[c_.joint]-initial_q_),speed=std::abs(sample.dq[c_.joint]);
  if(distance>=c_.maximum_displacement) {latch("position_limit",now,sample.sample_ns,true);return;}
  if(speed>=c_.maximum_speed) {latch("velocity_limit",now,sample.sample_ns,true);return;}
  if(distance>=c_.displacement_threshold && speed>=c_.speed_threshold) {
    if(candidate_<0)candidate_=sample.sample_ns;
    if(sample.sample_ns-candidate_>=c_.sustain_ns)latch("breakaway_confirmed",now,sample.sample_ns,false);
  } else candidate_=-1;
}
void ArmFeedbackGuard::apply(ArmCommand& command,Nanoseconds now) {
  if(command.send_ns!=now || command.q.size()!=joints_ || command.dq.size()!=joints_ || command.torque.size()!=joints_ ||
     command.kp.size()!=joints_ || command.kd.size()!=joints_)throw std::invalid_argument("Guard command shape/time mismatch");
  if(now>=c_.end_ns) {
    if(!latched_ && !finished_) {candidate_=-1;latch("not_detected",now,last_sample_,true);}
    finished_=true;return;
  }
  if(now<c_.begin_ns)return;
  if(!anchored_) {
    initial_q_=have_sample_?latest_.q[c_.joint]:command.q[c_.joint];anchored_=true;
  }
  if(!have_sample_ || now-last_sample_>c_.max_sample_age_ns) {
    if(latched_) {if(!events_.back().fault) {events_.back().fault=true;events_.back().reason="recovery_feedback_timeout";}}
    else latch("feedback_timeout",now,last_sample_,true);
  }
  if(have_sample_ && latest_.status[c_.joint]!=0)latch("invalid_feedback",now,last_sample_,true);
  if(latched_) {
    command.q[c_.joint]=initial_q_;command.dq[c_.joint]=0.;command.torque[c_.joint]=c_.baseline_torque;
    command.kp[c_.joint]=c_.restore_kp;command.kd[c_.joint]=c_.restore_kd;
  }
}
ArmGuardedResult run_guarded_arm(const ArmDynamics& plant,const ArmSimulationConfig& config,
 const std::vector<ArmCommand>& commands,Nanoseconds duration,const Eigen::VectorXd& q,const Eigen::VectorXd& v,const ArmGuardConfig& guard) {
  if(commands.empty() || commands.front().send_ns!=0 || duration<=guard.end_ns || commands.back().send_ns>=duration || commands.back().send_ns<guard.end_ns)
    throw std::invalid_argument("Guarded batch needs a time-zero command and recovery after the guard window");
  ArmSimulationSession session(plant,config,q,v,true);ArmFeedbackGuard policy(guard,static_cast<int>(q.size()));
  ArmGuardedResult result;std::deque<ArmSample> delivery;
  auto advance=[&](Nanoseconds time) {
    auto chunk=session.advance_to(time);
    for(const auto& sample:chunk.observations) {delivery.push_back(sample);result.simulation.observations.push_back(sample);}
    result.simulation.truth.insert(result.simulation.truth.end(),std::make_move_iterator(chunk.truth.begin()),std::make_move_iterator(chunk.truth.end()));
    while(!delivery.empty() && delivery.front().receive_ns<=time) {policy.observe(delivery.front(),time);delivery.pop_front();}
  };
  for(auto command:commands) {
    advance(command.send_ns);policy.apply(command,command.send_ns);session.submit(command);result.executed_commands.push_back(std::move(command));
  }
  advance(duration);result.events=policy.events();return result;
}
}
