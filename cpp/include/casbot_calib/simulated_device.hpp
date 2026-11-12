#pragma once
#include "casbot_calib/arm_simulator.hpp"
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>
#include <utility>

namespace casbot {
// Device-side transport adapter. The existing session remains the only owner
// of mechanics, local PD, current response and virtual time.
class SimulatedArmDevice {
 public:
  SimulatedArmDevice(ArmDynamics model,ArmSimulationConfig config,const Eigen::VectorXd& q)
    :model_(std::move(model)),session_(model_,config,q,Eigen::VectorXd::Zero(q.size()),false) {
    latest_.q=q;latest_.dq=Eigen::VectorXd::Zero(q.size());latest_.current=latest_.dq;
    latest_.status=Eigen::VectorXi::Zero(q.size());
  }
  void submit(const ArmCommand& command){session_.submit(command);}
  std::vector<ArmSample> advance_to(Nanoseconds now) {
    auto chunk=session_.advance_to(now);
    for(auto& sample:chunk.observations)pending_.push_back(std::move(sample));
    std::vector<ArmSample> delivered;
    while(!pending_.empty()&&pending_.front().receive_ns<=now) {
      latest_=std::move(pending_.front());pending_.pop_front();has_feedback_=true;
      delivered.push_back(latest_);
    }
    return delivered;
  }
  const ArmSample& latest() const{return latest_;}
  bool has_feedback() const{return has_feedback_;}
  Nanoseconds time_ns() const{return session_.time_ns();}
  const ArmDynamics& model() const{return model_;}
 private:
  ArmDynamics model_;
  ArmSimulationSession session_;
  std::deque<ArmSample> pending_;
  ArmSample latest_{};
  bool has_feedback_=false;
};

struct SimulatedArmSettings {
  ArmModel model;
  ArmSimulationConfig simulation;
  Eigen::VectorXd initial_q;
};

// Both ROS endpoints read the same plant settings. The reader is injected so
// this header and the device itself have no ROS or platform dependency.
template<class Read>
SimulatedArmSettings simulated_arm_settings(const std::string& path,Read read) {
  SimulatedArmSettings result;result.model=ArmDynamics::from_urdf(path).model();
  const auto n=result.model.joints.size();
  const auto inertia=read("inertia_scales",std::vector<double>(n,1.));
  const auto viscous=read("friction_viscous",std::vector<double>(n,0.));
  const auto coulomb=read("friction_coulomb",std::vector<double>(n,0.));
  const auto friction_static=read("friction_static",std::vector<double>(n,0.));
  if(inertia.size()!=n||viscous.size()!=n||coulomb.size()!=n||friction_static.size()!=n)
    throw std::invalid_argument("One synthetic inertia/friction setting per model joint required");
  for(std::size_t j=0;j<n;++j) {
    if(!std::isfinite(inertia[j])||inertia[j]<.1||inertia[j]>5.||!std::isfinite(viscous[j])||
       !std::isfinite(coulomb[j])||!std::isfinite(friction_static[j])||viscous[j]<0||coulomb[j]<0||friction_static[j]<coulomb[j])
      throw std::invalid_argument("Invalid synthetic inertia/friction");
    auto& joint=result.model.joints[j];joint.body.inertia_com*=inertia[j];
    joint.friction.viscous=viscous[j];joint.friction.coulomb=coulomb[j];
    joint.friction.static_friction=friction_static[j];joint.friction.stribeck_velocity=.005;
  }
  if(read("zero_gravity",true))result.model.gravity_world.setZero();
  auto& c=result.simulation;c.gain=Eigen::VectorXd::Constant(n,2.);
  c.stiction_enabled=read("stiction_enabled",false);
  c.sample_period_ns=read("sample_period_ns",std::int64_t(1000000));
  c.integration_ns=read("integration_ns",std::int64_t(50000));
  c.current_tau=read("current_tau",.001);
  c.torque_limit=read("torque_limit_nm",100.);
  c.command_delay_ns=read("drive_delay_ns",std::int64_t(1350000));
  c.feedback_delay_ns=read("feedback_delay_ns",std::int64_t(700000));
  const auto q=read("initial_q",std::vector<double>(n,0.));
  if(q.size()!=n)throw std::invalid_argument("Initial pose must match model joints");
  result.initial_q=Eigen::Map<const Eigen::VectorXd>(q.data(),q.size());return result;
}
}
