#include <casbot_calib/simulated_device.hpp>
#include <stdexcept>
#include <iostream>
#include <type_traits>

void check(bool condition){if(!condition)throw std::runtime_error("Shared simulated device check failed");}

int main(int argc,char** argv) {
  if(argc!=2)return 2;
  auto model=casbot::ArmDynamics::from_urdf(argv[1]).model();model.gravity_world.setZero();
  casbot::ArmDynamics dynamics(model);const auto n=model.joints.size();
  casbot::ArmSimulationConfig config;config.gain=Eigen::VectorXd::Constant(n,2.);
  config.position_noise=0.;config.current_noise=0.;config.feedback_delay_ns=3000000;
  Eigen::VectorXd q=Eigen::VectorXd::Zero(n);q[3]=-.5;q[10]=-.5;
  casbot::SimulatedArmDevice device(dynamics,config,q);
  casbot::ArmSimulationSession direct(dynamics,config,q,Eigen::VectorXd::Zero(n),false);
  casbot::ArmCommand command{0,q,Eigen::VectorXd::Zero(n),Eigen::VectorXd::Zero(n),Eigen::VectorXd::Constant(n,2.),Eigen::VectorXd::Constant(n,.1)};
  command.q[0]=.03;device.submit(command);direct.submit(command);
  std::deque<casbot::ArmSample> queued;
  for(std::int64_t now=1000000;now<=12000000;now+=1000000) {
    auto expected=direct.advance_to(now);for(auto& row:expected.observations)queued.push_back(std::move(row));
    auto actual=device.advance_to(now);std::size_t index=0;
    while(!queued.empty()&&queued.front().receive_ns<=now) {
      check(index<actual.size());const auto& a=actual[index++];const auto& b=queued.front();
      check(a.sequence==b.sequence&&a.sample_ns==b.sample_ns&&a.receive_ns==b.receive_ns);
      check((a.q-b.q).norm()==0.&&(a.dq-b.dq).norm()==0.&&(a.current-b.current).norm()==0.);queued.pop_front();
    }
    check(index==actual.size());check(device.has_feedback()==(now>=3000000));
    if(device.has_feedback())check(device.latest().receive_ns<=now);
  }
  // Configured total PD+feedforward clamp is independent of SDK effort acceptance.
  auto bounded=config;bounded.current_tau=0.;bounded.command_delay_ns=0;bounded.feedback_delay_ns=0;bounded.torque_limit=7.;
  casbot::SimulatedArmDevice saturated(dynamics,bounded,q);
  auto force=command;force.q=q;force.kp.setConstant(100.);force.kd.setZero();force.q[0]+=.1;force.torque.setZero();force.torque[0]=120.;
  saturated.submit(force);saturated.advance_to(2000000);
  check(std::abs(saturated.latest().current[0]-3.5)<1e-12);
  // The physics adapter reuses the established plant settings factory.
  auto settings=casbot::simulated_arm_settings(argv[1],[&](const std::string& key,auto fallback){
    using T=std::decay_t<decltype(fallback)>;
    if constexpr(std::is_same_v<T,std::vector<double>>) {
      if(key=="inertia_scales")return std::vector<double>(n,1.5);
      if(key=="friction_viscous")return std::vector<double>(n,.02);
    }
    return fallback;
  });
  casbot::ArmDynamics changed(settings.model);
  check((changed.mass_matrix(q)-dynamics.mass_matrix(q)).norm()>1e-6);
  check((changed.friction(Eigen::VectorXd::Ones(n))-Eigen::VectorXd::Constant(n,.02)).norm()<1e-12);
  // Zero PD remains a valid enabled pure feedforward command, not disable.
  casbot::SimulatedArmDevice powered(dynamics,bounded,q);
  auto torque=command;torque.kp.setZero();torque.kd.setZero();torque.torque.setZero();torque.torque[0]=2.;
  powered.submit(torque);powered.advance_to(2000000);
  check(std::abs(powered.latest().current[0]-1.)<1e-12);
  auto mask=Eigen::VectorXi::Ones(n).eval();mask[0]=0;
  powered.set_drive_enabled(powered.time_ns(),mask);
  torque.send_ns=powered.time_ns();torque.torque[0]=6.;powered.submit(torque);
  powered.advance_to(4000000);check(powered.latest().current[0]==0.);
  mask[0]=1;powered.set_drive_enabled(powered.time_ns(),mask);
  powered.advance_to(6000000);check(std::abs(powered.latest().current[0]-3.)<1e-12);
  // Disabling cuts a stored current state immediately, even with delayed commands.
  auto lagged=config;lagged.feedback_delay_ns=0;
  casbot::SimulatedArmDevice pending(dynamics,lagged,q);
  torque.send_ns=0;pending.submit(torque);pending.advance_to(5000000);
  mask.setZero();pending.set_drive_enabled(pending.time_ns(),mask);
  torque.send_ns=pending.time_ns();pending.submit(torque);pending.advance_to(10000000);
  check(pending.latest().current.norm()==0.);
  // An unpowered, fixed-base arm still falls under physical gravity.
  auto gravity_model=model;gravity_model.gravity_world=Eigen::Vector3d(0,0,-9.81);
  auto gravity_config=bounded;gravity_config.integration_ns=25000;
  casbot::SimulatedArmDevice falling(casbot::ArmDynamics(gravity_model),gravity_config,q);
  falling.set_drive_enabled(0,Eigen::VectorXi::Zero(n));falling.advance_to(20000000);
  check((falling.latest().q-q).norm()>1e-6&&falling.latest().current.norm()==0.);
  auto half=gravity_config;half.integration_ns/=2;
  casbot::SimulatedArmDevice falling_half(casbot::ArmDynamics(gravity_model),half,q);
  falling_half.set_drive_enabled(0,Eigen::VectorXi::Zero(n));falling_half.advance_to(20000000);
  check((falling_half.latest().q-falling.latest().q).norm()<1e-7);
  check(device.latest().q[0]>q[0]);std::cout<<"shared device preserves session dynamics and delayed delivery\n";
}
