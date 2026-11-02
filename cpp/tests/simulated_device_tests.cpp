#include <casbot_calib/simulated_device.hpp>
#include <stdexcept>
#include <iostream>

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
  check(device.latest().q[0]>q[0]);std::cout<<"shared device preserves session dynamics and delayed delivery\n";
}
