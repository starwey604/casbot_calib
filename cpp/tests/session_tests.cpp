#include <casbot_calib/arm_simulator.hpp>
#include <Eigen/QR>
#include <cmath>
#include <chrono>
#include <array>
#include <future>
#include <iostream>
#include <stdexcept>
using namespace casbot;
namespace {
void check(bool value, const char* message) {if(!value)throw std::runtime_error(message);}
template<class F> void rejects(F action) {
  bool rejected=false;try {action();}catch(const std::invalid_argument&){rejected=true;}
  check(rejected,"session invalid input");
}
}
int test_session_contracts(int argc, char** argv) {
  if(argc!=2)return 2;
  auto engine=ArmDynamics::from_urdf(argv[1]);
  ArmSimulationConfig config;config.gain=Eigen::VectorXd::LinSpaced(14,1.5,2.5);
  config.current_tau=.002;config.position_noise=0.;config.current_noise=0.;
  const auto zero=Eigen::VectorXd::Zero(14).eval();
  ArmCommand command{0,zero,zero,Eigen::VectorXd::LinSpaced(14,.1,.5),zero,zero};
  auto result=ArmSimulationRunner(engine,config).run({command},10'000'000,zero,zero);
  for(const auto& row:result.truth) {
    const double elapsed=std::max(0.,(row.sample_ns-config.command_delay_ns)*1e-9);
    const auto expected=((1-std::exp(-elapsed/config.current_tau))*command.torque.array()/config.gain.array()).matrix();
    check((row.current-expected).cwiseAbs().maxCoeff()<1e-9,"analytic current response");
  }
  for(const auto& row:result.observations)check(row.receive_ns-row.sample_ns==config.feedback_delay_ns,"causal timestamps");
  config.gain.setConstant(2.);config.current_tau=0.;config.command_delay_ns=0;config.feedback_delay_ns=0;config.integration_ns=25000;
  auto q=zero;q[3]=q[10]=-.5;
  ArmSimulationSession session(engine,config,q,zero);
  command.q=q;command.torque.setZero();command.torque[0]=2.;session.submit(command);
  check(std::abs(session.advance_to(1'000'000).observations.back().current[0]-1.)<1e-12,"powered current");
  auto mask=Eigen::VectorXi::Ones(14).eval();mask[0]=0;
  session.set_drive_enabled(session.time_ns(),mask);command.send_ns=session.time_ns();session.submit(command);
  result=session.advance_to(3'000'000);
  for(const auto& row:result.observations)check(row.current[0]==0.,"pending commands cannot re-enable");
  check((result.observations.back().q-q).norm()>0.,"disabled physics evolves");
  mask[0]=1;session.set_drive_enabled(session.time_ns(),mask);
  check(std::abs(session.advance_to(4'000'000).observations.back().current[0]-1.)<1e-12,"per-axis re-enable");
  rejects([&]{session.set_drive_enabled(1,mask);});
  rejects([&]{session.set_drive_enabled(session.time_ns(),Eigen::VectorXi::Ones(13));});
  rejects([&]{session.set_drive_enabled(session.time_ns(),2*mask);});
  check(session.advance_to(4'000'010).observations.empty(),"no observation off sample grid");
  rejects([&]{ArmSimulationRunner(engine,config).run({},1'000'000,Eigen::VectorXd::Zero(3),zero);});

  ArmModel planar;planar.model_id="session_planar";planar.gravity_world.setZero();
  for(int j=0;j<2;++j) {
    ArmJoint joint;joint.name="joint_"+std::to_string(j);joint.link_name="link_"+std::to_string(j);
    joint.origin.translation={j==0?0.:.7,0.,0.};joint.body.mass=j==0?1.3:.8;
    joint.body.com={j==0?.35:.225,0.,0.};joint.body.inertia_com=(j==0?.054:.0135)*Eigen::Matrix3d::Identity();
    planar.joints.push_back(joint);
  }
  ArmDynamics free_arm(planar);config.gain=Eigen::Vector2d::Constant(2.);config.integration_ns=50000;
  Eigen::Vector2d pq(.3,-.4),pv(.2,-.5);
  auto coarse=ArmSimulationRunner(free_arm,config).run({},100'000'000,pq,pv);
  config.integration_ns=25000;
  auto fine=ArmSimulationRunner(free_arm,config).run({},100'000'000,pq,pv);
  for(std::size_t i=0;i<coarse.truth.size();++i) {
    const auto& row=coarse.truth[i];
    check(std::abs(free_arm.energy(row.q,row.dq).kinetic-free_arm.energy(pq,pv).kinetic)<1e-12,"runner energy");
    check((row.q-fine.truth[i].q).norm()<1e-11,"runner step convergence");
  }
  config.drop_probability=.1;config.repeat_probability=.2;config.position_noise=1e-6;
  auto runner=ArmSimulationRunner(free_arm,config);
  auto left=runner.run({},100'000'000,pq,pv),right=runner.run({},100'000'000,pq,pv);
  check(left.observations.size()==right.observations.size(),"seeded packet count");bool repeated_packet=false;
  for(std::size_t i=0;i<left.observations.size();++i) {
    const auto& a=left.observations[i];const auto& b=right.observations[i];
    check(a.sequence==b.sequence&&a.sample_ns==b.sample_ns&&a.receive_ns==b.receive_ns&&
      (a.q-b.q).norm()==0.&&(a.dq-b.dq).norm()==0.&&(a.current-b.current).norm()==0.&&(a.status-b.status).norm()==0.,"seeded packet repeatability");
    for(int k=0;k<a.status.size();++k) repeated_packet|=(a.status[k]&2)!=0;
  }
  check(repeated_packet,"repeated packets exercised");

  // Independent stationary torque balance fc+b*v exposes sample-phase aliasing.
  ArmModel model;model.model_id="sampling_oracle";model.gravity_world.setZero();
  ArmJoint joint;joint.name="axis";joint.link_name="body";joint.body.inertia_com=.1*Eigen::Matrix3d::Identity();
  joint.friction.viscous=.035;joint.friction.coulomb=.04;joint.friction.static_friction=.07;joint.friction.stribeck_velocity=.002;
  model.joints={joint};
  // Each sampling grid owns its plant, workspace, RNG and result buffers.
  const auto sample_grid=[model](Nanoseconds sample_ns) {
    ArmDynamics plant(model);
    const auto started=std::chrono::steady_clock::now();
    Eigen::Matrix<double,3,2> design;Eigen::Vector3d targets;int row_index=0;
    for(double speed:{.04,.08,.14}) {
      std::vector<ArmCommand> commands;
      for(Nanoseconds time=0;time<2'000'000'000;time+=2'000'000)
        commands.push_back({time,Eigen::VectorXd::Constant(1,time*1e-9*speed),Eigen::VectorXd::Constant(1,speed),
          Eigen::VectorXd::Zero(1),Eigen::VectorXd::Constant(1,150.),Eigen::VectorXd::Constant(1,10.)});
      ArmSimulationConfig c;c.gain=Eigen::VectorXd::Ones(1);c.sample_period_ns=sample_ns;
      c.position_noise=0.;c.current_noise=0.;c.stiction_enabled=true;
      auto samples=ArmSimulationRunner(plant,c).run(commands,2'000'000'000,Eigen::VectorXd::Zero(1),Eigen::VectorXd::Zero(1));
      double current=0.,velocity=0.;int count=0;
      for(const auto& row:samples.observations)if(row.sample_ns>=1'000'000'000&&row.sample_ns<2'000'000'000) {
        current+=row.current[0];velocity+=row.dq[0];++count;
      }
      design.row(row_index)<<velocity/count,1.;targets[row_index++]=current/count;
    }
    Eigen::RowVector2d fitted=design.colPivHouseholderQr().solve(targets).transpose();
    return std::make_pair(fitted,std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count());
  };
  const std::array<Nanoseconds,3> periods{1'000'000,250'000,125'000};
  std::vector<std::future<std::pair<Eigen::RowVector2d,double>>> runs;
  for(auto period:periods)runs.push_back(std::async(std::launch::async,sample_grid,period));
  Eigen::Matrix<double,3,2> coefficients;
  for(std::size_t i=0;i<runs.size();++i) {
    const auto result=runs[i].get();coefficients.row(i)=result.first;
    std::cout<<"TEST_TIMING sampling_"<<periods[i]<<" "<<result.second<<" s\n";
  }
  Eigen::RowVector2d truth(.035,.04);
  auto errors=((coefficients.rowwise()-truth).array().rowwise()/truth.array()).abs().eval();
  check(errors(0,0)>.05,"coarse sampling aliasing");
  check(errors.row(1).maxCoeff()<.05&&errors.row(2).maxCoeff()<.015,"sampling convergence");
  check(((coefficients.row(1)-coefficients.row(2)).array()/truth.array()).abs().maxCoeff()<.05,"fine-grid agreement");
  std::cout<<"Power, current response, energy, repeatability and sampling contracts passed\n";
  return 0;
}
