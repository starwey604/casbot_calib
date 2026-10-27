#include "casbot_calib/arm_simulator.hpp"
#include "casbot_calib/integration.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
namespace casbot {
ArmSimulationRunner::ArmSimulationRunner(ArmDynamics plant,ArmSimulationConfig config):plant_(std::move(plant)),config_(std::move(config)) {
  const auto n=static_cast<Eigen::Index>(plant_.model().joints.size());const auto& c=config_;
  if(c.integration_ns<=0 || c.sample_period_ns<=0 || c.command_delay_ns<0 || c.feedback_delay_ns<0 ||
     !std::isfinite(c.current_tau) || c.current_tau<0 || (c.current_tau>0 && c.integration_ns*1e-9>c.current_tau/2) ||
     !std::isfinite(c.torque_limit) || c.torque_limit<=0 || c.gain.size()!=n || !c.gain.allFinite() || c.gain.minCoeff()<=0)
    throw std::invalid_argument("Invalid arm drive or timing configuration");
  for(double x:{c.position_noise,c.current_noise,c.drop_probability,c.repeat_probability})
    if(!std::isfinite(x) || x<0) throw std::invalid_argument("Invalid arm noise/packet configuration");
  if(c.drop_probability>1 || c.repeat_probability>1) throw std::invalid_argument("Invalid packet probability");
}
ArmSimulationResult ArmSimulationRunner::run(const std::vector<ArmCommand>& commands,Nanoseconds duration,
                                             const Eigen::VectorXd& q0,const Eigen::VectorXd& v0) const {
  const auto& c=config_;const Eigen::Index n=c.gain.size();
  const auto valid=[&](const Eigen::VectorXd& x){return x.size()==n && x.allFinite();};
  if(duration<=0 || !valid(q0) || !valid(v0)) throw std::invalid_argument("Invalid arm initial state/duration");
  Nanoseconds previous=-1;
  for(const auto& cmd:commands) {
    if(cmd.send_ns<0 || cmd.send_ns<=previous || cmd.send_ns>=duration || !valid(cmd.q) || !valid(cmd.dq) ||
       !valid(cmd.torque) || !valid(cmd.kp) || !valid(cmd.kd) || cmd.kp.minCoeff()<0 || cmd.kd.minCoeff()<0)
      throw std::invalid_argument("Invalid arm command batch");
    previous=cmd.send_ns;
  }
  Eigen::VectorXd state=Eigen::VectorXd::Zero(3*n);state.head(n)=q0;state.segment(n,n)=v0;
  FixedStepIntegrator<Eigen::VectorXd> integrator(state,c.integration_ns);auto dynamics=plant_.workspace();
  ArmCommand held{0,q0,Eigen::VectorXd::Zero(n),Eigen::VectorXd::Zero(n),Eigen::VectorXd::Zero(n),Eigen::VectorXd::Zero(n)};
  const auto request=[&](const Eigen::VectorXd& x)->Eigen::VectorXd {
    return held.torque+held.kp.cwiseProduct(held.q-x.head(n))+held.kd.cwiseProduct(held.dq-x.segment(n,n));
  };
  const auto target=[&](const Eigen::VectorXd& x)->Eigen::VectorXd {
    return request(x).cwiseMax(-c.torque_limit).cwiseMin(c.torque_limit).cwiseQuotient(c.gain);
  };
  const auto rhs=[&](const Eigen::VectorXd& x,Eigen::VectorXd& rate,double) {
    const Eigen::VectorXd current=c.current_tau>0?Eigen::VectorXd(x.tail(n)):target(x);
    rate.head(n)=x.segment(n,n);
    rate.segment(n,n)=dynamics.forward(x.head(n),x.segment(n,n),c.gain.cwiseProduct(current));
    if(c.current_tau>0) rate.tail(n)=(target(x)-x.tail(n))/c.current_tau;
    else rate.tail(n).setZero();
  };
  std::mt19937 rng(c.seed);
  const auto uniform=[&](){return (static_cast<double>(rng())+.5)/4294967296.;};
  const auto normal=[&](){const double a=uniform(),b=uniform();return std::sqrt(-2.*std::log(a))*std::cos(2.*std::acos(-1.)*b);};
  ArmSimulationResult result;std::size_t command_index=0;Nanoseconds t=0,next_sample=0,seq=0;
  ArmSample last{};bool has_last=false;
  while(t<=duration) {
    while(command_index<commands.size() && commands[command_index].send_ns<=t-c.command_delay_ns)
      held=commands[command_index++];
    if(c.current_tau==0) state.tail(n)=target(state);
    if(t==next_sample) {
      if(uniform()>=c.drop_probability) {
        ArmSample sample;
        if(has_last && uniform()<c.repeat_probability) {sample=last;for(Eigen::Index j=0;j<n;++j) sample.status[j]|=2;}
        else {
          sample.sequence=seq;sample.sample_ns=t;sample.q=state.head(n);sample.dq=state.segment(n,n);sample.current=state.tail(n);
          sample.status=Eigen::VectorXi::Zero(n);const auto torque=request(state);
          for(Eigen::Index j=0;j<n;++j) {
            sample.q[j]+=c.position_noise*normal();sample.current[j]+=c.current_noise*normal();
            sample.status[j]=std::abs(torque[j])>c.torque_limit?1:0;
          }
          last=sample;has_last=true;
        }
        sample.receive_ns=t+c.feedback_delay_ns;result.observations.push_back(std::move(sample));
      }
      Eigen::VectorXd rate(state.size());rhs(state,rate,0.);
      result.truth.push_back({t,state.head(n),state.segment(n,n),rate.segment(n,n),state.tail(n),request(state)});
      next_sample+=c.sample_period_ns;++seq;
    }
    if(t==duration) break;
    Nanoseconds next=t+std::min(c.integration_ns,duration-t);next=std::min(next,next_sample);
    if(command_index<commands.size() && commands[command_index].send_ns<=duration-c.command_delay_ns)
      next=std::min(next,commands[command_index].send_ns+c.command_delay_ns);
    integrator.step(rhs,state,t,next);
    if(!state.allFinite() || state.cwiseAbs().maxCoeff()>1e8) throw std::runtime_error("Unstable arm simulation");
    t=next;
  }
  return result;
}
}
