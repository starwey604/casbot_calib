#include "casbot_calib/arm_simulator.hpp"
#include "casbot_calib/integration.hpp"
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <random>
#include <stdexcept>

namespace casbot {
namespace {
ArmSimulationConfig validated(const ArmDynamics& plant,ArmSimulationConfig c) {
  const auto n=static_cast<Eigen::Index>(plant.model().joints.size());
  if(c.integration_ns<=0 || c.sample_period_ns<=0 || c.command_delay_ns<0 || c.feedback_delay_ns<0 ||
     !std::isfinite(c.current_tau) || c.current_tau<0 || (c.current_tau>0 && c.integration_ns*1e-9>c.current_tau/2) ||
     !std::isfinite(c.torque_limit) || c.torque_limit<=0 || c.gain.size()!=n || !c.gain.allFinite() || c.gain.minCoeff()<=0 ||
     !std::isfinite(c.stick_velocity_tolerance) || c.stick_velocity_tolerance<0)
    throw std::invalid_argument("Invalid arm drive or timing configuration");
  for(double x:{c.position_noise,c.current_noise,c.drop_probability,c.repeat_probability})
    if(!std::isfinite(x) || x<0) throw std::invalid_argument("Invalid arm noise/packet configuration");
  if(c.drop_probability>1 || c.repeat_probability>1) throw std::invalid_argument("Invalid packet probability");
  return c;
}
Eigen::VectorXd initial_state(Eigen::Index n,const Eigen::VectorXd& q,const Eigen::VectorXd& v) {
  if(q.size()!=n || v.size()!=n || !q.allFinite() || !v.allFinite()) throw std::invalid_argument("Invalid arm initial state/duration");
  Eigen::VectorXd x=Eigen::VectorXd::Zero(3*n);x.head(n)=q;x.segment(n,n)=v;return x;
}
}

struct ArmSimulationSession::Impl {
  ArmDynamics plant;
  ArmSimulationConfig c;
  Eigen::Index n;
  Eigen::VectorXd state;
  FixedStepIntegrator<Eigen::VectorXd> integrator;
  ArmWorkspace dynamics;
  ArmCommand held;
  std::deque<ArmCommand> pending;
  std::mt19937 rng;
  Nanoseconds t=0,next_sample=0,sequence=0,last_send=-1;
  ArmSample last{};
  bool has_last=false,truth;
  std::vector<bool> stuck;
  Eigen::VectorXd slip_sign;
  std::vector<ArmFriction> friction_parameters;
  bool all_stuck=false;
  Impl(ArmDynamics p,ArmSimulationConfig config,const Eigen::VectorXd& q,const Eigen::VectorXd& v,bool keep)
    :plant(std::move(p)),c(validated(plant,std::move(config))),n(c.gain.size()),state(initial_state(n,q,v)),
     integrator(state,c.integration_ns),dynamics(plant.workspace()),
     held{0,q,Eigen::VectorXd::Zero(n),Eigen::VectorXd::Zero(n),Eigen::VectorXd::Zero(n),Eigen::VectorXd::Zero(n)},
     rng(c.seed),truth(keep),stuck(n,false),slip_sign(Eigen::VectorXd::Ones(n)) {
       for(const auto& joint:plant.model().joints)friction_parameters.push_back(joint.friction);
     }
  double uniform() {return (static_cast<double>(rng())+.5)/4294967296.;}
  double normal() {const double a=uniform(),b=uniform();return std::sqrt(-2.*std::log(a))*std::cos(2.*std::acos(-1.)*b);}
  Eigen::VectorXd request(const Eigen::VectorXd& x) const {
    return held.torque+held.kp.cwiseProduct(held.q-x.head(n))+held.kd.cwiseProduct(held.dq-x.segment(n,n));
  }
  Eigen::VectorXd target(const Eigen::VectorXd& x) const {
    return request(x).cwiseMax(-c.torque_limit).cwiseMin(c.torque_limit).cwiseQuotient(c.gain);
  }
  Eigen::VectorXd dry_friction(const Eigen::VectorXd& v) const {
    Eigen::VectorXd f(n);
    for(Eigen::Index j=0;j<n;++j) {
      const auto& p=friction_parameters[j];
      const double ratio=v[j]/p.stribeck_velocity;
      f[j]=stuck[j]?0.:p.viscous*v[j]+slip_sign[j]*(p.coulomb+(p.static_friction-p.coulomb)*std::exp(-ratio*ratio));
    }
    return f;
  }
  void project_stopped_velocity(const std::vector<bool>& active) {
    std::vector<int> indices;
    for(int j=0;j<n;++j)if(active[j])indices.push_back(j);
    if(indices.empty())return;
    Eigen::VectorXd stopped(indices.size());
    for(std::size_t k=0;k<indices.size();++k)stopped[k]=state[n+indices[k]];
    if(stopped.squaredNorm()==0.)return;
    if(static_cast<Eigen::Index>(indices.size())==n) {state.segment(n,n).setZero();return;}
    Eigen::MatrixXd selector=Eigen::MatrixXd::Zero(n,indices.size());
    for(std::size_t k=0;k<indices.size();++k)selector(indices[k],k)=1.;
    const Eigen::MatrixXd inverse_columns=dynamics.mass_matrix(state.head(n)).ldlt().solve(selector);
    const Eigen::MatrixXd effective=selector.transpose()*inverse_columns;
    state.segment(n,n)-=inverse_columns*effective.ldlt().solve(stopped);
    // Minimum kinetic-energy projection, including already-stuck coordinates.
    // Independent scalar clipping can add energy through off-diagonal inertia.
    for(int j:indices)state[n+j]=0.;
  }
  Eigen::VectorXd residual(const Eigen::VectorXd& x) {
    const Eigen::VectorXd current=c.current_tau>0?Eigen::VectorXd(x.tail(n)):target(x);
    return c.gain.cwiseProduct(current)-dynamics.gravity(x.head(n))-
      dynamics.velocity_bias(x.head(n),x.segment(n,n))-dry_friction(x.segment(n,n));
  }
  // For active sticking coordinates a_s=0. Solve the free block of Pinocchio's
  // mass matrix, then recover the coupled reaction at every sticking axis.
  std::pair<Eigen::VectorXd,Eigen::VectorXd> constrained(const Eigen::VectorXd& x) {
    Eigen::VectorXd acceleration=Eigen::VectorXd::Zero(n);
    std::vector<int> free;
    for(int j=0;j<n;++j)if(!stuck[j])free.push_back(j);
    if(static_cast<Eigen::Index>(free.size())==n) {
      const Eigen::VectorXd current=c.current_tau>0?Eigen::VectorXd(x.tail(n)):target(x);
      acceleration=dynamics.forward(x.head(n),x.segment(n,n),c.gain.cwiseProduct(current)+dynamics.friction(x.segment(n,n))-dry_friction(x.segment(n,n)));
      return {acceleration,Eigen::VectorXd::Zero(n)};
    }
    const auto r=residual(x);
    if(free.empty())return {acceleration,r};
    const auto mass=dynamics.mass_matrix(x.head(n));Eigen::MatrixXd block(free.size(),free.size());Eigen::VectorXd load(free.size());
    for(std::size_t i=0;i<free.size();++i) {
      load[i]=r[free[i]];
      for(std::size_t j=0;j<free.size();++j)block(i,j)=mass(free[i],free[j]);
    }
    const Eigen::VectorXd answer=block.ldlt().solve(load);
    for(std::size_t i=0;i<free.size();++i)acceleration[free[i]]=answer[i];
    return {acceleration,r-mass*acceleration};
  }
  void prepare_stiction() {
    for(int j=0;j<n;++j) {
      if(std::abs(state[n+j])<=c.stick_velocity_tolerance)stuck[j]=true;
      else {stuck[j]=false;slip_sign[j]=state[n+j]>0?1.:-1.;}
    }
    project_stopped_velocity(stuck);
    for(int j=0;j<n;++j)if(!stuck[j] && state[n+j]!=0.)slip_sign[j]=state[n+j]>0?1.:-1.;
    if(std::none_of(stuck.begin(),stuck.end(),[](bool value){return value;})) {all_stuck=false;return;}
    // Removing an over-limit constraint can change the other axes' reactions.
    bool resolved=false;
    for(int iteration=0;iteration<4*n+4;++iteration) {
      const auto solution=constrained(state);
      int release=-1;double violation=1e-12;
      for(int j=0;j<n;++j)if(stuck[j]) {
        const double excess=(std::abs(solution.second[j])-friction_parameters[j].static_friction)/std::max(1e-6,friction_parameters[j].static_friction);
        if(excess>violation) {release=j;violation=excess;}
      }
      if(release>=0) {
        stuck[release]=false;slip_sign[release]=solution.second[release]>0?1.:-1.;continue;
      }
      // Coupling may make a provisional zero-speed release accelerate against
      // its assumed slip direction. Reinsert that constraint and resolve.
      int reinsert=-1;
      for(int j=0;j<n;++j)if(!stuck[j] && state[n+j]==0. && solution.first[j]*slip_sign[j]<-1e-10) {reinsert=j;break;}
      if(reinsert>=0) {stuck[reinsert]=true;continue;}
      resolved=true;break;
    }
    if(!resolved)throw std::runtime_error("Stiction active set did not converge; refusing an inconsistent release");
    all_stuck=std::all_of(stuck.begin(),stuck.end(),[](bool value){return value;});
  }
  void rhs(const Eigen::VectorXd& x,Eigen::VectorXd& rate,double) {
    const Eigen::VectorXd current=c.current_tau>0?Eigen::VectorXd(x.tail(n)):target(x);
    rate.head(n)=x.segment(n,n);
    if(c.stiction_enabled) {
      if(all_stuck)rate.segment(n,n).setZero();
      else rate.segment(n,n)=constrained(x).first;
      for(int j=0;j<n;++j)if(stuck[j])rate[j]=0.;
    } else rate.segment(n,n)=dynamics.forward(x.head(n),x.segment(n,n),c.gain.cwiseProduct(current));
    if(c.current_tau>0) rate.tail(n)=(target(x)-x.tail(n))/c.current_tau;
    else rate.tail(n).setZero();
  }
  void sample(ArmSimulationResult& result) {
    if(uniform()>=c.drop_probability) {
      ArmSample sample{};
      if(has_last && uniform()<c.repeat_probability) {sample=last;for(Eigen::Index j=0;j<n;++j) sample.status[j]|=2;}
      else {
        sample.sequence=sequence;sample.sample_ns=t;sample.q=state.head(n);sample.dq=state.segment(n,n);sample.current=state.tail(n);
        sample.status=Eigen::VectorXi::Zero(n);const auto torque=request(state);
        for(Eigen::Index j=0;j<n;++j) {
          sample.q[j]+=c.position_noise*normal();sample.current[j]+=c.current_noise*normal();
          sample.status[j]=std::abs(torque[j])>c.torque_limit?1:0;
        }
        last=sample;has_last=true;
      }
      sample.receive_ns=t+c.feedback_delay_ns;result.observations.push_back(std::move(sample));
    }
    if(truth) {
      Eigen::VectorXd rate(state.size());rhs(state,rate,0.);
      result.truth.push_back({t,state.head(n),state.segment(n,n),rate.segment(n,n),state.tail(n),request(state)});
    }
    next_sample+=c.sample_period_ns;++sequence;
  }
};

ArmSimulationSession::ArmSimulationSession(ArmDynamics p,ArmSimulationConfig c,const Eigen::VectorXd& q,const Eigen::VectorXd& v,bool truth)
  :impl_(std::make_unique<Impl>(std::move(p),std::move(c),q,v,truth)) {}
ArmSimulationSession::~ArmSimulationSession()=default;
ArmSimulationSession::ArmSimulationSession(ArmSimulationSession&&) noexcept=default;
ArmSimulationSession& ArmSimulationSession::operator=(ArmSimulationSession&&) noexcept=default;
Nanoseconds ArmSimulationSession::time_ns() const {return impl_->t;}
void ArmSimulationSession::submit(const ArmCommand& command) {
  auto& s=*impl_;
  const auto valid=[&](const Eigen::VectorXd& x){return x.size()==s.n && x.allFinite();};
  if(command.send_ns<s.t || command.send_ns<=s.last_send ||
     command.send_ns>std::numeric_limits<Nanoseconds>::max()-s.c.command_delay_ns ||
     !valid(command.q) || !valid(command.dq) || !valid(command.torque) || !valid(command.kp) || !valid(command.kd) ||
     command.kp.minCoeff()<0 || command.kd.minCoeff()<0) throw std::invalid_argument("Invalid arm command batch or retroactive submission");
  s.pending.push_back(command);s.last_send=command.send_ns;
}
ArmSimulationResult ArmSimulationSession::advance_to(Nanoseconds end) {
  auto& s=*impl_;
  if(end<s.t || end>std::numeric_limits<Nanoseconds>::max()-std::max(s.c.sample_period_ns,s.c.feedback_delay_ns))
    throw std::invalid_argument("Invalid arm time advance");
  ArmSimulationResult result;
  const auto rhs=[&](const Eigen::VectorXd& x,Eigen::VectorXd& rate,double time){s.rhs(x,rate,time);};
  while(s.t<=end) {
    while(!s.pending.empty() && s.pending.front().send_ns<=s.t-s.c.command_delay_ns) {s.held=s.pending.front();s.pending.pop_front();}
    if(s.c.current_tau==0) s.state.tail(s.n)=s.target(s.state);
    if(s.c.stiction_enabled)s.prepare_stiction();
    if(s.t==s.next_sample) s.sample(result);
    if(s.t==end) break;
    Nanoseconds event=std::min(end,s.next_sample);
    if(!s.pending.empty() && s.pending.front().send_ns<=end-s.c.command_delay_ns)
      event=std::min(event,s.pending.front().send_ns+s.c.command_delay_ns);
    // An entirely stuck mechanism has an exact constant configuration. With
    // held commands the current follows one common exponential, so every
    // reaction is monotone between the two endpoints. If both are within the
    // static limits, advance exactly to the next event without RK substeps.
    if(s.c.stiction_enabled && s.all_stuck) {
      const Eigen::VectorXd target=s.target(s.state);
      const Eigen::VectorXd current=s.c.current_tau>0?
        Eigen::VectorXd(target+(s.state.tail(s.n)-target)*std::exp(-(event-s.t)*1e-9/s.c.current_tau)):target;
      const Eigen::VectorXd reaction=s.c.gain.cwiseProduct(current)-s.dynamics.gravity(s.state.head(s.n));
      bool within=true;
      for(int j=0;j<s.n;++j)within=within && std::abs(reaction[j])<=s.friction_parameters[j].static_friction+1e-12;
      if(within) {s.state.tail(s.n)=current;s.t=event;continue;}
    }
    const Nanoseconds next=s.t+std::min(s.c.integration_ns,event-s.t);
    const Eigen::VectorXd previous_velocity=s.state.segment(s.n,s.n);
    s.integrator.step(rhs,s.state,s.t,next);
    if(s.c.stiction_enabled) {
      auto stopped=s.stuck;
      for(int j=0;j<s.n;++j)if(!s.stuck[j] && previous_velocity[j]*s.state[s.n+j]<0.)stopped[j]=true;
      s.project_stopped_velocity(stopped);
    }
    if(!s.state.allFinite() || s.state.cwiseAbs().maxCoeff()>1e8) throw std::runtime_error("Unstable arm simulation");
    s.t=next;
  }
  return result;
}

ArmSimulationRunner::ArmSimulationRunner(ArmDynamics p,ArmSimulationConfig c)
  :plant_(std::move(p)),config_(validated(plant_,std::move(c))) {}
ArmSimulationResult ArmSimulationRunner::run(const std::vector<ArmCommand>& commands,Nanoseconds duration,
                                             const Eigen::VectorXd& q,const Eigen::VectorXd& v) const {
  if(duration<=0) throw std::invalid_argument("Invalid arm initial state/duration");
  ArmSimulationSession session(plant_,config_,q,v,true);
  for(const auto& command:commands) {
    if(command.send_ns>=duration) throw std::invalid_argument("Invalid arm command batch");
    session.submit(command);
  }
  return session.advance_to(duration);
}
}
