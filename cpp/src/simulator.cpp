#include "casbot_calib/simulator.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

namespace casbot {
namespace {
using State = Eigen::Matrix<double, 8, 1>;
struct Input { double request, target_current, force; bool contact, clipped; };
double quantize(double x, double step) { return step > 0 ? std::round(x / step) * step : x; }
Input input(const SimulationConfig& c, const State& s, const JointCommand& cmd) {
  double request = cmd.torque_ff;
  if (cmd.mode == Mode::mixed_pd)
    request += cmd.kp * (cmd.q_ref - s[0]) + cmd.kd * (cmd.dq_ref - s[1]);
  const double limit = std::min(c.torque_limit, c.gain * c.current_limit);
  double current = std::clamp(request, -limit, limit) / c.gain;
  // Symmetric compliant catch/release: an inclined pick spans +/- depth in
  // relative transverse displacement. Potential vanishes at entry/release.
  double relative = (c.lever * s[0] + c.pick_offset) * std::cos(c.attack_angle) - s[5];
  bool active = c.contact_enabled && std::abs(relative) < c.depth;
  double force = 0;
  if (active) {
    const double u = relative / c.depth;
    const double rel_v = c.lever * s[1] * std::cos(c.attack_angle) - s[6];
    // U=k*d^2/(2*pi^2)*(1+cos(pi*r/d)); F=-dU/dr.
    force = c.contact_spring * c.depth / (2 * std::acos(-1.0)) * std::sin(std::acos(-1.0) * u)
          - c.contact_damping * std::pow(1 - u*u, 2) * rel_v;
  }
  return {request, current, force, active, std::abs(request) > limit};
}
State derivative(const SimulationConfig& c, const State& s, const JointCommand& cmd) {
  auto u = input(c, s, cmd);
  State d = State::Zero();
  const double i = c.current_tau > 0 ? s[2] : u.target_current;
  const double friction = c.damping*s[1] + c.coulomb*std::tanh(s[1]/c.friction_velocity);
  const double load = friction + c.spring*(s[0]-c.spring_zero);
  const double contact_torque = c.lever * std::cos(c.attack_angle) * u.force;
  d[0] = s[1];
  if (c.double_inertia) {
    const double coupling = c.coupling_spring*(s[3]-s[0]) + c.coupling_damping*(s[4]-s[1]);
    d[1] = (coupling-load+contact_torque)/c.inertia;
    d[3] = s[4];
    d[4] = (c.gain*i-coupling)/c.motor_inertia;
  } else d[1] = (c.gain*i-load+contact_torque)/c.inertia;
  if (c.current_tau > 0) d[2] = (u.target_current-s[2])/c.current_tau;
  d[5] = s[6];
  d[6] = (-c.string_spring*s[5]-c.string_damping*s[6]-u.force)/c.string_mass;
  if (c.current_filter_tau > 0) d[7] = (i-s[7])/c.current_filter_tau;
  return d;
}
double energy(const SimulationConfig& c, const State& s) {
  double result = .5*c.inertia*s[1]*s[1] + .5*c.spring*std::pow(s[0]-c.spring_zero,2)
    + .5*c.string_mass*s[6]*s[6] + .5*c.string_spring*s[5]*s[5];
  if(c.double_inertia) result += .5*c.motor_inertia*s[4]*s[4]+.5*c.coupling_spring*std::pow(s[3]-s[0],2);
  double r = (c.lever*s[0]+c.pick_offset)*std::cos(c.attack_angle)-s[5];
  if(c.contact_enabled && std::abs(r)<c.depth)
    result += c.contact_spring*c.depth*c.depth/(2*std::pow(std::acos(-1.0),2))*(1+std::cos(std::acos(-1.0)*r/c.depth));
  return result;
}
}
SimulationRunner::SimulationRunner(SimulationConfig config) : config_(config) {
  const auto& c = config_;
  for(double x : {c.inertia,c.gain,c.friction_velocity,c.motor_inertia,c.string_mass,c.lever,c.depth,c.torque_limit,c.current_limit})
    if(!std::isfinite(x) || x<=0) throw std::invalid_argument("Mass, gain, limits and geometric scales must be positive finite");
  for(double x : {c.damping,c.coulomb,c.spring,c.current_tau,c.current_filter_tau,c.string_spring,c.string_damping,c.contact_spring,c.contact_damping,c.coupling_spring,c.coupling_damping,c.position_noise,c.velocity_noise,c.current_noise,c.position_quantum,c.current_quantum})
    if(!std::isfinite(x) || x<0) throw std::invalid_argument("Damping, stiffness, noise and time constants must be nonnegative finite");
  if(c.integration_ns<=0 || c.sample_period_ns<=0 || c.command_delay_ns<0 || c.feedback_delay_ns<0 || c.jitter_ns<0 || c.current_lag_ns<0)
    throw std::invalid_argument("Invalid nanosecond schedule");
  double step = c.integration_ns*1e-9;
  if((c.current_tau>0 && step>c.current_tau/2) || (c.current_filter_tau>0 && step>c.current_filter_tau/2))
    throw std::invalid_argument("Integration step too large for current time constant");
  if(c.drop_probability<0 || c.drop_probability>1 || c.repeat_probability<0 || c.repeat_probability>1)
    throw std::invalid_argument("Invalid packet probability");
}
SimulationResult SimulationRunner::run(MockJointDevice& device, Nanoseconds duration_ns) const {
  if(!device.samples_.empty()) throw std::logic_error("Drain previous batch feedback before starting a new mock experiment");
  auto result=run(device.commands_,duration_ns);
  device.commands_.clear();
  device.samples_.assign(result.observations.begin(),result.observations.end());
  return result;
}
SimulationResult SimulationRunner::run(const std::vector<JointCommand>& commands, Nanoseconds duration_ns) const {
  const auto& c=config_;
  if(duration_ns<=0) throw std::invalid_argument("Duration must be positive");
  std::mt19937 rng(c.seed);
  // mt19937 integer output is specified across platforms; avoid library-specific normal_distribution.
  auto uniform = [&]() { return (static_cast<double>(rng())+.5)/4294967296.; };
  auto normal = [&]() { double a=uniform(),b=uniform(); return std::sqrt(-2*std::log(a))*std::cos(2*std::acos(-1.0)*b); };
  auto jitter = [&]() -> Nanoseconds { return static_cast<Nanoseconds>((2*uniform()-1)*static_cast<double>(c.jitter_ns)); };
  struct Event { Nanoseconds time, order; JointCommand command; };
  std::vector<Event> events;
  Nanoseconds previous=-1;
  for(std::size_t n=0;n<commands.size();++n) {
    const auto& cmd=commands[n];
    if(cmd.send_ns<0 || cmd.send_ns<=previous || cmd.send_ns>=duration_ns || cmd.kp<0 || cmd.kd<0)
      throw std::invalid_argument("Commands must have increasing in-range times and nonnegative PD gains");
    for(double x : {cmd.q_ref,cmd.dq_ref,cmd.torque_ff,cmd.kp,cmd.kd})
      if(!std::isfinite(x)) throw std::invalid_argument("Non-finite command");
    previous=cmd.send_ns;
    // Packet loss here is feedback-only; command loss is deliberately not conflated.
    events.push_back({std::max(cmd.send_ns,cmd.send_ns+c.command_delay_ns+jitter()),static_cast<Nanoseconds>(n),cmd});
  }
  std::stable_sort(events.begin(),events.end(),[](const auto& a,const auto& b){return a.time<b.time;});
  State s=State::Zero(); s[0]=c.initial_q;s[1]=c.initial_v;s[3]=c.initial_q;s[4]=c.initial_v;
  s[5]=c.initial_string_x;s[6]=c.initial_string_v;
  JointCommand held;
  SimulationResult result;
  std::deque<std::pair<Nanoseconds,double>> history{{0,0}};
  Nanoseconds t=0, next_sample=0,last_order=-1,seq=0;
  std::size_t event_index=0;
  JointSample last; bool has_last=false;
  while(t<=duration_ns) {
    while(event_index<events.size() && events[event_index].time<=t) {
      const auto& event=events[event_index++];
      if(event.order>last_order) {held=event.command;last_order=event.order;}
    }
    auto u=input(c,s,held);
    if(c.current_tau==0) s[2]=u.target_current;
    if(c.current_filter_tau==0) s[7]=s[2];
    if(history.back().first==t) history.back().second=s[7];
    else history.emplace_back(t,s[7]);
    if(t==next_sample) {
      const Nanoseconds target=t-c.current_lag_ns;
      while(history.size()>2 && history[1].first<target) history.pop_front();
      double sensed_i=history.front().second;
      if(target>=history.back().first) sensed_i=history.back().second;
      else if(target>=history.front().first) {
        auto it=std::lower_bound(history.begin(),history.end(),target,[](const auto& x,Nanoseconds y){return x.first<y;});
        if(it==history.begin()) sensed_i=it->second;
        else {auto lo=it-1;double f=double(target-lo->first)/double(it->first-lo->first);sensed_i=lo->second+f*(it->second-lo->second);}
      }
      Nanoseconds arrival=std::max(t,t+c.feedback_delay_ns+jitter());
      if(uniform()>=c.drop_probability) {
        JointSample sample;
        if(has_last && uniform()<c.repeat_probability) {sample=last; sample.status|=SampleStatus::repeated;}
        else {
          sample.sequence=seq;
          sample.sample_ns=c.expose_sample_time?t:-1;
          sample.current_sample_ns=c.expose_sample_time?target:-1;
          sample.position=quantize(s[0]+c.position_noise*normal(),c.position_quantum);
          sample.velocity=s[1]+c.velocity_noise*normal();
          sample.current=quantize(sensed_i+c.current_noise*normal(),c.current_quantum);
          sample.estimated_torque=c.gain*sample.current;
          sample.temperature=c.temperature;
          sample.status=u.clipped?static_cast<std::uint32_t>(SampleStatus::saturated):0U;
          last=sample;has_last=true;
        }
        sample.receive_ns=arrival;
        result.observations.push_back(sample);
      }
      auto d=derivative(c,s,held);
      result.truth.push_back({t,s[0],s[1],d[1],s[2],u.request,u.force,s[5],s[6],energy(c,s),s[3],u.contact});
      ++seq;next_sample+=c.sample_period_ns;
    }
    if(t==duration_ns) break;
    Nanoseconds next=std::min({t+c.integration_ns,next_sample,duration_ns});
    if(event_index<events.size()) next=std::min(next,events[event_index].time);
    double dt=(next-t)*1e-9;
    auto a=derivative(c,s,held);
    auto b=derivative(c,(s+dt*.5*a).eval(),held);
    auto d=derivative(c,(s+dt*.5*b).eval(),held);
    auto e=derivative(c,(s+dt*d).eval(),held);
    s+=dt/6*(a+2*b+2*d+e);
    if(!s.allFinite() || s.cwiseAbs().maxCoeff()>1e8) throw std::runtime_error("Unstable simulation");
    t=next;
  }
  std::stable_sort(result.observations.begin(),result.observations.end(),[](const auto& a,const auto& b){return a.receive_ns<b.receive_ns;});
  return result;
}
}  // namespace casbot
