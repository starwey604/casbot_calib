#include "casbot_calib/integration.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace casbot;
namespace {
void check(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
double exponential_error(std::int64_t step_ns) {
  Eigen::VectorXd state=Eigen::VectorXd::Ones(1);
  FixedStepIntegrator<Eigen::VectorXd> integrator(state,step_ns);
  const auto rhs=[](const Eigen::VectorXd& x,Eigen::VectorXd& rate,double) {rate=x;};
  for(std::int64_t t=0;t<1000000000;t+=step_ns) integrator.step(rhs,state,t,t+step_ns);
  return std::abs(state[0]-std::exp(1.));
}
}
int main() {
  const double coarse=exponential_error(100000000),medium=exponential_error(50000000),fine=exponential_error(25000000);
  check(coarse/medium>14. && coarse/medium<18. && medium/fine>14. && medium/fine<18.,"fourth-order global convergence");
  check(fine<1e-8,"independent exponential response");

  // Fixed Eigen vectors use the same Boost adapter as dynamic whole-arm states.
  Eigen::Vector2d oscillator(1.,0.);FixedStepIntegrator<Eigen::Vector2d> fixed(oscillator,100000);
  const auto harmonic=[](const Eigen::Vector2d& x,Eigen::Vector2d& rate,double) {rate<<x[1],-4.*x[0];};
  for(std::int64_t t=0;t<1000000000;t+=100000) fixed.step(harmonic,oscillator,t,t+100000);
  check((oscillator-Eigen::Vector2d(std::cos(2.),-2.*std::sin(2.))).norm()<1e-12,"independent oscillator response");

  // Time-dependent RHS: the scheduler supplies an absolute start and a truncated interval.
  Eigen::VectorXd state=Eigen::VectorXd::Zero(1);FixedStepIntegrator<Eigen::VectorXd> dynamic(state,50000);
  std::vector<double> stage_times;
  const auto timed=[&](const Eigen::VectorXd&,Eigen::VectorXd& rate,double seconds) {
    rate[0]=3.*seconds*seconds;stage_times.push_back(seconds);
  };
  const std::int64_t begin=123456789,end=123500003;dynamic.step(timed,state,begin,end);
  check(std::abs(state[0]-(std::pow(end*1e-9,3)-std::pow(begin*1e-9,3)))<1e-17,"time-dependent truncated step");
  check(stage_times.size()==4,"RK4 stage count");
  const double t=begin*1e-9,dt=(end-begin)*1e-9;
  check(std::abs(stage_times[0]-t)<1e-16 && std::abs(stage_times[1]-(t+dt/2))<1e-16 &&
        std::abs(stage_times[2]-(t+dt/2))<1e-16 && std::abs(stage_times[3]-(t+dt))<1e-16,"RK stage times");

  int rejected=0;
  for(const auto interval:std::vector<std::pair<std::int64_t,std::int64_t>>{{-1,1},{1,1},{2,1},{0,50001}}) {
    try {dynamic.step(timed,state,interval.first,interval.second);}catch(const std::invalid_argument&){++rejected;}
  }
  state.resize(2);
  try {dynamic.step(timed,state,0,1);}catch(const std::invalid_argument&){++rejected;}
  try {FixedStepIntegrator<Eigen::VectorXd> invalid(Eigen::VectorXd(),1000);}catch(const std::invalid_argument&){++rejected;}
  try {FixedStepIntegrator<Eigen::Vector2d> invalid(oscillator,0);}catch(const std::invalid_argument&){++rejected;}
  check(rejected==7 && stage_times.size()==4,"reject invalid intervals/dimensions before calling RHS");
  check(integration_backend().find("boost.odeint.rk4/")==0,"backend identity");
  std::cout<<integration_backend()<<": analytic solutions, stage times, fourth order and boundaries passed\n";
}
