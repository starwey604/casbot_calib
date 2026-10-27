#pragma once
#include <Eigen/Core>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <boost/version.hpp>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace casbot {
inline std::string integration_backend() {
  return "boost.odeint.rk4/"+std::to_string(BOOST_VERSION/100000)+"."+
    std::to_string(BOOST_VERSION/100%1000)+"."+std::to_string(BOOST_VERSION%100);
}

// Single-owner integration workspace for a fixed-size or dynamic Eigen column vector.
// The caller owns integer-nanosecond scheduling and truncates at every discrete event.
// System has Odeint's signature (const State&, State& derivative, double time_seconds).
template<class State>
class FixedStepIntegrator {
  static_assert(std::is_same_v<typename State::Scalar,double>,"Integration requires double precision");
  static_assert(State::ColsAtCompileTime==1,"Integration requires an Eigen column vector");
 public:
  FixedStepIntegrator(const State& prototype,std::int64_t max_step_ns)
    :size_(prototype.size()),max_step_ns_(max_step_ns) {
    if(size_<=0 || max_step_ns_<=0) throw std::invalid_argument("Integrator requires a nonempty state and positive step limit");
    // Allocate dynamic RK scratch vectors once; dimension is fixed for this workspace.
    stepper_.adjust_size(prototype);
  }
  template<class System>
  void step(System&& system,State& state,std::int64_t begin_ns,std::int64_t end_ns) {
    if(begin_ns<0 || end_ns<=begin_ns || end_ns-begin_ns>max_step_ns_ || state.size()!=size_)
      throw std::invalid_argument("Invalid integer integration interval or changed state dimension");
    // Subtract integers before conversion. Never accumulate the scheduling clock in double.
    const double dt=static_cast<double>(end_ns-begin_ns)*1e-9;
    stepper_.do_step(std::ref(system),state,static_cast<double>(begin_ns)*1e-9,dt);
  }
 private:
  using Stepper=boost::numeric::odeint::runge_kutta4<State>;
  Stepper stepper_;
  Eigen::Index size_;
  std::int64_t max_step_ns_;
};
}  // namespace casbot
