#include "soft_return.hpp"
#include <cmath>
#include <stdexcept>
#include <string>

namespace {
void check(bool ok,const char* message) {
  if(!ok)throw std::runtime_error(message);
}
}

int test_shm_soft_return_tests() {
  using namespace casbot::shm;
  JointVector lower{},upper{},speed{},start{},target{},via{};
  lower.fill(-2.);upper.fill(2.);speed.fill(1.5);
  start[0]=.3;via[0]=-.3;
  const auto via_return=SoftReturn::plan(start,{},{},target,via,lower,upper,speed);
  const double intermediate_time=2.1875*.6/.25;
  const auto intermediate=via_return.at(intermediate_time);
  check(std::abs(intermediate.position[0]-via[0])<1e-12&&std::abs(intermediate.velocity[0])<1e-12,
        "Via point must be reached at rest");
  check(via_return.at(via_return.total_duration_s()-.49).position==target&&
        via_return.at(via_return.total_duration_s()+1.).velocity==JointVector{},"Missing final hold");
  for(double t=0.;t<via_return.total_duration_s();t+=.001) {
    const auto sample=via_return.at(t);
    check(sample.position[0]>=-.3&&sample.position[0]<=.3&&std::abs(sample.velocity[0])<=.250000001,
          "Return joint bounds exceeded");
  }

  JointVector velocity{},acceleration{};
  velocity[0]=.2;acceleration[0]=.3;
  const auto braking=SoftReturn::plan(start,velocity,acceleration,target,std::nullopt,lower,upper,speed);
  const auto begin=braking.at(0.);
  check(begin.position==start&&begin.velocity==velocity,"Brake start mismatch");
  constexpr double dt=1e-5;
  check(std::abs((braking.at(dt).velocity[0]-velocity[0])/dt-acceleration[0])<1e-3,
        "Brake acceleration is discontinuous");
  const double brake_time=std::sqrt(.3/5.);
  const auto stopped=braking.at(brake_time);
  check(std::abs(stopped.velocity[0])<1e-12&&std::abs(stopped.position[0]-(.3+.1*brake_time+.3*.06/12.))<1e-12,
        "Brake does not stop continuously");

  const auto direct=SoftReturn::plan(start,{},{},target,std::nullopt,lower,upper,speed);
  const auto redundant=SoftReturn::plan(start,{},{},target,target,lower,upper,speed);
  check(direct.total_duration_s()==redundant.total_duration_s()&&direct.total_duration_s()<via_return.total_duration_s(),
        "Redundant via point adds movement");
  const auto hold=SoftReturn::plan(target,{},{},target,via,lower,upper,speed);
  check(hold.total_duration_s()==.5&&hold.at(.25).position==target,"Stationary start leaves its target");

  start[0]=upper[0];
  try {
    SoftReturn::plan(start,velocity,{},target,std::nullopt,lower,upper,speed);
  }catch(const std::runtime_error& error) {
    check(std::string(error.what())=="soft_stop_brake_limit","Unexpected brake limit error");
    return 0;
  }
  throw std::runtime_error("Accepted outward brake at position limit");
}
