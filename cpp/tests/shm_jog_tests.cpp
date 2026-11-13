#include <casbot_calib/shm/arm_jog.hpp>
#include <iostream>
#include <stdexcept>
void check(bool v) { if(!v) throw std::runtime_error("Arm jog test failed"); }
int main() {
  using namespace casbot::shm;
  check(jog_within_limits(0,-.5,.5,.5));
  check(!jog_within_limits(.01,-.5,.5,.5));
  check(!jog_within_limits(-.01,-.5,.5,.5));
  check(jog_at(0,5,.5).position==0 && jog_at(5,5,.5).position==-.5);
  check(jog_at(10,5,.5).position==.5 && jog_at(15,5,.5).position==0);
  for(double t: {0.,5.,10.,15.}) check(jog_at(t,5,.5).velocity==0);
  for(int i=0;i<=15000;++i) {
    const auto a=jog_at(i*.001,5,.5);
    check(std::abs(a.position)<=.5000000001 && std::abs(a.velocity)<=.437500001);
  }
  CommandFrame c{}; StatusFrame s{};
  for(int j=0;j<14;++j) {
    arm_target(c,j,.3,-.02,20,2);
    s.joints[j+12].position=c.joints[j+12].position;
    check(std::abs(arm_position(s,j)-.3)<1.1e-6);
    check(c.joints[j+12].kp==200 && c.joints[j+12].kd==20 && c.joints[j+12].torque==0);
  }
  bool failed=false;
  try { arm_target(c,0,1e10,0,20,2); } catch(const std::invalid_argument&) { failed=true; }
  check(failed);
  std::cout<<"arm jog: signed offsets, integer scale, both limits and smooth waveform passed\n";
}
