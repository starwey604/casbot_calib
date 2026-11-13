#include <casbot_calib/shm/arm_jog.hpp>
#include <iostream>
#include <stdexcept>
void check(bool v) { if(!v) throw std::runtime_error("Arm jog test failed"); }
int main() {
  using namespace casbot::shm;
  for(unsigned word: {0x21,0x31,0x231}) {
    check(drive_state_is(word,0x21));
    check(!drive_state_is(word,0x27));
  }
  for(unsigned word: {0x27,0x37,0x237}) {
    check(drive_state_is(word,0x27));
    check(!drive_state_is(word,0x21));
  }
  for(unsigned word: {0x0,0x8,0xf,0x40,0x239,0x23f,0x207}) {
    check(!drive_state_is(word,0x21) && !drive_state_is(word,0x27));
  }
  check(arm_index(12)==0 && arm_index(25)==13);
  for(int id: {1,11,26}) {
    bool rejected=false;
    try { (void)arm_index(id); } catch(const std::out_of_range&) { rejected=true; }
    check(rejected);
  }
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
  for(int j=0;j<14;++j) {
    arm_target(c,j,-.2,.1234567,0,0,.129);
    const auto value=arm_command(c,j);
    check(std::abs(value.position+.2)<1.1e-6&&std::abs(value.velocity-.1234567)<1.1e-6);
    check(std::abs(value.effort-.12)<1e-12&&value.kp==0&&value.kd==0);
    arm_status(s,j,-.2,.1234567,.129);const auto observed=arm_feedback(s,j);
    check(std::abs(observed.position-value.position)<1e-12&&std::abs(observed.effort-.12)<1e-12);
    arm_target(c,j,0,0,20,2,.009);check(c.joints[j+12].torque==0);
  }
  for(double bad:{std::numeric_limits<double>::infinity(),-400.,400.}) {
    bool refused=false;try{arm_target(c,0,0,0,0,0,bad);}catch(const std::invalid_argument&){refused=true;}check(refused);
  }
  bool failed=false;
  try { arm_target(c,0,1e10,0,20,2); } catch(const std::invalid_argument&) { failed=true; }
  check(failed);
  std::cout<<"arm jog: signed offsets, integer scale, both limits and smooth waveform passed\n";
}
