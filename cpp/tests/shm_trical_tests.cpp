#include "trical_runtime.hpp"
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <iostream>
int test_shm_trical_tests(){
  using namespace casbot::shm;
  Joints hold{},observed{},lower{},upper{},velocity{};lower.fill(-2);upper.fill(2);velocity.fill(1.5);
  TricalSegment s;s.duration_ns=1000000000;s.to[0]=.1;
  assert(s.validate(hold,observed,lower,upper,velocity,.012).empty());
  assert(s.at(0).first==hold&&s.at(s.duration_ns).first==s.to);
  assert(s.at(0).second==hold&&s.at(s.duration_ns).second==hold);
  assert(std::abs(s.at(500000000).second[0]-.21875)<1e-14);
  s.windup=true;assert(s.validate(hold,observed,lower,upper,velocity,.012)=="spring_windup_limit");
  observed[0]=.095;assert(s.validate(hold,observed,lower,upper,velocity,.012).empty());
  s.from[1]=1e-4;assert(s.validate(hold,observed,lower,upper,velocity,.012)=="hold_start_mismatch");s.from[1]=0;
  s.duration_ns=1000;assert(s.validate(hold,observed,lower,upper,velocity,.012)=="reference_velocity_limit");
  s.duration_ns=0;assert(s.validate(hold,observed,lower,upper,velocity,.012)=="positive_duration_required");
  s.duration_ns=1000000000;s.to[0]=3;assert(s.validate(hold,observed,lower,upper,velocity,.012)=="target_outside_urdf");
  TricalBlock b;Joints q{};q[1]=1.2;b.add(100,q);q[0]=.002;b.add(200,q);
  assert(b.count==2&&b.start_ns==100&&b.end_ns==200);
  assert(b.mean()[0]==.001&&b.low[0]==0&&b.high[0]==.002&&b.mean()[1]==1.2);
  std::cout<<"TriCal C3 segments, windup and non-overlapping feedback blocks passed\n";return 0;
}
