#include <casbot_calib/shm/clock.hpp>
#include <iostream>
#include <stdexcept>
void check(bool ok){if(!ok)throw std::runtime_error("SHM source clock test failed");}
int main() {
  using namespace casbot::shm;
  SourceClock c;bool rejected=false;
  try{c.observation(1);}catch(const std::logic_error&){rejected=true;}
  check(rejected);
  // Device wall clock near epoch (1970) still maps to a positive monotonic time.
  auto p=c.accept(1000000000,200000000,1000000200);
  check(p.offset==-800000100&&p.segment==0&&!p.jumped);
  check(c.observation(200)==1000000100);
  check(c.observation(200)==c.observation(200)); // Same ms is not made unique.
  check(c.observation(199)<c.observation(200)); // Do not sort/mask reversal.
  p=c.accept(2000000000,1200000000,2000000200);
  check(!p.jumped&&p.segment==0);
  // A wall clock step, not a forged 1ms frame-counter timeline.
  p=c.accept(3000000000,5200000000,3000000200);
  check(p.jumped&&p.segment==1&&c.observation(5200)==3000000100);
  p=c.accept(4000000000,6200000000,4000000200);
  check(!p.jumped&&p.segment==1);
  rejected=false;
  try{c.observation(~std::uint64_t(0));}catch(const std::overflow_error&){rejected=true;}
  check(rejected);
  std::cout<<"Source milliseconds, epoch wall clock, bracket, repeats, reversal and clock segmentation passed\n";
}
