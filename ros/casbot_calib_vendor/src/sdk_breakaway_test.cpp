#include "sdk_breakaway.hpp"
#include <iostream>
SdkBreakawayConfig config(){SdkBreakawayConfig c;c.free_begin=0;c.ramp_begin=300000000;c.ramp_end=1000000000;c.return_begin=1100000000;c.return_end=1600000000;return c;}
int main(){
  SdkBreakaway p(config());SdkBreakaway::Values q{},v{},observed{};double increment;bool zero;
  for(std::int64_t t=0;t<=300000000;t+=10000000){p.observe(observed,v,t,t);q.fill(0);if(!p.command(q,v,observed,t,increment,zero))return 1;}
  observed[0]=.0003;v[0]=.006;p.observe(observed,v,330000000,331000000);p.observe(observed,v,345000000,346000000);
  q.fill(0);v.fill(0);if(!p.command(q,v,observed,350000000,increment,zero)||zero||increment!=0||!p.confirmed())return 2;
  if(std::abs(q[0]-.0003)>1e-12)return 3;
  q.fill(0);if(!p.command(q,v,observed,1600000000,increment,zero)||q[0]!=0)return 4;
  SdkBreakaway wrong(config());observed.fill(0);v.fill(0);
  for(std::int64_t t=0;t<=300000000;t+=10000000){wrong.observe(observed,v,t,t);q.fill(0);wrong.command(q,v,observed,t,increment,zero);}
  observed[0]=-.0003;v[0]=-.006;wrong.observe(observed,v,330000000,331000000);
  if(wrong.command(q,v,observed,340000000,increment,zero)||wrong.fault()!="ramp_wrong_direction")return 5;
  SdkBreakaway unstable(config());observed.fill(0);v.fill(.01);
  for(std::int64_t t=0;t<=300000000;t+=10000000){unstable.observe(observed,v,t,t);q.fill(0);unstable.command(q,v,observed,t,increment,zero);}
  if(unstable.fault()!="zero_pd_dwell_not_stationary")return 6;
  std::cout<<"SDK callback onset, ramp removal, recovery and fault counterexamples passed\n";
}
