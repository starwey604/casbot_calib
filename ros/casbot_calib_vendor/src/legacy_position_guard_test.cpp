#include "legacy_position_guard.hpp"
int main() {
  LegacyPositionGuard guard(.02,.03);
  std::array<double,14> q{};std::vector<double> available(29,0.),current(29,0.);
  if(!guard.check(q,available,current).empty())return 1;
  q[0]=.021;if(guard.check(q,available,current)!="tracking_envelope")return 2;
  q[0]=.01;current[1]=.021;if(guard.check(q,available,current)!="aged_tracking_envelope")return 3;
  current[1]=.01;if(!guard.check(q,available,current).empty())return 4;
}
