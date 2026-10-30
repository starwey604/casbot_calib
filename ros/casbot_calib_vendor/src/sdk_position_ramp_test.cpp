#include "sdk_position_ramp.hpp"
#include <cmath>
#include <iostream>
int main() {
  SdkPositionRamp ramp(2000000);SdkPositionRamp::Values q{},one{};one.fill(1.);
  ramp.reset(q,0);ramp.target(one,0);
  if(std::abs(ramp.at(1000000)[0]-.5)>1e-12)return 1;
  // Retarget from the current reference, never jump to the previous endpoint.
  q.fill(-1.);ramp.target(q,1000000);
  if(std::abs(ramp.at(1000000)[0]-.5)>1e-12)return 2;
  if(std::abs(ramp.at(2000000)[0]+.25)>1e-12)return 3;
  if(ramp.at(9000000)[0]!=-1.)return 4;
  ramp.reset(one,10000000);if(ramp.at(10000000)[0]!=1.)return 5;
  bool rejected=false;try{SdkPositionRamp invalid(0);}catch(const std::invalid_argument&){rejected=true;}
  if(!rejected)return 6;
  std::cout<<"Causal position-only interpolation policy passed\n";
}
