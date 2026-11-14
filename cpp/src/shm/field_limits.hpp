#pragma once
#include <casbot_calib/shm/arm_codec.hpp>
#include <vector>
#include <cmath>
namespace casbot::shm {
struct FieldLimit {
  int joint;
  const char* field;
  double value,threshold;
  const char* unit;
  bool end_control;
};
// Field H1 policy: observed positions warn, observed velocity ends control.
// Reference/input bounds remain independent and are checked before sending.
inline std::vector<FieldLimit> field_limits(const StatusFrame& state,
    const std::array<double,14>& lower,const std::array<double,14>& upper,double velocity) {
  std::vector<FieldLimit> result;
  for(int j=0;j<14;++j) {
    const auto v=arm_feedback(state,j);
    if(v.position<lower[j])result.push_back({j,"position_lower",v.position,lower[j],"rad",false});
    if(v.position>upper[j])result.push_back({j,"position_upper",v.position,upper[j],"rad",false});
    if(std::abs(v.velocity)>velocity)result.push_back({j,"velocity",v.velocity,velocity,"rad/s",true});
  }
  return result;
}
}
