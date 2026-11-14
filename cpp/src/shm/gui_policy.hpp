#pragma once
#include "json_input.hpp"
#include <algorithm>
namespace casbot::shm {
struct GuiAxisPolicy {
  double input_lower,input_upper,input_velocity,effort,kp,kd;
  double observed_lower,observed_upper,observed_velocity;
  bool end_velocity;
};
struct GuiPolicy {
  std::array<GuiAxisPolicy,14> axes{};
  double lease_ms=1000,feedback_ms=2000,send_ms=2000,transition_ms=3000;
};
inline double policy_number(const Json& p,const char* key,double fallback){const auto n=p.get_child_optional(key);return n?finite_number(*n):fallback;}
inline GuiPolicy gui_policy(const Json& json,const Joints& lower,const Joints& upper,const Joints& velocity) {
  GuiPolicy result;
  result.lease_ms=policy_number(json,"lease_ms",1000);result.feedback_ms=policy_number(json,"feedback_ms",2000);
  result.send_ms=policy_number(json,"send_ms",2000);result.transition_ms=policy_number(json,"transition_ms",3000);
  if(result.lease_ms<200||result.lease_ms>10000||result.feedback_ms<20||result.feedback_ms>30000||result.send_ms<20||result.send_ms>30000||result.transition_ms<20||result.transition_ms>30000)throw std::runtime_error("communication_settings_range");
  const auto overrides=json.get_child_optional("axes");std::set<int> seen;
  if(overrides)for(const auto& row:*overrides){std::size_t n;const int id=std::stoi(row.first,&n);if(n!=row.first.size()||id<12||id>25||!seen.insert(id).second)throw std::runtime_error("axis_override_id");}
  for(int j=0;j<14;++j) {
    const auto axis=overrides?overrides->get_child_optional(std::to_string(j+12)):boost::optional<const Json&>{};
    auto value=[&](const char* key,double base){return axis?policy_number(*axis,key,policy_number(json,key,base)):policy_number(json,key,base);};
    auto& a=result.axes[j];a.input_lower=value("input_lower",lower[j]);a.input_upper=value("input_upper",upper[j]);
    a.input_velocity=value("input_velocity_limit",1.5);a.effort=value("effort_limit",10);a.kp=value("kp_max",100);a.kd=value("kd_max",10);
    a.observed_lower=value("observed_lower",lower[j]);a.observed_upper=value("observed_upper",upper[j]);a.observed_velocity=value("velocity_limit",1.5);
    a.end_velocity=json.get<bool>("end_velocity",false);if(axis&&axis->get_child_optional("end_velocity"))a.end_velocity=json_bool(axis->get_child("end_velocity"));
    if(a.input_lower<lower[j]||a.input_upper>upper[j]||a.input_lower>a.input_upper||a.input_velocity<=0||a.input_velocity>velocity[j]||a.effort<=0||a.effort>327||a.kp<0||a.kp>3276||a.kd<0||a.kd>3276||a.observed_lower>a.observed_upper||a.observed_velocity<=0)throw std::runtime_error("axis_settings_range_id_"+std::to_string(j+12));
  }
  return result;
}
}
