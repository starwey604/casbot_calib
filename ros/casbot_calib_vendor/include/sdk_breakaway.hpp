#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <string>
#include <stdexcept>

struct SdkBreakawayConfig {
  int joint=0,direction=1;
  std::int64_t free_begin=0,ramp_begin=0,ramp_end=0,return_begin=0,return_end=0,sustain=10000000;
  double slope=.12,maximum_increment=.5,displacement=.0002,speed=.005,maximum_departure=.006,
    maximum_speed=.1,still_speed=.0005,other_allowance=.003;
};

// SDK callback-time evidence, deliberately distinct from acquisition-time guards.
// No device sequence or acquisition timestamp is manufactured from callbacks.
class SdkBreakaway {
 public:
  using Values=std::array<double,14>;
  explicit SdkBreakaway(SdkBreakawayConfig c):c_(c) {
    for(auto value:{c.slope,c.maximum_increment,c.displacement,c.speed,c.maximum_departure,c.maximum_speed,c.still_speed,c.other_allowance})
      if(!std::isfinite(value))throw std::invalid_argument("Finite SDK ramp policy required");
    if(c.joint<0||c.joint>=14||(c.direction!=1&&c.direction!=-1)||c.free_begin<0||c.ramp_begin<=c.free_begin||
       c.ramp_end<=c.ramp_begin||c.return_begin<c.ramp_end||c.return_end<=c.return_begin||c.sustain<0||
       c.slope<=0||c.maximum_increment<=0||c.displacement<=0||c.speed<=0||c.maximum_departure<=c.displacement||
       c.maximum_speed<=c.speed||c.still_speed<=0||c.other_allowance<=0)throw std::invalid_argument("Invalid SDK ramp policy");
  }
  void observe(const Values& q,const Values& v,std::int64_t available,std::int64_t decision) {
    if(available<c_.free_begin||available>=c_.return_begin)return;
    if(available<c_.ramp_begin) {
      if(std::abs(v[c_.joint])<c_.still_speed){if(still_since_<0)still_since_=available;}
      else still_since_=-1;
      return;
    }
    if(!anchored_)return;
    const double distance=c_.direction*(q[c_.joint]-baseline_),speed=c_.direction*v[c_.joint];
    if(std::abs(distance)>=c_.maximum_departure||std::abs(speed)>=c_.maximum_speed){fault_="ramp_motion_limit";return;}
    if(confirmed_>=0)return;
    if(distance< -c_.displacement||speed< -c_.speed){fault_="ramp_wrong_direction";return;}
    if(distance>=c_.displacement&&speed>=c_.speed) {
      if(candidate_<0)candidate_=available;
      if(available-candidate_>=c_.sustain){confirmed_=available;decision_=decision;offset_=q[c_.joint]-reference_;}
    } else candidate_=-1;
  }
  bool command(Values& q,Values& v,const Values& observed,std::int64_t now,double& increment,bool& zero_pd) {
    increment=0.;zero_pd=false;
    if(now<c_.free_begin)return true;
    if(!anchored_){baseline_=observed[c_.joint];reference_=q[c_.joint];anchored_=true;}
    if(now<c_.return_begin) {
      for(int j=0;j<14;++j)if(j!=c_.joint&&std::abs(observed[j]-q[j])>c_.other_allowance){fault_="ramp_other_joint_hold_limit";return false;}
    }
    if(!fault_.empty())return false;
    if(now<c_.ramp_begin){zero_pd=true;return true;}
    if(!ramp_started_) {
      if(still_since_<0||c_.ramp_begin-still_since_<200000000){fault_="zero_pd_dwell_not_stationary";return false;}
      baseline_=observed[c_.joint];ramp_started_=true;first_command_=now;
    }
    if(confirmed_<0) {
      if(now>=c_.ramp_end){fault_="breakaway_not_observed";return false;}
      increment=c_.direction*c_.slope*double(now-c_.ramp_begin)*1e-9;
      if(std::abs(increment)>c_.maximum_increment){fault_="ramp_effort_limit_without_onset";return false;}
      zero_pd=true;return true;
    }
    double fade=1.,derivative=0.;
    if(now>=c_.return_end)fade=0.;
    else if(now>c_.return_begin) {
      const double duration=double(c_.return_end-c_.return_begin)*1e-9;
      const double u=double(now-c_.return_begin)/double(c_.return_end-c_.return_begin);
      fade=1.-(35*std::pow(u,4)-84*std::pow(u,5)+70*std::pow(u,6)-20*std::pow(u,7));
      derivative=-(140*std::pow(u,3)-420*std::pow(u,4)+420*std::pow(u,5)-140*std::pow(u,6))/duration;
    }
    q[c_.joint]+=offset_*fade;v[c_.joint]+=offset_*derivative;
    return true;
  }
  const std::string& fault() const{return fault_;}
  bool confirmed() const{return confirmed_>=0;}
  void write(const std::string& path) const {
    std::ofstream out(path);out<<std::setprecision(17)<<"{\"confirmed\":"<<(confirmed()?"true":"false")
      <<",\"fault\":"<<std::quoted(fault_)<<",\"joint\":"<<c_.joint<<",\"direction\":"<<c_.direction
      <<",\"first_ramp_command_elapsed_ns\":"<<first_command_<<",\"confirmed_callback_elapsed_ns\":"<<confirmed_
      <<",\"decision_elapsed_ns\":"<<decision_<<",\"baseline_q_rad\":"<<baseline_<<",\"recovery_offset_rad\":"<<offset_
      <<",\"time_basis\":\"client callback availability, not acquisition\",\"sample_time_known\":false,\"device_sequence_known\":false}\n";
  }
 private:
  SdkBreakawayConfig c_;std::string fault_;
  std::int64_t still_since_=-1,candidate_=-1,confirmed_=-1,decision_=-1,first_command_=-1;
  bool anchored_=false,ramp_started_=false;double baseline_=0.,reference_=0.,offset_=0.;
};
