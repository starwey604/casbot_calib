#pragma once
#include "json_input.hpp"
#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>

namespace casbot::shm {
struct TricalSegment {
  std::uint64_t id=0;
  std::int64_t duration_ns=0;
  Joints from{},to{};
  bool windup=false;
  std::string validate(const Joints& hold,const Joints& observed,const Joints& lower,
      const Joints& upper,const Joints& velocity,double max_windup) const {
    if(duration_ns<=0)return "positive_duration_required";
    for(int j=0;j<14;++j){
      if(!std::isfinite(from[j])||!std::isfinite(to[j]))return "nonfinite_target";
      if(std::abs(from[j]-hold[j])>1e-9)return "hold_start_mismatch";
      if(to[j]<lower[j]||to[j]>upper[j])return "target_outside_urdf";
      if(2.1875*std::abs(to[j]-from[j])/(duration_ns*1e-9)>velocity[j])return "reference_velocity_limit";
      if(windup&&std::abs(to[j]-observed[j])>max_windup)return "spring_windup_limit";
    }
    return {};
  }
  std::pair<Joints,Joints> at(std::int64_t elapsed) const {
    const double u=std::clamp(double(elapsed)/duration_ns,0.,1.);
    const double s=u*u*u*u*(35+u*(-84+u*(70-20*u)));
    const double v=140*u*u*u*(1-u)*(1-u)*(1-u)/(duration_ns*1e-9);
    Joints q{},dq{};
    for(int j=0;j<14;++j){q[j]=from[j]+s*(to[j]-from[j]);dq[j]=v*(to[j]-from[j]);}
    return {q,dq};
  }
};
struct TricalBlock {
  Joints sum{},low{},high{};
  std::uint64_t count=0;
  std::int64_t start_ns=0,end_ns=0;
  void add(std::int64_t time,const Joints& q){
    if(!count){start_ns=time;low=high=q;}
    end_ns=time;++count;
    for(int j=0;j<14;++j){sum[j]+=q[j];low[j]=std::min(low[j],q[j]);high[j]=std::max(high[j],q[j]);}
  }
  Joints mean() const {Joints q{};for(int j=0;j<14;++j)q[j]=count?sum[j]/count:0.;return q;}
};
int trical_main(int argc,char** argv);
}
