#pragma once

#include "reference.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace casbot::shm {

struct SoftReturnSample {
  JointVector position{},velocity{};
};

class SoftReturn {
 public:
  static SoftReturn plan(const JointVector& position,const JointVector& velocity,const JointVector& acceleration,
                         const JointVector& target,const std::optional<JointVector>& via,
                         const JointVector& lower,const JointVector& upper,const JointVector& speed) {
    constexpr double brake_acc=1.,brake_jerk=5.,default_brake=.2,min_brake=.05,max_brake=1.5;
    constexpr double motion_epsilon=1e-4,via_epsilon=.02;
    SoftReturn result;
    result.start_={position,velocity};
    result.acceleration_=acceleration;
    result.target_=target;
    double max_velocity=0.,max_acceleration=0.;
    for(int j=0;j<14;++j) {
      const auto valid_position=[&](double q){return q>=lower[j]&&q<=upper[j];};
      if(!valid_position(position[j])||!valid_position(target[j])||(via&&!valid_position((*via)[j]))||
         !std::isfinite(velocity[j])||!std::isfinite(acceleration[j])||!std::isfinite(speed[j])||speed[j]<=0.)
        throw std::runtime_error("soft_stop_return_limit");
      max_velocity=std::max(max_velocity,std::abs(velocity[j]));
      max_acceleration=std::max(max_acceleration,std::abs(acceleration[j]));
    }

    JointVector stopped=position;
    if(max_velocity>motion_epsilon||max_acceleration>motion_epsilon) {
      result.brake_s_=std::clamp(std::max(max_velocity/brake_acc,std::sqrt(max_acceleration/brake_jerk)),
                                 default_brake,max_brake);
      while(!result.brake_within_limits(lower,upper,speed)) {
        if(result.brake_s_<=min_brake)throw std::runtime_error("soft_stop_brake_limit");
        result.brake_s_=std::max(min_brake,result.brake_s_*.5);
      }
      stopped=result.brake_at(1.).position;
    }

    if(via&&distance(stopped,*via)>via_epsilon&&distance(*via,target)>via_epsilon&&
       distance(stopped,target)>via_epsilon) {
      result.add_leg(stopped,*via,speed);
      stopped=*via;
    }
    if(distance(stopped,target)>motion_epsilon)result.add_leg(stopped,target,speed);
    return result;
  }

  double total_duration_s() const {
    return brake_s_+legs_[0].duration+legs_[1].duration+.5;
  }

  SoftReturnSample at(double seconds) const {
    double remaining=std::clamp(seconds,0.,total_duration_s());
    if(remaining==0.)return start_;
    if(remaining<brake_s_)return brake_at(remaining/brake_s_);
    remaining-=brake_s_;
    for(const auto& leg:legs_) {
      if(remaining<leg.duration) {
        const double u=remaining/leg.duration,w=1.-u;
        const double f=u*u*u*u*(35.+u*(-84.+u*(70.-20.*u)));
        const double df=140.*u*u*u*w*w*w/leg.duration;
        SoftReturnSample sample;
        for(int j=0;j<14;++j) {
          sample.position[j]=std::clamp(leg.from[j]+leg.delta[j]*f,
                                        std::min(leg.from[j],leg.to[j]),std::max(leg.from[j],leg.to[j]));
          sample.velocity[j]=leg.delta[j]*df;
        }
        return sample;
      }
      remaining-=leg.duration;
    }
    return {target_,{}};
  }

 private:
  struct Leg {JointVector from{},to{},delta{};double duration=0.;};
  SoftReturnSample start_;
  JointVector acceleration_{},target_{};
  double brake_s_=0.;
  std::array<Leg,2> legs_{};

  static double distance(const JointVector& a,const JointVector& b) {
    double result=0.;
    for(int j=0;j<14;++j)result=std::max(result,std::abs(a[j]-b[j]));
    return result;
  }

  void add_leg(const JointVector& from,const JointVector& to,const JointVector& speed) {
    auto& leg=legs_[legs_[0].duration>0. ? 1 : 0];
    leg.from=from;leg.to=to;leg.duration=1.;
    for(int j=0;j<14;++j) {
      leg.delta[j]=to[j]-from[j];
      const double d=std::abs(leg.delta[j]);
      leg.duration=std::max({leg.duration,2.1875*d/std::min(.25,.5*speed[j]),std::sqrt(7.52*d/.5)});
    }
  }

  double brake_position(int j,double u) const {
    return start_.position[j]+brake_s_*start_.velocity[j]*u*(1.+u*u*(-1.+.5*u))+
           brake_s_*brake_s_*acceleration_[j]*u*u*(.5+u*(-2./3.+.25*u));
  }

  double brake_velocity(int j,double u) const {
    const double w=1.-u;
    return w*w*(start_.velocity[j]*(1.+2.*u)+brake_s_*acceleration_[j]*u);
  }

  SoftReturnSample brake_at(double u) const {
    SoftReturnSample sample;
    for(int j=0;j<14;++j) {
      sample.position[j]=brake_position(j,u);
      sample.velocity[j]=brake_velocity(j,u);
    }
    return sample;
  }

  bool brake_within_limits(const JointVector& lower,const JointVector& upper,const JointVector& speed) const {
    for(int j=0;j<14;++j) {
      const auto position_ok=[&](double u){const double q=brake_position(j,u);return q>=lower[j]&&q<=upper[j];};
      const double v=start_.velocity[j],a=brake_s_*acceleration_[j],b=2.*v+a;
      if(!position_ok(1.)||std::abs(v)>speed[j]*(1.+1e-6))return false;
      // Check exact interior extrema of the quartic position and cubic velocity.
      if(b!=0.) {
        const double q_extremum=-v/b,v_extremum=a/(3.*b);
        if(q_extremum>0.&&q_extremum<1.&&!position_ok(q_extremum))return false;
        if(v_extremum>0.&&v_extremum<1.&&std::abs(brake_velocity(j,v_extremum))>speed[j]*(1.+1e-6))return false;
      }
    }
    return true;
  }
};

}  // namespace casbot::shm
