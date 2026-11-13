#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

// One acknowledged rest-to-rest C3 segment, followed by an indefinite hold.
// This is a local protocol-development source, not a hardware contact permit.
struct TricalStream {
  using Joints=std::array<double,14>;
  std::int64_t generation=0,start=0,duration=0;
  int action=0; Joints from{},to{};
  bool complete(std::int64_t now) const {return now-start>=duration;}
  double chord_distance(const Joints& q) const {
    double numerator=0,denominator=0;
    for(int j=0;j<14;++j){const auto d=to[j]-from[j];numerator+=(q[j]-from[j])*d;denominator+=d*d;}
    const auto u=denominator?std::clamp(numerator/denominator,0.,1.):0.;
    double distance=0;
    for(int j=0;j<14;++j)distance=std::max(distance,std::abs(q[j]-from[j]-u*(to[j]-from[j])));
    return distance;
  }
  void accept(const std::vector<double>& x,std::int64_t now,const Joints& observed,
              const Joints& lower,const Joints& upper,double velocity,double tolerance) {
    if(x.size()!=31)throw std::runtime_error("31 TriCal fields required");
    for(double v:x)if(!std::isfinite(v))throw std::runtime_error("Nonfinite TriCal packet");
    if(x[0]<1||x[0]>1e9||x[0]!=std::floor(x[0])||x[0]<=generation||
       x[1]!=std::floor(x[1])||x[1]<0||x[1]>2||x[2]<0||x[2]>600)
      throw std::runtime_error("Invalid TriCal generation/action/duration");
    if(generation&&!complete(now))throw std::runtime_error("TriCal segment still active");
    if(!generation&&x[1]!=0)throw std::runtime_error("TriCal first packet must hold measured start");
    for(int j=0;j<14;++j) {
      const double a=x[3+j],b=x[17+j];
      if(a<lower[j]||a>upper[j]||b<lower[j]||b>upper[j]||
         std::abs(a-(generation?to[j]:observed[j]))>(generation?1e-10:tolerance))
        throw std::runtime_error("TriCal start discontinuity or bounds");
      if(x[1]!=1&&(a!=b||x[2]!=0))throw std::runtime_error("TriCal hold/finish must be stationary");
      if(x[1]==1&&(x[2]<=0||2.1875*std::abs(b-a)/x[2]>velocity*(1+1e-9)))
        throw std::runtime_error("TriCal velocity bound");
    }
    generation=std::int64_t(x[0]);action=int(x[1]);duration=std::llround(x[2]*1e9);start=now;
    std::copy_n(x.begin()+3,14,from.begin());std::copy_n(x.begin()+17,14,to.begin());
  }
  std::vector<double> row(std::int64_t now) const {
    const double u=duration?std::clamp(double(now-start)/duration,0.,1.):1.;
    // Integral of 140*u^3*(1-u)^3. Identical to Motion's septic Hermite.
    const double s=u*u*u*u*(35+u*(-84+u*(70-20*u)));
    const double v=duration?140*std::pow(u*(1-u),3)/(duration*1e-9):0.;
    std::vector<double> r(29);r[0]=double(now-start);
    for(int j=0;j<14;++j){r[j+1]=from[j]+(to[j]-from[j])*s;r[j+15]=(to[j]-from[j])*v;}
    return r;
  }
};
