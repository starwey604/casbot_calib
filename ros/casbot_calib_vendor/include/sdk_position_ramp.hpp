#pragma once
#include <array>
#include <algorithm>
#include <cstdint>
#include <stdexcept>

// A declared Mock policy, not a reverse-engineered vendor interpolator.
// Only position is interpolated. The caller holds velocity/effort/PD separately.
class SdkPositionRamp {
 public:
  using Values=std::array<double,14>;
  explicit SdkPositionRamp(std::int64_t horizon):horizon_(horizon) {
    if(horizon<=0)throw std::invalid_argument("Positive interpolation horizon required");
  }
  Values at(std::int64_t now) const {
    const auto alpha=std::clamp(double(now-begin_)/double(horizon_),0.,1.);
    Values result{};
    for(int j=0;j<14;++j)result[j]=from_[j]+alpha*(target_[j]-from_[j]);
    return result;
  }
  void reset(const Values& q,std::int64_t now) {from_=target_=q;begin_=now;}
  void target(const Values& q,std::int64_t now) {from_=at(now);target_=q;begin_=now;}
 private:
  std::int64_t horizon_,begin_=0;
  Values from_{},target_{};
};
