#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

// Frozen localhost S2 contract only. Field/manual execution never constructs it.
class LegacyPositionGuard {
 public:
  LegacyPositionGuard(double tracking,double allowance):tracking_(tracking),allowance_(allowance) {
    if(tracking<=0)throw std::runtime_error("Invalid legacy tracking limit");
  }
  std::string check(const std::array<double,14>& q,const std::vector<double>& available,
                    const std::vector<double>& current) const {
    for(int j=0;j<14;++j) {
      const double error=std::abs(q[j]-available[j+1]);
      if(error>tracking_)return "tracking_envelope";
      if(error+std::abs(current[j+1]-available[j+1])>allowance_)return "aged_tracking_envelope";
    }
    return {};
  }
 private:
  double tracking_,allowance_;
};
