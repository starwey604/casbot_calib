#pragma once
#include <cstdint>
#include <array>
#include <vector>
#include <cmath>
#include <stdexcept>

// v1 wire: revision, ten 14-axis arrays, thirteen scalar fields (seconds).
// Keep order aligned with fieldwork_settings.py. Legacy executors opt out.
// ObservedAction controls velocity only; observed position always warns/records.
struct ManualPolicy {
  enum Axis {InputLower,InputUpper,InputVelocity,InputEffort,KpMax,KdMax,ObservedLower,ObservedUpper,ObservedVelocity,ObservedAction};
  enum Global {StartVelocity,StartStable,StopVelocity,StopStable,StopTimeout,FeedbackTimeout,ModeTimeout,RpcTimeout,HeartbeatTimeout,MaximumDuration,ShortAmplitude,ShortMin,ShortMax};
  std::int64_t revision=0;
  std::array<std::array<double,14>,10> axis{};
  std::array<double,13> general{};
  static ManualPolicy parse(const std::vector<double>& values) {
    if(values.size()!=154)throw std::runtime_error("Manual policy requires154 fields");
    for(double v:values)if(!std::isfinite(v))throw std::runtime_error("Manual policy must be finite");
    if(values[0]<0||values[0]>9007199254740991.||std::floor(values[0])!=values[0])throw std::runtime_error("Invalid policy revision");
    ManualPolicy p;p.revision=static_cast<std::int64_t>(values[0]);std::size_t k=1;
    for(auto& a:p.axis)for(double& v:a)v=values[k++];
    for(double& v:p.general)v=values[k++];
    for(int j=0;j<14;++j) {
      if(p.axis[InputLower][j]>=p.axis[InputUpper][j]||p.axis[ObservedLower][j]>=p.axis[ObservedUpper][j])throw std::runtime_error("Invalid policy position range");
      for(auto f:{InputVelocity,InputEffort,KpMax,KdMax,ObservedVelocity})if(p.axis[f][j]<=0)throw std::runtime_error("Positive policy bound required");
      if(p.axis[ObservedAction][j]!=0&&p.axis[ObservedAction][j]!=1)throw std::runtime_error("Invalid observed action");
    }
    for(int i=0;i<13;++i)if(p.general[i]<0||p.general[i]>9e9||(!p.general[i]&&i!=StartStable&&i!=StopStable&&i!=MaximumDuration))throw std::runtime_error("Invalid policy duration/threshold");
    if(p.general[ShortMin]>p.general[ShortMax])throw std::runtime_error("Invalid short duration range");
    return p;
  }
  bool same_pinned(const ManualPolicy& other) const {
    for(int i=0;i<6;++i)if(axis[i]!=other.axis[i])return false;
    return general==other.general;
  }
  std::int64_t ns(Global key)const{return static_cast<std::int64_t>(general[key]*1e9);}
};
