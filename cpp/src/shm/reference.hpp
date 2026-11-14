#pragma once
#include <casbot_calib/shm/arm_codec.hpp>
#include <array>
#include <cstdint>
#include <string>
#include <vector>
namespace casbot::shm {
using JointVector=std::array<double,14>;
struct ReferenceValue {
  // q, dq, ddq, jerk all come from the same polynomial in trajectory mode.
  std::array<JointVector,4> state{};
  JointVector kp{},kd{},effort{};
};
class Reference {
 public:
  static Reference load(const std::string& file);
  ReferenceValue at(std::int64_t relative_ns) const;
  void validate_limits(const JointVector& lower,const JointVector& upper,const JointVector& velocity) const;
  std::int64_t duration_ns() const{return knots_.back();}
  bool direct_fields() const{return direct_;}
  double observed_velocity_limit() const{return observed_velocity_;}
  bool measured_origin() const{return relative_;}
  const std::vector<std::pair<std::int64_t,std::string>>& markers() const{return markers_;}
  void resolve_origin(const JointVector& measured);
 private:
  bool direct_=false,relative_=false;
  double observed_velocity_=0.;
  std::vector<std::int64_t> knots_;
  std::vector<std::array<JointVector,4>> states_;
  std::vector<std::array<JointVector,8>> coefficients_;
  JointVector kp_{},kd_{};
  struct Gains {std::int64_t at;JointVector kp,kd;};
  std::vector<Gains> gains_;
  std::vector<std::pair<std::int64_t,std::string>> markers_;
  std::vector<std::pair<std::int64_t,JointVector>> effort_;
};
}  // namespace casbot::shm
