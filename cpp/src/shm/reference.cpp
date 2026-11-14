#include "reference.hpp"
#include "json_input.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace casbot::shm {
namespace {
JointVector casteljau(std::array<JointVector,8> coefficients,int degree,double u) {
  for(int n=degree;n>0;--n)for(int i=0;i<n;++i)for(int j=0;j<14;++j)
    coefficients[i][j]=(1-u)*coefficients[i][j]+u*coefficients[i+1][j];
  return coefficients[0];
}
void joint_names(const Json& node) {
  if(node.size()!=14)throw std::invalid_argument("14 canonical joint names required");
  int j=0;for(const auto& n:node)if(!n.first.empty()||n.second.get_value<std::string>()!=arm_names[j++])
    throw std::invalid_argument("Reference joint order mismatch");
}
}
Reference Reference::load(const std::string& file) {
  const auto json=read_json(file);Reference r;
  const auto schema=json.get<std::string>("schema");r.direct_=schema=="casbot_shm_fields_v1";
  if(!r.direct_&&schema!="casbot_shm_reference_v1")throw std::invalid_argument("Unsupported SHM reference schema");
  const auto origin=json.get<std::string>("origin","absolute");
  if(origin!="absolute"&&origin!="measured_start")throw std::invalid_argument("Unknown reference origin");
  r.relative_=origin=="measured_start";
  if(r.relative_&&r.direct_)throw std::invalid_argument("Direct fields use explicit absolute positions");
  if(json.get<std::string>("purpose","")=="shm_h1_collision_checked"||json.get<std::string>("purpose","")=="shm_gui_zero_collision_checked") {
    if(r.direct_||r.relative_)throw std::invalid_argument("H1 uses frozen absolute trajectories");
    r.observed_velocity_=json.get<double>("observed_velocity_limit_rad_s");
    if(!std::isfinite(r.observed_velocity_)||r.observed_velocity_<=0||r.observed_velocity_>1.5)throw std::invalid_argument("Invalid H1 observed velocity limit");
  }
  joint_names(json.get_child("joint_names"));r.kp_=joint_values(json.get_child("kp"));r.kd_=joint_values(json.get_child("kd"));
  for(int j=0;j<14;++j)if(r.kp_[j]<0||r.kp_[j]>500||r.kd_[j]<0||r.kd_[j]>50)throw std::invalid_argument("Reference PD outside explicit interface bounds");
  if(r.direct_) {
    const auto duration=json.get<std::int64_t>("duration_ns");r.knots_={0,duration};
    std::array<JointVector,4> state{};state[0]=joint_values(json.get_child("position"));state[1]=joint_values(json.get_child("velocity"));
    r.states_={state,state};r.effort_.push_back({0,joint_values(json.get_child("effort"))});
  }else {
    for(const auto& item:json.get_child("knots_ns")) {
      if(!item.first.empty())throw std::invalid_argument("Expected knot array");
      r.knots_.push_back(item.second.get_value<std::int64_t>());
    }
    for(const auto& item:json.get_child("states")) {
      if(!item.first.empty()||item.second.size()!=4)throw std::invalid_argument("Expected q/dq/ddq/jerk at every knot");
      std::array<JointVector,4> state{};int k=0;
      for(const auto& derivative:item.second){if(!derivative.first.empty())throw std::invalid_argument("Expected derivative array");state[k++]=joint_values(derivative.second);}
      r.states_.push_back(state);
    }
    for(const auto& item:json.get_child("effort"))r.effort_.push_back({item.second.get<std::int64_t>("t_ns"),joint_values(item.second.get_child("value"))});
  }
  if(r.knots_.size()<2||r.knots_.size()!=r.states_.size()||r.knots_[0]!=0||r.knots_.back()<=0||r.knots_.back()>3600000000000LL)
    throw std::invalid_argument("Invalid reference knots/states/duration");
  for(std::size_t k=1;k<r.knots_.size();++k)if(r.knots_[k]<=r.knots_[k-1])throw std::invalid_argument("Reference knots must increase strictly");
  if(r.effort_.empty()||r.effort_[0].first!=0)throw std::invalid_argument("Effort table must begin at zero");
  for(std::size_t k=0;k<r.effort_.size();++k) {
    if(r.effort_[k].first<0||r.effort_[k].first>r.duration_ns()||(k&&r.effort_[k].first<=r.effort_[k-1].first))throw std::invalid_argument("Invalid effort table times");
    CommandFrame frame{};for(int j=0;j<14;++j)arm_target(frame,j,0,0,r.kp_[j],r.kd_[j],r.effort_[k].second[j]);
  }
  if(r.relative_)for(double q:r.states_.front()[0])if(std::abs(q)>1e-12)throw std::invalid_argument("Measured-origin reference must start at zero offset");
  if(!r.direct_)for(const auto* state:{&r.states_.front(),&r.states_.back()})
    for(int d=1;d<4;++d)for(double v:(*state)[d])if(std::abs(v)>1e-10)throw std::invalid_argument("Trajectory must join stationary entry/exit holds");
  for(std::size_t k=0;k+1<r.knots_.size();++k) {
    std::array<JointVector,8> b{};const double h=(r.knots_[k+1]-r.knots_[k])*1e-9;
    for(int j=0;j<14;++j) {
      const auto& a=r.states_[k];const auto& z=r.states_[k+1];
      b[0][j]=a[0][j];b[1][j]=b[0][j]+h*a[1][j]/7;
      b[2][j]=h*h*a[2][j]/42+2*b[1][j]-b[0][j];
      b[3][j]=h*h*h*a[3][j]/210+3*b[2][j]-3*b[1][j]+b[0][j];
      b[7][j]=z[0][j];b[6][j]=b[7][j]-h*z[1][j]/7;
      b[5][j]=h*h*z[2][j]/42+2*b[6][j]-b[7][j];
      b[4][j]=b[7][j]-3*b[6][j]+3*b[5][j]-h*h*h*z[3][j]/210;
      for(int d=0;d<8;++d)if(!std::isfinite(b[d][j]))throw std::invalid_argument("Nonfinite reference coefficient");
    }
    r.coefficients_.push_back(b);
  }
  return r;
}
void Reference::resolve_origin(const JointVector& measured) {
  if(!relative_)throw std::logic_error("Reference has no unresolved measured origin");
  for(int j=0;j<14;++j)if(!std::isfinite(measured[j]))throw std::invalid_argument("Nonfinite measured origin");
  for(auto& state:states_)for(int j=0;j<14;++j)state[0][j]+=measured[j];
  for(auto& segment:coefficients_)for(auto& point:segment)for(int j=0;j<14;++j)point[j]+=measured[j];
  relative_=false;
}
ReferenceValue Reference::at(std::int64_t t)const {
  t=std::clamp<std::int64_t>(t,0,duration_ns());ReferenceValue result;result.kp=kp_;result.kd=kd_;
  const auto effort=std::upper_bound(effort_.begin(),effort_.end(),t,[](auto time,const auto& row){return time<row.first;});
  result.effort=(effort-1)->second;
  if(direct_){result.state=states_[0];return result;}
  const auto it=std::upper_bound(knots_.begin(),knots_.end(),t);
  const auto k=std::min<std::size_t>(it-knots_.begin()-1,coefficients_.size()-1);
  const double h=(knots_[k+1]-knots_[k])*1e-9,u=double(t-knots_[k])/double(knots_[k+1]-knots_[k]);
  auto c=coefficients_[k];
  for(int d=0;d<4;++d) {
    result.state[d]=casteljau(c,7-d,u);
    for(int i=0;i<7-d;++i)for(int j=0;j<14;++j)c[i][j]=(7-d)*(c[i+1][j]-c[i][j])/h;
  }
  return result;
}
void Reference::validate_limits(const JointVector& lower,const JointVector& upper,const JointVector& velocity)const {
  if(direct_) {
    const auto v=at(0);for(int j=0;j<14;++j)if(v.state[0][j]<lower[j]||v.state[0][j]>upper[j]||std::abs(v.state[1][j])>velocity[j])throw std::invalid_argument("Direct fields outside URDF/reference velocity limits");
    return;
  }
  for(std::size_t k=0;k<coefficients_.size();++k) {
    const double h=(knots_[k+1]-knots_[k])*1e-9;
    for(int j=0;j<14;++j)for(int i=0;i<8;++i) {
      if(coefficients_[k][i][j]<lower[j]-1e-12||coefficients_[k][i][j]>upper[j]+1e-12)throw std::invalid_argument("Reference Bernstein position envelope outside URDF");
      if(i<7&&7*std::abs(coefficients_[k][i+1][j]-coefficients_[k][i][j])/h>velocity[j]*(1+1e-10))throw std::invalid_argument("Reference Bernstein velocity envelope outside limit");
    }
  }
}
}  // namespace casbot::shm
