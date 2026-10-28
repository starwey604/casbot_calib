#include "casbot_calib/trical_mock.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <random>
#include <stdexcept>
namespace casbot {
namespace {
Eigen::VectorXd initial_physical(const Eigen::VectorXd& q,const TricalMockConfig& c){
  if(c.encoder_offset.size()==0)return q;
  if(c.encoder_offset.size()!=q.size() || !c.encoder_offset.allFinite())throw std::invalid_argument("Invalid Mock encoder offsets");
  return q+c.encoder_offset;
}
}
struct TricalMockSession::Impl {
  ArmSimulationSession arm;ArmWorkspace geometry;TricalMockConfig c;Nanoseconds grid;
  std::mt19937 rng;std::array<Nanoseconds,3> next,sequence{{0,0,0}};
  std::array<std::string,3> faults{{"none","none","none"}};
  std::array<IndicatorPacket,3> last;std::array<bool,3> has_last{{false,false,false}};
  std::deque<IndicatorPacket> pending;std::string frame,target;Eigen::Vector3d ball=Eigen::Vector3d::Zero();bool truth;
  std::vector<std::string> frame_names;Nanoseconds last_raw=-1;
  Impl(ArmDynamics model,ArmSimulationConfig drive,TricalMockConfig sensor,const Eigen::VectorXd& q,const Eigen::VectorXd& v,bool retain)
    :arm(model,drive,initial_physical(q,sensor),v,true),geometry(model.workspace()),c(std::move(sensor)),grid(drive.sample_period_ns),rng(c.seed),next(c.phase_ns),truth(retain),frame_names(model.frame_names()) {
    if(c.encoder_offset.size()==0)c.encoder_offset=Eigen::VectorXd::Zero(q.size());
    trical_offset(c.calibration.zero,c.calibration);
    if(c.period_ns<=0 || c.period_ns%grid || c.delay_ns<0 || c.jitter_ns<0 || c.jitter_ns>c.delay_ns ||
       !c.minimum.allFinite() || !c.maximum.allFinite() || !c.bias.allFinite() || (c.minimum.array()>=c.maximum.array()).any() ||
       !std::isfinite(c.face_radius) || c.face_radius<=0 || !std::isfinite(c.noise_std) || c.noise_std<0 ||
       !std::isfinite(c.quantum) || c.quantum<0 || !std::isfinite(c.drop_probability) || c.drop_probability<0 || c.drop_probability>1)
      throw std::invalid_argument("Invalid TriCal Mock timing/range/noise configuration");
    for(auto phase:c.phase_ns)if(phase<0 || phase>=c.period_ns || phase%grid)throw std::invalid_argument("Indicator ticks must lie on arm raw sample grid");
  }
  double uniform(){return (static_cast<double>(rng())+.5)/4294967296.;}
  double normal(){const double a=uniform(),b=uniform();return std::sqrt(-2.*std::log(a))*std::cos(2.*std::acos(-1.)*b);}
};
TricalMockSession::TricalMockSession(ArmDynamics plant,ArmSimulationConfig arm,TricalMockConfig sensor,const Eigen::VectorXd& q,const Eigen::VectorXd& v,bool retain)
  :impl_(std::make_unique<Impl>(std::move(plant),std::move(arm),std::move(sensor),q,v,retain)){}
TricalMockSession::~TricalMockSession()=default;
void TricalMockSession::submit(const ArmCommand& command){
  ArmCommand physical=command;
  if(physical.q.size()!=impl_->c.encoder_offset.size())throw std::invalid_argument("Wrong Mock command width");
  physical.q+=impl_->c.encoder_offset;impl_->arm.submit(physical);
}
Nanoseconds TricalMockSession::time_ns()const{return impl_->arm.time_ns();}
int TricalMockSession::joint_count()const{return static_cast<int>(impl_->c.encoder_offset.size());}
void TricalMockSession::select_target(const std::string& frame,const std::string& target,const Eigen::Vector3d& ball){
  auto& s=*impl_;
  if(frame.empty() || target.empty() || !ball.allFinite())throw std::invalid_argument("Invalid indicator target");
  if(std::find(s.frame_names.begin(),s.frame_names.end(),frame)==s.frame_names.end())throw std::invalid_argument("Unknown indicator frame");
  s.frame=frame;s.target=target;s.ball=ball;s.has_last.fill(false);
  for(int i=0;i<3;++i){s.next[i]=s.c.phase_ns[i];const auto minimum=std::max(s.arm.time_ns(),s.last_raw+1);
    if(s.next[i]<minimum)s.next[i]+=((minimum-s.next[i]+s.c.period_ns-1)/s.c.period_ns)*s.c.period_ns;
  }
}
void TricalMockSession::set_fault(int channel,const std::string& fault){
  if(channel<0 || channel>2 || (fault!="none" && fault!="disconnected" && fault!="frozen" && fault!="out_of_range" && fault!="no_contact"))
    throw std::invalid_argument("Unknown indicator fault/channel");
  impl_->faults[channel]=fault;
}
TricalMockResult TricalMockSession::advance_to(Nanoseconds end){
  auto& s=*impl_;TricalMockResult output;output.simulation=s.arm.advance_to(end);
  for(const auto& state:output.simulation.truth) {
    s.last_raw=state.sample_ns;
    if(s.frame.empty())continue;
    if(std::none_of(s.next.begin(),s.next.end(),[&](auto next){return next==state.sample_ns;}))continue;
    const auto transform=s.geometry.frame_pose(state.q,s.frame);ArmPose pose;
    pose.rotation=transform.topLeftCorner<3,3>();pose.translation=transform.topRightCorner<3,1>();
    const auto ideal=trical_readings(pose,s.ball,s.c.calibration);
    const Eigen::Vector3d offset=pose.rotation.transpose()*(s.ball-pose.translation);
    for(int channel=0;channel<3;++channel)if(state.sample_ns==s.next[channel]){
      s.next[channel]+=s.c.period_ns;IndicatorPacket packet;packet.channel=channel;
      packet.sequence=s.sequence[channel]++;packet.sample_known=s.c.expose_sample_time;
      packet.sample_ns=packet.sample_known?state.sample_ns:-1;
      packet.receive_ns=state.sample_ns+s.c.delay_ns+static_cast<Nanoseconds>((2*s.uniform()-1)*s.c.jitter_ns);
      packet.target_id=s.target;packet.tool_frame=s.frame;packet.status=0;
      const Eigen::Vector3d normal=s.c.calibration.axes.row(channel).transpose();
      const double transverse=(offset-normal*normal.dot(offset)).norm();
      packet.reading=ideal[channel]+s.c.bias[channel]+s.c.noise_std*s.normal();
      if(s.c.quantum>0)packet.reading=std::round(packet.reading/s.c.quantum)*s.c.quantum;
      if(transverse>s.c.face_radius){packet.status=1;packet.reading=std::numeric_limits<double>::quiet_NaN();}
      else if(ideal[channel]<s.c.minimum[channel] || ideal[channel]>s.c.maximum[channel] ||
              packet.reading<s.c.minimum[channel] || packet.reading>s.c.maximum[channel])packet.status=2;
      if(s.faults[channel]=="disconnected"){packet.status=4;packet.reading=std::numeric_limits<double>::quiet_NaN();}
      if(s.faults[channel]=="no_contact"){packet.status=1;packet.reading=std::numeric_limits<double>::quiet_NaN();}
      if(s.faults[channel]=="out_of_range"){packet.status=2;packet.reading=s.c.maximum[channel]+.001;}
      if(s.faults[channel]=="frozen" && s.has_last[channel]){
        const auto delivery=packet.receive_ns;packet=s.last[channel];packet.receive_ns=delivery;packet.status=3;
      }
      if(!s.has_last[channel] || s.faults[channel]!="frozen"){s.last[channel]=packet;s.has_last[channel]=true;}
      if(s.uniform()>=s.c.drop_probability)s.pending.push_back(packet);
    }
  }
  std::stable_sort(s.pending.begin(),s.pending.end(),[](const auto& a,const auto& b){return a.receive_ns<b.receive_ns;});
  while(!s.pending.empty() && s.pending.front().receive_ns<=end){output.indicators.push_back(s.pending.front());s.pending.pop_front();}
  for(auto& observation:output.simulation.observations)observation.q-=s.c.encoder_offset;
  if(!s.truth)output.simulation.truth.clear();
  return output;
}
}  // namespace casbot
