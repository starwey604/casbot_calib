#include "transport_common.hpp"
#include <casbot_calib/arm_simulator.hpp>
#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>

class MockHost : public rclcpp::Node {
  struct Incoming {bench::Command command;std::int64_t receive,release;};
  std::string experiment_,model_id_,clock_=bench::clock_id();
  std::array<std::string,14> names_;
  std::unique_ptr<casbot::ArmDynamics> model_;
  std::unique_ptr<casbot::ArmSimulationSession> plant_;
  std::deque<casbot::ArmSample> delivery_;
  std::vector<Incoming> inbox_;
  std::optional<Incoming> latest_;
  casbot::ArmSample sample_{};
  Eigen::VectorXd hold_=Eigen::VectorXd::Zero(14);
  std::int64_t epoch_=0,driver_receive_=0,last_sequence_=-1,adopted_sequence_=-1;
  std::int64_t inject_delay_=0,inject_every_=0;
  std::uint64_t event_sequence_=0,publish_sequence_=0,accepted_=0,adopted_=0,rejected_=0;
  bool watchdog_=true;
  std::ofstream receives_,writes_,samples_;
  rclcpp::Publisher<bench::Observation>::SharedPtr observations_;
  rclcpp::Publisher<bench::Event>::SharedPtr events_;
  rclcpp::Subscription<bench::Command>::SharedPtr commands_;
  rclcpp::TimerBase::SharedPtr control_,feedback_;
  void event(std::uint8_t kind,const std::string& reason,const Incoming* item,std::int64_t decision) {
    bench::Event value;value.experiment_id=experiment_;value.clock_id=clock_;value.event_sequence=event_sequence_++;
    value.kind=kind;value.reason=reason;value.decision_ns=decision;
    if(item) {value.command_sequence=item->command.sequence;value.source_send_ns=item->command.send_ns;value.receive_ns=item->receive;value.source_clock_id=item->command.clock_id;}
    value.event_publish_ns=bench::now_ns();events_->publish(value);
    receives_<<value.event_sequence<<','<<value.command_sequence<<','<<int(kind)<<','<<reason<<','
      <<value.source_send_ns<<','<<value.receive_ns<<','<<decision<<','<<value.event_publish_ns<<'\n';
  }
  std::string invalid(const Incoming& item,std::int64_t now) const {
    const auto& c=item.command;
    if(c.experiment_id!=experiment_ || c.model_id!=model_id_)return "experiment_or_model";
    if(c.joint_names!=names_)return "joint_order";
    if(c.mode!=bench::Command::TORQUE && c.mode!=bench::Command::POSITION_PD)return "mode";
    if(c.send_ns<=0 || c.clock_id.empty() || c.valid_for_ns<1'000'000 || c.valid_for_ns>1'000'000'000)return "time_contract";
    for(std::size_t j=0;j<14;++j)
      if(!std::isfinite(c.position[j]) || !std::isfinite(c.velocity[j]) || !std::isfinite(c.effort[j]) ||
         !std::isfinite(c.kp[j]) || !std::isfinite(c.kd[j]) || c.kp[j]<0 || c.kd[j]<0 || c.kp[j]>1000 || c.kd[j]>100 || std::abs(c.effort[j])>100)
        return "nonfinite_or_synthetic_limit";
    if(c.clock_id==clock_ && c.send_ns>now+1'000'000)return "future_timestamp";
    if(now-item.receive>c.valid_for_ns || (c.clock_id==clock_ && now-c.send_ns>c.valid_for_ns))return "expired";
    if(last_sequence_>=0 && c.sequence<=static_cast<std::uint64_t>(last_sequence_))return "duplicate_or_old";
    if(c.sequence>static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))return "sequence_range";
    return {};
  }
  void tick() {
    const auto tick=bench::now_ns();
    auto chunk=plant_->advance_to(tick-epoch_);
    for(auto& value:chunk.observations)delivery_.push_back(std::move(value));
    while(!delivery_.empty() && delivery_.front().receive_ns<=tick-epoch_) {
      sample_=std::move(delivery_.front());delivery_.pop_front();driver_receive_=tick;
      samples_<<sample_.sequence<<','<<sample_.sample_ns+epoch_<<','<<tick;
      bench::columns(samples_,sample_.q);bench::columns(samples_,sample_.dq);bench::columns(samples_,sample_.current);bench::columns(samples_,sample_.status);samples_<<'\n';
    }
    std::stable_sort(inbox_.begin(),inbox_.end(),[](const auto& a,const auto& b){return a.release<b.release;});
    auto ready=inbox_.begin();
    while(ready!=inbox_.end() && ready->release<=tick) {
      const auto reason=invalid(*ready,tick);
      if(reason.empty()) {latest_=*ready;last_sequence_=static_cast<std::int64_t>(ready->command.sequence);++accepted_;event(bench::Event::ACCEPTED,"valid",&*ready,tick);}
      else {++rejected_;event(bench::Event::REJECTED,reason,&*ready,tick);}
      ++ready;
    }
    inbox_.erase(inbox_.begin(),ready);
    bool fresh=latest_.has_value();
    if(fresh) {const auto& item=*latest_;fresh=tick-item.receive<=item.command.valid_for_ns &&
      (item.command.clock_id!=clock_ || tick-item.command.send_ns<=item.command.valid_for_ns);}
    if(fresh && adopted_sequence_!=last_sequence_) {adopted_sequence_=last_sequence_;++adopted_;event(bench::Event::ADOPTED,"latest_valid",&*latest_,tick);}
    if(!fresh && !watchdog_) {hold_=sample_.q;event(bench::Event::WATCHDOG,"mock_gravity_weak_damping",latest_?&*latest_:nullptr,tick);}
    watchdog_=!fresh;
    Eigen::VectorXd effort(14);
    if(fresh) {
      const auto& c=latest_->command;
      for(int j=0;j<14;++j)effort[j]=c.effort[j]+(c.mode==bench::Command::POSITION_PD?
        c.kp[j]*(c.position[j]-sample_.q[j])+c.kd[j]*(c.velocity[j]-sample_.dq[j]):0.);
    } else effort=model_->gravity(sample_.q)-.001*sample_.dq;
    effort=effort.cwiseMax(-100.).cwiseMin(100.);
    const auto write=bench::now_ns();
    casbot::ArmCommand command{write-epoch_,hold_,Eigen::VectorXd::Zero(14),effort,Eigen::VectorXd::Zero(14),Eigen::VectorXd::Zero(14)};
    plant_->submit(command);
    writes_<<tick<<','<<write<<','<<(fresh?adopted_sequence_:-1)<<','<<watchdog_;bench::columns(writes_,effort);writes_<<'\n';
  }
  void publish() {
    if(!driver_receive_)return;
    bench::Observation value;value.experiment_id=experiment_;value.model_id=model_id_;value.clock_id=clock_;
    value.sample_clock_id=clock_;value.sequence=sample_.sequence;value.publish_sequence=publish_sequence_++;value.sample_ns=sample_.sample_ns+epoch_;
    value.driver_receive_ns=driver_receive_;value.sample_time_known=true;value.joint_names=names_;
    for(int j=0;j<14;++j) {value.position[j]=sample_.q[j];value.velocity[j]=sample_.dq[j];value.current[j]=sample_.current[j];
      value.estimated_effort[j]=2.*sample_.current[j];value.status[j]=sample_.status[j];}
    value.watchdog_active=watchdog_;value.accepted_commands=accepted_;value.adopted_commands=adopted_;value.rejected_commands=rejected_;
    value.publish_ns=bench::now_ns();observations_->publish(value);
  }
 public:
  MockHost():Node("calibration_mock_host") {
    experiment_=declare_parameter<std::string>("experiment_id");model_id_=declare_parameter<std::string>("model_id");
    const auto output=declare_parameter<std::string>("output_dir"),path=declare_parameter<std::string>("model_path");
    if(experiment_.empty() || model_id_.empty() || output.empty() || path.empty())throw std::invalid_argument("Required experiment/model/output parameters missing");
    const auto control_hz=declare_parameter<int>("control_hz",500),feedback_hz=declare_parameter<int>("feedback_hz",500);
    if(control_hz<10 || control_hz>2000 || feedback_hz<10 || feedback_hz>2000)throw std::invalid_argument("Invalid benchmark rate");
    inject_every_=declare_parameter<int>("inject_every",0);inject_delay_=declare_parameter<std::int64_t>("inject_delay_ns",0);
    if(inject_every_<0 || inject_delay_<0 || inject_delay_>500'000'000)throw std::invalid_argument("Invalid injection");
    auto native=casbot::ArmDynamics::from_urdf(path).model();
    if(native.joints.size()!=14)throw std::invalid_argument("Mock requires the declared nominal 14-axis model");
    if(declare_parameter<bool>("zero_gravity",true))native.gravity_world.setZero();
    model_=std::make_unique<casbot::ArmDynamics>(native);
    for(int j=0;j<14;++j)names_[j]=native.joints[j].name;
    casbot::ArmSimulationConfig config;config.gain=Eigen::VectorXd::Constant(14,2.);
    config.current_tau=declare_parameter<double>("current_tau",.001);config.command_delay_ns=declare_parameter<std::int64_t>("drive_delay_ns",1'350'000);
    config.feedback_delay_ns=declare_parameter<std::int64_t>("feedback_delay_ns",700'000);
    const auto initial=declare_parameter<std::vector<double>>("initial_q",std::vector<double>(14,0.));
    if(initial.size()!=14)throw std::invalid_argument("initial_q must contain 14 values");
    hold_=Eigen::Map<const Eigen::VectorXd>(initial.data(),14);sample_.q=hold_;sample_.dq=Eigen::VectorXd::Zero(14);sample_.current=Eigen::VectorXd::Zero(14);sample_.status=Eigen::VectorXi::Zero(14);
    plant_=std::make_unique<casbot::ArmSimulationSession>(*model_,config,hold_,sample_.dq,false);
    receives_=bench::log(output+"/host_events.csv","event_sequence,command_sequence,kind,reason,source_send_ns,receive_ns,decision_ns,event_publish_ns");
    writes_=bench::log(output+"/host_writes.csv","tick_ns,write_ns,command_sequence,watchdog"+bench::indexed("effort"));
    samples_=bench::log(output+"/host_samples.csv","sequence,sample_ns,driver_receive_ns"+bench::indexed("q")+bench::indexed("dq")+bench::indexed("current")+bench::indexed("status"));
    events_=create_publisher<bench::Event>("events",bench::qos(true,1024));
    observations_=create_publisher<bench::Observation>("observations",bench::qos(declare_parameter<bool>("reliable",true),1));
    commands_=create_subscription<bench::Command>("commands",bench::qos(get_parameter("reliable").as_bool(),1),[this](bench::Command::SharedPtr command) {
      Incoming item{*command,bench::now_ns(),0};item.release=item.receive;
      event(bench::Event::RECEIVED,"dds_callback",&item,item.receive);
      if(inject_every_>0 && command->sequence%inject_every_==0)item.release+=inject_delay_;
      inbox_.push_back(std::move(item));
    });
    epoch_=bench::now_ns();
    control_=create_wall_timer(std::chrono::nanoseconds(1'000'000'000/control_hz),[this]{tick();});
    feedback_=create_wall_timer(std::chrono::nanoseconds(1'000'000'000/feedback_hz),[this]{publish();});
    RCLCPP_INFO(get_logger(),"Synthetic 14-axis host ready; clock=%s; inline CSV instrumentation; no hardware",clock_.c_str());
  }
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {auto node=std::make_shared<MockHost>();rclcpp::spin(node);node.reset();rclcpp::shutdown();return 0;}
  catch(const std::exception& error) {std::cerr<<error.what()<<'\n';rclcpp::shutdown();return 1;}
}
