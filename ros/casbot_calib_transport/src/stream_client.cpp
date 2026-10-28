#include "transport_common.hpp"
#include <casbot_calib/arm_dynamics.hpp>
#include <deque>

class StreamClient : public rclcpp::Node {
  std::vector<bench::Command> plan_;
  std::deque<bench::Command> history_;
  std::string experiment_,model_id_,clock_=bench::clock_id();
  std::array<std::string,14> names_;
  std::int64_t start_=0,period_=0,last_index_=-1,watch_start_=0;
  bool faults_=false,ready_=false;
  double tail_=0.;
  std::ofstream sent_,states_,events_;
  std::string output_;
  std::string host_clock_;
  std::string sample_clock_;
  rclcpp::Publisher<bench::Command>::SharedPtr commands_;
  rclcpp::Subscription<bench::Observation>::SharedPtr observations_;
  rclcpp::Subscription<bench::Event>::SharedPtr acknowledgements_;
  rclcpp::TimerBase::SharedPtr timer_;
  void send(const bench::Command& command,std::int64_t planned,const std::string& injection) {
    const auto publish=bench::now_ns();commands_->publish(command);
    sent_<<command.sequence<<','<<planned<<','<<command.send_ns<<','<<publish<<','<<int(command.mode)<<','<<injection;
    bench::columns(sent_,command.position);bench::columns(sent_,command.velocity);bench::columns(sent_,command.effort);bench::columns(sent_,command.kp);bench::columns(sent_,command.kd);sent_<<'\n';
  }
  void tick() {
    const auto now=bench::now_ns();
    if(!ready_) {if(now-watch_start_>10'000'000'000)throw std::runtime_error("No matching host observation within 10 seconds");return;}
    if(!start_)start_=now+200'000'000;
    if(now<start_)return;
    const auto index=(now-start_)/period_;
    if(index>=static_cast<std::int64_t>(plan_.size())) {
      if(now-start_>static_cast<std::int64_t>(plan_.size())*period_+static_cast<std::int64_t>(tail_*1e9))rclcpp::shutdown();
      return;
    }
    if(index<=last_index_)return;
    last_index_=index;
    auto command=plan_[index];command.send_ns=bench::now_ns();
    if(faults_ && command.sequence%19==0) {sent_<<command.sequence<<','<<start_+index*period_<<','<<command.send_ns<<",0,"<<int(command.mode)<<",omitted";
      bench::columns(sent_,command.position);bench::columns(sent_,command.velocity);bench::columns(sent_,command.effort);bench::columns(sent_,command.kp);bench::columns(sent_,command.kd);sent_<<'\n';return;}
    std::string injection="none";
    if(faults_ && command.sequence%37==0) {command.effort[0]=std::numeric_limits<double>::quiet_NaN();injection="nan";}
    else if(faults_ && command.sequence%41==0) {command.joint_names[0]="wrong_joint";injection="joint_order";}
    else if(faults_ && command.sequence%43==0) {command.mode=255;injection="mode";}
    else if(faults_ && command.sequence%47==0) {command.model_id="wrong_model";injection="model";}
    send(command,start_+index*period_,injection);
    if(faults_ && command.sequence%11==0)send(command,start_+index*period_,"duplicate");
    if(faults_ && command.sequence%13==0 && history_.size()>2)send(history_[history_.size()-3],start_+index*period_,"old");
    history_.push_back(command);if(history_.size()>4)history_.pop_front();
  }
 public:
  StreamClient():Node("calibration_stream_client") {
    experiment_=declare_parameter<std::string>("experiment_id");model_id_=declare_parameter<std::string>("model_id");
    const auto output=declare_parameter<std::string>("output_dir"),path=declare_parameter<std::string>("model_path");
    output_=output;
    const auto model=casbot::ArmDynamics::from_urdf(path).model();if(model.joints.size()!=14)throw std::invalid_argument("Expected 14 joint names");
    for(int j=0;j<14;++j)names_[j]=model.joints[j].name;
    const auto rate=declare_parameter<int>("command_hz",500);if(rate<10 || rate>2000)throw std::invalid_argument("Invalid command rate");period_=1'000'000'000/rate;
    tail_=declare_parameter<double>("tail_s",.2);faults_=declare_parameter<bool>("faults",false);
    const auto validity=declare_parameter<std::int64_t>("valid_for_ns",100'000'000);
    std::ifstream input(declare_parameter<std::string>("plan_file"));std::string line;std::getline(input,line);
    while(std::getline(input,line)) {
      const auto values=bench::split(line);if(values.size()!=72)throw std::invalid_argument("Expected plan: relative_ns,mode,5x14 arrays");
      bench::Command command;command.experiment_id=experiment_;command.model_id=model_id_;command.clock_id=clock_;command.sequence=plan_.size()+1;
      if(std::stoll(values[0])!=static_cast<std::int64_t>(plan_.size())*period_)throw std::invalid_argument("Nonuniform command plan");
      command.valid_for_ns=validity;command.joint_names=names_;command.mode=static_cast<std::uint8_t>(std::stoi(values[1]));
      for(int j=0;j<14;++j) {command.position[j]=std::stod(values[2+j]);command.velocity[j]=std::stod(values[16+j]);command.effort[j]=std::stod(values[30+j]);command.kp[j]=std::stod(values[44+j]);command.kd[j]=std::stod(values[58+j]);}
      plan_.push_back(command);
    }
    if(plan_.empty())throw std::invalid_argument("Empty command plan");
    sent_=bench::log(output+"/client_sent.csv","sequence,planned_ns,send_ns,publish_ns,mode,injection"+bench::indexed("q_ref")+bench::indexed("dq_ref")+bench::indexed("effort")+bench::indexed("kp")+bench::indexed("kd"));
    states_=bench::log(output+"/client_observations.csv","sequence,publish_sequence,sample_ns,driver_receive_ns,publish_ns,receive_ns,clock_match,sample_clock_match,sample_time_known,watchdog,accepted,adopted,rejected"+bench::indexed("q")+bench::indexed("dq")+bench::indexed("current")+bench::indexed("status"));
    events_=bench::log(output+"/client_events.csv","event_sequence,command_sequence,kind,reason,source_send_ns,host_receive_ns,decision_ns,event_publish_ns,client_receive_ns,clock_match,source_clock_match");
    const bool reliable=declare_parameter<bool>("reliable",true);
    commands_=create_publisher<bench::Command>("commands",bench::qos(reliable,1));
    observations_=create_subscription<bench::Observation>("observations",bench::qos(reliable,1),[this](bench::Observation::SharedPtr value) {
      const auto receive=bench::now_ns();if(value->experiment_id!=experiment_ || value->model_id!=model_id_ || value->joint_names!=names_)throw std::runtime_error("Host model/experiment contract mismatch");
      if(!ready_) {host_clock_=value->clock_id;sample_clock_=value->sample_clock_id;std::ofstream clocks(output_+"/clocks.json");clocks<<"{\"client\":"<<std::quoted(clock_)<<",\"host\":"<<std::quoted(value->clock_id)<<",\"sample\":"<<std::quoted(sample_clock_)<<",\"sample_time_known\":"<<(value->sample_time_known?"true":"false")<<"}\n";}
      else if(host_clock_!=value->clock_id || sample_clock_!=value->sample_clock_id)throw std::runtime_error("Host or acquisition clock changed within a capture");
      ready_=true;
      states_<<value->sequence<<','<<value->publish_sequence<<','<<value->sample_ns<<','<<value->driver_receive_ns<<','<<value->publish_ns<<','<<receive<<','<<(value->clock_id==clock_)<<','<<(value->sample_clock_id==value->clock_id)<<','<<value->sample_time_known<<','<<value->watchdog_active<<','<<value->accepted_commands<<','<<value->adopted_commands<<','<<value->rejected_commands;
      bench::columns(states_,value->position);bench::columns(states_,value->velocity);bench::columns(states_,value->current);bench::columns(states_,value->status);states_<<'\n';
    });
    acknowledgements_=create_subscription<bench::Event>("events",bench::qos(true,1024),[this](bench::Event::SharedPtr value) {
      if(value->experiment_id!=experiment_)return;
      events_<<value->event_sequence<<','<<value->command_sequence<<','<<int(value->kind)<<','<<value->reason<<','<<value->source_send_ns<<','<<value->receive_ns<<','<<value->decision_ns<<','<<value->event_publish_ns<<','<<bench::now_ns()<<','<<(value->clock_id==clock_)<<','<<(value->source_clock_id==clock_)<<'\n';
    });
    watch_start_=bench::now_ns();timer_=create_wall_timer(std::chrono::nanoseconds(period_),[this]{tick();});
  }
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {auto node=std::make_shared<StreamClient>();rclcpp::spin(node);node.reset();rclcpp::shutdown();return 0;}
  catch(const std::exception& error) {std::cerr<<error.what()<<'\n';rclcpp::shutdown();return 1;}
}
