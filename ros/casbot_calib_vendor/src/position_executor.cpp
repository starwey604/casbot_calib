// S2 position stream and fault state machine. Hardware execution remains blocked.
#include "vendor_common.hpp"
#include <crb_ros_msg/msg/upper_joint_data.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <csignal>

namespace {volatile std::sig_atomic_t interrupted=0; void on_signal(int) {interrupted=1;}}
class PositionExecutor final : public rclcpp::Node {
 public:
  PositionExecutor():Node("casbot_vendor_position_executor",rclcpp::NodeOptions()
      .enable_rosout(false).start_parameter_services(false).start_parameter_event_publisher(false)) {
    const auto output=declare_parameter<std::string>("output");
    names_=declare_parameter<std::vector<std::string>>("joint_names");
    snapshot_=declare_parameter<bool>("snapshot_only",false);
    const auto joint=declare_parameter<std::string>("joint_topic"),mode=declare_parameter<std::string>("mode_topic");
    const auto command=declare_parameter<std::string>("command_topic"),service=declare_parameter<std::string>("mode_service"),stop=declare_parameter<std::string>("stop_service");
    command_topic_=command;
    vendor::local_topics({joint,mode,command,service,stop});
    if (names_.size()!=14||std::set<std::string>(names_.begin(),names_.end()).size()!=14) throw std::runtime_error("Expected 14 unique joint names");
    for (const auto& n:names_) if (n.find("waist")!=std::string::npos || n.find("head")!=std::string::npos ||
        (n.find("wrist")==std::string::npos&&n.find("shoulder")==std::string::npos&&n.find("elbow")==std::string::npos))
      throw std::runtime_error("Only explicit arm joints are permitted");
    const std::filesystem::path root(output);
    if (!std::filesystem::create_directory(root)) throw std::runtime_error("Execution output must not exist");
    root_=root; events_=vendor::log(root/"events.csv","receive_ns,state,reason");
    received_=vendor::log(root/"feedback.csv","receive_ns"+vendor::columns("q")+vendor::columns("v"));
    commands_=vendor::log(root/"commands.csv","planned_index,planned_ns,publish_ns,phase"+vendor::columns("q")+vendor::columns("v"));
    max_age_=declare_parameter<std::int64_t>("max_feedback_age_ns",60000000);
    mode_age_=declare_parameter<std::int64_t>("max_mode_age_ns",300000000);
    timeout_=declare_parameter<std::int64_t>("transition_timeout_ns",1000000000);
    startup_timeout_=declare_parameter<std::int64_t>("startup_timeout_ns",10000000000);
    tolerance_=declare_parameter<double>("start_tolerance_rad",.004);
    tracking_=declare_parameter<double>("tracking_limit_rad",.02);
    allowance_=declare_parameter<double>("position_allowance_rad",.03);
    max_velocity_=declare_parameter<double>("maximum_velocity_rad_s",.5);
    vel_scale_=declare_parameter<double>("vel_scale",.1);
    period_=declare_parameter<std::int64_t>("command_period_ns",2000000);
    if(max_age_<=0||mode_age_<=0||timeout_<=0||period_<1000000||period_>10000000||tolerance_<=0||tracking_<=0||max_velocity_<=0||vel_scale_<=0||vel_scale_>1)
      throw std::runtime_error("Invalid execution limits");
    sub_=create_subscription<sensor_msgs::msg::JointState>(joint,rclcpp::QoS(10).reliable(),[this](sensor_msgs::msg::JointState::SharedPtr msg) {
      vendor::State value;
      if(!vendor::state(*msg,names_,value)) {if(active()) fault("invalid_feedback");return;}
      state_=value; received_<<value.receive;vendor::values(received_,value.q);vendor::values(received_,value.v);received_<<'\n';
    });
    status_=create_subscription<std_msgs::msg::String>(mode,rclcpp::QoS(10).reliable(),[this](std_msgs::msg::String::SharedPtr msg) {mode_=msg->data;mode_at_=vendor::now();});
    if(!snapshot_) {
      auto lower=declare_parameter<std::vector<double>>("lower"),upper=declare_parameter<std::vector<double>>("upper");
      if(lower.size()!=14||upper.size()!=14) throw std::runtime_error("14 joint bounds required");
      for(int j=0;j<14;++j)if(!std::isfinite(lower[j])||!std::isfinite(upper[j])||lower[j]>=upper[j])throw std::runtime_error("Invalid bounds");
      std::copy(lower.begin(),lower.end(),lower_.begin());std::copy(upper.begin(),upper.end(),upper_.begin());
      std::ifstream input(declare_parameter<std::string>("plan_file"));std::string line;std::getline(input,line);
      while(std::getline(input,line)) {
        auto row=vendor::csv(line);if(row.size()!=29||row[0]!=static_cast<double>(plan_.size())*period_) throw std::runtime_error("Invalid uniform position plan");
        for(int j=0;j<14;++j) if(row[j+1]<lower_[j]||row[j+1]>upper_[j]||std::abs(row[j+15])>max_velocity_) throw std::runtime_error("Plan exceeds configured bounds");
        plan_.push_back(row);
      }
      if(plan_.size()<2) throw std::runtime_error("Missing plan");
      for(int j=0;j<14;++j) if(std::abs(plan_.back()[j+1]-plan_[0][j+1])>1e-12) throw std::runtime_error("Plan must return to observed start");
      // Publisher and service clients are created only after local scope / plan validation.
      pub_=create_publisher<crb_ros_msg::msg::UpperJointData>(command,rclcpp::QoS(1).reliable());
      mode_client_=create_client<std_srvs::srv::SetBool>(service);stop_client_=create_client<std_srvs::srv::SetBool>(stop);
    }
    began_=vendor::now(); phase_at_=began_;event("WAIT_FEEDBACK","startup");
  }
  bool done() const {return done_;} bool success() const {return success_;}
  void tick() {
    const auto now=vendor::now();
    if(done_) return;
    if(interrupted && phase_!="STOPPING"&&phase_!="FAILED") fault("interrupted");
    if(phase_=="STOPPING") {
      if(stop_future_.valid() && stop_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
        stop_ack_=stop_future_.get()->success;stop_future_={};
      }
      if(stop_ack_ && fresh(now) && speed()<.02) {
        if(!stable_at_) stable_at_=now;
        if(now-stable_at_>=100000000) finish(false,"stop_ack_and_observed_rest");
      } else stable_at_=0;
      if(now-phase_at_>timeout_) finish(false,"stop_unconfirmed");
      return;
    }
    if(phase_=="WAIT_FEEDBACK") {
      if(fresh(now)&&speed()<.02&&now-mode_at_<mode_age_) {
        if(snapshot_) {save_snapshot();finish(true,"snapshot_only");return;}
        if(mode_!="mode: READY, state: READY"&&mode_!="mode: IDLE_MODE, state: STAND") {fault("unexpected_start_mode");return;}
        for(int j=0;j<14;++j) if(std::abs(state_.q[j]-plan_[0][j+1])>tolerance_) {fault("start_pose_changed");return;}
        if(!mode_client_->service_is_ready()||!stop_client_->service_is_ready()) {fault("required_service_unavailable");return;}
        mode_future_=request(mode_client_,true); event("ENTERING","request_upper_debug");
      } else if(now-began_>startup_timeout_) fault("initial_feedback_or_mode_timeout");
      return;
    }
    if(!fresh(now)) {fault("stale_feedback");return;}
    if(now-mode_at_>mode_age_) {fault("stale_mode");return;}
    for(int j=0;j<14;++j) if(state_.q[j]<lower_[j]||state_.q[j]>upper_[j]||std::abs(state_.v[j])>max_velocity_) {fault("observed_limit");return;}
    if(phase_=="ENTERING") {
      if(mode_future_.valid()&&mode_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
        mode_ack_=mode_future_.get()->success; mode_future_={};
        if(!mode_ack_) {fault("mode_rejected");return;}
      }
      if(mode_ack_&&mode_=="mode: UPPER_BODY_DEBUG, state: STAND") {start_=now;event("STREAMING","mode_confirmed");}
      else if(now-phase_at_>timeout_) fault("mode_transition_timeout");
      return;
    }
    if(phase_=="EXITING") {
      if(mode_future_.valid()&&mode_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
        exit_ack_=mode_future_.get()->success;mode_future_={};if(!exit_ack_) {fault("exit_rejected");return;}
      }
      if(exit_ack_&&mode_=="mode: READY, state: READY") finish(true,"completed_at_observed_start");
      else if(now-phase_at_>timeout_) fault("exit_timeout");
      return;
    }
    if(now-last_graph_check_>20000000) {
      last_graph_check_=now;
      if(count_publishers(command_topic_)>1) {fault("competing_command_publisher");return;}
    }
    if(mode_!="mode: UPPER_BODY_DEBUG, state: STAND") {fault("control_mode_lost");return;}
    const auto index=std::min<std::int64_t>((now-start_)/period_,plan_.size()-1);
    if(index<=last_index_ && phase_=="STREAMING") return;
    const auto& row=plan_[index];
    // Compare with the reference at callback availability, not a later timer
    // instant. Header sampling time is still unknown. Bound subsequent reference
    // motion separately inside the declared tube; do not invent sample alignment.
    const auto available_index=std::clamp<std::int64_t>((state_.receive-start_)/period_,0,plan_.size()-1);
    for(int j=0;j<14;++j) {
      const double error=std::abs(state_.q[j]-plan_[available_index][j+1]);
      if(error>tracking_) {fault("tracking_envelope");return;}
      if(error+std::abs(row[j+1]-plan_[available_index][j+1])>allowance_) {fault("aged_tracking_envelope");return;}
    }
    if(phase_=="STREAMING") {
      // Late cycles skip planned slots. Never replay a burst of old commands.
      if(index>last_index_+1) skips_+=index-last_index_-1;
      publish(row,index,"stream");last_index_=index;
      if(index==static_cast<std::int64_t>(plan_.size()-1)) event("SETTLING","final_hold");
    } else if(phase_=="SETTLING") {
      if(now-last_publish_>=period_) publish(plan_.back(),index,"settle");
      double error=0;for(int j=0;j<14;++j) error=std::max(error,std::abs(state_.q[j]-plan_[0][j+1]));
      if(error<tolerance_&&speed()<.02) {if(!stable_at_) stable_at_=now;} else stable_at_=0;
      if(stable_at_&&now-stable_at_>150000000) {mode_future_=request(mode_client_,false);event("EXITING","verified_rest_then_exit");}
      else if(now-phase_at_>timeout_*2) fault("settling_timeout");
    }
  }
  void finish(bool ok,const std::string& reason) {
    success_=ok;done_=true;event(ok?"COMPLETE":"FAILED",reason);commands_.flush();received_.flush();events_.flush();
    std::ofstream out(root_/"result.json");out<<"{\"passed\":"<<(ok?"true":"false")<<",\"reason\":"<<std::quoted(reason)
      <<",\"last_feedback_receive_ns\":"<<state_.receive<<",\"last_mode_receive_ns\":"<<mode_at_<<",\"last_mode\":"<<std::quoted(mode_)
      <<",\"fault\":"<<std::quoted(fault_)<<",\"skipped_slots\":"<<skips_<<",\"stop_ack\":"<<(stop_ack_?"true":"false")
      <<",\"snapshot_only\":"<<(snapshot_?"true":"false")<<",\"hardware_executable\":false,\"clock_id\":"<<std::quoted(vendor::clock_id())<<"}\n";
  }
 private:
  using Client=rclcpp::Client<std_srvs::srv::SetBool>;
  Client::SharedFuture request(const Client::SharedPtr& cli,bool value) {auto r=std::make_shared<std_srvs::srv::SetBool::Request>();r->data=value;return cli->async_send_request(r).share();}
  bool fresh(std::int64_t now) const {return state_.receive>0&&now-state_.receive<=max_age_;}
  double speed() const {double v=0;for(double x:state_.v)v=std::max(v,std::abs(x));return v;}
  bool active() const {return phase_!="WAIT_FEEDBACK"&&phase_!="STOPPING"&&!done_;}
  void event(const std::string& phase,const std::string& reason) {phase_=phase;phase_at_=vendor::now();stable_at_=0;events_<<phase_at_<<','<<phase<<','<<reason<<'\n';events_.flush();}
  void fault(const std::string& reason) {
    if(done_||phase_=="STOPPING")return;
    fault_=reason;
    if(snapshot_||(!mode_ack_ && phase_!="ENTERING")) {finish(false,reason);return;}
    event("STOPPING",reason);
    if(stop_client_&&stop_client_->service_is_ready())stop_future_=request(stop_client_,true);
  }
  void publish(const std::vector<double>& row,std::int64_t index,const std::string& phase) {
    crb_ros_msg::msg::UpperJointData msg;msg.header.stamp=this->now();msg.name=names_;msg.time_ref=0;msg.vel_scale=vel_scale_;
    msg.position.assign(row.begin()+1,row.begin()+15);msg.velocity.assign(row.begin()+15,row.end());msg.effort.assign(14,0.);
    last_publish_=vendor::now();pub_->publish(msg);commands_<<index<<','<<start_+index*period_<<','<<last_publish_<<','<<phase;
    vendor::values(commands_,msg.position);vendor::values(commands_,msg.velocity);commands_<<'\n';
  }
  void save_snapshot() {
    auto out=vendor::log(root_/"snapshot.csv","receive_ns"+vendor::columns("q")+vendor::columns("v"));
    out<<state_.receive;vendor::values(out,state_.q);vendor::values(out,state_.v);out<<'\n';
  }
  std::filesystem::path root_;std::vector<std::string> names_;vendor::State state_;
  std::array<double,14> lower_{},upper_{};std::vector<std::vector<double>> plan_;
  std::ofstream events_,received_,commands_;
  std::string phase_,mode_,fault_,command_topic_;std::int64_t began_=0,phase_at_=0,mode_at_=0,start_=0,last_index_=-1,last_publish_=0,stable_at_=0,skips_=0,last_graph_check_=0;
  std::int64_t max_age_,mode_age_,timeout_,period_,startup_timeout_;double tolerance_,tracking_,allowance_,max_velocity_,vel_scale_;
  bool snapshot_=false,done_=false,success_=false,mode_ack_=false,exit_ack_=false,stop_ack_=false;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<crb_ros_msg::msg::UpperJointData>::SharedPtr pub_;Client::SharedPtr mode_client_,stop_client_;
  Client::SharedFuture mode_future_,stop_future_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);std::signal(SIGINT,on_signal);std::signal(SIGTERM,on_signal);
  try {auto node=std::make_shared<PositionExecutor>();rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(node);
    while(rclcpp::ok()&&!node->done()) {exec.spin_once(std::chrono::microseconds(250));node->tick();}
    bool ok=node->success();node.reset();rclcpp::shutdown();return ok?0:1;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 2;}
}
