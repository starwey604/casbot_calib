// SDK-shaped server; mock_host remains the sole ArmSimulationSession owner.
#include "vendor_common.hpp"
#include "sdk_position_ramp.hpp"
#include <casbot_calib/arm_dynamics.hpp>
#include <casbot_calib_interfaces/msg/joint_command.hpp>
#include <casbot_calib_interfaces/msg/joint_observation.hpp>
#include <crb_ros_msg/msg/upper_joint_data.hpp>
#include <crb_ros_msg/msg/joint_state_data.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <limits>

class SdkMockServer final:public rclcpp::Node {
 public:
  SdkMockServer():Node("casbot_sdk_mock_server") {
    root_=declare_parameter<std::string>("endpoint_root");vendor::local_topics({root_});
    names_=declare_parameter<std::vector<std::string>>("joint_names");
    model_id_=declare_parameter<std::string>("model_id");experiment_=declare_parameter<std::string>("experiment_id");
    if(names_.size()!=14||std::set<std::string>(names_.begin(),names_.end()).size()!=14)throw std::runtime_error("14 unique joint names required");
    const auto path=declare_parameter<std::string>("model_path"),output=declare_parameter<std::string>("output");
    engine_=std::make_unique<casbot::ArmDynamics>(casbot::ArmDynamics::from_urdf(path));
    if(engine_->model().joints.size()!=14)throw std::runtime_error("SDK Mock requires14 model joints");
    workspace_=std::make_unique<casbot::ArmWorkspace>(engine_->workspace());
    kp_=gains("kp",1000.);kd_=gains("kd",100.);
    for(int j=0;j<14;++j)if(engine_->model().joints[j].name!=names_[j])throw std::runtime_error("Mock model joint order mismatch");
    const auto control_hz=declare_parameter<int>("control_hz",1000),publish_hz=declare_parameter<int>("publish_hz",1000);
    cache_period_=declare_parameter<std::int64_t>("cache_period_ns",3000000);
    watchdog_=declare_parameter<std::int64_t>("watchdog_ns",100000000);
    const auto horizon=declare_parameter<std::int64_t>("interpolation_ns",2000000);
    gravity_=declare_parameter<bool>("internal_gravity",false);
    allow_effort_=declare_parameter<bool>("allow_effort",false);allow_pd_=declare_parameter<bool>("allow_custom_pd",false);
    effort_limit_=declare_parameter<double>("effort_limit_nm",100.);
    if(control_hz<100||control_hz>2000||publish_hz<10||publish_hz>2000||cache_period_<1000000||watchdog_<10000000||
       horizon<1000000||!std::isfinite(effort_limit_)||effort_limit_<=0||effort_limit_>100)throw std::runtime_error("Invalid Mock timing/limit");
    ramp_=std::make_unique<SdkPositionRamp>(horizon);
    // These are Mock fault controls only; no matching hardware capability is claimed.
    declare_parameter<bool>("feedback_enabled",true);declare_parameter<bool>("reject_mode",false);
    declare_parameter<bool>("lose_mode",false);declare_parameter<int>("unstable_static_pairs",0);
    declare_parameter<double>("unstable_q0",.006);
    events_=vendor::log(std::filesystem::path(output)/"server_events.csv","receive_ns,event");
    commands_=vendor::log(std::filesystem::path(output)/"server_commands.csv","receive_ns"+vendor::columns("q")+vendor::columns("v")+vendor::columns("effort"));
    references_=vendor::log(std::filesystem::path(output)/"server_references.csv","tick_ns,active"+vendor::columns("q")+vendor::columns("v")+vendor::columns("effort"));
    publications_=vendor::log(std::filesystem::path(output)/"server_publications.csv","publish_ns,cache_sequence,cache_sample_ns,cache_driver_receive_ns,source_publication_sequence,duplicate_cache");
    native_=create_publisher<casbot_calib_interfaces::msg::JointCommand>(root_+"/plant/commands",rclcpp::QoS(1).reliable());
    feedback_=create_publisher<sensor_msgs::msg::JointState>(root_+"/joint_states",rclcpp::QoS(10).reliable());
    motion_feedback_=create_publisher<sensor_msgs::msg::JointState>(root_+"/motion/joint_state",rclcpp::QoS(10).reliable());
    debug_=create_publisher<crb_ros_msg::msg::JointStateData>(root_+"/motion/debug/joint_state",rclcpp::QoS(10).reliable());
    mode_=create_publisher<std_msgs::msg::String>(root_+"/motion/status",rclcpp::QoS(10).reliable());
    observed_=create_subscription<casbot_calib_interfaces::msg::JointObservation>(root_+"/plant/observations",rclcpp::QoS(1).reliable(),
      [this](casbot_calib_interfaces::msg::JointObservation::SharedPtr msg) {
        if(msg->model_id!=model_id_||msg->experiment_id!=experiment_||std::vector<std::string>(msg->joint_names.begin(),msg->joint_names.end())!=names_)return;
        latest_=*msg;state_at_=vendor::now();
        for(int j=0;j<14;++j){q_[j]=msg->position[j];v_[j]=msg->velocity[j];}
        if(!have_state_){ramp_->reset(q_,state_at_);have_state_=true;}
      });
    upper_=create_subscription<crb_ros_msg::msg::UpperJointData>(root_+"/upper_body_debug/joint_cmd",rclcpp::QoS(1).reliable(),
      [this](crb_ros_msg::msg::UpperJointData::SharedPtr msg){command(*msg);});
    modes_=create_service<std_srvs::srv::SetBool>(root_+"/motion/upper_body_debug",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,std_srvs::srv::SetBool::Response::SharedPtr resp){
        if(req->data&&(!have_state_||active_||get_parameter("reject_mode").as_bool())){resp->success=false;record("mode_rejected");return;}
        if(!req->data&&inject_pair_) {auto left=get_parameter("unstable_static_pairs").as_int();
          set_parameter(rclcpp::Parameter("unstable_static_pairs",std::max<std::int64_t>(0,left-1)));}
        active_=req->data;awaiting_first_command_=active_;inject_pair_=active_&&get_parameter("unstable_static_pairs").as_int()>0;
        ramp_->reset(q_,vendor::now());target_v_.fill(0);feedforward_.fill(0);custom_pd_=false;
        last_command_=vendor::now();record(active_?"enter_requested":"exit_after_hold");resp->success=true;
      });
    control_=create_wall_timer(std::chrono::nanoseconds(1000000000/control_hz),[this]{tick();});
    publish_=create_wall_timer(std::chrono::nanoseconds(1000000000/publish_hz),[this]{publish();});
    status_=create_wall_timer(std::chrono::milliseconds(20),[this]{
      std_msgs::msg::String msg;msg.data=active_?"mode: UPPER_BODY_DEBUG, state: STAND":"mode: READY, state: READY";mode_->publish(msg);
      events_.flush();commands_.flush();references_.flush();publications_.flush();
    });
    record("ready_position_only_linear_ramp_assumed_no_stop_service");
  }
 private:
  std::vector<double> gains(const std::string& name,double maximum) {
    auto values=declare_parameter<std::vector<double>>(name);
    if(values.size()!=14)throw std::runtime_error("14 gains required");
    for(auto x:values)if(!std::isfinite(x)||x<0||x>maximum)throw std::runtime_error("Invalid synthetic gain");
    return values;
  }
  void record(const std::string& what){events_<<vendor::now()<<','<<what<<'\n';events_.flush();}
  void hold(const std::string& reason) {
    active_=false;ramp_->reset(q_,vendor::now());target_v_.fill(0);feedforward_.fill(0);custom_pd_=false;record(reason);
  }
  void command(const crb_ros_msg::msg::UpperJointData& msg) {
    if(!active_||!have_state_||msg.name!=names_||msg.position.size()!=14||msg.velocity.size()!=14||msg.effort.size()!=14||
       msg.time_ref!=0||!std::isfinite(msg.vel_scale)||msg.vel_scale<=0||msg.vel_scale>1||
       ((!msg.kp.empty()||!msg.kd.empty())&&(!allow_pd_||msg.kp.size()!=14||msg.kd.size()!=14))) {record("rejected_command_shape_or_semantics");return;}
    for(int j=0;j<14;++j)if(!std::isfinite(msg.position[j])||!std::isfinite(msg.velocity[j])||!std::isfinite(msg.effort[j])||
       std::abs(msg.effort[j])>effort_limit_||(!allow_effort_&&msg.effort[j]!=0)||
       (!msg.kp.empty()&&(!std::isfinite(msg.kp[j])||!std::isfinite(msg.kd[j])||msg.kp[j]<0||msg.kp[j]>1000||msg.kd[j]<0||msg.kd[j]>100))) {
      record("rejected_command_value");return;}
    SdkPositionRamp::Values target{};std::copy(msg.position.begin(),msg.position.end(),target.begin());
    const auto now=vendor::now();ramp_->target(target,now);last_command_=now;
    awaiting_first_command_=false;
    std::copy(msg.velocity.begin(),msg.velocity.end(),target_v_.begin());std::copy(msg.effort.begin(),msg.effort.end(),feedforward_.begin());
    custom_pd_=!msg.kp.empty();if(custom_pd_){custom_kp_=msg.kp;custom_kd_=msg.kd;}
    commands_<<now;vendor::values(commands_,target);vendor::values(commands_,target_v_);vendor::values(commands_,feedforward_);commands_<<'\n';
  }
  void tick() {
    if(!have_state_)return;
    const auto now=vendor::now();
    if(active_&&now-last_command_>watchdog_)hold("assumed_watchdog_hold");
    if(active_&&get_parameter("lose_mode").as_bool())hold("injected_mode_loss");
    if(now-state_at_>20000000)return; // Do not drive from an unboundedly old plant state.
    if(!initial_ready_) {
      double speed=0;for(auto value:v_)speed=std::max(speed,std::abs(value));
      if(speed<.003) {
        if(!initial_stable_at_)initial_stable_at_=now;
        if(now-initial_stable_at_>=500000000){initial_ready_=true;record("settled_initial_hold");}
      } else initial_stable_at_=0;
    }
    const auto reference=ramp_->at(now);
    casbot_calib_interfaces::msg::JointCommand cmd;cmd.experiment_id=experiment_;cmd.model_id=model_id_;cmd.clock_id=clock_;
    cmd.sequence=++sequence_;cmd.send_ns=now;cmd.valid_for_ns=100000000;cmd.mode=cmd.POSITION_PD;
    Eigen::Map<const Eigen::VectorXd> q(q_.data(),14);
    // Inactive protection hold is synthetic, distinct from active SDK commands.
    const Eigen::VectorXd g=(!active_||awaiting_first_command_||gravity_)?workspace_->gravity(q):Eigen::VectorXd::Zero(14);
    for(int j=0;j<14;++j) {
      const auto p=custom_pd_?custom_kp_[j]:kp_[j],d=custom_pd_?custom_kd_[j]:kd_[j];
      cmd.joint_names[j]=names_[j];
      cmd.position[j]=reference[j];cmd.velocity[j]=target_v_[j];cmd.kp[j]=p;cmd.kd[j]=d;
      cmd.effort[j]=std::clamp(g[j]+feedforward_[j],-effort_limit_,effort_limit_);
    }
    native_->publish(cmd);
    references_<<now<<','<<active_;vendor::values(references_,reference);vendor::values(references_,target_v_);vendor::values(references_,feedforward_);references_<<'\n';
    if(!have_cache_||now-cache_at_>=cache_period_){cache_=latest_;cache_at_=now;have_cache_=true;}
  }
  void publish() {
    const auto now=vendor::now();
    if(!have_cache_||now-state_at_>20000000||!get_parameter("feedback_enabled").as_bool())return;
    sensor_msgs::msg::JointState msg;msg.header.stamp=this->now();msg.name=names_;
    msg.position.assign(cache_.position.begin(),cache_.position.end());msg.velocity.assign(cache_.velocity.begin(),cache_.velocity.end());
    msg.effort.assign(cache_.estimated_effort.begin(),cache_.estimated_effort.end());
    if(inject_pair_&&std::abs(msg.position[0]-get_parameter("unstable_q0").as_double())<.0005&&std::abs(msg.velocity[0])<.01)msg.velocity[0]=.02;
    msg.name.push_back("waist_yaw_joint");msg.position.push_back(.17);msg.velocity.push_back(0);msg.effort.push_back(0);
    feedback_->publish(msg);motion_feedback_->publish(msg);
    crb_ros_msg::msg::JointStateData debug;debug.header=msg.header;debug.name=msg.name;debug.position=msg.position;debug.velocity=msg.velocity;debug.effort=msg.effort;
    debug.kp=custom_pd_?custom_kp_:kp_;debug.kd=custom_pd_?custom_kd_:kd_;debug.kp.push_back(0);debug.kd.push_back(0);debug_->publish(debug);
    publications_<<now<<','<<cache_.sequence<<','<<cache_.sample_ns<<','<<cache_.driver_receive_ns<<','<<cache_.publish_sequence<<','<<(previous_cache_==cache_.sequence)<<'\n';
    previous_cache_=cache_.sequence;
  }
  std::string root_,model_id_,experiment_,clock_=vendor::clock_id();std::vector<std::string> names_;
  std::vector<double> kp_,kd_,custom_kp_,custom_kd_;std::array<double,14> q_{},v_{},target_v_{},feedforward_{};
  std::int64_t state_at_=0,cache_at_=0,last_command_=0,cache_period_=0,watchdog_=0,initial_stable_at_=0;
  std::uint64_t sequence_=0,previous_cache_=std::numeric_limits<std::uint64_t>::max();
  bool have_state_=false,have_cache_=false,active_=false,inject_pair_=false,custom_pd_=false,gravity_=false,allow_effort_=false,allow_pd_=false,initial_ready_=false,awaiting_first_command_=false;
  double effort_limit_=100.;
  casbot_calib_interfaces::msg::JointObservation latest_,cache_;
  std::unique_ptr<SdkPositionRamp> ramp_;std::unique_ptr<casbot::ArmDynamics> engine_;std::unique_ptr<casbot::ArmWorkspace> workspace_;
  std::ofstream events_,commands_,references_,publications_;
  rclcpp::Publisher<casbot_calib_interfaces::msg::JointCommand>::SharedPtr native_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_,motion_feedback_;
  rclcpp::Publisher<crb_ros_msg::msg::JointStateData>::SharedPtr debug_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_;
  rclcpp::Subscription<casbot_calib_interfaces::msg::JointObservation>::SharedPtr observed_;
  rclcpp::Subscription<crb_ros_msg::msg::UpperJointData>::SharedPtr upper_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr modes_;
  rclcpp::TimerBase::SharedPtr control_,publish_,status_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{auto node=std::make_shared<SdkMockServer>();rclcpp::spin(node);node.reset();rclcpp::shutdown();return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}}
