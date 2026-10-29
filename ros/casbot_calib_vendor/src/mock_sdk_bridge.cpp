// SDK protocol + declared synthetic position servo. mock_host owns all stepping.
#include "vendor_common.hpp"
#include <casbot_calib/arm_dynamics.hpp>
#include <crb_ros_msg/msg/upper_joint_data.hpp>
#include <casbot_calib_interfaces/msg/joint_command.hpp>
#include <casbot_calib_interfaces/msg/joint_observation.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

class MockBridge final : public rclcpp::Node {
 public:
  MockBridge():Node("casbot_mock_sdk_bridge") {
    root_=declare_parameter<std::string>("endpoint_root");
    vendor::local_topics({root_});
    names_=declare_parameter<std::vector<std::string>>("joint_names");
    model_id_=declare_parameter<std::string>("model_id");experiment_=declare_parameter<std::string>("experiment_id");
    fault_=declare_parameter<std::string>("fault","");
    if(names_.size()!=14)throw std::runtime_error("14 axis names required");
    kp_=declare_parameter<std::vector<double>>("kp",std::vector<double>(14,100.));
    kd_=declare_parameter<std::vector<double>>("kd",std::vector<double>(14,14.));
    if(kp_.size()!=14||kd_.size()!=14)throw std::runtime_error("14 PD gains required");
    auto model=casbot::ArmDynamics::from_urdf(declare_parameter<std::string>("model_path")).model();
    model.gravity_world.setZero();engine_=std::make_unique<casbot::ArmDynamics>(model);
    workspace_=std::make_unique<casbot::ArmWorkspace>(engine_->workspace());
    const auto output=declare_parameter<std::string>("output");
    events_=vendor::log(std::filesystem::path(output)/"bridge_events.csv","receive_ns,event");
    incoming_=vendor::log(std::filesystem::path(output)/"bridge_commands.csv","receive_ns"+vendor::columns("q")+vendor::columns("v"));
    native_=create_publisher<casbot_calib_interfaces::msg::JointCommand>(root_+"/plant/commands",rclcpp::QoS(1).reliable());
    feedback_=create_publisher<sensor_msgs::msg::JointState>(root_+"/joint_states",rclcpp::QoS(10).reliable());
    mode_=create_publisher<std_msgs::msg::String>(root_+"/status",rclcpp::QoS(10).reliable());
    observed_=create_subscription<casbot_calib_interfaces::msg::JointObservation>(root_+"/plant/observations",rclcpp::QoS(1).reliable(),
      [this](casbot_calib_interfaces::msg::JointObservation::SharedPtr msg) {
        if(msg->model_id!=model_id_||msg->experiment_id!=experiment_||std::vector<std::string>(msg->joint_names.begin(),msg->joint_names.end())!=names_)return;
        for(int j=0;j<14;++j){q_[j]=msg->position[j];v_[j]=msg->velocity[j];effort_[j]=msg->estimated_effort[j];}
        if(!have_state_){target_=q_;have_state_=true;}state_at_=vendor::now();
      });
    upper_=create_subscription<crb_ros_msg::msg::UpperJointData>(root_+"/upper_cmd",rclcpp::QoS(1).reliable(),
      [this](crb_ros_msg::msg::UpperJointData::SharedPtr msg) {
        if(!active_||!have_state_||msg->name!=names_||msg->position.size()!=14||msg->velocity.size()!=14||msg->effort.size()!=14||!msg->kp.empty()||!msg->kd.empty()) {record("rejected_upper_command");return;}
        if(msg->time_ref!=0||msg->vel_scale<=0||msg->vel_scale>1) {record("rejected_upper_semantics");return;}
        for(int j=0;j<14;++j)if(!std::isfinite(msg->position[j])||!std::isfinite(msg->velocity[j])||msg->effort[j]!=0) {record("rejected_upper_value");return;}
        std::copy(msg->position.begin(),msg->position.end(),target_.begin());std::copy(msg->velocity.begin(),msg->velocity.end(),target_v_.begin());
        last_command_=vendor::now();incoming_<<last_command_;vendor::values(incoming_,target_);vendor::values(incoming_,target_v_);incoming_<<'\n';
      });
    modes_=create_service<std_srvs::srv::SetBool>(root_+"/upper_debug",[this](const std_srvs::srv::SetBool::Request::SharedPtr req,std_srvs::srv::SetBool::Response::SharedPtr resp) {
      if(req->data) {
        if(fault_=="reject_mode") {resp->success=false;resp->message="injected rejection";record("mode_rejected");return;}
        if(!have_state_||active_){resp->success=false;return;}
        active_since_=last_command_=vendor::now();active_=fault_!="mode_no_confirm";target_=q_;target_v_.fill(0.);
        record("enter_requested");
      } else {active_=false;target_=q_;target_v_.fill(0.);record("exit_after_hold");}
      resp->success=true;
    });
    stop_=create_service<std_srvs::srv::SetBool>(root_+"/stop",[this](const std_srvs::srv::SetBool::Request::SharedPtr req,std_srvs::srv::SetBool::Response::SharedPtr resp) {
      if(fault_=="stop_rejected"){record("stop_rejected");resp->success=false;return;}
      if(req->data&&have_state_){active_=false;target_=q_;target_v_.fill(0.);record("stop_hold_current_pose");resp->success=true;}
    });
    tick_=create_wall_timer(std::chrono::milliseconds(2),[this]{tick();});
    status_=create_wall_timer(std::chrono::milliseconds(50),[this] {
      std_msgs::msg::String msg;msg.data=takeover_?"mode: TELEOPERATION, state: STAND":active_?"mode: UPPER_BODY_DEBUG, state: STAND":"mode: READY, state: READY";mode_->publish(msg);
    });
  }
 private:
  void record(const std::string& event){events_<<vendor::now()<<','<<event<<'\n';events_.flush();}
  void tick() {
    const auto now=vendor::now(); if(!have_state_)return;
    if(active_&&now-last_command_>100000000) {active_=false;target_=q_;target_v_.fill(0.);record("mock_watchdog_hold");}
    const auto since=active_since_?now-active_since_:0;
    if((fault_=="mode_loss"||fault_=="stop_rejected")&&since>600000000&&!takeover_){takeover_=true;active_=false;target_=q_;target_v_.fill(0.);record("injected_takeover");}
    casbot_calib_interfaces::msg::JointCommand cmd;cmd.experiment_id=experiment_;cmd.model_id=model_id_;cmd.clock_id=clock_;
    cmd.sequence=++sequence_;cmd.send_ns=now;cmd.valid_for_ns=100000000;cmd.mode=cmd.TORQUE;
    Eigen::Map<const Eigen::VectorXd> q(q_.data(),14),v(v_.data(),14);Eigen::VectorXd desired_acceleration(14);
    for(int j=0;j<14;++j)desired_acceleration[j]=kp_[j]*(target_[j]-q_[j])+kd_[j]*(target_v_[j]-v_[j]);
    auto torque=workspace_->inverse(q,v,desired_acceleration);
    for(int j=0;j<14;++j){cmd.joint_names[j]=names_[j];cmd.effort[j]=torque[j];}
    native_->publish(cmd);
    // A stale native state is never republished as fresh SDK feedback.
    if(now-state_at_>20000000)return;
    if(fault_=="feedback_gap"&&since>600000000&&since<900000000)return;
    sensor_msgs::msg::JointState msg;msg.header.stamp=this->now();msg.name=names_;
    msg.position.assign(q_.begin(),q_.end());msg.velocity.assign(v_.begin(),v_.end());msg.effort.assign(effort_.begin(),effort_.end());
    // Add an unchanged waist to ensure the client never commands it by padding.
    msg.name.push_back("waist_yaw_joint");msg.position.push_back(.17);msg.velocity.push_back(0);msg.effort.push_back(0);
    if(fault_=="invalid_feedback"&&since>600000000&&since<800000000)msg.name[0]=msg.name[1];
    feedback_->publish(msg);
  }
  std::string root_,model_id_,experiment_,fault_,clock_=vendor::clock_id();std::vector<std::string> names_;std::vector<double> kp_,kd_;
  std::array<double,14> q_{},v_{},effort_{},target_{},target_v_{};
  bool have_state_=false,active_=false,takeover_=false;std::int64_t state_at_=0,active_since_=0,last_command_=0;std::uint64_t sequence_=0;
  std::ofstream events_,incoming_;
  std::unique_ptr<casbot::ArmDynamics> engine_;std::unique_ptr<casbot::ArmWorkspace> workspace_;
  rclcpp::Publisher<casbot_calib_interfaces::msg::JointCommand>::SharedPtr native_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_;
  rclcpp::Subscription<casbot_calib_interfaces::msg::JointObservation>::SharedPtr observed_;
  rclcpp::Subscription<crb_ros_msg::msg::UpperJointData>::SharedPtr upper_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr modes_,stop_;
  rclcpp::TimerBase::SharedPtr tick_,status_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{auto node=std::make_shared<MockBridge>();rclcpp::spin(node);node.reset();rclcpp::shutdown();return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}}
