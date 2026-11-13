#include "file_stamp.hpp"
#include "trical_stream.hpp"
#include "async_csv.hpp"
#include "manual_policy.hpp"
#include "legacy_position_guard.hpp"
// Shared S2/field position stream. Default S2 remains private/localhost only.
#include "vendor_common.hpp"
#include "sdk_breakaway.hpp"
#include <casbot_calib/arm_dynamics.hpp>
#include <crb_ros_msg/msg/upper_joint_data.hpp>
#include <crb_ros_msg/msg/joint_state_data.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <csignal>
#include <limits>
#include <iostream>
#ifdef __linux__
#include <sys/prctl.h>
#include <unistd.h>
#endif

namespace {volatile std::sig_atomic_t interrupted=0; void on_signal(int) {interrupted=1;}}
class PositionExecutor final : public rclcpp::Node {
  struct Observation {std::string joint,field;double value,threshold;std::string unit;};
 public:
  PositionExecutor():Node("casbot_vendor_position_executor",rclcpp::NodeOptions()
      .enable_rosout(false).start_parameter_services(false).start_parameter_event_publisher(false)) {
    const auto output=declare_parameter<std::string>("output");
    names_=declare_parameter<std::vector<std::string>>("joint_names");
    snapshot_=declare_parameter<bool>("snapshot_only",false);
    manual_=declare_parameter<bool>("manual_stream",false);
    trical_file_=declare_parameter<std::string>("trical_stream_file","");
    if(!trical_file_.empty()) {
      if(manual_)throw std::runtime_error("TriCal and manual sources are exclusive");
      manual_=true;
    }
    recording_pause_file_=declare_parameter<std::string>("recording_pause_file","");
    if(manual_) {
      if(snapshot_)throw std::runtime_error("Manual stream is not a snapshot");
      manual_file_=trical_file_.empty()?declare_parameter<std::string>("manual_command_file"):trical_file_;
      policy_file_=declare_parameter<std::string>("manual_policy_file","");
      manual_effort_limit_=declare_parameter<double>("manual_effort_limit_nm",10.);
      if(!std::isfinite(manual_effort_limit_)||manual_effort_limit_<=0||(policy_file_.empty()&&manual_effort_limit_>100.))throw std::runtime_error("Invalid manual effort range");
    }
    const bool ramp_enabled=declare_parameter<bool>("breakaway_enabled",false);
    const bool gravity_enabled=declare_parameter<bool>("gravity_feedforward_enabled",false);
    const auto feedforward_path=declare_parameter<std::string>("feedforward_plan_file","");
    const double feedforward_limit=declare_parameter<double>("feedforward_limit_nm",100.);
    hold_kp_=declare_parameter<std::vector<double>>("command_kp",std::vector<double>{});hold_kd_=declare_parameter<std::vector<double>>("command_kd",std::vector<double>{});
    experiment_commands_=manual_||ramp_enabled||gravity_enabled||!feedforward_path.empty()||!hold_kp_.empty()||!hold_kd_.empty();
    if(manual_&&trical_file_.empty()&&(ramp_enabled||gravity_enabled||!feedforward_path.empty()))throw std::runtime_error("Manual fields must not include hidden compensation");
    if(!trical_file_.empty()&&(ramp_enabled||!feedforward_path.empty()||!policy_file_.empty()||hold_kp_.size()!=14||hold_kd_.size()!=14))
      throw std::runtime_error("TriCal needs 14 explicit PD gains and an exclusive trajectory source");
    const auto joint=declare_parameter<std::string>("joint_topic"),mode=declare_parameter<std::string>("mode_topic");
    const auto command=declare_parameter<std::string>("command_topic"),service=declare_parameter<std::string>("mode_service"),stop=declare_parameter<std::string>("stop_service");
    command_topic_=command;
    scope_=declare_parameter<std::string>("execution_scope","localhost_contract_mock");
    if(scope_=="localhost_contract_mock") vendor::local_topics({joint,mode,command,service,stop});
    else if(scope_!="hardware_position_stream") throw std::runtime_error("Unknown execution scope");
    hardware_=scope_=="hardware_position_stream";
    if(!trical_file_.empty()&&(!hardware_||!std::getenv("ROS_LOCALHOST_ONLY")||std::string(std::getenv("ROS_LOCALHOST_ONLY"))!="1"||
       get_node_topics_interface()->resolve_topic_name(command).rfind("/casbot_calib_s2_",0)!=0))
      throw std::runtime_error("TriCal development stream requires private localhost SDK Mock");
    if(!trical_file_.empty()) {
      const auto resolved=get_node_topics_interface()->resolve_topic_name(command);
      const auto prefix=resolved.substr(0,resolved.find('/',1))+"/";
      for(const auto& topic:{joint,mode})if(get_node_topics_interface()->resolve_topic_name(topic).rfind(prefix,0)!=0)
        throw std::runtime_error("TriCal feedback and command must use the same private Mock");
      if(get_node_services_interface()->resolve_service_name(service).rfind(prefix,0)!=0)
        throw std::runtime_error("TriCal mode service must use the same private Mock");
    }
    #ifdef __linux__
    if(hardware_) {
      const auto parent=getppid();
      if(parent<=1||prctl(PR_SET_PDEATHSIG,SIGTERM)!=0||getppid()!=parent)
        throw std::runtime_error("Field publisher requires a live supervising process");
    }
    #endif
    if(hardware_ && (command!="/upper_body_debug/joint_cmd"||service!="/motion/upper_body_debug"||!stop.empty()))
      throw std::runtime_error("Field profile requires SDK upper-body endpoints and no invented stop service");
    const auto feedback_type=declare_parameter<std::string>("feedback_type","sensor_msgs/msg/JointState");
    if (names_.size()!=14||std::set<std::string>(names_.begin(),names_.end()).size()!=14) throw std::runtime_error("Expected 14 unique joint names");
    for (const auto& n:names_) if (n.find("waist")!=std::string::npos || n.find("head")!=std::string::npos ||
        (n.find("wrist")==std::string::npos&&n.find("shoulder")==std::string::npos&&n.find("elbow")==std::string::npos))
      throw std::runtime_error("Only explicit arm joints are permitted");
    const std::filesystem::path root(output);
    if (!std::filesystem::create_directory(root)) throw std::runtime_error("Execution output must not exist");
    root_=root;manual_archive_=manual_&&(!policy_file_.empty()||!trical_file_.empty());
    if(manual_archive_)archive_pool_=std::make_shared<spdlog::details::thread_pool>(archive_capacity_,1);
    const auto open=[this](AsyncCsv& stream,const auto& path,const auto& columns) {
      if(manual_archive_)stream.open(path,columns,archive_pool_,archive_capacity_);
      else stream.open_synchronous(path,columns);
    };
    open(events_,root/"events.csv","receive_ns,state,reason");
    open(received_,root/"feedback.csv","receive_ns"+vendor::columns("q")+vendor::columns("v")+
      (hardware_?",publish_stamp_ns"+vendor::columns("effort_raw"):""));
    open(commands_,root/"commands.csv","planned_index,planned_ns,publish_ns,phase"+vendor::columns("q")+vendor::columns("v")+
      (experiment_commands_?vendor::columns("effort")+vendor::columns("kp")+vendor::columns("kd")+",ramp_increment_nm":"")+
      (trical_file_.empty()?"":",source_evaluation_ns,segment_generation"));
    if(hardware_&&!snapshot_)open(limits_log_,root/"observed_limits.csv","receive_ns,phase,joint,field,value,threshold,unit,action");
    max_age_=declare_parameter<std::int64_t>("max_feedback_age_ns",60000000);
    mode_age_=declare_parameter<std::int64_t>("max_mode_age_ns",300000000);
    timeout_=declare_parameter<std::int64_t>("transition_timeout_ns",1000000000);
    startup_timeout_=declare_parameter<std::int64_t>("startup_timeout_ns",10000000000);
    tolerance_=declare_parameter<double>("start_tolerance_rad",.004);
    if(!hardware_&&!manual_)legacy_guard_=std::make_unique<LegacyPositionGuard>(
        declare_parameter<double>("tracking_limit_rad",.02),declare_parameter<double>("position_allowance_rad",.03));
    max_velocity_=declare_parameter<double>("maximum_velocity_rad_s",hardware_?1.5:.5);
    vel_scale_=declare_parameter<double>("vel_scale",.1);
    period_=declare_parameter<std::int64_t>("command_period_ns",2000000);
    if(max_age_<=0||mode_age_<=0||timeout_<=0||startup_timeout_<=0||period_<1000000||period_>10000000||(!manual_&&tolerance_<=0)||max_velocity_<=0||vel_scale_<=0||vel_scale_>1)
      throw std::runtime_error("Invalid execution limits");
    if(feedback_type=="sensor_msgs/msg/JointState")
      sub_=create_subscription<sensor_msgs::msg::JointState>(joint,rclcpp::QoS(10).reliable(),[this](sensor_msgs::msg::JointState::SharedPtr msg) {receive(*msg);});
    else if(hardware_&&feedback_type=="crb_ros_msg/msg/JointStateData")
      debug_sub_=create_subscription<crb_ros_msg::msg::JointStateData>(joint,rclcpp::QoS(10).reliable(),[this](crb_ros_msg::msg::JointStateData::SharedPtr msg) {
        sensor_msgs::msg::JointState value;value.header=msg->header;value.name=msg->name;value.position=msg->position;value.velocity=msg->velocity;value.effort=msg->effort;receive(value);
      });
    else throw std::runtime_error("Unsupported execution feedback type");
    status_=create_subscription<std_msgs::msg::String>(mode,rclcpp::QoS(10).reliable(),[this](std_msgs::msg::String::SharedPtr msg) {mode_=msg->data;mode_at_=vendor::now();});
    if(!snapshot_) {
      auto lower=declare_parameter<std::vector<double>>("lower"),upper=declare_parameter<std::vector<double>>("upper");
      if(lower.size()!=14||upper.size()!=14) throw std::runtime_error("14 joint bounds required");
      for(int j=0;j<14;++j)if(!std::isfinite(lower[j])||!std::isfinite(upper[j])||lower[j]>=upper[j])throw std::runtime_error("Invalid bounds");
      std::copy(lower.begin(),lower.end(),lower_.begin());std::copy(upper.begin(),upper.end(),upper_.begin());
      std::string line;
      if(!manual_) {
        std::ifstream input(declare_parameter<std::string>("plan_file"));std::getline(input,line);
        while(std::getline(input,line)) {
          auto row=vendor::csv(line);if(row.size()!=29||row[0]!=static_cast<double>(plan_.size())*period_) throw std::runtime_error("Invalid uniform position plan");
          for(int j=0;j<14;++j) if(row[j+1]<lower_[j]||row[j+1]>upper_[j]||std::abs(row[j+15])>max_velocity_) throw std::runtime_error("Plan exceeds configured bounds");
          plan_.push_back(row);
        }
        if(plan_.size()<2) throw std::runtime_error("Missing plan");
        for(int j=0;!hardware_&&j<14;++j) if(std::abs(plan_.back()[j+1]-plan_[0][j+1])>1e-12) throw std::runtime_error("Plan must return to observed start");
      }
      if(!feedforward_path.empty()) {
        if(!hardware_||gravity_enabled||ramp_enabled||!std::isfinite(feedforward_limit)||feedforward_limit<=0||feedforward_limit>100.)throw std::runtime_error("Exclusive bounded H4 feedforward schedule required");
        std::ifstream feedforward(feedforward_path);std::getline(feedforward,line);
        while(std::getline(feedforward,line)) {
          auto row=vendor::csv(line);
          if(row.size()!=15||row[0]!=static_cast<double>(feedforward_plan_.size())*period_)throw std::runtime_error("Invalid uniform feedforward schedule");
          for(int j=0;j<14;++j)if(!std::isfinite(row[j+1])||std::abs(row[j+1])>feedforward_limit)throw std::runtime_error("Feedforward effort limit");
          feedforward_plan_.push_back(row);
        }
        if(feedforward_plan_.size()!=plan_.size())throw std::runtime_error("Feedforward/position schedule length mismatch");
      }
      if(ramp_enabled) {
        if(!hardware_)throw std::runtime_error("SDK ramp uses the shared field path only");
        SdkBreakawayConfig c;c.joint=declare_parameter<int>("breakaway_joint");c.direction=declare_parameter<int>("breakaway_direction");
        c.free_begin=declare_parameter<std::int64_t>("breakaway_free_begin_ns");c.ramp_begin=declare_parameter<std::int64_t>("breakaway_ramp_begin_ns");
        c.ramp_end=declare_parameter<std::int64_t>("breakaway_ramp_end_ns");c.return_begin=declare_parameter<std::int64_t>("breakaway_return_begin_ns");c.return_end=declare_parameter<std::int64_t>("breakaway_return_end_ns");
        c.sustain=declare_parameter<std::int64_t>("breakaway_sustain_ns");c.slope=declare_parameter<double>("breakaway_slope_nm_s");
        c.maximum_increment=declare_parameter<double>("breakaway_maximum_increment_nm");c.displacement=declare_parameter<double>("breakaway_displacement_rad");
        c.speed=declare_parameter<double>("breakaway_speed_rad_s");c.maximum_departure=declare_parameter<double>("breakaway_maximum_departure_rad");
        c.maximum_speed=declare_parameter<double>("breakaway_maximum_speed_rad_s");c.still_speed=declare_parameter<double>("breakaway_still_speed_rad_s");
        c.other_allowance=declare_parameter<double>("breakaway_other_allowance_rad");ramp_joint_=c.joint;ramp_=std::make_unique<SdkBreakaway>(c);
        if(c.return_end>plan_.back()[0])throw std::runtime_error("Ramp recovery exceeds the frozen plan");
      }
      if((!hold_kp_.empty()||!hold_kd_.empty()||ramp_||!feedforward_plan_.empty())&&(hold_kp_.size()!=14||hold_kd_.size()!=14))throw std::runtime_error("14 explicit holding gains required");
      for(std::size_t j=0;j<hold_kp_.size();++j)if(!std::isfinite(hold_kp_[j])||!std::isfinite(hold_kd_[j])||hold_kp_[j]<0||(!hardware_&&hold_kp_[j]==0)||hold_kp_[j]>1000||hold_kd_[j]<0||(!hardware_&&hold_kd_[j]==0)||hold_kd_[j]>100)throw std::runtime_error("Invalid holding gains");
      if(gravity_enabled) {
        auto model=casbot::ArmDynamics::from_urdf(declare_parameter<std::string>("gravity_model_path")).model();
        const auto g=declare_parameter<std::vector<double>>("gravity_world");
        if(g.size()!=3)throw std::runtime_error("3 gravity components required");
        model.gravity_world=Eigen::Map<const Eigen::Vector3d>(g.data());
        for(int j=0;j<14;++j)if(model.joints.size()!=14||model.joints[j].name!=names_[j])throw std::runtime_error("Gravity joint order mismatch");
        gravity_engine_=std::make_unique<casbot::ArmDynamics>(model);gravity_workspace_=std::make_unique<casbot::ArmWorkspace>(gravity_engine_->workspace());
      }
      if(!policy_file_.empty()) {
        std::string columns="receive_ns,revision";
        for(const auto* key:{"input_lower","input_upper","input_velocity","input_effort","kp_max","kd_max","observed_lower","observed_upper","observed_velocity","observed_action"})columns+=vendor::columns(key);
        columns+=",start_velocity,start_stable_s,stop_velocity,stop_stable_s,stop_timeout_s,feedback_timeout_s,mode_timeout_s,rpc_timeout_s,heartbeat_timeout_s,maximum_duration_s,short_amplitude_deg,short_min_s,short_max_s";
        open(policy_log_,root/"settings_applied.csv",columns);
        load_policy(true);
      }
      // Publisher and service clients are created only after local scope / plan validation.
      pub_=create_publisher<crb_ros_msg::msg::UpperJointData>(command,rclcpp::QoS(1).reliable());
      mode_client_=create_client<std_srvs::srv::SetBool>(service);
      if(!hardware_)stop_client_=create_client<std_srvs::srv::SetBool>(stop);
    }
    began_=vendor::now(); phase_at_=began_;event("WAIT_FEEDBACK","startup");
  }
  ~PositionExecutor() override {if(archive_pool_){try{flush_archive();}catch(...){} }}
  bool done() const {return done_;} bool success() const {return success_;}
  void tick() {
    const auto now=vendor::now();
    if(done_) return;
    if(!policy_file_.empty())load_policy(false);
    if(interrupted && phase_!="STOPPING"&&phase_!="FAILED") fault("interrupted");
    if(phase_=="STOPPING") {
      if(manual_)notice(fresh(now)?"observing_stop":"observing_stop_no_feedback","observe_after_stop",motion_observations(policy_file_.empty()?.02:policy_.general[ManualPolicy::StopVelocity]));
      if(stop_future_.valid() && stop_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
        stop_ack_=stop_future_.get()->success;stop_future_={};
      }
      if((stop_ack_||hardware_) && fresh(now) && speed()<(policy_file_.empty()?.02:policy_.general[ManualPolicy::StopVelocity])) {
        if(!stable_at_) stable_at_=now;
        if(now-stable_at_>=(policy_file_.empty()?100000000:policy_.ns(ManualPolicy::StopStable))) {stop_rest_observed_=true;finish(false,hardware_?"stream_stopped_and_feedback_rest_observed":"stop_ack_and_observed_rest");}
      } else stable_at_=0;
      if(now-phase_at_>(policy_file_.empty()?timeout_:policy_.ns(ManualPolicy::StopTimeout))) finish(false,"stop_unconfirmed");
      return;
    }
    if(phase_=="WAIT_FEEDBACK") {
      if(manual_) {
        // A pending manual start is a visible, cancellable wait, not a 10 s deadline.
        if(!fresh(now)) {stable_at_=0;notice("waiting_feedback","wait",{{"","feedback_callback_age",state_.receive?double(now-state_.receive)*1e-6:std::numeric_limits<double>::quiet_NaN(),double(max_age_)*1e-6,"ms"}});return;}
        if(now-mode_at_>=mode_age_) {stable_at_=0;notice("waiting_mode_feedback","wait",{{"","mode_callback_age",mode_at_?double(now-mode_at_)*1e-6:std::numeric_limits<double>::quiet_NaN(),double(mode_age_)*1e-6,"ms"}});return;}
        auto moving=motion_observations(policy_file_.empty()?.02:policy_.general[ManualPolicy::StartVelocity]);
        if(!moving.empty()) {stable_at_=0;notice("waiting_stationary","wait",moving);return;}
        if(!stable_at_)stable_at_=now;
        if(!policy_file_.empty()&&now-stable_at_<policy_.ns(ManualPolicy::StartStable)) {notice("waiting_stationary_window","wait");return;}
        if(!std::filesystem::exists(manual_file_)) {notice("waiting_start_command","wait");return;}
        if(!load_manual())return;
        if(mode_!="mode: READY, state: READY"&&mode_!="mode: IDLE_MODE, state: STAND") {notice("waiting_start_mode","wait");return;}
        const auto publishers=count_publishers(command_topic_);
        if(publishers>1) {notice("waiting_command_owner","wait",{{"","publishers",double(publishers),1.,"publishers"}});return;}
        if(!mode_client_->service_is_ready()) {notice("waiting_mode_service","wait");return;}
        mode_future_=request(mode_client_,true);event("ENTERING","request_upper_debug");return;
      }
      if(fresh(now)&&speed()<.02&&now-mode_at_<mode_age_) {
        if(snapshot_) {save_snapshot();finish(true,"snapshot_only");return;}
        if(manual_&&!std::filesystem::exists(manual_file_))return;
        if(manual_&&!load_manual())return;
        if(mode_!="mode: READY, state: READY"&&mode_!="mode: IDLE_MODE, state: STAND") {fault("unexpected_start_mode");return;}
        if(hardware_&&count_publishers(command_topic_)>1) {fault("competing_command_publisher");return;}
        bool changed=false;
        for(int j=0;!manual_&&j<14;++j) {
          start_error_[j]=state_.q[j]-plan_[0][j+1];
          changed=changed||std::abs(start_error_[j])>tolerance_;
        }
        if(changed) {
          if(hardware_) {
            // No mode request or command has been sent. The planner may take a
            // new read-only snapshot and certify a new attempt, not resume this one.
            requires_replan_=true;finish(false,"start_pose_changed");
          } else fault("start_pose_changed"); // Preserve the original S2 contract.
          return;
        }
        // Topic and service discovery are independent. Stay read-only until
        // all required services appear, bounded by the existing startup limit.
        if(!mode_client_->service_is_ready()||(!hardware_&&!stop_client_->service_is_ready())) {
          if(now-began_>startup_timeout_)fault("required_service_unavailable");
          return;
        }
        mode_future_=request(mode_client_,true); event("ENTERING","request_upper_debug");
      } else if(now-began_>startup_timeout_) fault("initial_feedback_or_mode_timeout");
      return;
    }
    if(!fresh(now)) {fault("stale_feedback",{{"","feedback_callback_age",double(now-state_.receive)*1e-6,double(max_age_)*1e-6,"ms"}});return;}
    if(now-mode_at_>mode_age_) {fault("stale_mode",{{"","mode_callback_age",double(now-mode_at_)*1e-6,double(mode_age_)*1e-6,"ms"}});return;}
    std::vector<Observation> violations,warnings;
    for(int j=0;j<14;++j) {
      const double lo=policy_file_.empty()?lower_[j]:policy_.axis[ManualPolicy::ObservedLower][j];
      const double hi=policy_file_.empty()?upper_[j]:policy_.axis[ManualPolicy::ObservedUpper][j];
      const double velocity=policy_file_.empty()?max_velocity_:policy_.axis[ManualPolicy::ObservedVelocity][j];
      auto& target=policy_file_.empty()||policy_.axis[ManualPolicy::ObservedAction][j]==1?violations:warnings;
      // Model position bounds are diagnostic on the field path, regardless of
      // the selected velocity action. Input/reference bounds are separate.
      auto& position_target=hardware_?warnings:target;
      if(state_.q[j]<lo)position_target.push_back({names_[j],"position_lower",state_.q[j],lo,"rad"});
      if(state_.q[j]>hi)position_target.push_back({names_[j],"position_upper",state_.q[j],hi,"rad"});
      if(std::abs(state_.v[j])>velocity)target.push_back({names_[j],"velocity",state_.v[j],velocity,"rad/s"});
    }
    if(hardware_)record_limits(warnings,violations);
    if(!violations.empty()) {fault("observed_limit",violations);return;}
    if(!warnings.empty()&&phase_=="STREAMING") {notice("observed_warning","warning",warnings);warning_active_=true;}
    else if(warning_active_) {notice("observed_recovered","sending",{},true);warning_active_=false;}
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
      if(exit_ack_&&(mode_=="mode: READY, state: READY"||(hardware_&&mode_=="mode: IDLE_MODE, state: STAND"))) {
        finish(true,hardware_?"trajectory_sent_and_mode_exited":"completed_at_observed_start");return;
      }
      if(now-phase_at_>timeout_) fault("exit_timeout");
      return;
    }
    if(now-last_graph_check_>20000000) {
      last_graph_check_=now;
      if(count_publishers(command_topic_)>1) {fault("competing_command_publisher");return;}
    }
    if(mode_!="mode: UPPER_BODY_DEBUG, state: STAND") {fault("control_mode_lost");return;}
    if(manual_) {
      if(!policy_file_.empty()&&policy_.general[ManualPolicy::MaximumDuration]>0&&now-start_>=policy_.ns(ManualPolicy::MaximumDuration)) {fault("duration_complete");return;}
      const auto index=(now-start_)/period_;
      if(index<=last_index_)return;
      if(!load_manual())return;
      if(!trical_file_.empty()) {
        if(trical_.action==2) {mode_future_=request(mode_client_,false);event("EXITING","trical_complete_exit_requested");return;}
        manual_row_=trical_.row(now);
      }
      if(index>last_index_+1)skips_+=index-last_index_-1;
      publish(manual_row_,index,"manual");last_index_=index;return;
    }
    const auto index=std::min<std::int64_t>((now-start_)/period_,plan_.size()-1);
    if(index<=last_index_ && phase_=="STREAMING") return;
    const auto& row=plan_[index];
    if(legacy_guard_) {
      const auto available_index=std::clamp<std::int64_t>((state_.receive-start_)/period_,0,plan_.size()-1);
      const auto reason=legacy_guard_->check(state_.q,plan_[available_index],row);
      if(!reason.empty()){fault(reason);return;}
    }
    if(phase_=="STREAMING") {
      // Late cycles skip planned slots. Never replay a burst of old commands.
      if(index>last_index_+1) skips_+=index-last_index_-1;
      publish(row,index,"stream");last_index_=index;
      if(phase_=="STOPPING"||done_)return;
      if(index==static_cast<std::int64_t>(plan_.size()-1)) {
        if(hardware_) {mode_future_=request(mode_client_,false);event("EXITING","reference_complete_exit_requested");}
        else event("SETTLING","final_hold");
      }
    } else if(phase_=="SETTLING") {
      if(now-last_publish_>=period_) publish(plan_.back(),index,"settle");
      double error=0;for(int j=0;j<14;++j) error=std::max(error,std::abs(state_.q[j]-plan_[0][j+1]));
      if(error<tolerance_&&speed()<.02) {if(!stable_at_) stable_at_=now;} else stable_at_=0;
      if(stable_at_&&now-stable_at_>150000000) {mode_future_=request(mode_client_,false);event("EXITING","verified_rest_then_exit");}
      else if(now-phase_at_>timeout_*2) fault("settling_timeout");
    }
  }
  void finish(bool ok,const std::string& reason) {
    if(ramp_)ramp_->write((root_/"guard.json").string());
    success_=ok;done_=true;event(ok?"COMPLETE":"FAILED",reason);flush_archive();
    std::ofstream out(root_/"result.json");out<<"{\"passed\":"<<(ok?"true":"false")<<",\"reason\":"<<std::quoted(reason)
      <<",\"last_feedback_receive_ns\":"<<state_.receive<<",\"last_mode_receive_ns\":"<<mode_at_<<",\"last_mode\":"<<std::quoted(mode_)
      <<",\"fault\":"<<std::quoted(fault_)<<",\"skipped_slots\":"<<skips_<<",\"stop_ack\":"<<(stop_ack_?"true":"false")
      <<",\"requires_start_replan\":"<<(requires_replan_?"true":"false")
      <<",\"mode_requested\":"<<(mode_requested_?"true":"false")<<",\"published_commands\":"<<published_commands_
      <<",\"recording_paused\":"<<(recording_paused_?"true":"false")
      <<",\"execution_recording_mode\":"<<std::quoted(manual_archive_?"bounded_async_single_producer":"legacy_synchronous")
      <<",\"snapshot_only\":"<<(snapshot_?"true":"false")<<",\"hardware_executable\":"<<(hardware_?"true":"false")
      <<",\"execution_scope\":"<<std::quoted(scope_)<<",\"device_disconnect_protection_assumed\":"<<(hardware_?"true":"false")
      <<",\"position_limit_action\":"<<std::quoted(hardware_?"warn":"end_control")
      <<",\"tracking_gate_enabled\":"<<(legacy_guard_?"true":"false")
      <<",\"feedback_rest_observed_after_fault\":"<<(stop_rest_observed_?"true":"false")
      <<",\"physical_stop_verified\":false,\"clock_id\":"<<std::quoted(vendor::clock_id())<<",\"start_pose_error_rad\":[";
    for(int j=0;j<14;++j){if(j)out<<',';out<<std::setprecision(17)<<start_error_[j];}out<<"]}\n";
  }
 private:
  void archive_error(const std::string& reason) {
    std::string cause=reason;
    try {commands_.check();received_.check();events_.check();policy_log_.check();limits_log_.check();}
    catch(const std::exception& e){cause=e.what();}
    const auto explanation=cause.find("queue full")!=std::string::npos?"异步写盘积压达到队列容量":"CSV文件写入失败";
    recording_paused_=true;
    std::cout<<"FIELDWORK_NOTICE {\"code\":\"recording_io_error\",\"message\":"
      <<vendor::json_quote(std::string(explanation)+"；控制CSV记录已暂停，已写前缀保留；发送与反馈继续："+cause)<<"}"<<std::endl;
  }
  void flush_archive() {
    if(!manual_archive_){commands_.flush();received_.flush();events_.flush();limits_log_.flush();return;}
    try {commands_.flush();received_.flush();events_.flush();policy_log_.flush();limits_log_.flush();}
    catch(const std::exception& e){if(!recording_paused_)archive_error(e.what());}
    archive_pool_.reset(); // Drain accepted rows only after publishing has ended.
    try {commands_.check();received_.check();events_.check();policy_log_.check();limits_log_.check();}
    catch(const std::exception& e){if(!recording_paused_)archive_error(e.what());}
  }
  bool recording_enabled() {
    if(recording_paused_)return false;
    if(manual_archive_)try {commands_.check();received_.check();events_.check();policy_log_.check();limits_log_.check();}
    catch(const std::exception& e){archive_error(e.what());return false;}
    if(!recording_pause_file_.empty()&&std::filesystem::exists(recording_pause_file_)) {
      recording_paused_=true;
      std::cout<<"FIELDWORK_NOTICE {\"code\":\"recording_io_error\",\"message\":\"缓存已满，已停止追加记录；控制和反馈继续\"}"<<std::endl;
      return false;
    }
    return true;
  }
  std::vector<Observation> motion_observations(double limit) const {
    std::vector<Observation> result;
    for(int j=0;j<14;++j)if(std::abs(state_.v[j])>=limit)result.push_back({names_[j],"velocity",state_.v[j],limit,"rad/s"});
    return result;
  }
  void record_limits(const std::vector<Observation>& warnings,const std::vector<Observation>& violations) {
    std::string key;
    for(const auto* list:{&warnings,&violations})for(const auto& o:*list)key+=o.joint+o.field+std::to_string(o.threshold)+(list==&warnings?"warn":"end");
    const auto now=vendor::now();
    if(key==limits_key_&&(key.empty()||now-last_limits_<1000000000))return;
    limits_key_=key;last_limits_=now;
    const bool recording=recording_enabled();
    try {
      if(key.empty()) {
        std::cout<<"FIELDWORK_LIMIT {\"action\":\"recovered\"}"<<std::endl;
        if(recording)limits_log_<<now<<','<<phase_<<",,,,,,recovered\n";
      } else for(const auto* list:{&warnings,&violations})for(const auto& o:*list) {
        const std::string action=list==&warnings?"warn":"end_control";
        std::cout<<std::setprecision(17)<<"FIELDWORK_LIMIT {\"joint\":"<<vendor::json_quote(o.joint)
          <<",\"field\":"<<vendor::json_quote(o.field)<<",\"value\":"<<o.value<<",\"threshold\":"<<o.threshold
          <<",\"unit\":"<<vendor::json_quote(o.unit)<<",\"action\":"<<vendor::json_quote(action)<<"}"<<std::endl;
        if(recording)limits_log_<<now<<','<<phase_<<','<<o.joint<<','<<o.field<<','<<o.value<<','<<o.threshold<<','<<o.unit<<','<<action<<'\n';
      }
      if(recording)limits_log_.flush();
    } catch(const std::exception& e){if(!manual_archive_)throw;archive_error(e.what());}
  }
  void notice(const std::string& code,const std::string& action,const std::vector<Observation>& observations={},bool force=false) {
    if(!manual_)return;
    const auto now=vendor::now();
    const auto key=phase_+":"+code;
    if(key!=notice_key_) {notice_since_=now;notice_key_=key;force=true;}
    if(!force&&now-last_notice_<200000000)return;
    last_notice_=now;
    std::ostringstream out;out<<std::setprecision(17);
    out<<"{\"code\":"<<vendor::json_quote(code)<<",\"phase\":"<<vendor::json_quote(phase_)
      <<",\"action\":"<<vendor::json_quote(action)<<",\"event_ns\":"<<now<<",\"since_ns\":"<<notice_since_
      <<",\"feedback_receive_ns\":"<<state_.receive<<",\"mode\":"<<vendor::json_quote(mode_)<<",\"observations\":[";
    bool first=true;
    for(const auto& item:observations) {
      if(!first)out<<',';
      first=false;
      out<<"{\"joint\":"<<vendor::json_quote(item.joint)<<",\"field\":"<<vendor::json_quote(item.field)<<",\"value\":";
      if(std::isfinite(item.value))out<<item.value;else out<<"null";
      out<<",\"threshold\":"<<item.threshold<<",\"unit\":"<<vendor::json_quote(item.unit)<<'}';
    }
    out<<"]}";
    // Live status remains independent of archival log success.
    std::cout<<"FIELDWORK_EVENT "<<out.str()<<std::endl;
  }
  void load_policy(bool initial) {
    try {
      const auto stamp=mailbox_stamp(policy_file_);
      if(policy_seen_&&stamp==policy_stamp_)return;
      policy_stamp_=stamp;policy_seen_=true;
      std::ifstream in(policy_file_);std::string line;std::getline(in,line);
      auto candidate=ManualPolicy::parse(vendor::csv(line));
      if(!initial&&(candidate.revision<=policy_.revision||!candidate.same_pinned(policy_)))throw std::runtime_error("Policy revision must advance; non-observed fields are pinned until next control");
      // Queue the accepted-value record before changing live state. A later disk
      // failure is reported as recording paused, independently of control.
      policy_log_<<vendor::now()<<','<<line<<'\n';policy_log_.flush();
      policy_=candidate;
      max_age_=policy_.ns(ManualPolicy::FeedbackTimeout);mode_age_=policy_.ns(ManualPolicy::ModeTimeout);timeout_=policy_.ns(ManualPolicy::RpcTimeout);
      std::cout<<"FIELDWORK_SETTINGS {\"code\":\"settings_applied\",\"revision\":"<<policy_.revision<<",\"action\":\"settings\"}"<<std::endl;
    } catch(const std::exception& e) {
      if(initial)throw;
      std::cout<<"FIELDWORK_SETTINGS {\"code\":\"settings_rejected\",\"revision\":"<<policy_.revision<<",\"action\":\"settings\",\"message\":"<<vendor::json_quote(e.what())<<"}"<<std::endl;
    }
  }
  bool load_manual() {
    if(!trical_file_.empty())return load_trical();
    // GUI replaces the complete file atomically. No partial frames or replay queue.
    try {
      const auto stamp=mailbox_stamp(manual_file_);
      if(manual_cached_&&stamp==manual_stamp_)return true;
      std::ifstream input(manual_file_);std::string line;std::getline(input,line);
      auto fields=vendor::csv(line);
      if(fields.size()!=71)throw std::runtime_error("71 manual fields required");
      for(int j=0;j<14;++j) {
        if(fields[1+j]<(policy_file_.empty()?lower_[j]:policy_.axis[ManualPolicy::InputLower][j])||fields[1+j]>(policy_file_.empty()?upper_[j]:policy_.axis[ManualPolicy::InputUpper][j])||
           std::abs(fields[15+j])>(policy_file_.empty()?max_velocity_:policy_.axis[ManualPolicy::InputVelocity][j])||
           std::abs(fields[29+j])>(policy_file_.empty()?manual_effort_limit_:policy_.axis[ManualPolicy::InputEffort][j])||
           fields[43+j]<0||fields[43+j]>(policy_file_.empty()?1000:policy_.axis[ManualPolicy::KpMax][j])||
           fields[57+j]<0||fields[57+j]>(policy_file_.empty()?100:policy_.axis[ManualPolicy::KdMax][j]))
          throw std::runtime_error("Manual command outside configured ranges");
      }
      manual_stamp_=stamp;manual_cached_=true;
      manual_fields_=std::move(fields);manual_row_.assign(manual_fields_.begin(),manual_fields_.begin()+29);
      return true;
    } catch(const std::exception&) {fault("invalid_manual_command");return false;}
  }
  bool load_trical() {
    try {
      const auto stamp=mailbox_stamp(trical_file_);
      if(manual_cached_&&stamp==manual_stamp_)return true;
      std::ifstream in(trical_file_);std::string line;std::getline(in,line);
      auto fields=vendor::csv(line);
      trical_.accept(fields,vendor::now(),state_.q,lower_,upper_,max_velocity_,tolerance_);
      manual_stamp_=stamp;manual_cached_=true;manual_row_=trical_.row(vendor::now());
      std::cout<<"TRICAL_ACK {\"generation\":"<<trical_.generation<<",\"start_receive_clock_ns\":"<<trical_.start
        <<",\"duration_ns\":"<<trical_.duration<<"}"<<std::endl;
      return true;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';fault("invalid_trical_segment");return false;}
  }
  void receive(const sensor_msgs::msg::JointState& msg) {
    vendor::State value;
    if(!vendor::state(msg,names_,value)) {if(active())fault("invalid_feedback");return;}
    state_=value;
    // Spatial chord-tube diagnostic on received SDK q only. It uses no invented
    // acquisition time and does not certify motion between cached feedbacks.
    if(!trical_file_.empty()&&phase_=="STREAMING"&&trical_.generation&&trical_.chord_distance(value.q)>.001)
      fault("trical_received_position_outside_reference_tube");
    if(!trical_file_.empty()&&value.receive-last_trical_feedback_>=20000000) {
      last_trical_feedback_=value.receive;
      std::cout<<std::setprecision(17)<<"TRICAL_FEEDBACK {\"receive_ns\":"<<value.receive<<",\"phase\":"<<std::quoted(phase_)<<",\"q\":[";
      for(int j=0;j<14;++j){if(j)std::cout<<',';std::cout<<value.q[j];}
      std::cout<<"],\"dq\":[";for(int j=0;j<14;++j){if(j)std::cout<<',';std::cout<<value.v[j];}
      std::cout<<"]}"<<std::endl;
    }
    if(recording_enabled()) try {received_<<value.receive;vendor::values(received_,value.q);vendor::values(received_,value.v);
    if(hardware_) {
      const auto stamp=static_cast<std::int64_t>(msg.header.stamp.sec)*1000000000LL+msg.header.stamp.nanosec;
      received_<<','<<stamp;
      for(const auto& name:names_) {
        const auto i=std::find(msg.name.begin(),msg.name.end(),name)-msg.name.begin();
        if(msg.effort.size()==msg.name.size()&&std::isfinite(msg.effort[i]))received_<<','<<msg.effort[i];
        else received_<<",nan";
      }
    }
    received_<<'\n';if(manual_archive_)received_.flush();}
    catch(const std::exception& e){if(!manual_archive_)throw;archive_error(e.what());}

    if(ramp_&&start_>0&&active()) {
      ramp_->observe(value.q,value.v,value.receive-start_,vendor::now()-start_);
      if(!ramp_->fault().empty())fault(ramp_->fault());
    }
  }
  using Client=rclcpp::Client<std_srvs::srv::SetBool>;
  Client::SharedFuture request(const Client::SharedPtr& cli,bool value) {auto r=std::make_shared<std_srvs::srv::SetBool::Request>();r->data=value;if(value&&cli==mode_client_)mode_requested_=true;return cli->async_send_request(r).share();}
  bool fresh(std::int64_t now) const {return state_.receive>0&&now-state_.receive<=max_age_;}
  double speed() const {double v=0;for(double x:state_.v)v=std::max(v,std::abs(x));return v;}
  bool active() const {return phase_!="WAIT_FEEDBACK"&&phase_!="STOPPING"&&!done_;}
  void event(const std::string& phase,const std::string& reason) {
    phase_=phase;phase_at_=vendor::now();stable_at_=0;
    notice(reason,phase=="STREAMING"?"sending":phase=="STOPPING"?"observe_after_stop":phase=="ENTERING"?"request_mode":phase=="FAILED"||phase=="COMPLETE"?"control_ended":"wait",{},true);
    if(recording_enabled())try {events_<<phase_at_<<','<<phase<<','<<reason<<'\n';events_.flush();}
    catch(const std::exception& e){if(!manual_archive_)throw;archive_error(e.what());}
  }
  void fault(const std::string& reason,const std::vector<Observation>& observations={}) {
    if(done_||phase_=="STOPPING")return;
    fault_=reason;
    notice(reason,(reason=="interrupted"||reason=="duration_complete")?"cancel_or_stop":"end_control",observations,true);
    if(snapshot_||(!mode_ack_ && phase_!="ENTERING")) {finish(false,reason);return;}
    event("STOPPING",reason);
    if(stop_client_&&stop_client_->service_is_ready())stop_future_=request(stop_client_,true);
  }
  void publish(const std::vector<double>& row,std::int64_t index,const std::string& phase) {
    crb_ros_msg::msg::UpperJointData msg;msg.header.stamp=this->now();msg.name=names_;msg.time_ref=0;msg.vel_scale=vel_scale_;
    if(hardware_)msg.header.frame_id="base_link";
    msg.position.assign(row.begin()+1,row.begin()+15);msg.velocity.assign(row.begin()+15,row.end());msg.effort.assign(14,0.);
    msg.kp=hold_kp_;msg.kd=hold_kd_;
    if(manual_&&trical_file_.empty()) {
      msg.effort.assign(manual_fields_.begin()+29,manual_fields_.begin()+43);
      msg.kp.assign(manual_fields_.begin()+43,manual_fields_.begin()+57);
      msg.kd.assign(manual_fields_.begin()+57,manual_fields_.end());
    }
    if(gravity_workspace_) {
      Eigen::Map<const Eigen::VectorXd> observed(state_.q.data(),14);auto g=gravity_workspace_->gravity(observed);
      msg.effort.assign(g.data(),g.data()+14);
    }
    if(!feedforward_plan_.empty())msg.effort.assign(feedforward_plan_[index].begin()+1,feedforward_plan_[index].end());
    double increment=0.;
    if(ramp_) {
      SdkBreakaway::Values q{},v{};std::copy(msg.position.begin(),msg.position.end(),q.begin());std::copy(msg.velocity.begin(),msg.velocity.end(),v.begin());bool zero=false;
      if(!ramp_->command(q,v,state_.q,vendor::now()-start_,increment,zero)){fault(ramp_->fault());return;}
      msg.position.assign(q.begin(),q.end());msg.velocity.assign(v.begin(),v.end());msg.kp=hold_kp_;msg.kd=hold_kd_;
      if(zero){msg.kp[ramp_joint_]=0.;msg.kd[ramp_joint_]=0.;}
      msg.effort[ramp_joint_]+=increment;
    }
    last_publish_=vendor::now();pub_->publish(msg);++published_commands_;
    if(!recording_enabled())return;
    try {commands_<<index<<','<<start_+index*period_<<','<<last_publish_<<','<<phase;
    vendor::values(commands_,msg.position);vendor::values(commands_,msg.velocity);
    if(experiment_commands_){vendor::values(commands_,msg.effort);
      if(msg.kp.empty()){std::array<double,14> unknown;unknown.fill(std::numeric_limits<double>::quiet_NaN());vendor::values(commands_,unknown);vendor::values(commands_,unknown);}
      else {vendor::values(commands_,msg.kp);vendor::values(commands_,msg.kd);}commands_<<','<<increment;}
    if(!trical_file_.empty())commands_<<','<<trical_.start+std::int64_t(manual_row_[0])<<','<<trical_.generation;
    commands_<<'\n';
    if(manual_)commands_.flush();}
    catch(const std::exception& e){if(!manual_archive_)throw;archive_error(e.what());}
  }
  void save_snapshot() {
    auto out=vendor::log(root_/"snapshot.csv","receive_ns"+vendor::columns("q")+vendor::columns("v"));
    out<<state_.receive;vendor::values(out,state_.q);vendor::values(out,state_.v);out<<'\n';
  }
  TricalStream trical_;std::string trical_file_;std::int64_t last_trical_feedback_=0;
  std::filesystem::path root_;std::vector<std::string> names_;vendor::State state_;
  std::array<double,14> lower_{},upper_{};std::vector<std::vector<double>> plan_;
  // Only the new manual workspace opts in; H1–H4/S2 keep synchronous files.
  static constexpr std::size_t archive_capacity_=8192;
  bool manual_archive_=false;std::shared_ptr<spdlog::details::thread_pool> archive_pool_;
  AsyncCsv events_,received_,commands_,limits_log_;
  std::string phase_,mode_,fault_,command_topic_,scope_;std::int64_t began_=0,phase_at_=0,mode_at_=0,start_=0,last_index_=-1,last_publish_=0,stable_at_=0,skips_=0,last_graph_check_=0;
  std::int64_t max_age_,mode_age_,timeout_,period_,startup_timeout_;double tolerance_,max_velocity_,vel_scale_;
  std::unique_ptr<LegacyPositionGuard> legacy_guard_;
  std::string limits_key_;std::int64_t last_limits_=0;
  std::string notice_key_;std::int64_t notice_since_=0,last_notice_=0;
  bool recording_paused_=false;std::string recording_pause_file_;
  ManualPolicy policy_;std::string policy_file_;AsyncCsv policy_log_;
  bool policy_seen_=false,warning_active_=false;std::array<std::uint64_t,8> policy_stamp_{};
  bool manual_cached_=false;std::array<std::uint64_t,8> manual_stamp_{};
  bool manual_=false;std::string manual_file_;double manual_effort_limit_=10.;std::vector<double> manual_fields_,manual_row_;
  bool snapshot_=false,done_=false,success_=false,mode_ack_=false,exit_ack_=false,stop_ack_=false,hardware_=false,stop_rest_observed_=false;
  bool requires_replan_=false,mode_requested_=false;std::uint64_t published_commands_=0;std::array<double,14> start_error_{};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<crb_ros_msg::msg::JointStateData>::SharedPtr debug_sub_;
  rclcpp::Publisher<crb_ros_msg::msg::UpperJointData>::SharedPtr pub_;Client::SharedPtr mode_client_,stop_client_;
  Client::SharedFuture mode_future_,stop_future_;
  int ramp_joint_=0;std::unique_ptr<SdkBreakaway> ramp_;std::vector<double> hold_kp_,hold_kd_;
  bool experiment_commands_=false;
  std::vector<std::vector<double>> feedforward_plan_;
  std::unique_ptr<casbot::ArmDynamics> gravity_engine_;std::unique_ptr<casbot::ArmWorkspace> gravity_workspace_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);std::signal(SIGINT,on_signal);std::signal(SIGTERM,on_signal);
  try {auto node=std::make_shared<PositionExecutor>();rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(node);
    while(rclcpp::ok()&&!node->done()) {exec.spin_once(std::chrono::microseconds(250));node->tick();}
    bool ok=node->success();node.reset();rclcpp::shutdown();return ok?0:1;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 2;}
}
