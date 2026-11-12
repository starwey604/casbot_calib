// S1 has subscriptions and file I/O only. No publisher, mode client or controller.
#include "numeric_text.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <crb_ros_msg/msg/joint_state_data.hpp>
#include <std_msgs/msg/string.hpp>
#include <rmw/rmw.h>
#include <array>
#include <chrono>
#include <csignal>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace {
std::int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string quote(const std::string& s) {
  std::ostringstream out; out << '"';
  for (unsigned char c : s) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
    else out << c;
  }
  out << '"'; return out.str();
}
template<class T> void numbers(std::ostream& out, const T& values) {
  out << '['; bool first = true;
  for (double v : values) {
    if (!first) out << ',';
    first = false;
    if(std::isfinite(v))vendor_text::number(out,v);else out<<"null";
  }
  out << ']';
}
template<class T> void strings(std::ostream& out, const T& values) {
  out << '['; bool first = true;
  for (const auto& value : values) { if (!first) out << ','; first = false; out << quote(value); }
  out << ']';
}
template<class T> void joint_json(std::ostream& out, const T& msg,const std::string& names) {
  out << "{\"header\":{\"sec\":" << msg.header.stamp.sec
      << ",\"nanosec\":" << msg.header.stamp.nanosec
      << ",\"frame_id\":" << quote(msg.header.frame_id) << "},\"name\":[";
  out<<names;
  out << "],\"position\":"; numbers(out, msg.position);
  out << ",\"velocity\":"; numbers(out, msg.velocity);
  out << ",\"effort\":"; numbers(out, msg.effort);
}
}

class Recorder final : public rclcpp::Node {
 public:
  Recorder() : Node("casbot_vendor_state_recorder", rclcpp::NodeOptions()
      .enable_rosout(false).start_parameter_services(false).start_parameter_event_publisher(false)) {
    const auto output = declare_parameter<std::string>("output", "");
    const auto duration = declare_parameter<double>("duration_s", 10.);
    const auto depth = declare_parameter<int>("depth", 10);
    const auto reliable = declare_parameter<bool>("reliable", true);
    const auto flush_ms=declare_parameter<std::int64_t>("recording_flush_period_ms",0);
    if(flush_ms<0||flush_ms>std::numeric_limits<std::int64_t>::max()/1000000)throw std::invalid_argument("Recording flush period outside nanosecond range");
    flush_period_ns_=flush_ms*1000000;
    pause_file_=declare_parameter<std::string>("recording_pause_file","");
    live_=declare_parameter<bool>("stream_packets",false);
    until_stopped_=declare_parameter<bool>("until_stopped",false);
    tolerate_io_=declare_parameter<bool>("tolerate_recording_io_error",false);
    if(tolerate_io_&&!live_)throw std::invalid_argument("Tolerant recording requires a live observation consumer");
    // With a file-size limit Linux otherwise kills the recorder before the
    // stream can report EFBIG. Only opt-in live observation tolerates this I/O
    // failure; the original finite S1 capture retains its existing policy.
    if(tolerate_io_)std::signal(SIGXFSZ,SIG_IGN);
    if (output.empty() || !std::isfinite(duration) || duration <= 0 || depth < 1)
      throw std::invalid_argument("output, positive duration and depth are required");
    output_ = output;
    if (!std::filesystem::create_directory(output_)) throw std::runtime_error("Raw output must not exist");
    cdr_.open(output_ / "cdr.bin", std::ios::binary);
    rows_.open(output_ / "packets.jsonl"); rows_ << std::setprecision(17);
    graph_.open(output_ / "publishers.jsonl");
    cdr_.exceptions(std::ios::badbit | std::ios::failbit);
    rows_.exceptions(std::ios::badbit | std::ios::failbit);
    graph_.exceptions(std::ios::badbit | std::ios::failbit);
    if (declare_parameter<bool>("record_discovery", false)) {
      discovery_.open(output_ / "discovery.jsonl");
      discovery_.exceptions(std::ios::badbit | std::ios::failbit);
    }
    std::ifstream boot("/proc/sys/kernel/random/boot_id"); std::string id; boot >> id;
    if (id.empty()) throw std::runtime_error("Linux monotonic clock identity is unavailable");
    clock_ = "linux_monotonic:" + id + ":" + std::filesystem::read_symlink("/proc/self/ns/time").string();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(depth))).durability_volatile();
    if (reliable) qos.reliable(); else qos.best_effort();
    const auto primary = declare_parameter<std::string>("joint_topic", "/joint_states");
    const auto type = declare_parameter<std::string>("joint_type", "sensor_msgs/msg/JointState");
    if (type != "sensor_msgs/msg/JointState" && type != "crb_ros_msg/msg/JointStateData")
      throw std::invalid_argument("Unsupported joint feedback type");
    add("joint", primary, type, qos);
    add("debug", declare_parameter<std::string>("debug_topic", ""), "crb_ros_msg/msg/JointStateData", qos);
    add("mode", declare_parameter<std::string>("mode_topic", "/motion/status"), "std_msgs/msg/String", qos);
    add("robot_state", declare_parameter<std::string>("robot_state_topic", "/motion/robot_state"), "std_msgs/msg/String", qos);
    // Optional sensor channel; best-effort also receives reliable IMU publishers.
    auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(depth))).best_effort().durability_volatile();
    add("imu", declare_parameter<std::string>("imu_topic", ""), "sensor_msgs/msg/Imu", imu_qos);
    start_ = now_ns(); end_ = start_ + static_cast<std::int64_t>(duration * 1e9);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {
      record([this]{commit(false);});
      if (!until_stopped_ && now_ns() >= end_) done_ = true;
      if (now_ns() >= next_graph_) { record([this]{graph_snapshot();});if(live_)live_graph(); next_graph_ = now_ns() + 500000000; }
    });
    std::ofstream meta(output_ / "recorder.pending");
    meta.exceptions(std::ios::badbit | std::ios::failbit);
    meta << "{\"schema_version\":\"casbot_vendor_raw_v1\",\"clock_id\":" << quote(clock_)
         << ",\"rmw\":" << quote(rmw_get_implementation_identifier())
         << ",\"node\":" << quote(get_fully_qualified_name())
         << ",\"depth\":" << depth << ",\"reliable\":" << (reliable ? "true" : "false")
         << ",\"read_only\":true,\"sample_time_known\":false,\"device_sequence_known\":false,"
            "\"nonfinite_json\":\"null; original IEEE values retained in CDR\","
            "\"cdr_source\":\"received generic subscription bytes before deserialization\","
         << "\"recording_flush_period_ns\":"<<flush_period_ns_<<",\"stream_packets\":"<<(live_?"true":"false")<<",\"until_stopped\":"<<(until_stopped_?"true":"false")<<"}\n";
    meta.close();
    std::filesystem::rename(output_ / "recorder.pending", output_ / "recorder.json");
  }
  bool done() const { return done_; }
  void finish(const std::string& reason) {
    record([this]{commit(true);graph_snapshot();});
    try {
      std::ofstream summary(output_ / "summary.json");summary.exceptions(std::ios::badbit | std::ios::failbit);
      summary<<"{\"receive_packets\":"<<sequence_<<",\"start_ns\":"<<start_
        <<",\"end_ns\":"<<now_ns()<<",\"reason\":"<<quote(recording_paused_?"recording_paused":recording_failed_?"recording_error":reason)
        <<",\"recording_failed\":"<<(recording_failed_?"true":"false")
        <<",\"read_only\":true,\"application_publishers\":0,\"mode_clients\":0}\n";
    } catch(const std::ios_base::failure& error) {io_failure(error);}
  }
 private:
  void commit(bool force) {
    const auto now=now_ns();
    if(!force&&flush_period_ns_&&now-last_flush_<flush_period_ns_)return;
    if(!pending_cdr_.empty()) {
      cdr_.write(pending_cdr_.data(),pending_cdr_.size());cdr_.flush();
      // Publish complete JSON rows only after their entire CDR prefix is visible.
      rows_.write(pending_rows_.data(),pending_rows_.size());rows_.flush();
      pending_cdr_.clear();pending_rows_.clear();
    }
    last_flush_=now;
  }
  void io_failure(const std::ios_base::failure& error) {
    if(!tolerate_io_)throw;
    if(!recording_failed_)std::cout<<"FIELDWORK_NOTICE {\"code\":\"recording_io_error\",\"message\":"<<quote(error.what())
      <<",\"event_ns\":"<<now_ns()<<",\"recording\":false}"<<std::endl;
    recording_failed_=true;
  }
  template<class Write> void record(Write write) {
    if(!pause_file_.empty()&&std::filesystem::exists(pause_file_)){recording_failed_=true;recording_paused_=true;}
    if(recording_failed_)return;
    try {write();}catch(const std::ios_base::failure& error){io_failure(error);}
  }
  void live_graph() {
    std::ostringstream out;out<<"FIELDWORK_GRAPH {\"observed_ns\":"<<now_ns()<<",\"channels\":{";
    bool first=true;
    for(const auto& pair:channels_) {
      if(!first)out<<',';
      first=false;
      auto publishers=get_publishers_info_by_topic(pair.second);
      out<<quote(pair.first)<<":{\"topic\":"<<quote(pair.second)<<",\"publishers\":"<<publishers.size()<<'}';
    }
    out<<"}}";std::cout<<out.str()<<std::endl;
  }
  void graph_snapshot() {
    if (!rclcpp::ok()) return;
    for (const auto& topic : topics_) for (const auto& info : get_publishers_info_by_topic(topic)) {
      const auto& p = info.qos_profile().get_rmw_qos_profile();
      graph_ << "{\"observed_ns\":" << now_ns() << ",\"topic\":" << quote(topic) << ",\"node\":" << quote(info.node_name())
            << ",\"namespace\":" << quote(info.node_namespace()) << ",\"type\":" << quote(info.topic_type())
            << ",\"qos_reliability\":" << p.reliability << ",\"qos_durability\":" << p.durability
            << ",\"qos_history\":" << p.history << ",\"qos_depth\":" << p.depth << "}\n";
    }
    graph_.flush();
    if (discovery_.is_open()) {
      // Include empty publisher lists: absence cannot be represented by the
      // legacy publishers.jsonl stream. Graph discovery is observational only.
      discovery_ << "{\"observed_ns\":" << now_ns() << ",\"topics\":{";
      bool first = true;
      for (const auto& item : get_topic_names_and_types()) {
        if (!first) discovery_ << ',';
        first = false; discovery_ << quote(item.first) << ':'; strings(discovery_, item.second);
      }
      discovery_ << "},\"services\":{"; first = true;
      for (const auto& item : get_service_names_and_types()) {
        if (!first) discovery_ << ',';
        first = false; discovery_ << quote(item.first) << ':'; strings(discovery_, item.second);
      }
      discovery_ << "},\"monitored_topics\":{"; first = true;
      for (const auto& topic : topics_) {
        if (!first) discovery_ << ',';
        first = false; discovery_ << quote(topic) << ":[";
        bool first_endpoint = true;
        for (const auto& info : get_publishers_info_by_topic(topic)) {
          if (!first_endpoint) discovery_ << ',';
          first_endpoint = false;
          const auto& p = info.qos_profile().get_rmw_qos_profile();
          std::ostringstream gid;
          for (auto byte : info.endpoint_gid()) gid << std::hex << std::setfill('0') << std::setw(2) << int(byte);
          discovery_ << "{\"node\":" << quote(info.node_name()) << ",\"namespace\":" << quote(info.node_namespace())
              << ",\"gid\":" << quote(gid.str()) << ",\"type\":" << quote(info.topic_type())
              << ",\"qos_reliability\":" << p.reliability << ",\"qos_durability\":" << p.durability
              << ",\"qos_history\":" << p.history << ",\"qos_depth\":" << p.depth << '}';
        }
        discovery_ << ']';
      }
      discovery_ << "}}\n"; discovery_.flush();
    }
  }
  void add(const std::string& channel, const std::string& topic, const std::string& type, const rclcpp::QoS& qos) {
    if (topic.empty()) return;
    topics_.push_back(topic);channels_[channel]=topic;
    const auto prefix=",\"channel\":"+quote(channel)+",\"topic\":"+quote(topic)+",\"type\":"+quote(type);
    subscriptions_.push_back(create_generic_subscription(topic, type, qos,
      [this, channel, topic, type,prefix,cached_names=std::vector<std::string>{},names_json=std::string{}](std::shared_ptr<rclcpp::SerializedMessage> raw) mutable {
        const auto received = now_ns(); const auto& bytes = raw->get_rcl_serialized_message();
        std::streamoff offset=-1;
        record([&]{offset=cdr_bytes_;pending_cdr_.append(reinterpret_cast<const char*>(bytes.buffer),bytes.buffer_length);cdr_bytes_+=bytes.buffer_length;});
        const auto names=[&](const auto& value)->const std::string& {
          if(names_json.empty()||cached_names!=value) {
            cached_names=value;std::ostringstream text;
            for(std::size_t j=0;j<value.size();++j){if(j)text<<',';text<<quote(value[j]);}
            names_json=text.str();
          }
          return names_json;
        };
        std::ostringstream decoded; decoded << std::setprecision(17);
        try {
          if (type == "sensor_msgs/msg/JointState") {
            sensor_msgs::msg::JointState msg; rclcpp::Serialization<sensor_msgs::msg::JointState> codec;
            codec.deserialize_message(raw.get(), &msg); joint_json(decoded,msg,names(msg.name)); decoded << '}';
          } else if (type == "crb_ros_msg/msg/JointStateData") {
            crb_ros_msg::msg::JointStateData msg; rclcpp::Serialization<crb_ros_msg::msg::JointStateData> codec;
            codec.deserialize_message(raw.get(), &msg); joint_json(decoded,msg,names(msg.name));
            decoded << ",\"kp\":"; numbers(decoded, msg.kp); decoded << ",\"kd\":"; numbers(decoded, msg.kd); decoded << '}';
          } else if (type == "sensor_msgs/msg/Imu") {
            sensor_msgs::msg::Imu msg; rclcpp::Serialization<sensor_msgs::msg::Imu> codec;
            codec.deserialize_message(raw.get(), &msg);
            decoded << "{\"header\":{\"sec\":" << msg.header.stamp.sec
                << ",\"nanosec\":" << msg.header.stamp.nanosec
                << ",\"frame_id\":" << quote(msg.header.frame_id) << "},\"orientation\":";
            numbers(decoded, std::array<double,4>{msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w});
            decoded << ",\"angular_velocity\":";
            numbers(decoded, std::array<double,3>{msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z});
            decoded << ",\"linear_acceleration\":";
            numbers(decoded, std::array<double,3>{msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z});
            decoded << ",\"orientation_covariance\":"; numbers(decoded,msg.orientation_covariance);
            decoded << ",\"angular_velocity_covariance\":"; numbers(decoded,msg.angular_velocity_covariance);
            decoded << ",\"linear_acceleration_covariance\":"; numbers(decoded,msg.linear_acceleration_covariance);
            decoded << '}';
          } else {
            std_msgs::msg::String msg; rclcpp::Serialization<std_msgs::msg::String> codec;
            codec.deserialize_message(raw.get(), &msg); decoded << "{\"data\":" << quote(msg.data) << '}';
          }
        } catch (const std::exception& e) {
          decoded.str(""); decoded.clear(); decoded << "{\"decode_error\":" << quote(e.what()) << '}';
        }
        std::ostringstream row;row<<std::setprecision(17);
        row << "{\"receive_sequence\":" << sequence_++ << ",\"receive_ns\":" << received
              <<prefix
              << ",\"cdr_offset\":";
        if(offset>=0&&!recording_failed_)row<<offset;else row<<"null";
        row << ",\"cdr_size\":" << bytes.buffer_length << ",\"message\":" << decoded.str() << "}";
        const auto text=row.str();
        record([&]{pending_rows_+=text;pending_rows_+='\n';commit(false);});
        // Full CDR/JSON recording retains every callback. Only the live display
        // is decimated by callback time, never by equality of numeric values.
        if(live_&&(last_live_.count(channel)==0||received-last_live_[channel]>=50000000)) {
          std::cout<<"FIELDWORK_PACKET "<<text<<std::endl;last_live_[channel]=received;
        }
      }));
  }
  std::filesystem::path output_;
  std::ofstream cdr_, rows_, graph_, discovery_;
  std::string pending_cdr_,pending_rows_;std::uint64_t cdr_bytes_=0;
  std::int64_t flush_period_ns_=0,last_flush_=0;
  std::string clock_,pause_file_;std::map<std::string,std::string> channels_;
  std::uint64_t sequence_ = 0;
  std::int64_t start_ = 0, end_ = 0, next_graph_ = 0;
  bool recording_paused_=false;
  bool done_ = false,live_=false,until_stopped_=false,tolerate_io_=false,recording_failed_=false;
  std::map<std::string,std::int64_t> last_live_;
  std::vector<std::string> topics_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<Recorder> node;
  try {
    node = std::make_shared<Recorder>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !node->done()) executor.spin_once(std::chrono::milliseconds(20));
    node->finish(rclcpp::ok() ? "duration_complete" : "interrupted");
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    if (node) { try { node->finish("error"); } catch (...) {} }
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
