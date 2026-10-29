// S1 has subscriptions and file I/O only. No publisher, mode client or controller.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <crb_ros_msg/msg/joint_state_data.hpp>
#include <std_msgs/msg/string.hpp>
#include <rmw/rmw.h>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
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
    if (std::isfinite(v)) out << v; else out << "null";
  }
  out << ']';
}
template<class T> void joint_json(std::ostream& out, const T& msg) {
  out << "{\"header\":{\"sec\":" << msg.header.stamp.sec
      << ",\"nanosec\":" << msg.header.stamp.nanosec
      << ",\"frame_id\":" << quote(msg.header.frame_id) << "},\"name\":[";
  for (std::size_t i = 0; i < msg.name.size(); ++i) {
    if (i) out << ',';
    out << quote(msg.name[i]);
  }
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
    start_ = now_ns(); end_ = start_ + static_cast<std::int64_t>(duration * 1e9);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {
      if (now_ns() >= end_) done_ = true;
      if (now_ns() >= next_graph_) { graph_snapshot(); next_graph_ = now_ns() + 500000000; }
    });
    std::ofstream meta(output_ / "recorder.pending");
    meta.exceptions(std::ios::badbit | std::ios::failbit);
    meta << "{\"schema_version\":\"casbot_vendor_raw_v1\",\"clock_id\":" << quote(clock_)
         << ",\"rmw\":" << quote(rmw_get_implementation_identifier())
         << ",\"node\":" << quote(get_fully_qualified_name())
         << ",\"depth\":" << depth << ",\"reliable\":" << (reliable ? "true" : "false")
         << ",\"read_only\":true,\"sample_time_known\":false,\"device_sequence_known\":false,"
            "\"nonfinite_json\":\"null; original IEEE values retained in CDR\","
            "\"cdr_source\":\"received generic subscription bytes before deserialization\"}\n";
    meta.close();
    std::filesystem::rename(output_ / "recorder.pending", output_ / "recorder.json");
  }
  bool done() const { return done_; }
  void finish(const std::string& reason) {
    cdr_.flush(); rows_.flush();
    std::ofstream summary(output_ / "summary.json");
    summary.exceptions(std::ios::badbit | std::ios::failbit);
    summary << "{\"receive_packets\":" << sequence_ << ",\"start_ns\":" << start_
            << ",\"end_ns\":" << now_ns() << ",\"reason\":" << quote(reason)
            << ",\"read_only\":true,\"application_publishers\":0,\"mode_clients\":0}\n";
    graph_snapshot();
  }
 private:
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
  }
  void add(const std::string& channel, const std::string& topic, const std::string& type, const rclcpp::QoS& qos) {
    if (topic.empty()) return;
    topics_.push_back(topic);
    subscriptions_.push_back(create_generic_subscription(topic, type, qos,
      [this, channel, topic, type](std::shared_ptr<rclcpp::SerializedMessage> raw) {
        const auto received = now_ns(); const auto& bytes = raw->get_rcl_serialized_message();
        const auto offset = cdr_.tellp();
        cdr_.write(reinterpret_cast<const char*>(bytes.buffer), bytes.buffer_length); cdr_.flush();
        std::ostringstream decoded; decoded << std::setprecision(17);
        try {
          if (type == "sensor_msgs/msg/JointState") {
            sensor_msgs::msg::JointState msg; rclcpp::Serialization<sensor_msgs::msg::JointState> codec;
            codec.deserialize_message(raw.get(), &msg); joint_json(decoded, msg); decoded << '}';
          } else if (type == "crb_ros_msg/msg/JointStateData") {
            crb_ros_msg::msg::JointStateData msg; rclcpp::Serialization<crb_ros_msg::msg::JointStateData> codec;
            codec.deserialize_message(raw.get(), &msg); joint_json(decoded, msg);
            decoded << ",\"kp\":"; numbers(decoded, msg.kp); decoded << ",\"kd\":"; numbers(decoded, msg.kd); decoded << '}';
          } else {
            std_msgs::msg::String msg; rclcpp::Serialization<std_msgs::msg::String> codec;
            codec.deserialize_message(raw.get(), &msg); decoded << "{\"data\":" << quote(msg.data) << '}';
          }
        } catch (const std::exception& e) {
          decoded.str(""); decoded.clear(); decoded << "{\"decode_error\":" << quote(e.what()) << '}';
        }
        rows_ << "{\"receive_sequence\":" << sequence_++ << ",\"receive_ns\":" << received
              << ",\"channel\":" << quote(channel) << ",\"topic\":" << quote(topic) << ",\"type\":" << quote(type)
              << ",\"cdr_offset\":" << offset << ",\"cdr_size\":" << bytes.buffer_length
              << ",\"message\":" << decoded.str() << "}\n";
        rows_.flush();
      }));
  }
  std::filesystem::path output_;
  std::ofstream cdr_, rows_, graph_;
  std::string clock_;
  std::uint64_t sequence_ = 0;
  std::int64_t start_ = 0, end_ = 0, next_graph_ = 0;
  bool done_ = false;
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
