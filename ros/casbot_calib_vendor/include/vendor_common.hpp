#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>

namespace vendor {
inline std::int64_t now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline std::string clock_id() {
  std::ifstream input("/proc/sys/kernel/random/boot_id"); std::string id; input >> id;
  if (id.empty()) throw std::runtime_error("Missing local monotonic clock identity");
  return "linux_monotonic:"+id+":"+std::filesystem::read_symlink("/proc/self/ns/time").string();
}
inline std::ofstream log(const std::filesystem::path& path, const std::string& header) {
  std::ofstream out(path); out.exceptions(std::ios::badbit|std::ios::failbit);
  out << std::setprecision(17) << header << '\n'; return out;
}
inline std::string columns(const std::string& name) {
  std::string s; for (int j=0;j<14;++j) s+=","+name+std::to_string(j); return s;
}
template<class Values> void values(std::ostream& out, const Values& v) {for (auto x:v) out << ',' << x;}
inline std::vector<double> csv(const std::string& line) {
  std::istringstream in(line); std::string s; std::vector<double> result;
  while (std::getline(in,s,',')) {std::size_t used; double value=std::stod(s,&used);
    if (used!=s.size()||!std::isfinite(value)) throw std::runtime_error("Invalid finite CSV value");
    result.push_back(value);}
  return result;
}
struct State {std::array<double,14> q{},v{}; std::int64_t receive=0;};
inline bool state(const sensor_msgs::msg::JointState& msg, const std::vector<std::string>& names, State& result) {
  if (names.size()!=14 || msg.name.size()!=msg.position.size() || msg.name.size()!=msg.velocity.size()
      || std::set<std::string>(msg.name.begin(),msg.name.end()).size()!=msg.name.size()) return false;
  for (int j=0;j<14;++j) {
    auto found=std::find(msg.name.begin(),msg.name.end(),names[j]);
    if (found==msg.name.end()) return false;
    auto i=static_cast<std::size_t>(found-msg.name.begin());
    if (!std::isfinite(msg.position[i])||!std::isfinite(msg.velocity[i])) return false;
    result.q[j]=msg.position[i]; result.v[j]=msg.velocity[i];
  }
  result.receive=now(); return true;
}
inline void local_topics(const std::vector<std::string>& topics) {
  const auto local=std::getenv("ROS_LOCALHOST_ONLY");
  if (!local||std::string(local)!="1") throw std::runtime_error("S2 currently permits isolated localhost contract execution only");
  for (const auto& topic:topics) if (topic.rfind("/casbot_calib_s2_",0)!=0)
    throw std::runtime_error("S2 requires private /casbot_calib_s2_* endpoints");
}
}
