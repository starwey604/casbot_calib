#pragma once
#include <rclcpp/rclcpp.hpp>
#include <casbot_calib_interfaces/msg/joint_command.hpp>
#include <casbot_calib_interfaces/msg/joint_observation.hpp>
#include <casbot_calib_interfaces/msg/command_event.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
namespace bench {
using Command=casbot_calib_interfaces::msg::JointCommand;
using Observation=casbot_calib_interfaces::msg::JointObservation;
using Event=casbot_calib_interfaces::msg::CommandEvent;
inline std::int64_t now_ns() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
inline std::string clock_id() {
  std::ifstream input("/proc/sys/kernel/random/boot_id");std::string id;input>>id;
  if(id.empty()) throw std::runtime_error("Cannot identify the monotonic clock domain");
  return "linux_monotonic:"+id+":"+std::filesystem::read_symlink("/proc/self/ns/time").string();
}
inline rclcpp::QoS qos(bool reliable,std::size_t depth=1) {
  auto profile=rclcpp::QoS(rclcpp::KeepLast(depth));profile.durability_volatile();
  return reliable?profile.reliable():profile.best_effort();
}
inline std::ofstream log(const std::string& path,const std::string& header) {
  std::ofstream stream(path);if(!stream) throw std::runtime_error("Cannot open log: "+path);
  stream<<std::setprecision(17)<<header<<'\n';return stream;
}
template<class Values> void columns(std::ostream& out,const Values& values) {for(const auto& value:values)out<<','<<value;}
inline std::string indexed(const std::string& name) {std::string s;for(int j=0;j<14;++j)s+=","+name+std::to_string(j);return s;}
inline std::vector<std::string> split(const std::string& line) {std::istringstream in(line);std::vector<std::string> values;std::string value;while(std::getline(in,value,','))values.push_back(value);return values;}
}
