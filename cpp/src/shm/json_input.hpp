#pragma once
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <array>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
namespace casbot::shm {
using Json=boost::property_tree::ptree;
using Joints=std::array<double,14>;
inline Json read_json(const std::string& path) {
  Json result;boost::property_tree::read_json(path,result);
  std::set<std::string> keys;
  for(const auto& entry:result)if(!keys.insert(entry.first).second)throw std::invalid_argument("Duplicate JSON key: "+entry.first);
  return result;
}
inline double finite_number(const Json& node) {
  const double v=node.get_value<double>();
  if(!std::isfinite(v)||!node.empty())throw std::invalid_argument("Expected finite scalar");return v;
}
inline Joints joint_values(const Json& node) {
  if(node.size()!=14||!node.data().empty())throw std::invalid_argument("Expected 14 joint values");
  Joints result{};std::size_t j=0;
  for(const auto& item:node){if(!item.first.empty())throw std::invalid_argument("Expected joint array");result[j++]=finite_number(item.second);}
  return result;
}
inline bool json_bool(const Json& node) {
  if(node.data()=="true")return true;if(node.data()=="false")return false;
  throw std::invalid_argument("Expected JSON boolean");
}
}  // namespace casbot::shm
