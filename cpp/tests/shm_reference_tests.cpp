#include "reference.hpp"
#include "soft_return.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace {

static void check(bool cond, const char* msg) {
  if (!cond) throw std::runtime_error(msg);
}

std::string json_vec(const casbot::shm::JointVector& v) {
  std::ostringstream out;
  out << std::setprecision(17) << '[';
  for (int i = 0; i < 14; ++i) {
    if (i) out << ',';
    out << v[i];
  }
  out << ']';
  return out.str();
}

std::string write_temp_reference(
    std::int64_t duration_ns,
    const casbot::shm::JointVector& q0,
    const casbot::shm::JointVector& q1,
    const casbot::shm::JointVector& dq0 = {},
    const casbot::shm::JointVector& dq1 = {},
    std::string schema = "casbot_shm_reference_v1", std::string fields = "",
    const std::vector<std::pair<std::int64_t,casbot::shm::JointVector>>& waypoints = {}) {
  const auto path = std::filesystem::temp_directory_path() /
      ("casbot_ref_test_" + std::to_string(getpid()) + ".json");
  std::ofstream out(path);
  out << std::setprecision(17);
  out << "{\n"
      << "  \"schema\": \"" << schema << "\",\n" << fields
      << "  \"joint_names\": [";
  for (int i = 0; i < 14; ++i) {
    if (i) out << ',';
    out << '"' << casbot::shm::arm_names[i] << '"';
  }
  casbot::shm::JointVector kp{}, kd{}, zeros{};
  kp.fill(20.0);
  kd.fill(2.0);
  out << "],\n"
      << "  \"kp\": " << json_vec(kp) << ",\n"
      << "  \"kd\": " << json_vec(kd) << ",\n"
      << "  \"knots_ns\": [0";
  for(const auto& waypoint:waypoints)out<<','<<waypoint.first;
  out << ',' << duration_ns << "],\n"
      << "  \"states\": [\n"
      << "    [" << json_vec(q0) << ',' << json_vec(dq0) << ',' << json_vec(zeros) << ',' << json_vec(zeros) << "],\n";
  for(const auto& waypoint:waypoints)
    out<<"    ["<<json_vec(waypoint.second)<<','<<json_vec(zeros)<<','<<json_vec(zeros)<<','<<json_vec(zeros)<<"],\n";
  out << "    [" << json_vec(q1) << ',' << json_vec(dq1) << ',' << json_vec(zeros) << ',' << json_vec(zeros) << "]\n"
      << "  ],\n"
      << "  \"effort\": [{\"t_ns\": 0, \"value\": " << json_vec(zeros) << "}]\n"
      << "}\n";
  return path.string();
}

}  // namespace

int test_shm_reference_tests() {
  using namespace casbot::shm;
  JointVector lower{}, upper{}, velocity{};
  lower.fill(-2.0);
  upper.fill(2.0);
  velocity.fill(1.5);

  const double dq = 1.0471975511965976;  // 60 deg
  // 0.5 rad/s planned peak (unsubdivided bound is 1.6 > 1.5, 8-subdivision bound is 0.5 <= 1.5).
  const auto h_valid_ns = static_cast<std::int64_t>(std::llround((2.1875 * dq / 0.5) * 1e9));
  JointVector q0{}, q1{};
  q1[3] = dq;
  const auto path_valid = write_temp_reference(h_valid_ns, q0, q1);
  auto ref_valid = Reference::load(path_valid);
  std::filesystem::remove(path_valid);
  ref_valid.validate_limits(lower, upper, velocity);

  const std::string mit=R"({"slot":0,"mode":"mit","kp":400,"kd":20})";
  const std::string cst=R"({"slot":4,"mode":"cst_pd","kp":120,"kd":3,"torque_limit_nm":30})";
  const std::string limit=R"("observed_velocity_limit_rad_s":1.5,)";
  auto block=[](const std::string& slots) {
    return R"("non_arm_hold":{"ramp_s":1,"cst_comm_timeout_ms":25,"slots":[)"+slots+"]},";
  };
  auto load=[&](const std::string& schema,const std::string& fields) {
    return Reference::load(write_temp_reference(h_valid_ns,q0,q1,{},{},schema,fields));
  };
  check(!load("casbot_shm_reference_v1",limit+R"("purpose":"shm_calib_collision_checked",)").has_ready_pose(),
        "Plain reference includes a preparation pose");
  const std::string pd_vector=json_vec(q0);
  const std::string pd_rows="\"pd_schedule\":[{\"t_ns\":0,\"kp\":"+pd_vector+",\"kd\":"+pd_vector+
      "},{\"t_ns\":1000000000,\"kp\":"+pd_vector+",\"kd\":"+pd_vector+"}],";
  for(const auto* purpose:{"shm_calib_ready_then_collision_checked","shm_h1_ready_then_collision_checked"}) {
    const auto ready=load("casbot_shm_reference_v1",limit+pd_rows+"\"purpose\":\""+purpose+"\",");
    check(ready.has_ready_pose()&&ready.ready_move_end_ns()==1000000000,"Wrong preparation boundary");
  }
  const std::string v2="casbot_shm_reference_v2";
  auto mixed=load(v2,limit+block(mit+","+cst));
  check(mixed.non_arm_hold().slots.size()==2&&mixed.non_arm_hold().has_cst_pd(),"Missing body slots");
  check(mixed.non_arm_hold().cst_comm_timeout_ns==25000000&&mixed.observed_velocity_limit()==1.5,"Wrong v2 limits");
  check(load(v2,R"("observed_velocity_limit_rad_s":2.0,)"+block(mit)).observed_velocity_limit()==2.0,
        "Wrong verification feedback velocity limit");
  auto rejects=[&](const std::string& schema,const std::string& fields) {
    try { load(schema,fields); } catch(const std::exception&) { return; }
    throw std::runtime_error("Accepted invalid hold: "+fields);
  };
  rejects("casbot_shm_reference_v1",limit+block(mit));
  rejects(v2,limit);rejects(v2,block(cst));rejects(v2,limit+block(""));
  rejects(v2,R"("observed_velocity_limit_rad_s":2.01,)"+block(mit));
  const std::string prepared=limit+R"("purpose":"shm_calib_ready_then_collision_checked",)";
  JointVector anchor{},middle{};
  anchor[0]=.4;middle=anchor;middle[1]=.4;
  const std::string retreat_schema="casbot_shm_retreat_reference_v1";
  const auto route_path=write_temp_reference(11000000000LL,q0,q0,{}, {},retreat_schema,
      prepared+R"("soft_stop_checkpoints_ns":[1000000000,3000000000,9000000000],)",
      {{1000000000,q0},{3000000000,anchor},{6000000000,middle},{9000000000,anchor}});
  const auto route=Reference::load(route_path);
  check(route.retreat_checkpoints().size()==5,"Missing retreat checkpoints");
  const std::int64_t interrupt_ns=4000000000;
  const ReferenceRetreat retreat(route,interrupt_ns);
  const auto interrupted=route.at(interrupt_ns);
  const auto initial=retreat.at(0.);
  check(initial.position==interrupted.state[0]&&initial.velocity==interrupted.state[1],"Retreat changes the initial command");
  check(retreat.at(1.).velocity[1]<0.,"Retreat did not reverse toward the nearby anchor");
  check(retreat.at(3.25).position==anchor&&retreat.at(3.25).velocity==JointVector{},"Retreat did not reach the anchor at rest");
  check(retreat.at(4.).position[0]<anchor[0]&&retreat.at(4.).position[1]==0.,"Retreat skipped the checked exit");
  check(retreat.at(retreat.total_duration_s()).position==q0,"Retreat did not return to ready");
  rejects(retreat_schema,prepared);
  rejects(retreat_schema,prepared+R"("soft_stop_checkpoints_ns":[1,2],)");
  rejects(v2,limit+block(mit+","+mit));
  for(auto bad:{R"({"slot":12,"kp":1,"kd":1,"mode":"mit"})",R"({"slot":29,"kp":1,"kd":1,"mode":"mit"})",
                R"({"slot":0,"kp":1,"kd":1,"mode":"other"})",R"({"slot":4,"kp":1,"kd":1,"mode":"cst_pd"})",
                R"({"slot":0,"kp":1001,"kd":1,"mode":"mit"})"})rejects(v2,limit+block(bad));
  std::filesystem::remove(path_valid);
  // Genuine peak above 1.5 rad/s (1.55 rad/s) must still be rejected.
  const auto h_fast_ns = static_cast<std::int64_t>(std::llround((2.1875 * dq / 1.55) * 1e9));
  const auto path_fast = write_temp_reference(h_fast_ns, q0, q1);
  auto ref_fast = Reference::load(path_fast);
  std::filesystem::remove(path_fast);
  bool caught_vel = false;
  try {
    ref_fast.validate_limits(lower, upper, velocity);
  } catch (const std::invalid_argument& e) {
    caught_vel = std::string(e.what()).find("velocity envelope") != std::string::npos;
  }
  check(caught_vel, "Expected over-limit velocity reference to throw");

  // Position limit violation must still be rejected.
  JointVector q_oob{};
  q_oob[3] = 2.05;
  const auto h_pos_ns = static_cast<std::int64_t>(std::llround((2.1875 * 2.05 / 0.5) * 1e9));
  const auto path_oob = write_temp_reference(h_pos_ns, q0, q_oob);
  auto ref_oob = Reference::load(path_oob);
  std::filesystem::remove(path_oob);
  bool caught_pos = false;
  try {
    ref_oob.validate_limits(lower, upper, velocity);
  } catch (const std::invalid_argument& e) {
    caught_pos = std::string(e.what()).find("position envelope") != std::string::npos;
  }
  check(caught_pos, "Expected out-of-limit position reference to throw");

  std::cout << "SHM reference Bernstein subdivision limits test passed\n";
  return 0;
  return 0;
}
