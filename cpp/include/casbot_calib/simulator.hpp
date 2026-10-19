#pragma once
#include <Eigen/Core>
#include <cstdint>
#include <deque>
#include <vector>

namespace casbot {
using Nanoseconds = std::int64_t;
enum class Mode : std::int32_t { torque = 0, mixed_pd = 1 };
enum SampleStatus : std::uint32_t { saturated = 1, repeated = 2 };
struct JointCommand {
  Nanoseconds send_ns = 0;
  Mode mode = Mode::torque;
  double q_ref = 0, dq_ref = 0, torque_ff = 0, kp = 0, kd = 0;
};
struct JointSample {
  Nanoseconds sequence = 0, sample_ns = -1, current_sample_ns = -1, receive_ns = 0;
  std::uint32_t status = 0;
  double position = 0, velocity = 0, current = 0, estimated_torque = 0, temperature = 25;
};
struct DeviceCapabilities {
  bool position = true, velocity = true, current = true, torque_command = true;
  bool independent_torque_sensor = false, motor_encoder_exposed = false;
  bool mixed_pd = true, inner_pi_tunable = false, displacement_sensor = false;
};
// SDK adapters implement only this interface, never virtual-time advance().
class JointDevice {
 public:
  virtual ~JointDevice() = default;
  virtual DeviceCapabilities capabilities() const = 0;
  virtual void write(const JointCommand&) = 0;
  virtual bool read(JointSample&) = 0;
};
struct SimulationConfig {
  double inertia = 0.08, damping = 0.12, coulomb = 0.06, spring = 3.0;
  double spring_zero = 0, friction_velocity = 0.03, gain = 2.0;
  double current_tau = 0, current_filter_tau = 0;
  double torque_limit = 20, current_limit = 10;
  double initial_q = 0, initial_v = 0, initial_string_x = 0, initial_string_v = 0;
  double position_noise = 0, velocity_noise = 0, current_noise = 0;
  double position_quantum = 0, current_quantum = 0;
  double temperature = 25, drop_probability = 0, repeat_probability = 0;
  Nanoseconds integration_ns = 50000, sample_period_ns = 1000000;
  Nanoseconds command_delay_ns = 0, feedback_delay_ns = 0, jitter_ns = 0;
  Nanoseconds current_lag_ns = 0;
  bool expose_sample_time = true, double_inertia = false, contact_enabled = false;
  double motor_inertia = 0.02, coupling_spring = 90, coupling_damping = 0.25;
  double string_mass = 0.010, string_spring = 400, string_damping = 2;
  double contact_spring = 700, contact_damping = 0.2;
  double lever = 0.2, pick_offset = 0, depth = 0.002, attack_angle = 0;
  std::uint32_t seed = 0;
};
struct TruthSample {
  Nanoseconds time_ns;
  double q, v, acceleration, current, torque_request, contact_force, string_x, string_v;
  double energy, motor_q;
  bool contact;
};
struct SimulationResult {
  std::vector<JointSample> observations;
  std::vector<TruthSample> truth;
};
// A batch mock implements the exact read/write SDK contract. Only the runner
// can execute queued commands in virtual time. Each run starts a new experiment.
class MockJointDevice final : public JointDevice {
 public:
  DeviceCapabilities capabilities() const override { return {}; }
  void write(const JointCommand& command) override { commands_.push_back(command); }
  bool read(JointSample& sample) override {
    if(samples_.empty()) return false;
    sample=samples_.front(); samples_.pop_front(); return true;
  }
 private:
  friend class SimulationRunner;
  std::vector<JointCommand> commands_;
  std::deque<JointSample> samples_;
};
class SimulationRunner {
 public:
  explicit SimulationRunner(SimulationConfig config);
  SimulationResult run(const std::vector<JointCommand>& commands, Nanoseconds duration_ns) const;
  SimulationResult run(MockJointDevice& device, Nanoseconds duration_ns) const;
 private:
  SimulationConfig config_;
};
}  // namespace casbot
