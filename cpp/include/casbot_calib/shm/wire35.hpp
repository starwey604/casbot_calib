#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace casbot::shm {
// Installation ABI: embed-fusion 2.3.7 / casbot_02 35-slot compatibility mode.
// No IPC objects live in these messages. The outer messages are NOT packed.
#pragma pack(push, 1)
struct JointCommand {
  std::uint16_t control_word;
  std::int32_t position, velocity;
  std::int16_t torque, mode, kp, kd;
};
struct JointStatus {
  std::uint16_t status_word;
  std::int32_t position, velocity;
  std::int16_t torque, mode;
  std::uint16_t error_code;
};
struct HandJointCommand {
  std::int16_t target, param;
  std::uint8_t mode, options;
};
struct HandCommand {
  std::uint16_t control_word;
  std::uint8_t hand_id, reserved;
  std::array<HandJointCommand, 6> joints;
};
struct HandJointStatus {
  std::int16_t position, velocity, current;
  std::uint16_t status;
};
struct HandStatus {
  std::uint16_t global_status, error_code;
  std::array<HandJointStatus, 6> joints;
};
#pragma pack(pop)

struct alignas(8) CommandFrame {
  std::uint64_t frame_counter;
  std::uint8_t joint_count, reserved0;
  std::uint16_t hand_count;
  std::array<JointCommand, 35> joints;
  std::array<HandCommand, 2> hands;
  std::array<std::uint8_t, 6> reserved1;
  std::uint64_t timestamp;
  std::uint32_t crc;
  std::array<std::uint8_t, 4> reserved2;
};
struct alignas(8) StatusFrame {
  std::uint64_t frame_counter;
  std::uint8_t joint_count, reserved0;
  std::uint16_t hand_count;
  std::array<JointStatus, 35> joints;
  std::array<HandStatus, 2> hands;
  std::array<std::uint8_t, 4> reserved1;
  std::uint64_t timestamp;
  std::uint32_t crc;
  std::array<std::uint8_t, 4> reserved2;
};

static_assert(sizeof(JointCommand) == 18 && sizeof(JointStatus) == 16);
static_assert(offsetof(JointCommand, position) == 2 && offsetof(JointCommand, velocity) == 6);
static_assert(offsetof(JointCommand, torque) == 10 && offsetof(JointCommand, mode) == 12);
static_assert(offsetof(JointCommand, kp) == 14 && offsetof(JointCommand, kd) == 16);
static_assert(offsetof(JointStatus, position) == 2 && offsetof(JointStatus, velocity) == 6);
static_assert(offsetof(JointStatus, torque) == 10 && offsetof(JointStatus, mode) == 12);
static_assert(offsetof(JointStatus, error_code) == 14);
static_assert(sizeof(HandJointCommand) == 6 && sizeof(HandCommand) == 40);
static_assert(offsetof(HandJointCommand, mode) == 4 && offsetof(HandCommand, joints) == 4);
static_assert(sizeof(HandJointStatus) == 8 && sizeof(HandStatus) == 52);
static_assert(offsetof(HandStatus, joints) == 4);
static_assert(std::is_standard_layout_v<CommandFrame> && std::is_trivially_copyable_v<CommandFrame>);
static_assert(std::is_standard_layout_v<StatusFrame> && std::is_trivially_copyable_v<StatusFrame>);
static_assert(alignof(CommandFrame) == 8 && alignof(StatusFrame) == 8);
static_assert(sizeof(CommandFrame) == 744 && sizeof(StatusFrame) == 696);
static_assert(offsetof(CommandFrame, joint_count) == 8 && offsetof(CommandFrame, hand_count) == 10);
static_assert(offsetof(CommandFrame, joints) == 12 && offsetof(CommandFrame, hands) == 642);
static_assert(offsetof(CommandFrame, timestamp) == 728 && offsetof(CommandFrame, crc) == 736);
static_assert(offsetof(StatusFrame, joint_count) == 8 && offsetof(StatusFrame, hand_count) == 10);
static_assert(offsetof(StatusFrame, joints) == 12 && offsetof(StatusFrame, hands) == 572);
static_assert(offsetof(StatusFrame, timestamp) == 680 && offsetof(StatusFrame, crc) == 688);

using CommandBytes = std::array<std::uint8_t, sizeof(CommandFrame)>;
using StatusBytes = std::array<std::uint8_t, sizeof(StatusFrame)>;

// Copies into local storage, never dereferences a potentially unaligned mapped
// frame. Retains every reserved byte when decoding and re-encoding evidence.
bool native_wire_compatible() noexcept;
CommandFrame decode_command(const void* data, std::size_t size);
StatusFrame decode_status(const void* data, std::size_t size);
CommandBytes encode_command(const CommandFrame& frame);
StatusBytes encode_status(const StatusFrame& frame);
}  // namespace casbot::shm
