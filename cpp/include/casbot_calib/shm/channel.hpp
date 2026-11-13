#pragma once
#include <casbot_calib/shm/wire35.hpp>
#include <memory>
#include <string>

namespace casbot::shm {
struct QueueHeader {
  std::int32_t magic, capacity;
  std::int64_t count, write_index, read_index, published_index;
};
static_assert(sizeof(QueueHeader) == 40 && offsetof(QueueHeader, published_index) == 32);
constexpr std::size_t channel_bytes = 16 * 1024 * 1024;
constexpr std::size_t status_offset = channel_bytes / 2;
enum class ChannelRole { client, fixture_master };

// Attach never truncates, clears indices, creates semaphores, or posts _start.
// Only an explicit private fixture master may create/unlink its own objects.
class Channel {
 public:
  Channel(const std::string& name, ChannelRole role = ChannelRole::client);
  ~Channel();
  Channel(const Channel&) = delete;
  Channel& operator=(const Channel&) = delete;
  bool send(const CommandFrame& frame);
  bool receive(StatusFrame& frame);
  bool fixture_receive(CommandFrame& frame);
  bool fixture_send(const StatusFrame& frame);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct PassiveSnapshot { CommandFrame command; StatusFrame status; };
// O_RDONLY/pread only. Evidence snapshot, not a coherent control observation.
PassiveSnapshot passive_snapshot(const std::string& name);
}  // namespace casbot::shm
