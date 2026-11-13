#include <casbot_calib/shm/channel.hpp>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <semaphore.h>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace casbot::shm {
namespace {
constexpr std::int32_t magic = 0x11223344;
std::string normalized(std::string name) {
  if (name.empty()) throw std::invalid_argument("Empty SHM name");
  if (name[0] != '/') name.insert(name.begin(), '/');
  if (name.find('/', 1) != std::string::npos) throw std::invalid_argument("SHM name must not contain paths");
  return name;
}
std::runtime_error error(const char* operation) {
  return std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}
std::int64_t load(const std::int64_t* value) { return __atomic_load_n(value, __ATOMIC_SEQ_CST); }
void store(std::int64_t* value, std::int64_t next) { __atomic_store_n(value, next, __ATOMIC_SEQ_CST); }
void validate(const QueueHeader& h) {
  if (h.magic != magic || h.capacity != 2 || h.read_index < 0 || h.write_index < 0 || h.published_index < 0)
    throw std::runtime_error("Unsupported/corrupt SHM queue header (expected 35-compat, capacity 2)");
}
class Lock {
 public:
  explicit Lock(sem_t* sem):sem_(sem) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::microseconds(400);
    do {
      if (sem_trywait(sem_) == 0) { owned_ = true; return; }
      if (errno != EAGAIN && errno != EINTR) throw error("sem_trywait");
      const timespec pause{0, 20000}; nanosleep(&pause, nullptr);
    } while (std::chrono::steady_clock::now() < end);
  }
  ~Lock() { if (owned_) sem_post(sem_); }
  explicit operator bool() const { return owned_; }
 private:
  sem_t* sem_; bool owned_ = false;
};
template<class Frame> bool push(void* memory, sem_t* sem, const Frame& frame) {
  Lock lock(sem); if (!lock) return false;
  auto* h = static_cast<QueueHeader*>(memory); validate(*h);
  const auto w = load(&h->write_index), r = load(&h->read_index);
  if (w == std::numeric_limits<std::int64_t>::max()) throw std::runtime_error("Queue index exhausted");
  // Vendor ring reserves one slot: capacity 2 means ONE pending frame.
  if ((w + 1) % h->capacity == r % h->capacity) return false;
  store(&h->write_index, w + 1);
  std::memcpy(static_cast<char*>(memory) + sizeof(QueueHeader) + (w % h->capacity) * sizeof(Frame), &frame, sizeof(Frame));
  store(&h->published_index, w + 1);
  __atomic_fetch_add(&h->count, 1, __ATOMIC_SEQ_CST);
  return true;
}
template<class Frame> bool pop(void* memory, sem_t* sem, Frame& frame) {
  Lock lock(sem); if (!lock) return false;
  auto* h = static_cast<QueueHeader*>(memory); validate(*h);
  const auto r = load(&h->read_index), p = load(&h->published_index);
  if (r % h->capacity == p % h->capacity) return false;
  if (r == std::numeric_limits<std::int64_t>::max()) throw std::runtime_error("Queue index exhausted");
  std::memcpy(&frame, static_cast<char*>(memory) + sizeof(QueueHeader) + (r % h->capacity) * sizeof(Frame), sizeof(Frame));
  store(&h->read_index, r + 1);
  __atomic_fetch_sub(&h->count, 1, __ATOMIC_SEQ_CST);
  return true;
}
}
struct Channel::Impl {
  std::string name;
  int fd = -1;
  void* memory = MAP_FAILED;
  sem_t* front = SEM_FAILED;
  sem_t* back = SEM_FAILED;
  sem_t* start = SEM_FAILED;
  bool owns_shm = false, owns_front = false, owns_back = false, owns_start = false;
  ChannelRole role;
  Impl(std::string n, ChannelRole r):name(normalized(std::move(n))),role(r) {}
  ~Impl() {
    if (front != SEM_FAILED) sem_close(front);
    if (back != SEM_FAILED) sem_close(back);
    if (start != SEM_FAILED) sem_close(start);
    if (memory != MAP_FAILED) munmap(memory, channel_bytes);
    if (fd >= 0) close(fd);
    if (owns_front) sem_unlink((name + "_front").c_str());
    if (owns_back) sem_unlink((name + "_back").c_str());
    if (owns_start) sem_unlink((name + "_start").c_str());
    if (owns_shm) shm_unlink(name.c_str());
  }
};
Channel::Channel(const std::string& name, ChannelRole role):impl_(std::make_unique<Impl>(name, role)) {
  if (!native_wire_compatible()) throw std::runtime_error("Channel requires little-endian host");
  auto& i = *impl_;
  const bool create = role == ChannelRole::fixture_master;
  if (create && i.name.rfind("/casbot_calib_test_", 0) != 0)
    throw std::invalid_argument("Fixtures may only create /casbot_calib_test_* objects");
  i.fd = shm_open(i.name.c_str(), create ? O_RDWR | O_CREAT | O_EXCL : O_RDWR, 0600);
  if (i.fd < 0) throw error("shm_open");
  i.owns_shm = create;
  if (create && ftruncate(i.fd, channel_bytes) != 0) throw error("ftruncate private fixture");
  struct stat st{};
  if (fstat(i.fd, &st) != 0) throw error("fstat");
  if (st.st_size != static_cast<off_t>(channel_bytes)) throw std::runtime_error("Unexpected SHM mapping size");
  i.memory = mmap(nullptr, channel_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, i.fd, 0);
  if (i.memory == MAP_FAILED) throw error("mmap");
  if (create) {
    auto* cmd = static_cast<QueueHeader*>(i.memory);
    auto* status = reinterpret_cast<QueueHeader*>(static_cast<char*>(i.memory) + status_offset);
    *cmd = {magic, 2, 0, 0, 0, 0}; *status = *cmd;
  }
  i.front = create ? sem_open((i.name + "_front").c_str(), O_CREAT | O_EXCL, 0600, 0) : sem_open((i.name + "_front").c_str(), 0);
  if (i.front == SEM_FAILED) throw error("sem_open front");
  i.owns_front = create;
  i.back = create ? sem_open((i.name + "_back").c_str(), O_CREAT | O_EXCL, 0600, 0) : sem_open((i.name + "_back").c_str(), 0);
  if (i.back == SEM_FAILED) throw error("sem_open back");
  i.owns_back = create;
  i.start = create ? sem_open((i.name + "_start").c_str(), O_CREAT | O_EXCL, 0600, 0) : sem_open((i.name + "_start").c_str(), 0);
  if (i.start == SEM_FAILED) throw error("sem_open start");
  i.owns_start = create;
  validate(*static_cast<QueueHeader*>(i.memory));
  validate(*reinterpret_cast<QueueHeader*>(static_cast<char*>(i.memory) + status_offset));
}
Channel::~Channel() = default;
void Channel::activate_fresh_master() {
  if(impl_->role != ChannelRole::client)throw std::logic_error("Only client can initialize fresh master");
  const auto* a=static_cast<QueueHeader*>(impl_->memory);
  const auto* b=reinterpret_cast<QueueHeader*>(static_cast<char*>(impl_->memory)+status_offset);
  auto empty=[](const QueueHeader& h) {validate(h);return h.count==0&&h.write_index==0&&h.read_index==0&&h.published_index==0;};
  if(!empty(*a)||!empty(*b))throw std::runtime_error("Refusing handshake on non-fresh queues");
  for(auto* sem:{impl_->back,impl_->front,impl_->start}) {
    int value=-1;if(sem_getvalue(sem,&value))throw error("sem_getvalue");
    if(value!=0)throw std::runtime_error("Refusing handshake: semaphore is already active");
  }
  // MotionEtherCAT::init 0x9b3c..0x9b68 posts in this exact order. Master
  // wait_semaphore 0x9de98 waits on start before starting shared I/O.
  for(auto* sem:{impl_->back,impl_->front,impl_->start})if(sem_post(sem))throw error("startup sem_post");
}
bool Channel::send(const CommandFrame& f) {
  if (impl_->role != ChannelRole::client) throw std::logic_error("Wrong channel role");
  return push(impl_->memory, impl_->front, f);
}
bool Channel::receive(StatusFrame& f) {
  if (impl_->role != ChannelRole::client) throw std::logic_error("Wrong channel role");
  return pop(static_cast<char*>(impl_->memory) + status_offset, impl_->back, f);
}
QueueDiagnostics Channel::diagnostics() const {
  auto sample=[](const void* memory) {
    const auto* h=static_cast<const QueueHeader*>(memory);
    return QueueHeader{h->magic,h->capacity,load(&h->count),load(&h->write_index),load(&h->read_index),load(&h->published_index)};
  };
  return {sample(impl_->memory),sample(static_cast<const char*>(impl_->memory)+status_offset)};
}
bool Channel::fixture_receive(CommandFrame& f) {
  if (impl_->role != ChannelRole::fixture_master) throw std::logic_error("Wrong channel role");
  return pop(impl_->memory, impl_->front, f);
}
bool Channel::fixture_send(const StatusFrame& f) {
  if (impl_->role != ChannelRole::fixture_master) throw std::logic_error("Wrong channel role");
  return push(static_cast<char*>(impl_->memory) + status_offset, impl_->back, f);
}
PassiveSnapshot passive_snapshot(const std::string& name) {
  const int fd = shm_open(normalized(name).c_str(), O_RDONLY, 0);
  if (fd < 0) throw error("passive shm_open");
  struct Close { int fd; ~Close() { close(fd); } } close_fd{fd};
  auto read_latest = [&](std::size_t offset, void* frame, std::size_t size, QueueHeader& header) {
    for (int attempt = 0; attempt < 32; ++attempt) {
      QueueHeader before{}, after{};
      if (pread(fd, &before, sizeof(before), offset) != sizeof(before)) throw error("pread header");
      validate(before);
      if (before.published_index == 0) throw std::runtime_error("No published frame in passive snapshot");
      const auto slot = (before.published_index - 1) % before.capacity;
      if (pread(fd, frame, size, offset + sizeof(QueueHeader) + slot * size) != static_cast<ssize_t>(size)) throw error("pread frame");
      if (pread(fd, &after, sizeof(after), offset) != sizeof(after)) throw error("pread header");
      if (std::memcmp(&before, &after, sizeof(before)) == 0) {header=after;return;}
    }
    throw std::runtime_error("Passive snapshot changed during read; retry later");
  };
  PassiveSnapshot result{};
  read_latest(0, &result.command, sizeof(result.command), result.command_queue);
  read_latest(status_offset, &result.status, sizeof(result.status), result.status_queue);
  return result;
}
}  // namespace casbot::shm
