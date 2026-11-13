#include <casbot_calib/shm/wire35.hpp>
#include <cstring>
#include <stdexcept>

namespace casbot::shm {
bool native_wire_compatible() noexcept {
  const std::uint16_t value = 1;
  unsigned char first = 0;
  std::memcpy(&first, &value, 1);
  return first == 1;
}
namespace {
void require_native() {
  if (!native_wire_compatible())
    throw std::runtime_error("wire35 requires a little-endian host");
}
template<class Frame> Frame decode(const void* data, std::size_t size) {
  require_native();
  if (!data || size != sizeof(Frame))
    throw std::invalid_argument("wire35 frame size mismatch or null input");
  Frame frame{};
  std::memcpy(&frame, data, sizeof(frame));
  return frame;
}
template<class Frame, class Bytes> Bytes encode(const Frame& frame) {
  require_native();
  Bytes bytes{};
  std::memcpy(bytes.data(), &frame, bytes.size());
  return bytes;
}
}
CommandFrame decode_command(const void* data, std::size_t size) {
  return decode<CommandFrame>(data, size);
}
StatusFrame decode_status(const void* data, std::size_t size) {
  return decode<StatusFrame>(data, size);
}
CommandBytes encode_command(const CommandFrame& frame) {
  return encode<CommandFrame, CommandBytes>(frame);
}
StatusBytes encode_status(const StatusFrame& frame) {
  return encode<StatusFrame, StatusBytes>(frame);
}
}  // namespace casbot::shm
