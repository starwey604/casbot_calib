#include <casbot_calib/shm/wire35.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
std::vector<std::uint8_t> read(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("Cannot read fixture: " + path.string());
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
}
int main(int argc, char** argv) {
  if (argc != 2) return 2;
  using namespace casbot::shm;
  const auto command = read(std::filesystem::path(argv[1]) / "command_slot_0.bin");
  const auto status = read(std::filesystem::path(argv[1]) / "status_slot_0.bin");
  // Fixed expected values decoded independently with Python struct, not a
  // round-trip alone through this same C++ implementation.
  const auto c = decode_command(command.data(), command.size());
  check(c.frame_counter == 869953 && c.joint_count == 29 && c.hand_count == 2, "command header");
  check(c.timestamp == 925003 && c.crc == 0, "command stamp/crc");
  check(c.joints[12].position == 106947 && c.joints[12].torque == -25, "left shoulder");
  check(c.joints[19].position == -101226 && c.joints[19].torque == 27, "right shoulder");
  check(c.joints[28].kp == 5000 && c.joints[28].kd == 50, "waist gains");
  check(c.joints[34].control_word == 0 && c.joints[12].mode == -1, "reserved/mode");
  check(c.hands[0].control_word == 20 && c.hands[0].joints[0].param == 1000 &&
        c.hands[0].joints[0].mode == 3 && c.hands[0].joints[0].options == 150, "hand command layout");
  const auto s = decode_status(status.data(), status.size());
  check(s.frame_counter == 869944 && s.joint_count == 0 && s.hand_count == 2, "status header");
  check(s.timestamp == 937742 && s.crc == 65535, "status stamp/crc");
  check(s.joints[12].position == 106622 && s.joints[12].velocity == -21978 &&
        s.joints[12].torque == -92 && s.joints[12].mode == -1 &&
        s.joints[12].status_word == 567 && s.joints[12].error_code == 0, "status joint fields");
  check(s.joints[25].position == -8201 && s.joints[28].torque == -44, "right/waist status");
  const auto cb = encode_command(c);
  const auto sb = encode_status(s);
  check(std::vector<std::uint8_t>(cb.begin(), cb.end()) == command, "command reserved bytes retained");
  check(std::vector<std::uint8_t>(sb.begin(), sb.end()) == status, "status reserved bytes retained");
  auto unaligned = status; unaligned.insert(unaligned.begin(), 0);
  check(decode_status(unaligned.data() + 1, status.size()).timestamp == s.timestamp, "unaligned input");
  int rejected = 0;
  try { decode_command(command.data(), command.size() - 1); } catch (const std::invalid_argument&) { ++rejected; }
  try { decode_status(status.data(), status.size() + 1); } catch (const std::invalid_argument&) { ++rejected; }
  try { decode_status(nullptr, status.size()); } catch (const std::invalid_argument&) { ++rejected; }
  check(rejected == 3, "reject invalid buffers before access");
  std::cout << "wire35: installation fixtures, fields, reserved bytes and unaligned input passed\n";
}
