#include <casbot_calib/shm/wire35.hpp>
#include <iostream>
int main() {
  casbot::shm::CommandFrame frame{};
  frame.joints[12].position = -123456;
  const auto bytes = casbot::shm::encode_command(frame);
  const auto copy = casbot::shm::decode_command(bytes.data(), bytes.size());
  if (copy.joints[12].position != -123456) return 1;
  std::cout << "installed casbot_calib::shm consumer passed\n";
}
