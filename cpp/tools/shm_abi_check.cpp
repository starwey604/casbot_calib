#include <casbot_calib/shm/wire35.hpp>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc > 1) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
      std::cout << "Usage: shm_abi_check\nOffline wire35 ABI check; never opens shared memory.\n";
      return 0;
    }
    std::cerr << "Usage: shm_abi_check [--help]\n";
    return 2;
  }
  using namespace casbot::shm;
  std::cout << "{\"abi\":\"casbot_02_35_compat\",\"command_bytes\":" << sizeof(CommandFrame)
            << ",\"status_bytes\":" << sizeof(StatusFrame)
            << ",\"pointer_bits\":" << sizeof(void*) * 8
            << ",\"little_endian\":" << (native_wire_compatible() ? "true" : "false")
            << ",\"opens_shm\":false}\n";
  return native_wire_compatible() ? 0 : 1;
}
