#include <casbot_calib/arm_dynamics.hpp>
#include <casbot_calib/integration.hpp>
#include <iostream>
int main() {
  std::cout << casbot::ArmDynamics::backend_version() << '\n'
            << casbot::integration_backend() << '\n';
  return 0;
}
