#include <casbot_calib/shm/channel.hpp>
#include <stdexcept>
#include <iostream>
#include <unistd.h>

void check(bool value) { if (!value) throw std::runtime_error("SHM channel test failed"); }
int main() {
  using namespace casbot::shm;
  const auto name = "/casbot_calib_test_" + std::to_string(getpid());
  {
    Channel master(name, ChannelRole::fixture_master);
    Channel client(name);
    CommandFrame command{}; command.frame_counter = 17; command.joint_count = 29;
    check(client.send(command)); check(!client.send(command));
    // Attaching another object must NOT reset shared indices.
    Channel observer(name);
    CommandFrame received{}; check(master.fixture_receive(received));
    check(received.frame_counter == 17); check(!master.fixture_receive(received));
    for (int j = 0; j < 30; ++j) {
      command.frame_counter++; check(client.send(command));
      check(master.fixture_receive(received)); check(received.frame_counter == command.frame_counter);
    }
    StatusFrame status{}; status.frame_counter = 42; status.joints[12].position = -123456;
    check(master.fixture_send(status));
    const auto snapshot = passive_snapshot(name);
    check(snapshot.status.frame_counter == 42);
    StatusFrame feedback{}; check(client.receive(feedback));
    check(feedback.joints[12].position == -123456); check(!client.receive(feedback));
  }
  bool absent = false;
  try { Channel removed(name); } catch (const std::runtime_error&) { absent = true; }
  check(absent);
  bool refused = false;
  try { Channel unsafe("/shm_mc", ChannelRole::fixture_master); } catch (const std::invalid_argument&) { refused = true; }
  check(refused);
  std::cout << "private channel: vendor ring indices, capacity-1, attach, snapshot and cleanup passed\n";
}
