// Explicit private protocol fixture, NOT a physics simulator or real master.
#include <casbot_calib/shm/channel.hpp>
#include <casbot_calib/shm/arm_jog.hpp>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
namespace {volatile std::sig_atomic_t finished=0;void stop(int){finished=1;}}
int main(int argc,char** argv) {
  if(argc<4||argc>5){std::cerr<<"Usage: shm_field_fixture PRIVATE_NAME READY_FILE COMMAND_LOG [FAULT_AFTER]\n";return 2;}
  using namespace casbot::shm;
  std::signal(SIGTERM,stop);std::signal(SIGINT,stop);
  try {
    Channel master(argv[1],ChannelRole::fixture_master);
    {Channel start(argv[1]);start.activate_fresh_master();}
    std::ofstream log(argv[3],std::ios::binary);log.exceptions(std::ios::badbit|std::ios::failbit);
    StatusFrame state{};state.hand_count=2;CommandFrame target{};
    for(int j=0;j<14;++j)arm_target(target,j,j==1?.3490658504:j==8?-.3490658504:0,0,20,2);
    for(int j=0;j<29;++j){state.joints[j].status_word=0x231;state.joints[j].mode=0;}
    for(int j=12;j<26;++j)state.joints[j].position=target.joints[j].position;
    // Make hand feedback preservation testable, including a nonzero current.
    state.hands[0].joints[0].position=123;state.hands[1].joints[5].current=-7;
    std::ofstream(argv[2])<<"Private protocol fixture ready (not physics)\n";
    const int fault_after=argc==5?std::stoi(argv[4]):-1;int enabled=0;
    while(!finished) {
      if(master.fixture_receive(target)) {
        const auto at=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        log.write(reinterpret_cast<const char*>(&at),8);log.write(reinterpret_cast<const char*>(&target),sizeof(target));log.flush();
        for(int j=0;j<29;++j) {
          const auto& c=target.joints[j];state.joints[j].status_word=c.control_word==15?0x237:0x231;
          state.joints[j].mode=c.mode;state.joints[j].position=c.position;state.joints[j].velocity=c.velocity;state.joints[j].torque=c.torque;
        }
        if(target.joints[12].control_word==15)++enabled;
        if(fault_after>=0&&enabled>fault_after)state.joints[12].error_code=42;
      }
      ++state.frame_counter;
      state.timestamp=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      master.fixture_send(state);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
