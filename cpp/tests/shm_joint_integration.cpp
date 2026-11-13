#include <casbot_calib/shm/channel.hpp>
#include <casbot_calib/shm/arm_jog.hpp>
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

void check(bool x,const char* m){if(!x)throw std::runtime_error(m);}
int main(int argc,char** argv) {
  if(argc!=5){std::cerr<<"Usage: shm_joint_integration EXECUTABLE URDF BACKGROUND OUTPUT\n";return 2;}
  using namespace casbot::shm;
  const auto root=std::filesystem::path(argv[4]);
  check(std::filesystem::create_directory(root),"New output required");
  for(int test=0;test<3;++test) {
    const bool fault=test==1,limit=test==2;
    const auto name="/casbot_calib_test_"+std::to_string(getpid())+"_"+std::to_string(test);
    Channel master(name,ChannelRole::fixture_master);
    {Channel startup(name);startup.activate_fresh_master();}
    std::atomic<bool> finished{false},disabled{false},bad_background{false};std::atomic<int> commands_seen{0};
    std::thread server([&] {
      StatusFrame state{};state.hand_count=2;
      CommandFrame target{};
      for(int j=0;j<14;++j)arm_target(target,j,j==1?.3490658504:j==8?-.3490658504:0,0,20,2);
      for(int j=0;j<29;++j){state.joints[j].mode=-1;state.joints[j].status_word=0x21;}
      for(int j=12;j<26;++j)state.joints[j].position=target.joints[j].position;
      int enabled_ticks=0;
      while(!finished) {
        if(master.fixture_receive(target)) {
          ++commands_seen;
          disabled=true;
          for(int j=0;j<29;++j) {
            const auto& c=target.joints[j];
            if(j<12||j>25)if(c.control_word!=6||c.torque!=0)bad_background=true;
            if(c.control_word==15)disabled=false;
            if(c.control_word==0x80)bad_background=true;
            state.joints[j].status_word=c.control_word==15?(fault?0x37:0x237):(fault?0x21:0x231);
            state.joints[j].mode=c.mode;
            state.joints[j].position=c.position;
          }
          if(!disabled)++enabled_ticks;
          if(fault&&enabled_ticks>30)state.joints[12].error_code=42;
        }
        state.frame_counter++;
        state.timestamp=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
        master.fixture_send(state);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    const auto output=(root/(limit?"limit":fault?"fault":"nominal")).string();
    const auto log=(root/(limit?"limit.log":fault?"fault.log":"nominal.log")).string();
    const auto pid=fork();
    if(pid==0) {
      const int fd=open(log.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);dup2(fd,1);dup2(fd,2);close(fd);
      execl(argv[1],argv[1],"--execute","--shm",name.c_str(),"--joint-id",limit?"15":"17","--amplitude",limit?"0.5":"0.01","--segment-s",limit?"5":"1",
        "--urdf",argv[2],"--hand-background",argv[3],"--output",output.c_str(),static_cast<char*>(nullptr));
      _exit(127);
    }
    if(pid<0){finished=true;server.join();throw std::runtime_error("fork failed");}
    int status=0;waitpid(pid,&status,0);finished=true;server.join();
    check(WIFEXITED(status)&&WEXITSTATUS(status)==(limit?4:fault?1:0),"Unexpected child exit; inspect fixture logs");
    if(limit)check(commands_seen==0,"URDF limit rejection must send NO command");
    else check(disabled&&!bad_background,"Exit must disable, preserve background and never reset faults");
    std::ifstream result(std::filesystem::path(output)/"result.txt");
    const std::string text{std::istreambuf_iterator<char>(result),std::istreambuf_iterator<char>()};
    check(text.find(limit?"SKIP_urdf_limit":fault?"drive_error_slot_12_42":"execution_completed=1")!=std::string::npos,"Missing outcome");
  }
  std::cout<<"Protocol fixture only (no physics): enable/jog/disable, fault termination, URDF skip-without-send passed\n";
}
