// Private EtherCAT-compatible SHM master with the shared double-arm physical plant.
#include <casbot_calib/shm/channel.hpp>
#include <casbot_calib/shm/arm_codec.hpp>
#include <casbot_calib/simulated_device.hpp>
#include "json_input.hpp"
#include "native_clock.hpp"
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <type_traits>
#include <semaphore.h>
#include <sys/prctl.h>
#include <unistd.h>
namespace {
volatile std::sig_atomic_t stopped=0;
void stop(int){stopped=1;}
}
int main(int argc,char** argv) {
  using namespace casbot::shm;
  std::signal(SIGINT,stop);std::signal(SIGTERM,stop);std::signal(SIGHUP,stop);
  std::string name,config_file,output;
  try {
    for(int i=1;i<argc;++i){const std::string key=argv[i];if(key=="--help"){std::cout<<"shm_mock_server --shm /casbot_calib_test_* --config FILE --output NEW_DIR\n";return 0;}
      if(i+1==argc)throw std::runtime_error("Missing mock option value");const std::string value=argv[++i];
      if(key=="--shm")name=value;else if(key=="--config")config_file=value;else if(key=="--output")output=value;else throw std::runtime_error("Unknown mock option");}
    if(name.rfind("/casbot_calib_test_",0)!=0||config_file.empty()||output.empty())throw std::runtime_error("Explicit private name, config and new output required; never shm_mc");
    const auto config=read_json(config_file);
    if(config.get<std::string>("schema")!="casbot_shm_mock_v1")throw std::runtime_error("Unsupported mock config");
    const auto settings=casbot::simulated_arm_settings(config.get<std::string>("urdf"),[&](const std::string& key,auto fallback) {
      using T=std::decay_t<decltype(fallback)>;
      if constexpr(std::is_same_v<T,bool>){if(key=="zero_gravity")return false;}
      const auto value=config.get_child_optional(key);
      if(!value)return fallback;
      if constexpr(std::is_same_v<T,std::vector<double>>){const auto a=joint_values(*value);return std::vector<double>(a.begin(),a.end());}
      else if constexpr(std::is_same_v<T,bool>)return json_bool(*value);
      else return value->get_value<T>();
    });
    auto model=settings.model;
    if(model.joints.size()!=14)throw std::runtime_error("Mock requires 14 arm axes");
    for(int j=0;j<14;++j)if(model.joints[j].name!=arm_names[j])throw std::runtime_error("Mock model joint order mismatch");
    const auto initial=settings.initial_q;
    for(int j=0;j<14;++j)if(initial[j]<model.joints[j].lower||initial[j]>model.joints[j].upper)throw std::runtime_error("Mock initial q outside URDF");
    const auto gravity=config.get_child("gravity_world");if(gravity.size()!=3)throw std::runtime_error("Three gravity values required");int gi=0;
    for(const auto& g:gravity)model.gravity_world[gi++]=finite_number(g.second);
    auto physical=settings.simulation;
    for(const auto& joint:model.joints)if((joint.friction.coulomb>0||joint.friction.static_friction>0)&&!physical.stiction_enabled)
      throw std::runtime_error("Dry friction requires explicit coupled stiction_enabled=true");
    physical.integration_ns=config.get<std::int64_t>("integration_ns");physical.sample_period_ns=1000000;
    physical.command_delay_ns=0;physical.feedback_delay_ns=0;physical.position_noise=0;physical.current_noise=0;
    physical.current_tau=config.get<double>("current_tau_s");physical.torque_limit=config.get<double>("torque_limit_nm");physical.gain=Eigen::VectorXd::Constant(14,config.get<double>("gain_nm_a"));
    if(physical.integration_ns>50000)throw std::runtime_error("Mock integration must be <=50us");
    casbot::SimulatedArmDevice device(casbot::ArmDynamics(model),physical,initial);
    CommandFrame held{};Eigen::VectorXi enabled=Eigen::VectorXi::Zero(14);
    const bool initial_on=config.get<bool>("initial_drive_enabled",false);
    if(initial_on) {
      const auto f=config.get_child("initial_fields");
      const auto q=joint_values(f.get_child("position")),v=joint_values(f.get_child("velocity")),
        torque=joint_values(f.get_child("effort")),kp=joint_values(f.get_child("kp")),kd=joint_values(f.get_child("kd"));
      casbot::ArmCommand command;command.send_ns=0;command.q.resize(14);command.dq.resize(14);command.torque.resize(14);command.kp.resize(14);command.kd.resize(14);
      for(int j=0;j<14;++j) {
        if(kp[j]<0||kp[j]>500||kd[j]<0||kd[j]>50)throw std::runtime_error("Initial PD outside bounds");
        arm_target(held,j,q[j],v[j],kp[j],kd[j],torque[j]);held.joints[j+12].control_word=15;held.joints[j+12].mode=-1;
        const auto value=arm_command(held,j);command.q[j]=value.position;command.dq[j]=value.velocity;command.torque[j]=value.effort;command.kp[j]=value.kp;command.kd[j]=value.kd;
      }
      enabled.setOnes();device.submit(command);
    }
    device.set_drive_enabled(0,enabled);device.advance_to(0);
    const auto root=std::filesystem::path(output);if(!root.parent_path().empty())std::filesystem::create_directories(root.parent_path());
    if(!std::filesystem::create_directory(root))throw std::runtime_error("Mock output must be new");
    std::filesystem::copy_file(config_file,root/"config.json");
    // Same queue ABI and semaphores, but exclusive ownership of a private object.
    Channel channel(name,ChannelRole::fixture_master);
    const auto parent=getppid();if(parent<=1||prctl(PR_SET_PDEATHSIG,SIGTERM)||getppid()!=parent)throw std::runtime_error("Mock requires live owner");
    std::ofstream(root/"ready.txt")<<"private_physics_master_waiting_for_handshake\n";
    sem_t* start=sem_open((name+"_start").c_str(),0);if(start==SEM_FAILED)throw std::runtime_error("Mock start semaphore missing");
    struct Close{sem_t* s;~Close(){sem_close(s);}} close{start};
    while(!stopped){int value=0;if(sem_getvalue(start,&value))throw std::runtime_error("Mock startup sem_getvalue");if(value>0)break;std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    if(stopped)return 0;
    StatusFrame status{};status.hand_count=2;status.crc=65535; // Actual source joint_count=0, all slots retained.
    for(int j=0;j<29;++j)status.joints[j].status_word=0x231;
    if(initial_on)for(int j=12;j<26;++j){status.joints[j].status_word=0x237;status.joints[j].mode=-1;}
    const auto origin=mono();auto next=origin;std::int64_t received=0,published=0,dropped=0,max_late=0;
    const auto fault_after=config.get<std::int64_t>("fault_after_commands",-1);
    while(!stopped) {
      const auto now=mono(),virtual_time=now-origin;
      device.advance_to(virtual_time);
      if(channel.fixture_receive(held)) {
        ++received;
        casbot::ArmCommand command;command.send_ns=device.time_ns();command.q.resize(14);command.dq.resize(14);command.torque.resize(14);command.kp.resize(14);command.kd.resize(14);
        for(int j=0;j<29;++j) {
          auto& a=status.joints[j];const auto& c=held.joints[j];
          if(j>=12&&j<26) {
            if(a.mode!=c.mode&&(a.status_word&4)){a.error_code=0x1001;}
            if(c.control_word!=6&&c.control_word!=15)a.error_code=0x1002;
            if(fault_after>=0&&received>fault_after&&j==12)a.error_code=42;
            if(!(a.status_word&4))a.mode=c.mode;
            const bool on=!a.error_code&&c.control_word==15&&a.mode==-1;
            a.status_word=a.error_code?0x238:on?0x237:0x231;enabled[j-12]=on;
            const auto value=arm_command(held,j-12);command.q[j-12]=value.position;command.dq[j-12]=value.velocity;command.torque[j-12]=value.effort;command.kp[j-12]=value.kp;command.kd[j-12]=value.kd;
          }else {
            // Protocol fixture only: no leg/waist/head mechanics or invented SI.
            a.mode=c.mode;a.status_word=c.control_word==15?0x237:0x231;
          }
        }
        device.set_drive_enabled(device.time_ns(),enabled);device.submit(command);
      }
      const auto& feedback=device.latest();
      for(int j=0;j<14;++j)arm_status(status,j,feedback.q[j],feedback.dq[j],feedback.current[j]*physical.gain[j]);
      ++status.frame_counter;
      status.timestamp=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      if(channel.fixture_send(status))++published;else ++dropped;
      next+=1000000;max_late=std::max(max_late,mono()-next);
      timespec deadline{next/1000000000LL,next%1000000000LL};
      while(clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&deadline,nullptr)==EINTR&&!stopped){}
    }
    std::ofstream(root/"result.txt")<<"physics_model=shared_ArmSimulationSession\ninitial_drive_enabled="<<initial_on<<"\ncommands_adopted="<<received<<"\nstatus_published="<<published<<"\nstatus_queue_full="<<dropped<<"\nmax_late_ns="<<max_late<<"\nvirtual_time_ns="<<device.time_ns()<<"\n";
    return 0;
  }catch(const std::exception& e){std::cerr<<"MOCK ERROR "<<e.what()<<'\n';if(!output.empty()&&std::filesystem::is_directory(output))std::ofstream(std::filesystem::path(output)/"failure.txt")<<e.what()<<'\n';return 1;}
}
