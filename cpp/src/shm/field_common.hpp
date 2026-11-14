#pragma once
#include <casbot_calib/shm/channel.hpp>
#include <casbot_calib/shm/arm_codec.hpp>
#include <casbot_calib/shm/clock.hpp>
#include "native_clock.hpp"
#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <memory>
#include <stdexcept>
namespace casbot::shm {
struct OwnerLease {
  int fd=-1;
  explicit OwnerLease(std::string name) {
    if(name.empty()||name[0]!='/')name='/'+name;
    fd=shm_open(name.c_str(),O_RDONLY,0);
    if(fd<0)throw std::runtime_error("Owner lease: SHM not available");
    if(flock(fd,LOCK_EX|LOCK_NB)){close(fd);fd=-1;throw std::runtime_error("Another fieldwork owner is using this SHM endpoint");}
  }
  OwnerLease(const OwnerLease&)=delete;
  ~OwnerLease(){if(fd>=0)close(fd);}
};
class ClockMap : public SourceClock {
 public:
  ClockPair pair() {
    const auto before=mono();timespec w{};
    if(clock_gettime(CLOCK_REALTIME,&w))throw std::runtime_error("clock_gettime realtime");
    const auto after=mono();
    return accept(before,w.tv_sec*1000000000LL+w.tv_nsec,after);
  }
};
using ArmMask=std::array<bool,14>;
inline bool drive_ack(const StatusFrame& s,unsigned word,const ArmMask& selected,bool mit=false) {
  for(int j=0;j<14;++j) {
    const auto& a=s.joints[j+12];
    if(!drive_state_is(a.status_word,selected[j]?word:0x21)||(mit&&selected[j]&&a.mode!=-1))return false;
  }
  return true;
}
// The same CiA402 handshake is used by finite CLI cases and persistent GUI sessions.
inline int drive_phase_next(int phase,const StatusFrame& s,const ArmMask& selected) {
  if(phase==0) {
    if(!drive_ack(s,0x21,selected))return phase;
    for(int j=0;j<29;++j)if((j<12||j>25)&&(s.joints[j].status_word&4))return phase;
    return 1;
  }
  if(phase==1&&drive_ack(s,0x21,selected,true))return 2;
  if(phase==2&&drive_ack(s,0x27,selected,true))return 3;
  if(phase==4&&drive_ack(s,0x21,selected))return -1;
  return phase;
}
inline CommandFrame initial_command(const StatusFrame& s,const CommandFrame& hand,
    const std::array<double,14>& q,double kp,double kd) {
  CommandFrame c{};c.joint_count=29;c.hand_count=hand.hand_count;c.hands=hand.hands;
  for(int i=0;i<29;++i){c.joints[i].position=s.joints[i].position;c.joints[i].control_word=6;c.joints[i].mode=s.joints[i].mode;}
  for(int j=0;j<14;++j)arm_target(c,j,q[j],0,kp,kd);
  return c;
}
}
