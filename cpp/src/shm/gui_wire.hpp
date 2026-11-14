#pragma once
#include "field_common.hpp"
#include <cstring>
#include <vector>
namespace casbot::shm {
// Versioned, explicit little-endian wire schema. No compiler struct layout on the network.
template<class T> void wire_int(std::vector<unsigned char>& b,T value) {
  using U=std::make_unsigned_t<T>;const auto v=static_cast<U>(value);
  for(unsigned i=0;i<sizeof(T);++i)b.push_back(static_cast<unsigned char>(v>>(8*i)));
}
inline void wire_double(std::vector<unsigned char>& b,double x){std::uint64_t bits;std::memcpy(&bits,&x,8);wire_int(b,bits);}
struct GuiSnapshot {
  std::uint64_t seq=0,control=0,revision=0,challenge=0,writes=0,skips=0;
  std::int64_t at=0,receive=0,observation=0,applied_at=0,expires=0,max_gap=0;
  int segment=0,phase=-1;bool ack=false,recording_stopped=false;
  StatusFrame status{};CommandFrame command{};
  std::vector<unsigned char> encode() const {
    std::vector<unsigned char> b{'S','G','T','1'};b.reserve(1800);wire_int<std::uint32_t>(b,1);
    wire_int(b,seq);wire_int(b,at);wire_int(b,receive);wire_int(b,observation);wire_int<std::int32_t>(b,segment);wire_int<std::int32_t>(b,phase);
    wire_int(b,control);wire_int(b,revision);wire_int(b,applied_at);wire_int(b,challenge);wire_int(b,expires);
    wire_int<std::uint32_t>(b,ack);wire_int<std::uint32_t>(b,recording_stopped);wire_int(b,writes);wire_int(b,skips);wire_int(b,max_gap);
    wire_int(b,status.frame_counter);wire_int(b,status.timestamp);
    for(int field=0;field<3;++field)for(int j=0;j<14;++j){auto a=arm_feedback(status,j);wire_double(b,field==0?a.position:field==1?a.velocity:a.effort);}
    for(int field=0;field<5;++field)for(int j=0;j<14;++j){auto a=arm_command(command,j);wire_double(b,field==0?a.position:field==1?a.velocity:field==2?a.effort:field==3?a.kp:a.kd);}
    for(const auto& a:status.joints){wire_int(b,a.status_word);wire_int(b,a.position);wire_int(b,a.velocity);wire_int(b,a.torque);wire_int(b,a.mode);wire_int(b,a.error_code);}
    for(const auto& h:status.hands){wire_int(b,h.global_status);wire_int(b,h.error_code);for(const auto& a:h.joints){wire_int(b,a.position);wire_int(b,a.velocity);wire_int(b,a.current);wire_int(b,a.status);}}
    return b;
  }
};
}
