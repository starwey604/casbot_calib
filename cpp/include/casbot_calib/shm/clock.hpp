#pragma once
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace casbot::shm {
struct ClockPair {
  std::int64_t before=0,wall=0,after=0,offset=0;
  int segment=0;
  bool jumped=false;
};
// Map the master's millisecond wall timestamp to this machine's monotonic clock.
// A segment keeps its initial offset; clock discontinuities start a new segment.
class SourceClock {
 public:
  ClockPair accept(std::int64_t before,std::int64_t wall,std::int64_t after) {
    if(before<0||wall<0||after<before)throw std::invalid_argument("Invalid local clock bracket");
    ClockPair p{before,wall,after,wall-(before+(after-before)/2),current_.segment,false};
    if(have_) {
      p.jumped=std::abs(static_cast<long double>(p.offset)-current_.offset)>
        5000000.L+(after-before)+(current_.after-current_.before);
      if(p.jumped)++p.segment;else p.offset=current_.offset;
    }
    have_=true;current_=p;return p;
  }
  std::int64_t observation(std::uint64_t stamp_ms)const {
    if(!have_)throw std::logic_error("Clock pair required before source time mapping");
    const auto maximum=std::numeric_limits<std::int64_t>::max();
    if(stamp_ms>static_cast<std::uint64_t>(maximum/1000000))throw std::overflow_error("Source timestamp overflow");
    const auto source=static_cast<std::int64_t>(stamp_ms)*1000000;
    if(current_.offset<0&&source>maximum+current_.offset)throw std::overflow_error("Mapped timestamp overflow");
    return source-current_.offset;
  }
  int segment()const{return current_.segment;}
 private:
  ClockPair current_{};bool have_=false;
};
}  // namespace casbot::shm
