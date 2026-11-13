#pragma once
#include <cstdint>
#include <stdexcept>
#include <time.h>
namespace casbot::shm {
inline std::int64_t mono() {
  timespec t{};if(clock_gettime(CLOCK_MONOTONIC,&t))throw std::runtime_error("clock_gettime monotonic");
  return t.tv_sec*1000000000LL+t.tv_nsec;
}
}
