#pragma once
#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <sys/stat.h>

// Atomic mailbox replacements and ordinary in-place edits both invalidate this
// cache. Content is still parsed/validated on every changed file; no digest or
// generation convention is imposed on existing producers.
inline std::array<std::uint64_t,8> mailbox_stamp(const std::string& path) {
  struct stat value{};
  if(::stat(path.c_str(),&value))throw std::runtime_error("Command/settings mailbox unavailable");
  return {std::uint64_t(value.st_dev),std::uint64_t(value.st_ino),std::uint64_t(value.st_size),
    std::uint64_t(value.st_mtim.tv_sec),std::uint64_t(value.st_mtim.tv_nsec),
    std::uint64_t(value.st_ctim.tv_sec),std::uint64_t(value.st_ctim.tv_nsec),std::uint64_t(value.st_mode)};
}
