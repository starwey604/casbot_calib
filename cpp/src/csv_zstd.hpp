#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <string_view>

namespace casbot::recording {

struct CommandRow {
  std::int64_t send_mono_ns{};
  int phase{};
  std::uint64_t frame_counter{};
  std::uint16_t hand_count{};
  std::array<double, 14> q{}, v{}, effort{}, kp{}, kd{};
  std::array<std::int32_t, 29> cmd_pos_raw{}, cmd_vel_raw{};
  std::array<std::int16_t, 29> cmd_trq_raw{}, cmd_kp_raw{}, cmd_kd_raw{};
  std::array<std::uint16_t, 29> cmd_cw{};
  std::array<std::int16_t, 29> cmd_mode{};
};

struct HandFeedback {
  std::uint16_t global_status{}, error{};
  std::array<std::int16_t, 6> pos{}, vel{}, cur{};
  std::array<std::uint16_t, 6> status{};
};

struct FeedbackRow {
  std::int64_t receive_mono_ns{};
  std::uint64_t source_timestamp_ms{}, frame_counter{};
  std::int64_t observation_time_ns{};
  int clock_segment{};
  std::uint8_t passive{};
  int phase{};
  std::array<double, 14> q{}, dq{}, effort_raw{};
  std::array<std::int32_t, 35> pos_raw{}, vel_raw{};
  std::array<std::int16_t, 35> trq_raw{};
  std::array<std::uint16_t, 35> status{};
  std::array<std::int16_t, 35> mode{};
  std::array<std::uint16_t, 35> error{};
  std::array<HandFeedback, 2> hands{};
};

struct VideoRow {
  std::int64_t receive_mono_ns{};
  std::array<double, 14> q{};
};

struct CommandTimingRow {
  std::int64_t send_mono_ns{};
  int phase{};
  std::uint64_t frame_counter{};
};

enum class Table { commands, feedback };
const std::string& csv_header(Table table);

class CsvWriter {
 public:
  explicit CsvWriter(const std::filesystem::path& path, bool append = false);
  ~CsvWriter();
  void append(std::string_view text);
  void append(const CommandRow& row);
  void append(const FeedbackRow& row);
  void flush_frame();
  std::uint64_t bytes() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class CsvReader {
 public:
  CsvReader(const std::filesystem::path& path, Table table);
  ~CsvReader();
  bool next(CommandRow& row);
  bool next(CommandTimingRow& row);
  bool next(FeedbackRow& row);
  bool next(VideoRow& row);
  bool truncated_tail() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

std::string last_feedback_line(const std::filesystem::path& path);
std::size_t copy_complete_prefix(const std::filesystem::path& src,
                                 const std::filesystem::path& dst,
                                 const std::function<bool()>& cancel = {});

}  // namespace casbot::recording
