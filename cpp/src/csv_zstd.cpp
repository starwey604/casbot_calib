#include "csv_zstd.hpp"
#include "../third_party/fast_float.h"
#include <zstd.h>

#include <algorithm>
#include <charconv>
#include <fstream>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace casbot::recording {
namespace {

constexpr std::size_t page_size = 64 * 1024;
constexpr std::size_t command_columns = 4 + 14 * 5 + 29 * 7;
constexpr std::size_t feedback_columns = 7 + 14 * 3 + 35 * 6 + 2 * (2 + 6 * 4);

void add_columns(std::string& header, std::size_t count,
                 std::initializer_list<std::string_view> prefixes) {
  for (const auto prefix : prefixes) {
    for (std::size_t index = 0; index < count; ++index) {
      header += ',';
      header += prefix;
      header += std::to_string(index);
    }
  }
}

template <typename Hand>
auto hand_fields(Hand& h) {
  return std::tie(h.global_status, h.error, h.pos, h.vel, h.cur, h.status);
}

template <typename Row>
auto feedback_fields(Row& r) {
  return std::tuple_cat(
      std::tie(r.receive_mono_ns, r.source_timestamp_ms, r.frame_counter,
               r.observation_time_ns, r.clock_segment, r.passive, r.phase,
               r.q, r.dq, r.effort_raw, r.pos_raw, r.vel_raw, r.trq_raw, r.status, r.mode, r.error),
      hand_fields(r.hands[0]), hand_fields(r.hands[1]));
}

template <typename Row>
auto command_fields(Row& r) {
  return std::tie(r.send_mono_ns, r.phase, r.frame_counter, r.hand_count, r.q, r.v, r.effort, r.kp, r.kd,
                  r.cmd_pos_raw, r.cmd_vel_raw, r.cmd_trq_raw, r.cmd_kp_raw, r.cmd_kd_raw,
                  r.cmd_cw, r.cmd_mode);
}

std::size_t check_zstd(std::size_t result) {
  if (ZSTD_isError(result)) throw std::runtime_error(ZSTD_getErrorName(result));
  return result;
}

struct LineWriter {
  std::string& output;

  template <typename T>
  void add(T value) {
    char digits[32];
    const auto result = std::to_chars(digits, digits + sizeof(digits), value);
    if (result.ec != std::errc{}) throw std::runtime_error("CSV number formatting failed");
    output.append(digits, result.ptr);
    output += ',';
  }

  template <typename T, std::size_t N>
  void add(const std::array<T, N>& values) {
    for (auto value : values) add(value);
  }

  template <typename Tuple>
  void write(Tuple fields) {
    std::apply([this](const auto&... values) { (add(values), ...); }, fields);
    output.back() = '\n';
  }
};

class LineParser {
 public:
  explicit LineParser(std::string_view line) : current_(line.data()), end_(current_ + line.size()) {}

  template <typename T>
  bool read(T& value) {
    const auto result = fast_float::from_chars(current_, end_, value);
    if (result.ec != std::errc{}) return false;
    current_ = result.ptr;
    if (current_ == end_) return true;
    if (*current_ != ',') return false;
    ++current_;
    return current_ != end_;
  }

  template <typename T, std::size_t N>
  bool read(std::array<T, N>& values) {
    for (auto& value : values) if (!read(value)) return false;
    return true;
  }

  template <typename Tuple>
  bool parse(Tuple fields) {
    return std::apply([this](auto&... values) { return (read(values) && ...); }, fields)
        && current_ == end_;
  }

  bool read_video(VideoRow& row) {
    if (!read(row.receive_mono_ns)) return false;
    for (int field = 0; field < 6; ++field) {
      current_ = std::find(current_, end_, ',');
      if (current_ == end_) return false;
      ++current_;
    }
    return read(row.q) && std::count(current_, end_, ',') == feedback_columns - 7 - 14 - 1;
  }

  bool read_timing(CommandTimingRow& row) {
    return read(row.send_mono_ns) && read(row.phase) && read(row.frame_counter)
        && std::count(current_, end_, ',') == command_columns - 3 - 1;
  }

 private:
  const char* current_;
  const char* end_;
};

std::ifstream open_input(const std::filesystem::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot open CSV recording: " + path.string());
  stream.exceptions(std::ios::badbit);
  return stream;
}

class FrameReader {
 public:
  explicit FrameReader(const std::filesystem::path& path)
      : stream_(open_input(path)), decoder_(ZSTD_createDCtx(), ZSTD_freeDCtx) {
    if (!decoder_) throw std::bad_alloc();
    stream_.seekg(0, std::ios::end);
    const auto size = stream_.tellg();
    if (size < 0) throw std::runtime_error("Cannot determine CSV recording length");
    unread_ = static_cast<std::uint64_t>(size);
    stream_.seekg(0);
  }

  bool next() {
    frame.clear();
    while (!truncated) {
      if (input_.pos == input_.size && unread_) {
        const auto length = static_cast<std::streamsize>(std::min<std::uint64_t>(unread_, input_page_.size()));
        stream_.read(input_page_.data(), length);
        if (stream_.gcount() != length) throw std::runtime_error("CSV recording shrank while reading");
        input_ = {input_page_.data(), static_cast<std::size_t>(length), 0};
        unread_ -= static_cast<std::uint64_t>(length);
      }
      if (input_.pos == input_.size && !in_frame_) return false;
      const auto before = input_.pos;
      ZSTD_outBuffer out{output_page_.data(), output_page_.size(), 0};
      const auto result = ZSTD_decompressStream(decoder_.get(), &out, &input_);
      if (ZSTD_isError(result)) {
        truncated = true;
        return false;
      }
      consumed += input_.pos - before;
      frame.append(output_page_.data(), out.pos);
      in_frame_ = result != 0;
      if (!in_frame_) return true;
      if (input_.pos == before && out.pos == 0) {
        truncated = true;
        return false;
      }
    }
    return false;
  }

  std::string frame;
  std::uint64_t consumed = 0;
  bool truncated = false;

 private:
  std::ifstream stream_;
  std::unique_ptr<ZSTD_DCtx, decltype(&ZSTD_freeDCtx)> decoder_;
  std::array<char, page_size> input_page_, output_page_;
  ZSTD_inBuffer input_{nullptr, 0, 0};
  std::uint64_t unread_ = 0;
  bool in_frame_ = false;
};

std::string_view take_line(std::string_view& text) {
  const auto size = text.find('\n');
  if (size == std::string_view::npos) return {};
  const auto line = text.substr(0, size);
  text.remove_prefix(size + 1);
  return line;
}

std::string_view last_line(std::string_view text) {
  if (text.empty() || text.back() != '\n') return {};
  text.remove_suffix(1);
  return text.substr(text.rfind('\n') + 1);
}

bool is_feedback_data(std::string_view line) {
  FeedbackRow row;
  return !line.empty() && LineParser(line).parse(feedback_fields(row));
}

}  // namespace

const std::string& csv_header(Table table) {
  static const std::string commands = [] {
    std::string header = "send_mono_ns,phase,frame_counter,hand_count";
    add_columns(header, 14, {"q", "v", "effort", "kp", "kd"});
    add_columns(header, 29, {"cmd_pos_raw", "cmd_vel_raw", "cmd_trq_raw", "cmd_kp_raw",
                             "cmd_kd_raw", "cmd_cw", "cmd_mode"});
    return header + '\n';
  }();
  static const std::string feedback = [] {
    std::string header = "receive_mono_ns,source_timestamp_ms,frame_counter,observation_time_ns,clock_segment,passive,phase";
    add_columns(header, 14, {"q", "v", "effort"});
    add_columns(header, 35, {"pos_raw", "vel_raw", "trq_raw", "status", "mode", "error"});
    for (int hand = 0; hand < 2; ++hand) {
      const std::string prefix = "hand" + std::to_string(hand) + "_";
      header += ',' + prefix + "global_status," + prefix + "error";
      add_columns(header, 6, {prefix + "pos", prefix + "vel", prefix + "cur", prefix + "status"});
    }
    return header + '\n';
  }();
  return table == Table::feedback ? feedback : commands;
}

struct CsvWriter::Impl {
  std::ofstream file;
  std::unique_ptr<ZSTD_CCtx, decltype(&ZSTD_freeCCtx)> encoder{ZSTD_createCCtx(), ZSTD_freeCCtx};
  std::string page;
  std::vector<char> compressed;
  std::uint64_t bytes = 0;

  Impl(const std::filesystem::path& path, bool append)
      : file(path, std::ios::binary | (append ? std::ios::app : std::ios::trunc)) {
    file.exceptions(std::ios::failbit | std::ios::badbit);
    if (!encoder) throw std::bad_alloc();
    check_zstd(ZSTD_CCtx_setParameter(encoder.get(), ZSTD_c_compressionLevel, 1));
    check_zstd(ZSTD_CCtx_setParameter(encoder.get(), ZSTD_c_checksumFlag, 1));
    page.reserve(page_size);
    compressed.resize(ZSTD_compressBound(page_size));
  }
};

CsvWriter::CsvWriter(const std::filesystem::path& path, bool append)
    : impl_(std::make_unique<Impl>(path, append)) {}
CsvWriter::~CsvWriter() = default;

void CsvWriter::append(std::string_view text) { impl_->page += text; }
void CsvWriter::append(const CommandRow& row) { LineWriter{impl_->page}.write(command_fields(row)); }
void CsvWriter::append(const FeedbackRow& row) { LineWriter{impl_->page}.write(feedback_fields(row)); }
std::uint64_t CsvWriter::bytes() const { return impl_->bytes; }

void CsvWriter::flush_frame() {
  auto& writer = *impl_;
  if (writer.page.empty()) return;
  const auto bound = ZSTD_compressBound(writer.page.size());
  if (writer.compressed.size() < bound) writer.compressed.resize(bound);
  ZSTD_inBuffer in{writer.page.data(), writer.page.size(), 0};
  ZSTD_outBuffer out{writer.compressed.data(), writer.compressed.size(), 0};
  check_zstd(ZSTD_CCtx_setPledgedSrcSize(writer.encoder.get(), writer.page.size()));
  const auto pending = check_zstd(ZSTD_compressStream2(writer.encoder.get(), &out, &in, ZSTD_e_end));
  if (pending != 0) throw std::runtime_error("CSV zstd frame did not fit its compression bound");
  writer.file.write(writer.compressed.data(), out.pos);
  writer.file.flush();
  writer.bytes += out.pos;
  writer.page.clear();
}

struct CsvReader::Impl {
  FrameReader frames;
  std::string_view remaining;
  bool truncated = false;

  Impl(const std::filesystem::path& path, Table table) : frames(path) {
    if (!frames.next()) {
      truncated = true;
      return;
    }
    remaining = frames.frame;
    const auto& header = csv_header(table);
    if (remaining.substr(0, header.size()) != header) {
      throw std::invalid_argument("CSV recording has an unexpected header: " + path.string());
    }
    remaining.remove_prefix(header.size());
  }

  template <typename Parse>
  bool next(Parse parse) {
    if (truncated) return false;
    while (remaining.empty()) {
      if (!frames.next()) {
        truncated = frames.truncated;
        return false;
      }
      remaining = frames.frame;
    }
    const auto line = take_line(remaining);
    if (!line.empty() && parse(LineParser(line))) return true;
    // Only an incomplete last row is a recoverable CSV tail.
    if (!remaining.empty() && remaining.back() == '\n') {
      throw std::invalid_argument("Invalid row in CSV recording");
    }
    if (frames.next()) throw std::invalid_argument("Invalid row before end of CSV recording");
    truncated = true;
    return false;
  }
};

CsvReader::CsvReader(const std::filesystem::path& path, Table table)
    : impl_(std::make_unique<Impl>(path, table)) {}
CsvReader::~CsvReader() = default;
bool CsvReader::truncated_tail() const { return impl_->truncated; }
bool CsvReader::next(CommandRow& row) {
  return impl_->next([&](LineParser parser) { return parser.parse(command_fields(row)); });
}
bool CsvReader::next(CommandTimingRow& row) {
  return impl_->next([&](LineParser parser) { return parser.read_timing(row); });
}
bool CsvReader::next(FeedbackRow& row) {
  return impl_->next([&](LineParser parser) { return parser.parse(feedback_fields(row)); });
}
bool CsvReader::next(VideoRow& row) {
  return impl_->next([&](LineParser parser) { return parser.read_video(row); });
}

std::string last_feedback_line(const std::filesystem::path& path) {
  auto stream = open_input(path);
  stream.seekg(0, std::ios::end);
  const auto size = static_cast<std::streamoff>(stream.tellg());
  if (size < 0) throw std::runtime_error("Cannot determine CSV recording length");
  const auto span = std::min<std::streamoff>(size, page_size);
  std::string tail(static_cast<std::size_t>(span), '\0');
  stream.seekg(size - span);
  stream.read(tail.data(), span);
  if (stream.gcount() != span) throw std::runtime_error("CSV recording shrank while reading");

  const std::string_view magic("\x28\xb5\x2f\xfd", 4);
  auto candidate = tail.size();
  while (candidate > 0) {
    candidate = tail.rfind(magic, candidate - 1);
    if (candidate == std::string::npos) break;
    const auto* start = tail.data() + candidate;
    const auto length = tail.size() - candidate;
    if (ZSTD_findFrameCompressedSize(start, length) != length) continue;
    const auto decoded_size = ZSTD_getFrameContentSize(start, length);
    if (decoded_size > page_size) continue;
    std::string decoded(static_cast<std::size_t>(decoded_size), '\0');
    if (ZSTD_decompress(decoded.data(), decoded.size(), start, length) != decoded.size()) continue;
    const auto line = last_line(decoded);
    if (is_feedback_data(line)) return std::string(line);
  }

  // Recover the last complete row from a damaged tail or an unusually large frame.
  FrameReader frames(path);
  std::string last;
  while (frames.next()) {
    auto remaining = std::string_view(frames.frame);
    while (!remaining.empty()) {
      const auto line = take_line(remaining);
      if (line.empty()) break;
      if (is_feedback_data(line)) last = line;
    }
  }
  if (last.empty()) throw std::invalid_argument("快照没有完整反馈");
  return last;
}

std::size_t copy_complete_prefix(const std::filesystem::path& src,
                                 const std::filesystem::path& dst,
                                 const std::function<bool()>& cancel) {
  FrameReader frames(src);
  const auto check_cancel = [&] {
    if (cancel && cancel()) throw std::invalid_argument("保存已取消，已写前缀保留");
  };
  std::uint64_t bound = 0;
  std::size_t rows = 0;
  std::size_t columns = 0;
  while (frames.next()) {
    check_cancel();
    std::string_view body(frames.frame);
    if (!bound) {
      const auto header = take_line(body);
      const auto& feedback = csv_header(Table::feedback);
      const auto& commands = csv_header(Table::commands);
      if (header == std::string_view(feedback).substr(0, feedback.size() - 1)) {
        columns = feedback_columns;
      } else if (header == std::string_view(commands).substr(0, commands.size() - 1)) {
        columns = command_columns;
      } else {
        throw std::invalid_argument("CSV recording has an unexpected header");
      }
    }
    std::size_t frame_rows = 0;
    bool complete = true;
    while (!body.empty()) {
      const auto line = take_line(body);
      if (line.empty() || static_cast<std::size_t>(std::count(line.begin(), line.end(), ',')) != columns - 1) {
        complete = false;
        break;
      }
      ++frame_rows;
    }
    if (!complete) break;
    bound = frames.consumed;
    rows += frame_rows;
  }
  if (!bound) throw std::invalid_argument("CSV recording has no complete zstd header frame");

  auto input = open_input(src);
  std::ofstream output(dst, std::ios::binary);
  output.exceptions(std::ios::failbit | std::ios::badbit);
  std::array<char, page_size> page;
  while (bound) {
    check_cancel();
    const auto size = static_cast<std::streamsize>(std::min<std::uint64_t>(bound, page.size()));
    input.read(page.data(), size);
    if (input.gcount() != size) throw std::runtime_error("CSV recording shrank while saving");
    output.write(page.data(), size);
    bound -= size;
  }
  output.close();
  return rows;
}

}  // namespace casbot::recording
