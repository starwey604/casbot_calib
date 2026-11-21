#include "csv_zstd.hpp"
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include <stdexcept>
#include <vector>

namespace py = pybind11;
namespace rec = casbot::recording;
using namespace py::literals;
namespace {

using Size = py::ssize_t;

template <typename Row>
class Columns {
 public:
  explicit Columns(std::unique_ptr<std::vector<Row>> rows)
      : first_(rows->data()), count_(static_cast<Size>(rows->size())),
        owner_(rows.release(), [](void* data) { delete static_cast<std::vector<Row>*>(data); }) {}

  template <typename T>
  py::array scalar(T Row::* field) const {
    return py::array(py::dtype::of<T>(), {count_}, {sizeof(Row)},
                     count_ ? &(first_->*field) : nullptr, owner_);
  }

  template <typename T, std::size_t N>
  py::array matrix(std::array<T, N> Row::* field) const {
    return py::array(py::dtype::of<T>(), {count_, static_cast<Size>(N)}, {sizeof(Row), sizeof(T)},
                     count_ ? (first_->*field).data() : nullptr, owner_);
  }

 private:
  Row* first_;
  Size count_;
  py::capsule owner_;
};

template <typename Row>
auto read_rows(const std::filesystem::path& path, rec::Table table) {
  auto rows = std::make_unique<std::vector<Row>>();
  py::gil_scoped_release release;
  rec::CsvReader reader(path, table);
  Row row;
  while (reader.next(row)) rows->push_back(row);
  return std::make_pair(std::move(rows), reader.truncated_tail());
}

py::dict read_commands(const std::filesystem::path& path, bool timing_only) {
  if (timing_only) {
    using R = rec::CommandTimingRow;
    auto [rows, truncated] = read_rows<R>(path, rec::Table::commands);
    Columns<R> out(std::move(rows));
    return py::dict("send_mono_ns"_a=out.scalar(&R::send_mono_ns),
                    "phase"_a=out.scalar(&R::phase), "frame_counter"_a=out.scalar(&R::frame_counter),
                    "truncated_tail"_a=truncated);
  }
  auto [rows, truncated] = read_rows<rec::CommandRow>(path, rec::Table::commands);
  using R = rec::CommandRow;
  Columns<R> out(std::move(rows));
  return py::dict(
      "send_mono_ns"_a=out.scalar(&R::send_mono_ns), "phase"_a=out.scalar(&R::phase),
      "frame_counter"_a=out.scalar(&R::frame_counter), "hand_count"_a=out.scalar(&R::hand_count),
      "q"_a=out.matrix(&R::q),
      "v"_a=out.matrix(&R::v), "effort"_a=out.matrix(&R::effort),
      "kp"_a=out.matrix(&R::kp), "kd"_a=out.matrix(&R::kd),
      "cmd_pos_raw"_a=out.matrix(&R::cmd_pos_raw), "cmd_vel_raw"_a=out.matrix(&R::cmd_vel_raw),
      "cmd_trq_raw"_a=out.matrix(&R::cmd_trq_raw), "cmd_kp_raw"_a=out.matrix(&R::cmd_kp_raw),
      "cmd_kd_raw"_a=out.matrix(&R::cmd_kd_raw), "cmd_cw"_a=out.matrix(&R::cmd_cw),
      "cmd_mode"_a=out.matrix(&R::cmd_mode), "truncated_tail"_a=truncated);
}

py::dict read_feedback(const std::filesystem::path& path) {
  using R = rec::FeedbackRow;
  using H = rec::HandFeedback;
  auto [rows, truncated] = read_rows<R>(path, rec::Table::feedback);
  const auto count = static_cast<Size>(rows->size());
  const auto* first = rows->data();
  Columns<R> out(std::move(rows));
  py::dict result(
      "receive_mono_ns"_a=out.scalar(&R::receive_mono_ns),
      "source_timestamp_ms"_a=out.scalar(&R::source_timestamp_ms),
      "frame_counter"_a=out.scalar(&R::frame_counter),
      "observation_time_ns"_a=out.scalar(&R::observation_time_ns),
      "clock_segment"_a=out.scalar(&R::clock_segment), "passive"_a=out.scalar(&R::passive),
      "phase"_a=out.scalar(&R::phase), "q"_a=out.matrix(&R::q), "dq"_a=out.matrix(&R::dq),
      "effort_raw"_a=out.matrix(&R::effort_raw), "pos_raw"_a=out.matrix(&R::pos_raw),
      "vel_raw"_a=out.matrix(&R::vel_raw), "trq_raw"_a=out.matrix(&R::trq_raw),
      "status"_a=out.matrix(&R::status), "mode"_a=out.matrix(&R::mode), "error"_a=out.matrix(&R::error),
      "truncated_tail"_a=truncated);
  const auto owner = result["q"].cast<py::array>().base();
  const auto hand_scalar = [&](auto H::* field) {
    using T = std::decay_t<decltype(first->hands[0].*field)>;
    return py::array(py::dtype::of<T>(), {count, Size{2}}, {sizeof(R), sizeof(H)},
                     count ? &(first->hands[0].*field) : nullptr, owner);
  };
  const auto hand_matrix = [&](auto H::* field) {
    using Array = std::decay_t<decltype(first->hands[0].*field)>;
    using T = typename Array::value_type;
    return py::array(py::dtype::of<T>(), {count, Size{2}, static_cast<Size>(std::tuple_size_v<Array>)},
                     {sizeof(R), sizeof(H), sizeof(T)},
                     count ? (first->hands[0].*field).data() : nullptr, owner);
  };
  result["hand_global_status"] = hand_scalar(&H::global_status);
  result["hand_error"] = hand_scalar(&H::error);
  result["hand_pos"] = hand_matrix(&H::pos);
  result["hand_vel"] = hand_matrix(&H::vel);
  result["hand_cur"] = hand_matrix(&H::cur);
  result["hand_status"] = hand_matrix(&H::status);
  return result;
}

class FeedbackChunks {
 public:
  FeedbackChunks(const std::filesystem::path& path, std::size_t chunk_rows)
      : reader_(path, rec::Table::feedback), chunk_rows_(chunk_rows) {
    if (!chunk_rows_) throw std::invalid_argument("chunk_rows must be positive");
  }

  py::tuple next() {
    auto rows = std::make_unique<std::vector<rec::VideoRow>>();
    rows->reserve(chunk_rows_);
    {
      py::gil_scoped_release release;
      rec::VideoRow row;
      while (rows->size() < chunk_rows_ && reader_.next(row)) rows->push_back(row);
      if (reader_.truncated_tail()) throw std::invalid_argument("SHM反馈记录尾部不完整");
    }
    if (rows->empty()) throw py::stop_iteration();
    Columns<rec::VideoRow> out(std::move(rows));
    return py::make_tuple(out.scalar(&rec::VideoRow::receive_mono_ns), out.matrix(&rec::VideoRow::q));
  }

 private:
  rec::CsvReader reader_;
  std::size_t chunk_rows_;
};

py::dict last_feedback(const std::filesystem::path& path) {
  std::string line;
  {
    py::gil_scoped_release release;
    line = rec::last_feedback_line(path);
  }
  const auto& header = rec::csv_header(rec::Table::feedback);
  const auto names = py::str(header.substr(0, header.size() - 1)).attr("split")(",");
  const auto values = py::str(line).attr("split")(",");
  return py::dict(py::module_::import("builtins").attr("zip")(names, values));
}

}  // namespace

void bind_csv(py::module_& m) {
  m.def("read_feedback_csv_zst", read_feedback);
  m.def("read_commands_csv_zst", read_commands, "path"_a, "timing_only"_a=false);
  py::class_<FeedbackChunks>(m, "_FeedbackChunks")
      .def("__iter__", [](FeedbackChunks& self) -> FeedbackChunks& { return self; },
           py::return_value_policy::reference_internal)
      .def("__next__", &FeedbackChunks::next);
  m.def("iter_feedback_chunks_zst", [](const std::filesystem::path& path, std::size_t chunk_rows) {
    return std::make_unique<FeedbackChunks>(path, chunk_rows);
  }, "path"_a, "chunk_rows"_a=4096);
  m.def("last_feedback_csv_zst", last_feedback);
  m.def("copy_complete_csv_zst_prefix", [](const std::filesystem::path& src,
                                           const std::filesystem::path& dst, py::object cancel) {
    std::function<bool()> check;
    if (!cancel.is_none()) check = [&] {
      py::gil_scoped_acquire acquire;
      return cancel().cast<bool>();
    };
    py::gil_scoped_release release;
    return rec::copy_complete_prefix(src, dst, check);
  }, "src"_a, "dst"_a, "cancel"_a=py::none());
  m.def("_recording_header", [](bool feedback) {
    return rec::csv_header(feedback ? rec::Table::feedback : rec::Table::commands);
  });
  m.def("_write_csv_zst", [](const std::filesystem::path& path, const std::string& text, bool append) {
    rec::CsvWriter writer(path, append);
    writer.append(text);
    writer.flush_frame();
  }, "path"_a, "text"_a, "append"_a=false, py::call_guard<py::gil_scoped_release>());
}
