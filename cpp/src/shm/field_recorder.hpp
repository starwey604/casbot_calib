#pragma once
#include <casbot_calib/shm/arm_codec.hpp>
#include <casbot_calib/shm/channel.hpp>
#include <casbot_calib/shm/clock.hpp>
#include "native_clock.hpp"
#include "../csv_zstd.hpp"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <optional>
#include <thread>
#include <sstream>
namespace casbot::shm {
struct Record {
  std::int64_t at=0,send_at=0,receive_at=0,observation=0;
  int phase=0,segment=0;bool sent=false,received=false,passive=false;
  CommandFrame cmd{};StatusFrame state{};QueueDiagnostics queues{};bool has_queues=false;std::string event,limit,metadata;ClockPair clock{};bool has_pair=false;
};
class Recorder {
  std::mutex mutex_;std::condition_variable ready_;std::deque<Record> queue_;
  std::uint64_t budget_;
  bool done_=false;std::thread worker_;std::filesystem::path root_;
  std::ofstream metadata_;
  recording::CsvWriter commands_, feedback_;
  std::optional<recording::CsvWriter> passive_commands_;
  std::ofstream events_, pairs_, queues_, limits_;

  static recording::CommandRow command_row(const Record& r, std::int64_t at) {
    recording::CommandRow row;
    row.send_mono_ns = at;
    row.phase = r.phase;
    row.frame_counter = r.cmd.frame_counter;
    row.hand_count = r.cmd.hand_count;
    for (int j = 0; j < 14; ++j) {
      const auto fields = arm_command(r.cmd, j);
      row.q[j] = fields.position;
      row.v[j] = fields.velocity;
      row.effort[j] = fields.effort;
      row.kp[j] = fields.kp;
      row.kd[j] = fields.kd;
    }
    for (int slot = 0; slot < 29; ++slot) {
      const auto& joint = r.cmd.joints[slot];
      row.cmd_pos_raw[slot] = joint.position;
      row.cmd_vel_raw[slot] = joint.velocity;
      row.cmd_trq_raw[slot] = joint.torque;
      row.cmd_kp_raw[slot] = joint.kp;
      row.cmd_kd_raw[slot] = joint.kd;
      row.cmd_cw[slot] = joint.control_word;
      row.cmd_mode[slot] = joint.mode;
    }
    return row;
  }

  static recording::FeedbackRow feedback_row(const Record& r) {
    recording::FeedbackRow row;
    row.receive_mono_ns = r.receive_at;
    row.source_timestamp_ms = r.state.timestamp;
    row.frame_counter = r.state.frame_counter;
    row.observation_time_ns = r.observation;
    row.clock_segment = r.segment;
    row.passive = r.passive;
    row.phase = r.phase;
    for (int j = 0; j < 14; ++j) {
      const auto fields = arm_feedback(r.state, j);
      row.q[j] = fields.position;
      row.dq[j] = fields.velocity;
      row.effort_raw[j] = fields.effort;
    }
    for (int slot = 0; slot < 35; ++slot) {
      const auto& joint = r.state.joints[slot];
      row.pos_raw[slot] = joint.position;
      row.vel_raw[slot] = joint.velocity;
      row.trq_raw[slot] = joint.torque;
      row.status[slot] = joint.status_word;
      row.mode[slot] = joint.mode;
      row.error[slot] = joint.error_code;
    }
    for (int h = 0; h < 2; ++h) {
      const auto& hand = r.state.hands[h];
      auto& out = row.hands[h];
      out.global_status = hand.global_status;
      out.error = hand.error_code;
      for (int j = 0; j < 6; ++j) {
        const auto& joint = hand.joints[j];
        out.pos[j] = joint.position;
        out.vel[j] = joint.velocity;
        out.cur[j] = joint.current;
        out.status[j] = joint.status;
      }
    }
    return row;
  }

  void flush(std::int64_t at) {
    commands_.flush_frame();
    feedback_.flush_frame();
    if (passive_commands_) passive_commands_->flush_frame();
    std::uint64_t bytes = commands_.bytes() + feedback_.bytes();
    if (passive_commands_) bytes += passive_commands_->bytes();
    for (auto* stream : {&events_, &pairs_, &queues_, &limits_}) {
      stream->flush();
      if (budget_) bytes += static_cast<std::uint64_t>(stream->tellp());
    }
    if (!budget_) return;
    metadata_.flush();
    bytes += static_cast<std::uint64_t>(metadata_.tellp());
    bytes_written = bytes;
    if (bytes >= budget_ && !stopped.exchange(true)) {
      std::ofstream marker(root_/"recording_stopped.json");
      marker << "{\"reason\":\"capacity\",\"mono_ns\":" << at
             << ",\"bytes\":" << bytes << ",\"budget_bytes\":" << budget_ << "}\n";
    }
  }

 public:
  std::atomic<bool> stopped{false};
  std::atomic<std::uint64_t> bytes_written{0};
  explicit Recorder(const std::filesystem::path& root, std::uint64_t budget = 0)
      : budget_(budget), root_(root), commands_(root/"commands.csv.zst"),
        feedback_(root/"feedback.csv.zst") {
    if (budget_) {
      metadata_.open(root/"controls.jsonl");
      metadata_.exceptions(std::ios::failbit | std::ios::badbit);
    }
    limits_.open(root/"observed_limits.csv");
    events_.open(root/"events.csv");
    pairs_.open(root/"clock_pairs.csv");
    queues_.open(root/"queues.csv");
    for (auto* stream : {&events_, &pairs_, &queues_, &limits_})
      stream->exceptions(std::ios::badbit | std::ios::failbit);
    limits_ << "receive_mono_ns,phase,joint_id,field,value,threshold,unit,action\n";
    events_ << "mono_ns,phase,event\n";
    commands_.append(recording::csv_header(recording::Table::commands));
    feedback_.append(recording::csv_header(recording::Table::feedback));
    commands_.flush_frame();
    feedback_.flush_frame();
    pairs_<<"mono_before_ns,wall_ns,mono_after_ns,offset_ns,clock_segment,jump_detected\n";
    queues_<<"mono_ns,passive,cmd_count,cmd_write,cmd_read,cmd_published,status_count,status_write,status_read,status_published\n";
    worker_=std::thread([this] {
      try {
        std::int64_t last_flush=0;
        while(true) {
          Record r;
          {std::unique_lock<std::mutex> lock(mutex_);ready_.wait(lock,[&]{return done_||!queue_.empty();});
            if(queue_.empty())break;r=std::move(queue_.front());queue_.pop_front();}
          if(stopped)continue;
          if(!r.metadata.empty()&&metadata_.is_open())metadata_<<r.metadata<<'\n';
          if(!r.limit.empty())limits_<<r.at<<','<<r.phase<<','<<r.limit<<'\n';
          if(!r.event.empty()) {
            events_<<r.at<<','<<r.phase<<','<<r.event<<'\n';events_.flush();
            if(r.event=="kill_ready"||r.event=="passive_ready") {
              std::ofstream marker(root_/(r.event+".txt"));marker<<r.at<<'\n';marker.flush();
            }
          }
          if(r.has_queues) {
            const auto& c=r.queues.command;const auto& s=r.queues.status;
            queues_<<r.at<<','<<r.passive<<','<<c.count<<','<<c.write_index<<','<<c.read_index<<','<<c.published_index
              <<','<<s.count<<','<<s.write_index<<','<<s.read_index<<','<<s.published_index<<'\n';
          }
          if(r.has_pair) {const auto& p=r.clock;pairs_<<p.before<<','<<p.wall<<','<<p.after<<','<<p.offset<<','<<p.segment<<','<<p.jumped<<'\n';}
          if (r.sent) commands_.append(command_row(r, r.send_at));
          if (r.received && r.passive) {
            if (!passive_commands_) {
              passive_commands_.emplace(root_/"passive_commands.csv.zst");
              passive_commands_->append(recording::csv_header(recording::Table::commands));
            }
            passive_commands_->append(command_row(r, r.receive_at));
          }
          if (r.received) feedback_.append(feedback_row(r));
          if (r.at - last_flush >= 20000000) {
            flush(r.at);
            last_flush = r.at;
          }
        }
        flush(mono());
      }catch(const std::exception&){stopped=true;}
    });
  }
  void push(Record r) {
    if(stopped)return;
    {std::lock_guard<std::mutex> lock(mutex_);if(done_)return;if(queue_.size()>=4096){stopped=true;return;}queue_.push_back(std::move(r));}
    ready_.notify_one();
  }
  void metadata(std::string text) {Record r;r.at=mono();r.metadata="{\"recorded_mono_ns\":"+std::to_string(r.at)+",\"message\":"+text+"}";push(std::move(r));}
  void limit(std::int64_t receive,int phase,int joint,const char* field,double value,double threshold,const char* unit,const char* action) {
    Record r;r.at=receive;r.phase=phase;std::ostringstream text;
    text<<std::setprecision(17)<<joint<<','<<field<<','<<value<<','<<threshold<<','<<unit<<','<<action;
    r.limit=text.str();push(std::move(r));
  }
  void event(int phase,const std::string& event) {Record r;r.at=mono();r.phase=phase;r.event=event;push(std::move(r));}
  void finish(){{std::lock_guard<std::mutex> lock(mutex_);done_=true;}ready_.notify_one();if(worker_.joinable())worker_.join();}
  ~Recorder(){finish();}
};
}  // namespace casbot::shm
