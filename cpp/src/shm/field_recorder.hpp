#pragma once
#include <casbot_calib/shm/arm_codec.hpp>
#include <casbot_calib/shm/channel.hpp>
#include <casbot_calib/shm/clock.hpp>
#include "native_clock.hpp"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <thread>
#include <sstream>
namespace casbot::shm {
struct Record {
  std::int64_t at=0,send_at=0,receive_at=0,observation=0;
  int phase=0,segment=0;bool sent=false,received=false,passive=false;
  CommandFrame cmd{};StatusFrame state{};QueueDiagnostics queues{};bool has_queues=false;std::string event,limit;ClockPair clock{};bool has_pair=false;
};
class Recorder {
  std::mutex mutex_;std::condition_variable ready_;std::deque<Record> queue_;
  bool done_=false;std::thread worker_;std::filesystem::path root_;
  std::ofstream commands_,feedback_,events_,raw_cmd_,raw_status_,pairs_,passive_cmd_,queues_,limits_;
 public:
  std::atomic<bool> stopped{false};
  explicit Recorder(const std::filesystem::path& root):root_(root) {
    limits_.open(root/"observed_limits.csv");
    commands_.open(root/"commands.csv");feedback_.open(root/"feedback.csv");events_.open(root/"events.csv");
    raw_cmd_.open(root/"commands.bin",std::ios::binary);raw_status_.open(root/"feedback.bin",std::ios::binary);pairs_.open(root/"clock_pairs.csv");passive_cmd_.open(root/"passive_commands.bin",std::ios::binary);queues_.open(root/"queues.csv");
    for(auto* s:{&commands_,&feedback_,&events_,&raw_cmd_,&raw_status_,&pairs_,&passive_cmd_,&queues_,&limits_})s->exceptions(std::ios::badbit|std::ios::failbit);
    limits_<<"receive_mono_ns,phase,joint_id,field,value,threshold,unit,action\n";
    commands_<<std::setprecision(17)<<"send_mono_ns,phase,frame_counter";
    feedback_<<std::setprecision(17)<<"receive_mono_ns,source_timestamp_ms,frame_counter,observation_time_ns,clock_segment,passive";
    for(int j=0;j<14;++j){commands_<<",q"<<j;feedback_<<",q"<<j;}
    for(int j=0;j<14;++j){commands_<<",v"<<j<<",effort"<<j<<",kp"<<j<<",kd"<<j;feedback_<<",v"<<j<<",effort"<<j;}
    for(int j=0;j<35;++j)feedback_<<",status"<<j<<",mode"<<j<<",error"<<j;
    commands_<<'\n';feedback_<<'\n';events_<<"mono_ns,phase,event\n";
    pairs_<<"mono_before_ns,wall_ns,mono_after_ns,offset_ns,clock_segment,jump_detected\n";
    queues_<<"mono_ns,passive,cmd_count,cmd_write,cmd_read,cmd_published,status_count,status_write,status_read,status_published\n";
    worker_=std::thread([this] {
      try {
        std::int64_t last_flush=0;
        while(true) {
          Record r;
          {std::unique_lock<std::mutex> lock(mutex_);ready_.wait(lock,[&]{return done_||!queue_.empty();});
            if(queue_.empty())break;r=std::move(queue_.front());queue_.pop_front();}
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
          if(r.sent) {
            raw_cmd_.write(reinterpret_cast<const char*>(&r.send_at),8);raw_cmd_.write(reinterpret_cast<const char*>(&r.cmd),sizeof(r.cmd));
            commands_<<r.send_at<<','<<r.phase<<','<<r.cmd.frame_counter;
            for(int j=0;j<14;++j)commands_<<','<<(arm_direction[j]*double(r.cmd.joints[j+12].position)*1e-6+arm_offset[j]);
            for(int j=0;j<14;++j){const auto& a=r.cmd.joints[j+12];commands_<<','<<arm_direction[j]*static_cast<double>(a.velocity)*1e-6<<','<<arm_direction[j]*a.torque*.01<<','<<a.kp*.1<<','<<a.kd*.1;}
            commands_<<'\n';
          }
          if(r.received && r.passive) {
            passive_cmd_.write(reinterpret_cast<const char*>(&r.receive_at),8);
            passive_cmd_.write(reinterpret_cast<const char*>(&r.cmd),sizeof(r.cmd));
          }
          if(r.received) {
            raw_status_.write(reinterpret_cast<const char*>(&r.receive_at),8);raw_status_.write(reinterpret_cast<const char*>(&r.state),sizeof(r.state));
            feedback_<<r.receive_at<<','<<r.state.timestamp<<','<<r.state.frame_counter<<','<<r.observation<<','<<r.segment<<','<<r.passive;
            for(int j=0;j<14;++j)feedback_<<','<<arm_position(r.state,j);
            for(int j=0;j<14;++j){const auto& a=r.state.joints[j+12];feedback_<<','<<arm_direction[j]*static_cast<double>(a.velocity)*1e-6<<','<<arm_direction[j]*a.torque*.01;}
            for(const auto& a:r.state.joints)feedback_<<','<<a.status_word<<','<<a.mode<<','<<a.error_code;
            feedback_<<'\n';
          }
          if(r.at-last_flush>=20000000){flush();last_flush=r.at;}
        }
        flush();
      }catch(const std::exception&){stopped=true;}
    });
  }
  void flush(){for(auto* s:{&commands_,&feedback_,&events_,&raw_cmd_,&raw_status_,&pairs_,&passive_cmd_,&queues_,&limits_})s->flush();}
  void push(Record r) {
    if(stopped)return;
    {std::lock_guard<std::mutex> lock(mutex_);if(queue_.size()>=4096){stopped=true;return;}queue_.push_back(std::move(r));}
    ready_.notify_one();
  }
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
