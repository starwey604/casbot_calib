#include <casbot_calib/shm/channel.hpp>
#include <casbot_calib/shm/arm_jog.hpp>
#include <casbot_calib/arm_dynamics.hpp>
#include <casbot_calib/collision.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <locale>
#include <mutex>
#include <set>
#include <sstream>
#include <sys/prctl.h>
#include <thread>
#include <time.h>
#include <unistd.h>

using namespace casbot::shm;
namespace {
volatile std::sig_atomic_t interrupted=0;
void signal_stop(int) { interrupted=1; }
std::int64_t mono() {
  timespec t{}; if(clock_gettime(CLOCK_MONOTONIC,&t)) throw std::runtime_error("clock_gettime");
  return t.tv_sec*1000000000LL+t.tv_nsec;
}
void sleep_to(std::int64_t deadline) {
  timespec t{deadline/1000000000LL,deadline%1000000000LL};
  while(clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&t,nullptr)==EINTR && !interrupted) {}
}
template<class Frame> void binary(const std::filesystem::path& path,const Frame& frame) {
  std::ofstream f(path,std::ios::binary); f.exceptions(std::ios::badbit|std::ios::failbit);
  f.write(reinterpret_cast<const char*>(&frame),sizeof(frame));
}
CommandFrame read_background(const std::string& file) {
  std::ifstream f(file,std::ios::binary);
  const std::vector<char> data{std::istreambuf_iterator<char>(f),std::istreambuf_iterator<char>()};
  auto c=decode_command(data.data(),data.size());
  if(c.hand_count>2) throw std::runtime_error("Invalid hand background count");
  return c;
}
Eigen::VectorXd positions(const StatusFrame& s) {
  Eigen::VectorXd q(14); for(int j=0;j<14;++j) q[j]=arm_position(s,j); return q;
}
void print_status(const StatusFrame& s) {
  std::cout<<"STATE frame="<<s.frame_counter<<" source_timestamp_ms="<<s.timestamp
           <<" joint_count="<<int(s.joint_count)<<" (print slots, not count-derived validity)\n";
  for(int j=0;j<35;++j) {
    const auto& a=s.joints[j];
    std::cout<<"id="<<j;
    if(j>=12 && j<=25) std::cout<<" "<<arm_names[j-12]<<" q_rad="<<arm_position(s,j-12);
    else std::cout<<(j>=29?" reserved":" non_arm");
    std::cout<<" pos_raw="<<a.position<<" vel_raw="<<a.velocity<<" torque_raw="<<a.torque
      <<" status=0x"<<std::hex<<a.status_word<<std::dec<<" mode="<<a.mode<<" error="<<a.error_code<<'\n';
  }
  for(int h=0;h<2;++h) for(int j=0;j<6;++j) {
    const auto& a=s.hands[h].joints[j];
    std::cout<<"hand="<<h<<" item="<<j<<" pos_raw="<<a.position<<" vel_raw="<<a.velocity
      <<" current_raw="<<a.current<<" status="<<a.status<<'\n';
  }
}
StatusFrame fresh(Channel& channel) {
  StatusFrame s{}; const auto end=mono()+2000000000LL;
  bool have=false; std::uint64_t first=0;
  while(mono()<end && !interrupted) {
    if(channel.receive(s)) { if(have && s.frame_counter!=first) return s; first=s.frame_counter;have=true; }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  throw std::runtime_error("No fresh changing master frame counter within 2 seconds");
}
struct Scene {
  std::unique_ptr<casbot::CollisionChecker> checker;
  Eigen::VectorXd thresholds;
  Scene(const std::string& path,const casbot::ArmDynamics& model,const std::string& urdf) {
    std::ifstream in(path); std::string tag; int n=0; in>>tag>>n;
    if(tag!="CASBOT_SHM_COLLISION_V1" || n!=14) throw std::runtime_error("Invalid collision scene header");
    for(int j=0;j<14;++j) { std::string name;in>>name;if(name!=arm_names[j])throw std::runtime_error("Collision scene joint order mismatch"); }
    std::ifstream source(urdf,std::ios::binary);const std::string xml{std::istreambuf_iterator<char>(source),std::istreambuf_iterator<char>()};
    std::string expected;constexpr char hex[]="0123456789abcdef";
    for(unsigned char c:xml){expected+=hex[c>>4];expected+=hex[c&15];}
    std::string binding;in>>tag>>binding;
    if(tag!="URDF_HEX" || binding!=expected || xml.empty())throw std::runtime_error("Collision scene is bound to a different URDF");
    std::size_t count=0;in>>count;
    if(count==0 || count>2000)throw std::runtime_error("Invalid geometry count");
    std::vector<casbot::CollisionGeometry> geometry(count);
    for(auto& g:geometry) {
      int attached=0; std::size_t vertices=0,triangles=0;
      in>>g.name>>g.parent>>attached>>vertices>>triangles;g.attached_to_base=attached;
      if(vertices<4 || vertices>1000000 || triangles>2000000 || g.parent < -1 || g.parent>=14)
        throw std::runtime_error("Invalid collision geometry");
      for(int r=0;r<3;++r)for(int c=0;c<3;++c)in>>g.placement.rotation(r,c);
      for(int r=0;r<3;++r)in>>g.placement.translation[r];
      g.vertices.resize(vertices,3);g.triangles.resize(triangles,3);
      for(Eigen::Index r=0;r<g.vertices.rows();++r)for(int c=0;c<3;++c)in>>g.vertices(r,c);
      for(Eigen::Index r=0;r<g.triangles.rows();++r)for(int c=0;c<3;++c)in>>g.triangles(r,c);
      if(!in || !g.vertices.allFinite() || !g.placement.rotation.allFinite() || !g.placement.translation.allFinite())
        throw std::runtime_error("Truncated/nonfinite collision geometry");
    }
    std::size_t pairs=0; in>>pairs;
    if(pairs==0 || pairs>count*(count-1)/2) throw std::runtime_error("Invalid collision pairs");
    thresholds.resize(pairs); std::vector<std::pair<int,int>> enabled;
    std::set<std::pair<int,int>> active;
    for(std::size_t i=0;i<pairs;++i) {
      int a,b; in>>a>>b>>thresholds[i];
      if(a<0 || b<=a || b>=static_cast<int>(count) || !std::isfinite(thresholds[i]) || thresholds[i]<.01 || !active.emplace(a,b).second)
        throw std::runtime_error("Invalid pair or clearance below 10mm");
      enabled.emplace_back(a,b);
    }
    std::string extra;if(!in || (in>>extra))throw std::runtime_error("Collision scene truncated or has trailing data");
    std::vector<std::pair<int,int>> excluded;
    for(int a=0;a<static_cast<int>(count);++a)for(int b=a+1;b<static_cast<int>(count);++b)
      if(!active.count({a,b}))excluded.emplace_back(a,b);
    checker=std::make_unique<casbot::CollisionChecker>(model,geometry,excluded);
    if(checker->pairs()!=enabled)throw std::runtime_error("Native collision pair order differs from frozen scene");
  }
  bool certify(const Eigen::VectorXd& q,int joint,double amplitude,std::ostream& report) {
    const auto radii=checker->motion_radii();
    struct Interval { double a,b; int depth; };
    std::vector<Interval> pending{{-amplitude,amplitude,0}};
    std::size_t queries=0;double minimum=std::numeric_limits<double>::infinity();
    while(!pending.empty()) {
      const auto x=pending.back();pending.pop_back();
      Eigen::MatrixXd pose(1,14);pose.row(0)=q.transpose();pose(0,joint)+=(x.a+x.b)/2;
      const auto distances=checker->check_batch(pose).distances.row(0).transpose().eval();
      minimum=std::min(minimum,distances.minCoeff());++queries;
      if((distances.array()<=thresholds.array()).any()) {report<<"collision_or_clearance min_m="<<minimum<<" queries="<<queries<<'\n';return false;}
      if(((distances-radii.col(joint)*(x.b-x.a)/2).array()>thresholds.array()).all())continue;
      if(x.depth>=20 || queries>=200000) {report<<"collision_interval_unresolved min_m="<<minimum<<'\n';return false;}
      const auto mid=(x.a+x.b)/2;
      pending.push_back({x.a,mid,x.depth+1});pending.push_back({mid,x.b,x.depth+1});
    }
    report<<"collision_certified min_sampled_m="<<minimum<<" queries="<<queries<<" nominal_reference_only\n";
    return true;
  }
};
struct Record { std::int64_t at,send_at,receive_at; unsigned char phase; bool sent,received; CommandFrame cmd; StatusFrame state; };
class Recorder {
  std::mutex mutex_; std::condition_variable ready_; std::deque<Record> queue_;
  bool done_=false;std::thread worker_;std::ofstream commands_,feedback_,events_,raw_cmd_,raw_status_;
 public:
  std::atomic<bool> stopped{false};
  Recorder(const std::filesystem::path& root) {
    commands_.open(root/"commands.csv");feedback_.open(root/"feedback.csv");events_.open(root/"events.csv");
    raw_cmd_.open(root/"commands.bin",std::ios::binary);raw_status_.open(root/"feedback.bin",std::ios::binary);
    for(auto* s:{&commands_,&feedback_,&events_,&raw_cmd_,&raw_status_})s->exceptions(std::ios::badbit|std::ios::failbit);
    commands_<<std::setprecision(17)<<"send_mono_ns,phase,frame_counter";
    feedback_<<std::setprecision(17)<<"receive_mono_ns,source_timestamp_ms,frame_counter";
    for(int j=0;j<14;++j) {commands_<<",q"<<j;feedback_<<",q"<<j;}
    commands_<<'\n';feedback_<<'\n';events_<<"mono_ns,phase\n";
    worker_=std::thread([this] {
      try {
        int last_phase=-1;std::int64_t last_print=0;
        while(true) {
          Record r{};
          {std::unique_lock<std::mutex> lock(mutex_);ready_.wait(lock,[&]{return done_||!queue_.empty();});
            if(queue_.empty())break;r=queue_.front();queue_.pop_front();}
          if(last_phase!=r.phase){events_<<r.at<<','<<int(r.phase)<<'\n';last_phase=r.phase;std::cout<<"PHASE "<<last_phase<<'\n';}
          if(r.sent) {
            raw_cmd_.write(reinterpret_cast<const char*>(&r.send_at),8);raw_cmd_.write(reinterpret_cast<const char*>(&r.cmd),sizeof(r.cmd));
            commands_<<r.send_at<<','<<int(r.phase)<<','<<r.cmd.frame_counter;
            for(int j=0;j<14;++j)commands_<<','<<(arm_direction[j]*double(r.cmd.joints[j+12].position)*1e-6+arm_offset[j]);commands_<<'\n';
          }
          if(r.received) {
            raw_status_.write(reinterpret_cast<const char*>(&r.receive_at),8);raw_status_.write(reinterpret_cast<const char*>(&r.state),sizeof(r.state));
            feedback_<<r.receive_at<<','<<r.state.timestamp<<','<<r.state.frame_counter;
            for(int j=0;j<14;++j)feedback_<<','<<arm_position(r.state,j);feedback_<<'\n';
            if(r.at-last_print>=1000000000LL){print_status(r.state);last_print=r.at;}
          }
        }
        for(auto* s:{&commands_,&feedback_,&events_,&raw_cmd_,&raw_status_})s->flush();
      } catch(const std::exception& e) { stopped=true;std::cerr<<"Recording stopped: "<<e.what()<<'\n'; }
    });
  }
  void push(Record r) {
    if(stopped)return;
    {std::lock_guard<std::mutex> lock(mutex_);if(queue_.size()>=4096){stopped=true;return;}queue_.push_back(std::move(r));}
    ready_.notify_one();
  }
  void finish() {{std::lock_guard<std::mutex> lock(mutex_);done_=true;}ready_.notify_one();if(worker_.joinable())worker_.join();}
  ~Recorder() {finish();}
};
CommandFrame baseline(const StatusFrame& s,const CommandFrame& hand,const Eigen::VectorXd& q,double kp,double kd) {
  CommandFrame c{};c.joint_count=29;c.hand_count=hand.hand_count;c.hands=hand.hands;
  for(int i=0;i<29;++i) {c.joints[i].position=s.joints[i].position;c.joints[i].control_word=6;c.joints[i].mode=s.joints[i].mode;}
  for(int j=0;j<14;++j)arm_target(c,j,q[j],0,kp,kd);
  return c;
}
bool arm_ack(const StatusFrame& s,unsigned word,bool mit=false) {
  for(int j=12;j<26;++j)if((s.joints[j].status_word&255)!=word || (mit&&s.joints[j].mode!=-1))return false;
  return true;
}
void run(Channel& channel,StatusFrame state,const CommandFrame& hand,const Eigen::VectorXd& q0,
         int joint,double amplitude,double segment_s,double kp,double kd,const std::filesystem::path& root) {
  Recorder recorder(root);
  auto command=baseline(state,hand,q0,kp,kd);
  const auto t0=mono();std::int64_t next=t0,phase_begin=t0,last_receive=t0,last_send=t0,first_stream=0,max_send_gap=0;
  int phase=0;bool phase_sent=false,done=false;std::string failure;
  auto send=[&](std::int64_t now) {command.frame_counter++;command.timestamp=now/1000000;return channel.send(command);};
  try {
    while(!done) {
      const auto now=mono();
      if(interrupted)throw std::runtime_error("operator_interrupt");
      const bool received=channel.receive(state);
      const auto receive_at=received?mono():0;
      if(received)last_receive=receive_at;
      if(now-last_receive>2000000000LL)throw std::runtime_error("feedback_timeout");
      if(now-last_send>2000000000LL)throw std::runtime_error("command_queue_timeout");
      for(int i=0;i<29;++i)if(state.joints[i].error_code)throw std::runtime_error("drive_error_slot_"+std::to_string(i)+"_"+std::to_string(state.joints[i].error_code));
      if(phase!=3 && now-phase_begin>3000000000LL)throw std::runtime_error("transition_timeout_phase_"+std::to_string(phase));
      bool advance=false;
      if(phase_sent && now-phase_begin>20000000) {
        if(phase==0) {
          advance=arm_ack(state,0x21);
          for(int j=0;j<29;++j)if((j<12||j>25)&&(state.joints[j].status_word&4))advance=false;
        }
        if(phase==1)advance=arm_ack(state,0x21,true);
        if(phase==2)advance=arm_ack(state,0x37,true);
        if(phase==4 && arm_ack(state,0x21))done=true;
      }
      if(advance){++phase;phase_begin=now;phase_sent=false;}
      if(phase>=1)for(int j=12;j<26;++j)command.joints[j].mode=-1;
      for(int j=12;j<26;++j)command.joints[j].control_word=(phase==2||phase==3)?15:6;
      if(phase==3) {
        if(!arm_ack(state,0x37,true))throw std::runtime_error("enabled_mode_lost");
        const double elapsed=first_stream?(now-first_stream)*1e-9:0;
        const auto value=jog_at(elapsed,segment_s,amplitude);
        arm_target(command,joint,q0[joint]+value.position,value.velocity,kp,kd);
        if(elapsed>=3*segment_s+.5){phase=4;phase_begin=now;phase_sent=false;for(int j=12;j<26;++j)command.joints[j].control_word=6;}
      }
      const bool sent=send(now);
      const auto send_at=sent?mono():0;
      if(sent) {
        max_send_gap=std::max(max_send_gap,send_at-last_send);last_send=send_at;phase_sent=true;
        if(phase==3 && !first_stream)first_stream=send_at;
      }
      recorder.push({now,send_at,receive_at,static_cast<unsigned char>(phase),sent,received,command,state});
      next+=1000000;if(next<mono())next=mono()+1000000;sleep_to(next);
    }
  } catch(const std::exception& e) {
    failure=e.what();
    // One bounded best-effort disable, never reset faults or re-enable.
    for(int j=12;j<26;++j){command.joints[j].control_word=6;command.joints[j].velocity=0;command.joints[j].torque=0;}
    const auto end=mono()+200000000;
    while(mono()<end) {
      try {
        const auto now=mono();const bool received=channel.receive(state);const auto receive_at=received?mono():0;
        const bool sent=send(now);const auto send_at=sent?mono():0;
        recorder.push({now,send_at,receive_at,5,sent,received,command,state});
        if(received&&arm_ack(state,0x21))break;
      }catch(const std::exception& stop_error){std::cerr<<"Disable attempt failed: "<<stop_error.what()<<'\n';break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  recorder.finish();
  std::ofstream result(root/"result.txt");
  result<<"execution_completed="<<(failure.empty()?1:0)<<"\nreason="<<(failure.empty()?"completed_with_disable_ack":failure)
    <<"\njoint_id="<<joint+12<<"\namplitude_rad="<<amplitude<<"\nkp="<<kp<<"\nkd="<<kd
    <<"\nfirst_stream_mono_ns="<<first_stream<<"\nmax_send_gap_ns="<<max_send_gap
    <<"\nrecording_stopped="<<recorder.stopped<<"\nphysical_stop_observed=unknown\n";
  if(!failure.empty())throw std::runtime_error(failure);
}
void snapshot_output(const std::filesystem::path& root,const StatusFrame& state,const casbot::ArmModel& model) {
  binary(root/"status.bin",state);print_status(state);
  std::ofstream csv(root/"arms.csv");csv<<std::setprecision(17)<<"id,joint,q,lower,upper,minus_0_5,plus_0_5,within_limits\n";
  for(int j=0;j<14;++j) {
    const double q=arm_position(state,j);const auto& m=model.joints[j];
    csv<<j+12<<','<<arm_names[j]<<','<<q<<','<<m.lower<<','<<m.upper<<','<<q-.5<<','<<q+.5<<','<<jog_within_limits(q,m.lower,m.upper,.5)<<'\n';
    std::cout<<"LIMIT id="<<j+12<<" q="<<q<<" minus="<<q-.5<<" plus="<<q+.5<<" URDF=["<<m.lower<<','<<m.upper<<"] "
      <<(jog_within_limits(q,m.lower,m.upper,.5)?"eligible":"SKIP limit")<<'\n';
  }
}
void usage() {
  std::cout<<"shm_joint_test --inspect --output NEW_DIR [--shm /shm_mc]\n"
    <<"shm_joint_test --snapshot --urdf FILE --output NEW_DIR [--shm /shm_mc]\n"
    <<"shm_joint_test --execute --joint-id 12..25 --urdf FILE --scene FILE --hand-background command.bin --output NEW_DIR\n"
    <<"  [--amplitude .5] [--segment-s 5] [--kp 20] [--kd 2] [--shm /shm_mc]\n"
    <<"inspect is O_RDONLY; snapshot consumes status only, requires exclusive window.\n"
    <<"execute disables legs/waist/head, holds other arm axes; hand bytes explicitly frozen from given file.\n";
}
}
int main(int argc,char** argv) {
  std::cout<<std::setprecision(9);std::signal(SIGINT,signal_stop);std::signal(SIGTERM,signal_stop);std::signal(SIGHUP,signal_stop);std::signal(SIGPIPE,SIG_IGN);
  try {
    std::string mode,name="/shm_mc",output,urdf,scene,background;int id=12;double amplitude=.5,segment=5,kp=20,kd=2;
    for(int a=1;a<argc;++a) {
      const std::string key=argv[a];
      if(key=="--help"){usage();return 0;}
      if(key=="--inspect"||key=="--snapshot"||key=="--execute") {if(!mode.empty())throw std::runtime_error("Choose one mode");mode=key;continue;}
      if(a+1>=argc)throw std::runtime_error("Missing option value");const std::string value=argv[++a];
      if(key=="--shm")name=value;else if(key=="--output")output=value;else if(key=="--urdf")urdf=value;
      else if(key=="--scene")scene=value;else if(key=="--hand-background")background=value;
      else {
        std::size_t used=0;
        if(key=="--joint-id")id=std::stoi(value,&used);
        else if(key=="--amplitude")amplitude=std::stod(value,&used);else if(key=="--segment-s")segment=std::stod(value,&used);
        else if(key=="--kp")kp=std::stod(value,&used);else if(key=="--kd")kd=std::stod(value,&used);else throw std::runtime_error("Unknown option: "+key);
        if(used!=value.size())throw std::runtime_error("Invalid numeric option: "+key);
      }
    }
    if(output.empty()||mode.empty())throw std::runtime_error("Mode and new output directory required");
    const auto index=arm_index(id);
    if(!std::isfinite(amplitude)||amplitude<=0||amplitude>.5||!std::isfinite(segment)||segment<1||
       !std::isfinite(kp)||kp<0||kp>100||!std::isfinite(kd)||kd<0||kd>10)throw std::runtime_error("Invalid test settings");
    if(!std::filesystem::path(output).parent_path().empty())std::filesystem::create_directories(std::filesystem::path(output).parent_path());
    if(!std::filesystem::create_directory(output))throw std::runtime_error("Output must be a new directory");
    if(mode=="--inspect") {
      const auto s=passive_snapshot(name);binary(std::filesystem::path(output)/"command.bin",s.command);
      binary(std::filesystem::path(output)/"status.bin",s.status);print_status(s.status);return 0;
    }
    if(urdf.empty())throw std::runtime_error("Explicit 14-axis URDF required");
    const auto model=casbot::ArmDynamics::from_urdf(urdf);
    if(model.model().joints.size()!=14)throw std::runtime_error("Expected reduced 14-axis model");
    for(int j=0;j<14;++j)if(model.model().joints[j].name!=arm_names[j])throw std::runtime_error("URDF arm order mismatch");
    Channel channel(name);auto state=fresh(channel);
    snapshot_output(output,state,model.model());
    if(mode=="--snapshot")return 0;
    if(scene.empty()||background.empty())throw std::runtime_error("Collision scene and explicit hand command background required");
    const auto q0=positions(state);const auto& joint=model.model().joints[index];
    if(4.375*amplitude/segment>std::min(1.5,joint.velocity_limit))throw std::runtime_error("Jog reference exceeds configured/URDF velocity limit");
    if(!jog_within_limits(q0[index],joint.lower,joint.upper,amplitude)) {
      std::ofstream(std::filesystem::path(output)/"result.txt")<<"execution_completed=0\nreason=SKIP_urdf_limit\ncommands_sent=0\n";
      return 4;
    }
    for(int j=0;j<14;++j)if(q0[j]<model.model().joints[j].lower||q0[j]>model.model().joints[j].upper)
      throw std::runtime_error("Measured start outside URDF limit at joint ID "+std::to_string(j+12));
    Scene collision(scene,model,urdf);std::ofstream proof(std::filesystem::path(output)/"collision_reference.txt");
    if(!collision.certify(q0,index,amplitude,proof)) {
      std::ofstream(std::filesystem::path(output)/"result.txt")<<"execution_completed=0\nreason=SKIP_collision\ncommands_sent=0\n";
      return 4;
    }
    state=fresh(channel);
    if((positions(state)-q0).cwiseAbs().maxCoeff()>.005)throw std::runtime_error("Start changed during planning; no commands sent");
    const auto hand=read_background(background);
    binary(std::filesystem::path(output)/"hand_background_source.bin",hand);
    std::ofstream(std::filesystem::path(output)/"settings.txt")<<"mode=arm_jog_experiment\nnon_arm=disabled\nhand_policy=explicit_frozen_command_bytes\nurdf="<<urdf<<"\nscene="<<scene<<'\n';
    const auto parent=getppid();if(parent<=1 || prctl(PR_SET_PDEATHSIG,SIGTERM) || getppid()!=parent)throw std::runtime_error("No live supervisor");
    std::cout<<"START id="<<id<<" q0="<<q0[index]<<" amplitude="<<amplitude<<" kp="<<kp<<" kd="<<kd<<"\n";
    run(channel,state,hand,q0,index,amplitude,segment,kp,kd,output);
    return 0;
  } catch(const std::exception& e) {std::cerr<<"ERROR "<<e.what()<<'\n';return 1;}
}
