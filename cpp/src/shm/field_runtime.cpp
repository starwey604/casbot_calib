#include "field_runtime.hpp"
#include "field_common.hpp"
#include <casbot_calib/shm/channel.hpp>
#include <casbot_calib/shm/clock.hpp>
#include "reference.hpp"
#include "soft_return.hpp"
#include <casbot_calib/field_defaults.hpp>
#include <algorithm>
#include "field_limits.hpp"
#include "field_recorder.hpp"
#include "native_clock.hpp"
#include <casbot_calib/shm/arm_jog.hpp>
#include <casbot_calib/arm_dynamics.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <time.h>
#include <unistd.h>

namespace casbot::shm {
namespace {
volatile std::sig_atomic_t hard_stop=0,soft_stop=0;
void signal_hard_stop(int) { hard_stop=1; }
void signal_soft_stop(int) { soft_stop=1; }
void sleep_to(std::int64_t deadline) {
  timespec t{deadline/1000000000LL,deadline%1000000000LL};
  while(clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&t,nullptr)==EINTR && !hard_stop) {}
}
template<class Frame> void binary(const std::filesystem::path& path,const Frame& frame) {
  std::ofstream f(path,std::ios::binary); f.exceptions(std::ios::badbit|std::ios::failbit);
  f.write(reinterpret_cast<const char*>(&frame),sizeof(frame));
}
struct Options {
  std::string mode,name="/shm_mc",output,urdf,background,which="C03",reference;
  int id=12;double amplitude=.5,segment=5,kp=20,kd=2,duration=3,quiet=3;
  bool other_disabled=false,bare_flange=false;
};
CommandFrame read_background(const std::string& file) {
  std::ifstream f(file,std::ios::binary);
  const std::vector<char> data{std::istreambuf_iterator<char>(f),std::istreambuf_iterator<char>()};
  auto c=decode_command(data.data(),data.size());
  if(c.hand_count>2)throw std::runtime_error("Invalid hand background count");
  return c;
}
Eigen::VectorXd positions(const StatusFrame& s) {
  Eigen::VectorXd q(14); for(int j=0;j<14;++j)q[j]=arm_position(s,j); return q;
}
void print_status(const StatusFrame& s) {
  std::cout<<"STATE frame="<<s.frame_counter<<" source_timestamp_ms="<<s.timestamp<<" joint_count="<<int(s.joint_count)<<'\n';
  for(int j=0;j<35;++j) {
    const auto& a=s.joints[j];std::cout<<"id="<<j;
    if(j>=12&&j<=25)std::cout<<" "<<arm_names[j-12]<<" q_rad="<<arm_position(s,j-12);
    else std::cout<<(j>=29?" reserved":" non_arm");
    std::cout<<" pos_raw="<<a.position<<" vel_raw="<<a.velocity<<" torque_raw="<<a.torque
      <<" status=0x"<<std::hex<<a.status_word<<std::dec<<" mode="<<a.mode<<" error="<<a.error_code<<'\n';
  }
  for(int h=0;h<2;++h)for(int j=0;j<6;++j) {
    const auto& a=s.hands[h].joints[j];
    std::cout<<"hand="<<h<<" item="<<j<<" pos_raw="<<a.position<<" vel_raw="<<a.velocity
      <<" current_raw="<<a.current<<" status="<<a.status<<'\n';
  }
}
StatusFrame fresh(Channel& channel) {
  StatusFrame s{};const auto end=mono()+2000000000LL;bool have=false;std::uint64_t first=0;
  while(mono()<end&&!hard_stop&&!soft_stop) {
    if(channel.receive(s)){if(have&&s.frame_counter!=first)return s;first=s.frame_counter;have=true;}
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  throw std::runtime_error(hard_stop?"operator_interrupt":soft_stop?"operator_soft_stop":"No fresh changing master frame counter within 2 seconds");
}
bool selected(int j,const Options& o){return !o.other_disabled||((j<7)==(o.id<19));}
CommandFrame baseline(const StatusFrame& s,const CommandFrame& hand,const Eigen::VectorXd& q,const Options& o) {
  std::array<double,14> values{};for(int j=0;j<14;++j)values[j]=q[j];
  return initial_command(s,hand,values,o.kp,o.kd);
}
ArmMask selection(const Options& o){ArmMask mask{};for(int j=0;j<14;++j)mask[j]=selected(j,o);return mask;}
void body_targets(CommandFrame& cmd,const StatusFrame& state,const NonArmHoldConfig& config,
                  const std::array<std::int32_t,29>& origin,double elapsed) {
  const double u=config.ramp_s>0?std::clamp(elapsed/config.ramp_s,0.,1.):1.;
  const double scale=u*u*u*(10.+u*(-15.+6.*u));
  for(const auto& slot:config.slots) {
    auto& c=cmd.joints[slot.slot];
    if(slot.mode==HoldMode::mit) {
      c.kp=checked_truncate<std::int16_t>(slot.kp*scale*10);
      c.kd=checked_truncate<std::int16_t>(slot.kd*scale*10);
    }else {
      const auto& a=state.joints[slot.slot];
      const double error=(double(origin[slot.slot])-a.position)*1e-6;
      const double torque=scale*(slot.kp*error-slot.kd*(double(a.velocity)*1e-6));
      c.torque=checked_truncate<std::int16_t>(std::clamp(torque,-slot.torque_limit_nm,slot.torque_limit_nm)*100);
    }
  }
}

void write_result(const std::filesystem::path& root,const std::string& reason,bool completed,
                  bool recording_stopped,std::int64_t first=0,std::int64_t max_gap=0,
                  std::int64_t writes=0,std::int64_t failures=0,std::int64_t first_write=0,bool ack=false) {
  std::ofstream result(root/"result.txt");
  result<<"execution_completed="<<completed<<"\nreason="<<reason<<"\nfirst_stream_mono_ns="<<first
    <<"\nfirst_command_mono_ns="<<first<<"\nfirst_shm_write_mono_ns="<<first_write
    <<"\nmax_send_gap_ns="<<max_gap<<"\ncommands_sent="<<writes<<"\nsend_skipped="<<failures
    <<"\nrecording_stopped="<<recording_stopped<<"\ndisable_ack="<<ack<<"\nphysical_stop_observed=unknown\n";
}
void observe(const Options& o) {
  const auto root=std::filesystem::path(o.output);
  std::unique_ptr<OwnerLease> lease;std::unique_ptr<Channel> channel;
  const bool passive=o.mode=="--passive-observe";
  if(!passive){lease=std::make_unique<OwnerLease>(o.name);channel=std::make_unique<Channel>(o.name);}
  Recorder recorder(root);ClockMap clock;const auto begin=mono();auto next=begin,last_receive=begin,last_pair=begin-std::int64_t(1000000000);
  std::string failure;bool ready=false;
  recorder.event(-1,passive?"passive_observe_begin":"observe_begin");
  try {
    while(mono()-begin<static_cast<std::int64_t>(o.duration*1e9)&&!hard_stop&&!soft_stop) {
      Record r;r.at=mono();r.phase=-1;r.passive=passive;
      if(r.at-last_pair>=1000000000LL){r.clock=clock.pair();r.has_pair=true;last_pair=r.at;}
      if(passive){auto snap=passive_snapshot(o.name);r.state=snap.status;r.cmd=snap.command;r.queues={snap.command_queue,snap.status_queue};r.has_queues=true;r.received=true;}
      else {r.received=channel->receive(r.state);if(r.has_pair){r.queues=channel->diagnostics();r.has_queues=true;}}
      if(r.received) {
        r.receive_at=mono();last_receive=r.receive_at;r.observation=clock.observation(r.state.timestamp);r.segment=clock.segment();
        if(!ready){binary(root/"status.bin",r.state);print_status(r.state);ready=true;recorder.event(-1,passive?"passive_ready":"observe_ready");}
      }
      recorder.push(std::move(r));
      if(mono()-last_receive>2000000000LL)throw std::runtime_error("feedback_timeout");
      next+=passive?10000000:1000000;if(next<mono())next=mono()+(passive?10000000:1000000);sleep_to(next);
    }
    if(!ready)throw std::runtime_error("No feedback recorded");
  }catch(const std::exception& e){failure=e.what();}
  recorder.event(-1,failure.empty()?"observe_end":failure);recorder.finish();
  write_result(root,failure.empty()?(hard_stop?"operator_interrupt":soft_stop?"operator_soft_stop":"observe_completed"):failure,failure.empty(),recorder.stopped);
  if(!failure.empty())throw std::runtime_error(failure);
}
void run(Channel& channel,StatusFrame state,const CommandFrame& hand,const Eigen::VectorXd& q0,const Options& o,const casbot::ArmDynamics& dynamics,const Reference* reference=nullptr) {
  const auto& model=dynamics.model();
  auto workspace=dynamics.workspace();
  const auto root=std::filesystem::path(o.output);Recorder recorder(root);ClockMap clock;
  auto command=baseline(state,hand,q0,o);
  if(reference){auto initial=reference->at(0);for(int j=0;j<14;++j)arm_target(command,j,q0[j],0,initial.kp[j],initial.kd[j]);}
  const NonArmHoldConfig empty_hold{};
  const auto& hold=reference?reference->non_arm_hold():empty_hold;
  const auto arms=selection(o);
  JointMask selected_axes{};
  JointModes modes{};
  std::array<std::int32_t,29> origin{};
  for(int j=0;j<14;++j){selected_axes[j+12]=arms[j];modes[j+12]=-1;}
  for(const auto& slot:hold.slots) {
    selected_axes[slot.slot]=true;
    modes[slot.slot]=static_cast<std::int16_t>(slot.mode);
    origin[slot.slot]=state.joints[slot.slot].position;
  }
  const auto ack=[&](unsigned word,bool mode=false){
    return hold.enabled()?drive_ack(state,word,selected_axes,modes,mode):drive_ack(state,word,arms,mode);
  };
  const auto disable=[&](){
    for(int i=0;i<29;++i) {
      auto& c=command.joints[i];
      c.control_word=6;c.velocity=0;c.torque=0;
      if(hold.enabled()){c.kp=0;c.kd=0;}
    }
  };
  const bool cst=hold.has_cst_pd();
  binary(root/"background.bin",command);
  const auto t0=mono();std::int64_t next=t0,phase_begin=t0,last_receive=t0,last_send=t0,first_stream=0,max_gap=0,first_write=0,last_pair=t0-1000000000LL,writes=0,skips=0;
  int phase=0;bool phase_sent=false,done=false,disable_ack=false;std::string failure;
  bool soft_stop_handled=false,soft_stop_active=false,return_has_ff=false;
  std::int64_t return_begin_ns=0;
  std::optional<SoftReturn> returning;
  JointVector return_kp{},return_kd{};
  using ArmTorque=Eigen::Matrix<double,14,1>;
  ArmTorque return_tau_offset=ArmTorque::Zero();
  std::size_t marker_index=0;
  const bool jogging=o.which=="C03"||o.which=="C04";
  auto send=[&](std::int64_t now){command.frame_counter++;command.timestamp=now/1000000;return channel.send(command);};
  auto change=[&](int p,const char* event){phase=p;phase_begin=mono();phase_sent=false;recorder.event(p,event);};
  JointVector lower{},upper{},return_speed{},start{};std::string previous_limits;std::int64_t last_limit=0;
  for(int j=0;j<14;++j) {
    lower[j]=model.joints[j].lower;upper[j]=model.joints[j].upper;start[j]=q0[j];
    return_speed[j]=std::min(1.5,model.joints[j].velocity_limit);
    if(reference&&reference->observed_velocity_limit()>0)
      return_speed[j]=std::min(return_speed[j],reference->observed_velocity_limit());
  }
  recorder.event(0,"disabling");
  try {
    while(!done) {
      const auto now=mono();
      if(hard_stop)throw std::runtime_error("operator_interrupt");
      Record r;r.at=now;
      if(now-last_pair>=1000000000LL){r.clock=clock.pair();r.has_pair=true;last_pair=now;r.queues=channel.diagnostics();r.has_queues=true;}
      r.received=channel.receive(state);r.receive_at=r.received?mono():0;
      if(cst&&phase==3) {
        if(now-last_receive>hold.cst_comm_timeout_ns)throw std::runtime_error("cst_feedback_stale");
        if(now-last_send>hold.cst_comm_timeout_ns)throw std::runtime_error("cst_command_stale");
      }
      if(r.received)last_receive=r.receive_at;
      if(r.received&&reference&&reference->observed_velocity_limit()>0) {
        const double body_speed_limit=std::min(1.5,reference->observed_velocity_limit());
        for(const auto& slot:hold.slots) {
          const double speed=std::abs(double(state.joints[slot.slot].velocity))*1e-6;
          if(speed>body_speed_limit) {
            recorder.limit(r.receive_at,phase,slot.slot,"velocity",speed,body_speed_limit,"rad/s","end_control");
            r.phase=phase;r.state=state;r.segment=clock.segment();r.observation=clock.observation(state.timestamp);
            recorder.push(std::move(r));
            throw std::runtime_error("observed_velocity_limit_slot_"+std::to_string(slot.slot));
          }
        }
        const auto limits=field_limits(state,lower,upper,reference->observed_velocity_limit());
        std::string key;int end_joint=-1;
        for(const auto& limit:limits){key+=std::to_string(limit.joint)+limit.field;if(limit.end_control)end_joint=limit.joint;}
        if(key!=previous_limits||(!key.empty()&&now-last_limit>=1000000000LL)) {
          if(key.empty())recorder.limit(r.receive_at,phase,-1,"",0,0,"","recovered");
          for(const auto& limit:limits)recorder.limit(r.receive_at,phase,limit.joint+12,limit.field,limit.value,limit.threshold,limit.unit,limit.end_control?"end_control":"warn");
          previous_limits=key;last_limit=now;
        }
        if(end_joint>=0) {
          r.phase=phase;r.state=state;r.segment=clock.segment();r.observation=clock.observation(state.timestamp);recorder.push(std::move(r));
          throw std::runtime_error("observed_velocity_limit_slot_"+std::to_string(end_joint+12));
        }
      }
      if(now-last_receive>2000000000LL)throw std::runtime_error("feedback_timeout");
      if(phase!=6&&now-last_send>2000000000LL)throw std::runtime_error("command_queue_timeout");
      for(int i=0;i<29;++i)if(state.joints[i].error_code)throw std::runtime_error("drive_error_slot_"+std::to_string(i)+"_"+std::to_string(state.joints[i].error_code));
      if(phase!=3&&phase!=6&&now-phase_begin>3000000000LL)throw std::runtime_error("transition_timeout_phase_"+std::to_string(phase));
      if(soft_stop&&!soft_stop_handled) {
        soft_stop_handled=true;
        if(phase!=4) {
          soft_stop_active=true;
          if(phase<3||(phase==3&&!first_stream))change(4,"soft_stop_disable");
          else if(phase!=3||!reference||reference->direct_fields())throw std::runtime_error("operator_soft_stop");
          else {
            const auto relative_ns=now-first_stream;
            const auto current=reference->at(relative_ns);
            if(std::all_of(current.kp.begin(),current.kp.end(),[](double kp){return kp==0.;})) {
              change(4,"soft_stop_disable");
            }else {
              std::optional<JointVector> via;
              const auto ready_ns=reference->ready_move_end_ns();
              if(reference->has_ready_pose()&&ready_ns>0&&relative_ns>ready_ns)
                via=reference->at(ready_ns).state[0];
              returning=SoftReturn::plan(current.state[0],current.state[1],current.state[2],
                                         start,via,lower,upper,return_speed);
              return_kp=current.kp;return_kd=current.kd;
              const Eigen::Map<const ArmTorque> current_tau(current.effort.data());
              return_has_ff=current_tau.cwiseAbs().maxCoeff()>1e-6;
              if(return_has_ff) {
                workspace.gravity_into(Eigen::Map<const ArmTorque>(current.state[0].data()),return_tau_offset);
                return_tau_offset=current_tau-return_tau_offset;
              }
              return_begin_ns=now;
              recorder.event(3,"soft_stop_return_begin");
            }
          }
        }
      }
      if(phase_sent&&now-phase_begin>20000000) {
        const auto p=hold.enabled()?drive_phase_next(phase,state,selected_axes,modes):drive_phase_next(phase,state,arms);
        if(p==-1){done=true;disable_ack=true;recorder.event(4,"disable_ack");}
        else if(p!=phase)change(p,p==1?"switching_mit":p==2?"enabling_at_start":reference?"reference_begin":jogging?"jog_begin":"hold_begin");
      }
      if(phase>=1)for(int i=0;i<29;++i)if(selected_axes[i])command.joints[i].mode=modes[i];
      if(phase==2)for(const auto& slot:hold.slots)
        command.joints[slot.slot].position=slot.mode==HoldMode::mit?origin[slot.slot]:0;
      if(phase==2&&reference&&!reference->direct_fields()) {
        const auto initial=reference->at(0);
        for(int j=0;j<14;++j)arm_target(command,j,q0[j],0,initial.kp[j],initial.kd[j],initial.effort[j]);
      }
      if(phase==3||phase==7) {
        if(!ack(0x27,true))throw std::runtime_error("enabled_mode_lost");
        const std::int64_t relative_ns=first_stream?now-first_stream:0;
        const double elapsed=relative_ns*1e-9;
        if(hold.enabled())body_targets(command,state,hold,origin,elapsed);
        if(returning) {
          const double t=(now-return_begin_ns)*1e-9;
          const auto sample=returning->at(t);
          ArmTorque tau=ArmTorque::Zero();
          if(return_has_ff) {
            const double u=std::clamp(t/.5,0.,1.);
            const double blend=u*u*u*(10.+u*(-15.+6.*u));
            workspace.gravity_into(Eigen::Map<const ArmTorque>(sample.position.data()),tau);
            tau+=(1.-blend)*return_tau_offset;
            if(!tau.allFinite()||tau.cwiseAbs().maxCoeff()>100.)throw std::runtime_error("soft_stop_feedforward_limit");
          }
          for(int j=0;j<14;++j)
            arm_target(command,j,sample.position[j],sample.velocity[j],return_kp[j],return_kd[j],tau[j]);
          if(t>=returning->total_duration_s())change(4,"soft_stop_disable");
        }else if(reference) {
          while(marker_index<reference->markers().size()&&relative_ns>=reference->markers()[marker_index].first)
            recorder.event(3,reference->markers()[marker_index++].second);
          const auto value=reference->at(relative_ns);
          for(int j=0;j<14;++j)arm_target(command,j,value.state[0][j],value.state[1][j],value.kp[j],value.kd[j],value.effort[j]);
        }
        if(jogging){const auto value=jog_at(elapsed,o.segment,o.amplitude);arm_target(command,arm_index(o.id),q0[arm_index(o.id)]+value.position,value.velocity,o.kp,o.kd);}
        if(!returning&&phase==3&&elapsed>=(reference?reference->duration_ns()*1e-9+(reference->direct_fields()?0.:.5):jogging?3*o.segment+.5:o.duration)) {
          if(o.which=="C05")change(6,"command_writes_stopped");
          else if(o.which=="C06")change(7,"kill_ready");
          else change(4,"exiting_disable");
        }
      }
      if(phase==6&&now-phase_begin>=static_cast<std::int64_t>(o.quiet*1e9))change(4,"quiet_observation_end_disable");
      // Recompute controlword after a phase change; quiet means NO writes.
      for(int i=0;i<29;++i)command.joints[i].control_word=((phase==2||phase==3||phase==7)&&selected_axes[i])?15:6;
      if(phase==4)disable();
      if(phase!=6) {
        r.sent=send(now);r.send_at=r.sent?mono():0;
        if(r.sent){max_gap=std::max(max_gap,r.send_at-last_send);last_send=r.send_at;phase_sent=true;++writes;if(!first_write)first_write=r.send_at;if(phase==3&&!first_stream)first_stream=r.send_at;}
        else ++skips;
      }else last_send=now; // Deliberate silence is not a queue timeout.
      r.phase=phase;r.cmd=command;r.state=state;r.segment=clock.segment();
      if(r.received)r.observation=clock.observation(state.timestamp);
      recorder.push(std::move(r));
      next+=1000000;if(next<mono())next=mono()+1000000;sleep_to(next);
    }
  }catch(const std::exception& e) {
    failure=e.what();recorder.event(5,failure);
    disable();
    const auto end=mono()+200000000;
    bool disable_sent=false;
    while(mono()<end) {
      try {
        Record r;r.at=mono();r.phase=5;
        r.sent=send(r.at);r.send_at=r.sent?mono():0;r.cmd=command;
        r.received=channel.receive(state);r.receive_at=r.received?mono():0;
        r.state=state;r.segment=clock.segment();
        const bool acknowledged=disable_sent&&r.received&&ack(0x21);
        disable_sent=disable_sent||r.sent;
        if(r.received)r.observation=clock.observation(state.timestamp);
        if(r.sent)++writes;else ++skips;recorder.push(std::move(r));
        if(acknowledged){disable_ack=true;break;}
      }catch(const std::exception&){break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  if(soft_stop_active&&failure.empty())failure="operator_soft_stop";
  recorder.finish();
  write_result(root,failure.empty()?"completed_with_disable_ack":failure,failure.empty(),recorder.stopped,first_stream,max_gap,writes,skips,first_write,disable_ack);
  if(reference)std::ofstream(root/"result.txt",std::ios::app)<<"reference_kind="<<(reference->direct_fields()?"direct_fields":"septic_hermite")<<"\nreference_duration_ns="<<reference->duration_ns()<<'\n';
  else std::ofstream(root/"result.txt",std::ios::app)<<"joint_id="<<o.id<<"\namplitude_rad="<<o.amplitude<<"\nkp="<<o.kp<<"\nkd="<<o.kd<<'\n';
  if(!failure.empty())throw std::runtime_error(failure);
}
void snapshot_output(const std::filesystem::path& root,const StatusFrame& state,const casbot::ArmModel& model,double amplitude) {
  binary(root/"status.bin",state);print_status(state);
  std::ofstream csv(root/"arms.csv");csv<<std::setprecision(17)<<"id,joint,q,lower,upper,minus_0_5,plus_0_5,within_limits,requested_amplitude_rad,requested_minus,requested_plus,requested_within_limits\n";
  for(int j=0;j<14;++j){const double q=arm_position(state,j);const auto& m=model.joints[j];
    csv<<j+12<<','<<arm_names[j]<<','<<q<<','<<m.lower<<','<<m.upper<<','<<q-.5<<','<<q+.5<<','<<jog_within_limits(q,m.lower,m.upper,.5)<<','<<amplitude<<','<<q-amplitude<<','<<q+amplitude<<','<<jog_within_limits(q,m.lower,m.upper,amplitude)<<'\n';}
}
void usage() {
  std::cout<<"shm_executor --inspect|--snapshot|--observe|--passive-observe|--execute --output NEW_DIR\n"
    <<"  [--shm /shm_mc] [--urdf FILE] [--hand-background FILE | --bare-flange]\n"
    <<"  [--reference FILE (trajectory or independent five-field bundle)]\n"
    <<"  [--case C01..C06] [--joint-id 12..25] [--duration 3] [--quiet-s 3]\n"
    <<"  [--amplitude .5] [--segment-s 5] [--kp 20] [--kd 2] [--other-arm hold|disabled]\n"
    <<"C06 continues finite holding until supervisor terminates this owned sender; watchdog 3s.\n"
    <<"Inspect/passive are O_RDONLY; observe/snapshot consume status exclusively.\n";
}
}
int field_main(int argc,char** argv) {
  hard_stop=0;soft_stop=0;Options o;bool created=false;
  std::cout<<std::setprecision(9);std::signal(SIGINT,signal_hard_stop);std::signal(SIGTERM,signal_hard_stop);std::signal(SIGHUP,signal_hard_stop);
  std::signal(SIGUSR1,signal_soft_stop);std::signal(SIGPIPE,SIG_IGN);
  try {
    for(int a=1;a<argc;++a) {
      const std::string key=argv[a];
      if(key=="--help"){usage();return 0;}
      if(key=="--bare-flange"){o.bare_flange=true;continue;}
      if(key=="--inspect"||key=="--snapshot"||key=="--execute"||key=="--observe"||key=="--passive-observe"){
        if(!o.mode.empty())throw std::runtime_error("Choose one mode");o.mode=key;continue;}
      if(a+1>=argc)throw std::runtime_error("Missing option value");const std::string value=argv[++a];
      if(key=="--shm")o.name=value;else if(key=="--output")o.output=value;else if(key=="--urdf")o.urdf=value;
      else if(key=="--hand-background")o.background=value;else if(key=="--case")o.which=value;else if(key=="--reference")o.reference=value;
      else if(key=="--other-arm"){if(value!="hold"&&value!="disabled")throw std::runtime_error("Invalid other-arm policy");o.other_disabled=value=="disabled";}
      else {std::size_t used=0;
        if(key=="--joint-id")o.id=std::stoi(value,&used);
        else if(key=="--amplitude")o.amplitude=std::stod(value,&used);else if(key=="--segment-s")o.segment=std::stod(value,&used);
        else if(key=="--kp")o.kp=std::stod(value,&used);else if(key=="--kd")o.kd=std::stod(value,&used);
        else if(key=="--duration")o.duration=std::stod(value,&used);else if(key=="--quiet-s")o.quiet=std::stod(value,&used);
        else throw std::runtime_error("Unknown option: "+key);
        if(used!=value.size())throw std::runtime_error("Invalid numeric option: "+key);
      }
    }
    if(o.output.empty()||o.mode.empty())throw std::runtime_error("Mode and new output directory required");
    const auto index=arm_index(o.id);
    if(!o.reference.empty()){if(o.mode!="--execute"||o.other_disabled)throw std::runtime_error("Reference requires execute with all-arm policy");o.which="REF";}
    if(o.which!="C01"&&o.which!="C02"&&o.which!="C03"&&o.which!="C04"&&o.which!="C05"&&o.which!="C06"&&o.which!="REF")throw std::runtime_error("Unsupported active case");
    if(!std::isfinite(o.amplitude)||o.amplitude<=0||o.amplitude>.5||!std::isfinite(o.segment)||o.segment<1||
      !std::isfinite(o.kp)||o.kp<0||o.kp>100||!std::isfinite(o.kd)||o.kd<0||o.kd>10||
      !std::isfinite(o.duration)||o.duration<=0||o.duration>3600||!std::isfinite(o.quiet)||o.quiet<=0||o.quiet>60)throw std::runtime_error("Invalid test settings");
    const auto root=std::filesystem::path(o.output);if(!root.parent_path().empty())std::filesystem::create_directories(root.parent_path());
    if(!std::filesystem::create_directory(root))throw std::runtime_error("Output must be a new directory");created=true;
    std::ofstream settings(root/"settings.txt");
    settings<<"case="<<(o.mode=="--execute"?o.which:"read_only")<<"\nshm="<<o.name<<"\nmode="<<o.mode
      <<"\nhand_policy="<<(o.mode=="--execute"?"explicit_frozen_command_bytes":"not_commanded")<<"\nurdf="<<o.urdf
      <<"\ncollision_check="<<(o.mode=="--execute"?(o.reference.empty()?"disabled_by_user":"not_performed_by_native_reference_mode"):"not_applicable")<<'\n';
    if(o.reference.empty())settings<<"joint_id="<<o.id<<"\namplitude="<<o.amplitude<<"\nsegment_s="<<o.segment<<"\nkp="<<o.kp<<"\nkd="<<o.kd<<"\nother_arm="<<(o.other_disabled?"disabled":"hold")<<'\n';
    else settings<<"reference="<<o.reference<<"\nreference_targets=all_14_arms\n";
    if(o.reference.empty())settings<<"non_arm="<<(o.mode=="--execute"?"disabled":"not_commanded")<<'\n';
    settings.close();
    if(o.mode=="--inspect"){
      const auto s=passive_snapshot(o.name);binary(root/"command.bin",s.command);binary(root/"status.bin",s.status);
      binary(root/"command_queue.bin",s.command_queue);binary(root/"status_queue.bin",s.status_queue);print_status(s.status);
      write_result(root,"passive_snapshot_not_live_proof",true,false);return 0;
    }
    const auto parent=getppid();if(parent<=1||prctl(PR_SET_PDEATHSIG,SIGTERM)||getppid()!=parent)throw std::runtime_error("No live supervisor");
    if(o.mode=="--observe"||o.mode=="--passive-observe"){observe(o);return 0;}
    if(o.urdf.empty())throw std::runtime_error("Explicit 14-axis URDF required");
    const auto model=casbot::ArmDynamics::from_urdf(o.urdf);
    if(model.model().joints.size()!=14)throw std::runtime_error("Expected reduced 14-axis model");
    for(int j=0;j<14;++j)if(model.model().joints[j].name!=arm_names[j])throw std::runtime_error("URDF arm order mismatch");
    std::unique_ptr<Reference> reference;
    if(!o.reference.empty()) {
      std::filesystem::copy_file(o.reference,root/"reference.json");
      reference=std::make_unique<Reference>(Reference::load((root/"reference.json").string()));
      std::ofstream log(root/"settings.txt",std::ios::app);
      const auto& hold=reference->non_arm_hold();
      if(hold.enabled()) {
        log<<"non_arm=hold(";
        for(std::size_t i=0;i<hold.slots.size();++i)log<<(i?",":"")<<hold.slots[i].slot;
        log<<")\n";
      }else log<<"non_arm=disabled\n";
      JointVector lower{},upper{},velocity{};
      for(int j=0;j<14;++j){lower[j]=model.model().joints[j].lower;upper[j]=model.model().joints[j].upper;velocity[j]=std::min(1.5,model.model().joints[j].velocity_limit);}
      if(!reference->measured_origin())reference->validate_limits(lower,upper,velocity);
    }
    OwnerLease lease(o.name);Channel channel(o.name);auto state=fresh(channel);
    snapshot_output(root,state,model.model(),o.amplitude);
    if(o.mode=="--snapshot"){write_result(root,"snapshot_completed_no_commands",true,false);return 0;}
    if(o.bare_flange&&!o.background.empty())throw std::runtime_error("Choose bare flange or hand background");
    if(!o.bare_flange&&o.background.empty())throw std::runtime_error("Explicit hand command background required");
    const auto q0=positions(state);const auto joint=model.model().joints[index];
    if(reference&&reference->measured_origin()) {
      JointVector origin{},lower{},upper{},velocity{};
      std::ofstream origin_csv(root/"reference_origin.csv");origin_csv<<std::setprecision(17)<<"joint_id,q_rad\n";
      for(int j=0;j<14;++j){origin[j]=q0[j];origin_csv<<j+12<<','<<origin[j]<<'\n';lower[j]=model.model().joints[j].lower;upper[j]=model.model().joints[j].upper;velocity[j]=std::min(1.5,model.model().joints[j].velocity_limit);}
      reference->resolve_origin(origin);reference->validate_limits(lower,upper,velocity);
    }
    if(reference&&!reference->direct_fields())for(int j=0;j<14;++j)
      if(std::abs(reference->at(0).state[0][j]-q0[j])>.005)throw std::runtime_error("Reference start differs from fresh measured position; no commands sent");
    const bool jogging=o.which=="C03"||o.which=="C04";
    if(jogging&&4.375*o.amplitude/o.segment>std::min(1.5,joint.velocity_limit))throw std::runtime_error("Jog reference exceeds configured/URDF velocity limit");
    if(jogging&&!jog_within_limits(q0[index],joint.lower,joint.upper,o.amplitude)){
      write_result(root,"SKIP_urdf_limit",false,false);return 4;
    }
    for(int j=0;j<14;++j)if(q0[j]<model.model().joints[j].lower||q0[j]>model.model().joints[j].upper)throw std::runtime_error("Measured start outside URDF limit at joint ID "+std::to_string(j+12));
    const auto first_state=state;
    state=fresh(channel);
    if(reference&&reference->non_arm_hold().enabled()) {
      for(const auto& slot:reference->non_arm_hold().slots) {
        const auto& a=state.joints[slot.slot];
        if(std::abs(double(a.position)-first_state.joints[slot.slot].position)>5000||
           std::abs(double(a.velocity))*1e-6>=casbot::field_start_velocity_rad_s)
          throw std::runtime_error("Non-arm hold start not stationary at slot "+std::to_string(slot.slot)+"; no commands sent");
      }
      for(int i=0;i<29;++i)if(state.joints[i].error_code)
        throw std::runtime_error("drive_error_slot_"+std::to_string(i)+"; no commands sent");
    }
    if((positions(state)-q0).cwiseAbs().maxCoeff()>.005)throw std::runtime_error("Start changed during preflight; no commands sent");
    const auto hand=o.bare_flange?CommandFrame{}:read_background(o.background);binary(root/"hand_background_source.bin",hand);
    run(channel,state,hand,q0,o,model,reference.get());return 0;
  }catch(const std::exception& e){
    std::cerr<<"ERROR "<<e.what()<<'\n';
    if(created&&!std::filesystem::exists(std::filesystem::path(o.output)/"result.txt"))write_result(o.output,e.what(),false,false);
    return 1;
  }
}
}  // namespace casbot::shm
