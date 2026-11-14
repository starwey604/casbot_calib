#include "gui_runtime.hpp"
#include "gui_wire.hpp"
#include "field_recorder.hpp"
#include "json_input.hpp"
#include <casbot_calib/arm_dynamics.hpp>
#include <atomic>
#include <csignal>
#include <deque>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <poll.h>
#include <sstream>
#include <thread>
#include <sys/prctl.h>
namespace casbot::shm {namespace {
volatile std::sig_atomic_t stop_signal=0;
void signal_exit(int){stop_signal=1;}
std::string json_quote(const std::string& s){std::ostringstream o;o<<'"';for(char c:s){if(c=='"'||c=='\\')o<<'\\';if(c=='\n')o<<"\\n";else if(static_cast<unsigned char>(c)>=32)o<<c;}o<<'"';return o.str();}
struct Message {
  std::string type,error;std::uint64_t id=0,control=0,revision=0,challenge=0,seq=0;
  std::array<Joints,5> fields{};
  double velocity=1.5,effort=10,kp=100,kd=10,lease_ms=1000;bool end_velocity=false;
};
Message parse(const std::string& line) {
  Message m;
  try {
    Json p;std::istringstream stream(line);boost::property_tree::read_json(stream,p);
    std::set<std::string> keys;for(const auto& item:p)if(!keys.insert(item.first).second)throw std::runtime_error("duplicate_key");
    m.id=p.get<std::uint64_t>("id");m.type=p.get<std::string>("type");m.control=p.get<std::uint64_t>("control",0);
    if(m.type=="fields") {
      m.revision=p.get<std::uint64_t>("revision");
      int j=0;for(const auto* name:{"position","velocity","effort","kp","kd"})m.fields[j++]=joint_values(p.get_child(name));
    }else if(m.type=="renew") {m.challenge=p.get<std::uint64_t>("challenge");m.seq=p.get<std::uint64_t>("seq");}
    else if(m.type=="settings") {
      m.velocity=finite_number(p.get_child("velocity_limit"));m.effort=finite_number(p.get_child("effort_limit"));
      m.kp=finite_number(p.get_child("kp_max"));m.kd=finite_number(p.get_child("kd_max"));
      m.lease_ms=finite_number(p.get_child("lease_ms"));m.end_velocity=json_bool(p.get_child("end_velocity"));
      if(m.velocity<=0||m.velocity>1.5||m.effort<=0||m.effort>327||m.kp<20||m.kp>3276||m.kd<2||m.kd>3276||m.lease_ms<200||m.lease_ms>10000)throw std::runtime_error("settings_range");
    }else if(m.type!="start"&&m.type!="stop"&&m.type!="quit")throw std::runtime_error("unknown_type");
    if(!m.id)throw std::runtime_error("positive_request_id_required");
  }catch(const std::exception& e){m.error=e.what();}
  return m;
}
class IO {
  std::mutex in_mutex,out_mutex;std::deque<Message> requests;std::optional<Message> target,stop;
  std::deque<std::string> events;std::optional<GuiSnapshot> snapshot;
  std::thread reader,writer;
 public:
  std::atomic<bool> disconnected{false},finished{false};
  explicit IO(const std::filesystem::path& root,Recorder& recorder) {
    reader=std::thread([this]{
      std::string pending;char data[4096];
      while(!finished&&!disconnected) {
        pollfd p{STDIN_FILENO,POLLIN,0};int n=poll(&p,1,20);if(n<=0)continue;
        const auto count=read(STDIN_FILENO,data,sizeof(data));if(count<=0){disconnected=true;break;}
        pending.append(data,count);if(pending.size()>32768){disconnected=true;break;}
        std::size_t end;
        while((end=pending.find('\n'))!=std::string::npos) {
          auto m=parse(pending.substr(0,end));pending.erase(0,end+1);
          if(!m.error.empty()){event("{\"type\":\"reply\",\"id\":"+std::to_string(m.id)+",\"ok\":false,\"reason\":"+json_quote(m.error)+"}");continue;}
          std::lock_guard<std::mutex> lock(in_mutex);
          if(m.type=="stop"||m.type=="quit")stop=m;
          else if(m.type=="fields"){
            if(target)event("{\"type\":\"reply\",\"id\":"+std::to_string(target->id)+",\"ok\":false,\"reason\":\"superseded\"}");target=m;
          }else if(requests.size()<16)requests.push_back(m);
          else {disconnected=true;break;}
        }
      }
    });
    writer=std::thread([this,root,&recorder]{
      std::ofstream archive(root/"controls.jsonl");
      fcntl(STDOUT_FILENO,F_SETFL,fcntl(STDOUT_FILENO,F_GETFL)|O_NONBLOCK);
      while(true) {
        std::string event_value;std::optional<GuiSnapshot> snap;
        {std::lock_guard<std::mutex> lock(out_mutex);if(!events.empty()){event_value=std::move(events.front());events.pop_front();}else if(snapshot){snap=std::move(snapshot);snapshot.reset();}}
        if(event_value.empty()&&!snap){if(finished)break;std::this_thread::sleep_for(std::chrono::milliseconds(1));continue;}
        std::vector<unsigned char> body;
        if(snap)body=snap->encode();else {body.assign(event_value.begin(),event_value.end());if(!recorder.stopped){archive<<event_value<<'\n';archive.flush();if(archive)recorder.external_bytes=static_cast<std::uint64_t>(archive.tellp());else recorder.stopped=true;}}
        std::vector<unsigned char> packet;wire_int<std::uint32_t>(packet,body.size()+1);packet.push_back(snap?2:1);packet.insert(packet.end(),body.begin(),body.end());
        std::size_t pos=0;auto deadline=mono()+2000000000LL;
        while(pos<packet.size()) {
          pollfd p{STDOUT_FILENO,POLLOUT,0};const auto ready=poll(&p,1,20);
          if(ready>0){const auto n=write(STDOUT_FILENO,packet.data()+pos,packet.size()-pos);if(n>0)pos+=n;else if(errno!=EAGAIN&&errno!=EINTR){disconnected=true;return;}}
          if(mono()>deadline|| (finished&&disconnected)){disconnected=true;return;}
        }
      }
    });
  }
  std::vector<Message> take() {
    std::unique_lock<std::mutex> lock(in_mutex,std::try_to_lock);if(!lock.owns_lock())return {};
    std::vector<Message> messages;
    if(stop){messages.push_back(*stop);stop.reset();requests.clear();target.reset();return messages;}
    while(!requests.empty()){messages.push_back(std::move(requests.front()));requests.pop_front();}
    if(target){messages.push_back(std::move(*target));target.reset();}return messages;
  }
  void event(std::string value){std::lock_guard<std::mutex> lock(out_mutex);if(events.size()<128)events.push_back(std::move(value));else disconnected=true;}
  void publish(const GuiSnapshot& value){std::unique_lock<std::mutex> lock(out_mutex,std::try_to_lock);if(lock.owns_lock())snapshot=value;}
  ~IO(){finished=true;if(reader.joinable())reader.join();if(writer.joinable())writer.join();}
};
void reply(IO& io,const Message& m,bool ok,const std::string& reason){io.event("{\"type\":\"reply\",\"id\":"+std::to_string(m.id)+",\"ok\":"+(ok?"true":"false")+",\"reason\":"+json_quote(reason)+"}");}
}
int gui_main(int argc,char** argv) {
  stop_signal=0;std::signal(SIGINT,signal_exit);std::signal(SIGTERM,signal_exit);std::signal(SIGHUP,signal_exit);std::signal(SIGPIPE,SIG_IGN);
  std::string name,output,urdf,background;double budget=512;
  try {
    for(int a=1;a<argc;++a){std::string key=argv[a];if(a+1>=argc)throw std::runtime_error("option_value_required");std::string value=argv[++a];if(key=="--shm")name=value;else if(key=="--output")output=value;else if(key=="--urdf")urdf=value;else if(key=="--hand-background")background=value;else if(key=="--budget-mib")budget=std::stod(value);else throw std::runtime_error("unknown_option");}
    if(name.empty()||output.empty()||urdf.empty()||!std::isfinite(budget)||budget<=0)throw std::runtime_error("explicit_shm_output_urdf_required");
    const auto parent=getppid();if(parent<=1||prctl(PR_SET_PDEATHSIG,SIGTERM)||getppid()!=parent)throw std::runtime_error("no_supervisor");
    const auto model=casbot::ArmDynamics::from_urdf(urdf);const auto& joints=model.model().joints;
    if(joints.size()!=14)throw std::runtime_error("arm_model_required");
    for(int j=0;j<14;++j)if(joints[j].name!=arm_names[j])throw std::runtime_error("arm_order_mismatch");
    OwnerLease lease(name);Channel channel(name);
    const auto root=std::filesystem::path(output);std::filesystem::create_directories(root.parent_path());
    if(!std::filesystem::create_directory(root))throw std::runtime_error("new_output_required");
    CommandFrame hand{};bool have_background=!background.empty();
    if(have_background){std::ifstream file(background,std::ios::binary);std::vector<char> bytes{std::istreambuf_iterator<char>(file),{}};hand=decode_command(bytes.data(),bytes.size());if(hand.hand_count>2)throw std::runtime_error("invalid_hands");std::filesystem::copy_file(background,root/"hand_background_source.bin");}
    Recorder recorder(root,static_cast<std::uint64_t>(budget*1024*1024));IO io(root,recorder);ClockMap clock;
    GuiSnapshot snap;Message settings;std::int64_t next=mono(),last_pair=0,last_publish=0,last_new=0,phase_begin=0,last_send=0,lease_end=0,last_limit=0;
    std::int64_t stop_timeout=3000000000LL;std::uint64_t last_start=0,pending_revision=0,new_frames=0,previous_frame=0;
    std::array<std::int64_t,512> sent_times{};bool sent_phase=false,quit=false;std::string reason="read_only";
    ArmMask all{};all.fill(true);
    auto event=[&](std::string text){recorder.event(snap.phase,text);io.event("{\"type\":\"status\",\"control\":"+std::to_string(snap.control)+",\"phase\":"+std::to_string(snap.phase)+",\"reason\":"+json_quote(text)+",\"disable_ack\":"+(snap.ack?"true":"false")+"}");};
    auto change=[&](int phase,std::string text){snap.phase=phase;phase_begin=mono();sent_phase=false;event(text);};
    auto stopping=[&](const std::string& why,bool fault){if(snap.phase>=0&&snap.phase!=4){reason=why;stop_timeout=fault?200000000LL:3000000000LL;change(4,"stopping_"+why);}};
    auto finish=[&](bool ack){snap.ack=ack;change(-1,ack?"disabled_"+reason:"disable_unconfirmed_"+reason);pending_revision=0;snap.challenge=0;snap.expires=0;};
    {std::ostringstream hello;hello<<std::setprecision(17)<<"{\"type\":\"hello\",\"version\":1,\"lower\":[";for(int j=0;j<14;++j)hello<<(j?",":"")<<joints[j].lower;hello<<"],\"upper\":[";for(int j=0;j<14;++j)hello<<(j?",":"")<<joints[j].upper;hello<<"],\"velocity\":[";for(int j=0;j<14;++j)hello<<(j?",":"")<<std::min(1.5,joints[j].velocity_limit);hello<<"],\"hand_background\":"<<(have_background?"true":"false")<<"}";io.event(hello.str());}
    try {while(true) {
      const auto now=mono();Record r;r.at=now;r.received=channel.receive(snap.status);r.receive_at=r.received?mono():0;
      if(now-last_pair>=1000000000LL){r.clock=clock.pair();r.has_pair=true;last_pair=now;}
      if(r.received){snap.receive=r.receive_at;snap.observation=clock.observation(snap.status.timestamp);snap.segment=clock.segment();}
      if(r.received&&(!last_new||snap.status.frame_counter!=previous_frame)){last_new=now;previous_frame=snap.status.frame_counter;++new_frames;}
      if(stop_signal||io.disconnected){quit=true;stopping(stop_signal?"operator_interrupt":"bridge_eof",true);}
      for(auto& m:io.take()) {
        if(m.type=="quit"){quit=true;stopping("close",false);reply(io,m,true,"closing");}
        else if(m.type=="stop"){if(m.control&&m.control!=snap.control)reply(io,m,false,"old_control");else{stopping("operator_stop",false);reply(io,m,true,"stop_requested");}}
        else if(m.type=="settings") {
          if(snap.phase>=0){reply(io,m,false,"settings_require_read_only");continue;}
          settings=m;reply(io,m,true,"settings_applied");
          io.event("{\"type\":\"settings\",\"id\":"+std::to_string(m.id)+",\"velocity_limit\":"+std::to_string(settings.velocity)+",\"end_velocity\":"+(settings.end_velocity?"true":"false")+",\"lease_ms\":"+std::to_string(settings.lease_ms)+",\"effort_limit\":"+std::to_string(settings.effort)+",\"kp_max\":"+std::to_string(settings.kp)+",\"kd_max\":"+std::to_string(settings.kd)+"}");
        }else if(m.type=="start") {
          if(m.id<=last_start){reply(io,m,m.id==snap.control&&snap.phase>=0,"start_already_processed");continue;}
          last_start=m.id;
          std::string error;
          if(quit||snap.phase>=0)error="busy";else if(!have_background)error="hand_background_required";
          else if(new_frames<2||now-last_new>200000000LL)error="fresh_feedback_required";
          else for(int j=0;j<29;++j)if(snap.status.joints[j].error_code)error="drive_fault";
          std::array<double,14> q{};
          for(int j=0;j<14;++j){q[j]=arm_position(snap.status,j);if(q[j]<joints[j].lower||q[j]>joints[j].upper)error="start_outside_urdf";}
          if(!error.empty()){reply(io,m,false,error);continue;}
          snap.command=initial_command(snap.status,hand,q,20,2);snap.control=m.id;snap.ack=false;snap.revision=0;snap.applied_at=0;pending_revision=0;
          snap.challenge++;snap.expires=now+static_cast<std::int64_t>(settings.lease_ms*1e6);lease_end=snap.expires;last_send=now;
          reason="";change(0,"disabling");reply(io,m,true,"starting_at_measured_position");
        }else if(m.type=="renew") {
          bool valid=snap.phase>=0&&snap.phase<4&&m.control==snap.control&&m.challenge==snap.challenge&&now<snap.expires&&m.seq&&m.seq<=snap.seq&&snap.seq-m.seq<sent_times.size()&&now-sent_times[m.seq%sent_times.size()]<static_cast<std::int64_t>(settings.lease_ms*1e6);
          if(valid){lease_end=snap.expires;++snap.challenge;snap.expires=now+static_cast<std::int64_t>(settings.lease_ms*1e6);}
          // Lease renewals have no reply traffic; the next challenge is the acknowledgement.
        }else if(m.type=="fields") {
          std::string error;auto c=snap.command;
          if(snap.phase!=3||m.control!=snap.control)error="not_current_control";else if(!m.revision||m.revision<=pending_revision)error="old_revision";
          if(error.empty())try{for(int j=0;j<14;++j){const auto q=m.fields[0][j],v=m.fields[1][j],e=m.fields[2][j],kp=m.fields[3][j],kd=m.fields[4][j];if(q<joints[j].lower||q>joints[j].upper||std::abs(v)>std::min(settings.velocity,joints[j].velocity_limit)||std::abs(e)>settings.effort||kp>settings.kp||kd>settings.kd)throw std::runtime_error("target_range_id_"+std::to_string(j+12));arm_target(c,j,q,v,kp,kd,e);}}
          catch(const std::exception& e){error=e.what();}
          if(error.empty()){snap.command=c;pending_revision=m.revision;reply(io,m,true,"accepted_not_yet_enqueued");}else reply(io,m,false,error);
        }
      }
      if(snap.phase>=0&&snap.phase<4) {
        if(now>lease_end)stopping("lease_expired",true);
        if(now-last_new>2000000000LL)stopping("feedback_timeout",true);
        if(now-last_send>2000000000LL)stopping("command_queue_timeout",true);
        for(int j=0;j<29;++j)if(snap.status.joints[j].error_code){stopping("drive_error_slot_"+std::to_string(j),true);break;}
        if(snap.phase!=3&&now-phase_begin>3000000000LL)stopping("transition_timeout",true);
        if(snap.phase==3&&!drive_ack(snap.status,0x27,all,true))stopping("enabled_mode_lost",true);
      }
      bool limit_recorded=false;
      if(r.received)for(int j=0;j<14;++j){const auto a=arm_feedback(snap.status,j);const bool pos=a.position<joints[j].lower||a.position>joints[j].upper,vel=std::abs(a.velocity)>settings.velocity;
        if((pos||vel)&&now-last_limit>=1000000000LL){limit_recorded=true;recorder.limit(r.receive_at,snap.phase,j+12,pos?"position":"velocity",pos?a.position:a.velocity,pos?(a.position<joints[j].lower?joints[j].lower:joints[j].upper):settings.velocity,pos?"rad":"rad/s",vel&&settings.end_velocity?"end_control":"warn");}
        if(vel&&settings.end_velocity&&snap.phase>=0&&snap.phase<4)stopping("observed_velocity_slot_"+std::to_string(j+12),true);
      }
      if(limit_recorded)last_limit=now;
      if(snap.phase>=0&&sent_phase&&now-phase_begin>20000000LL) {
        int p=drive_phase_next(snap.phase,snap.status,all);
        if(p==-1)finish(true);else if(p!=snap.phase)change(p,p==1?"switching_mit":p==2?"enabling_at_start":"manual_ready");
      }
      if(snap.phase==4&&now-phase_begin>stop_timeout)finish(false);
      if(snap.phase>=0) {
        for(int j=12;j<26;++j){auto& a=snap.command.joints[j];a.control_word=(snap.phase==2||snap.phase==3)?15:6;if(snap.phase>=1)a.mode=-1;if(snap.phase==4){a.velocity=0;a.torque=0;}}
        snap.command.frame_counter++;snap.command.timestamp=now/1000000;r.sent=channel.send(snap.command);r.send_at=r.sent?mono():0;
        if(r.sent){snap.max_gap=std::max(snap.max_gap,r.send_at-last_send);last_send=r.send_at;++snap.writes;sent_phase=true;if(snap.phase==3&&(snap.applied_at==0||snap.revision!=pending_revision)){snap.revision=pending_revision;snap.applied_at=r.send_at;}}
        else ++snap.skips;
      }
      r.phase=snap.phase;r.cmd=snap.command;r.state=snap.status;r.observation=snap.observation;r.segment=snap.segment;recorder.push(std::move(r));
      if(now-last_publish>=5000000LL){snap.at=now;snap.recording_stopped=recorder.stopped;++snap.seq;sent_times[snap.seq%sent_times.size()]=now;io.publish(snap);last_publish=now;}
      if(quit&&snap.phase<0)break;
      next+=1000000;if(next<mono())next=mono()+1000000;timespec t{next/1000000000LL,next%1000000000LL};clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&t,nullptr);
    }
    }catch(const std::exception& error) {
      reason=std::string("native_fault_")+error.what();snap.ack=false;
      if(snap.phase>=0){for(int j=12;j<26;++j){snap.command.joints[j].control_word=6;snap.command.joints[j].velocity=0;snap.command.joints[j].torque=0;}
      const auto end=mono()+200000000LL;
      while(mono()<end)try {
        Record r;r.at=mono();r.phase=4;r.received=channel.receive(snap.status);r.receive_at=r.received?mono():0;
        snap.command.frame_counter++;snap.command.timestamp=r.at/1000000;
        r.sent=channel.send(snap.command);r.send_at=r.sent?mono():0;r.cmd=snap.command;r.state=snap.status;r.segment=clock.segment();r.observation=clock.observation(snap.status.timestamp);
        if(r.sent)++snap.writes;else ++snap.skips;recorder.push(std::move(r));
        if(drive_ack(snap.status,0x21,all)){snap.ack=true;break;}
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }catch(const std::exception&){break;}
      }snap.phase=-1;event(reason);
    }
    recorder.finish();std::ofstream result(root/"result.json");result<<"{\"reason\":"<<json_quote(reason)<<",\"disable_ack\":"<<(snap.ack?"true":"false")<<",\"commands_sent\":"<<snap.writes<<",\"recording_stopped\":"<<(recorder.stopped?"true":"false")<<"}\n";
    io.event("{\"type\":\"closed\",\"disable_ack\":"+std::string(snap.ack?"true":"false")+"}");return 0;
  }catch(const std::exception& e){std::cerr<<"shm_gui_owner: "<<e.what()<<'\n';return 1;}
}
}
