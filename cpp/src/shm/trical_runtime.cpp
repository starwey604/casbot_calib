#include "trical_runtime.hpp"
#include "field_common.hpp"
#include "field_recorder.hpp"
#include <casbot_calib/arm_dynamics.hpp>
#include <atomic>
#include <csignal>
#include <deque>
#include <iostream>
#include <optional>
#include <poll.h>
#include <sys/prctl.h>

namespace casbot::shm {namespace {
volatile std::sig_atomic_t interrupted=0;
void stop(int){interrupted=1;}
std::string quote(const std::string& s){std::ostringstream o;o<<'"';for(char c:s){if(c=='"'||c=='\\')o<<'\\';if(static_cast<unsigned char>(c)>=32)o<<c;}o<<'"';return o.str();}
std::string array(const Joints& a){std::ostringstream s;s<<std::setprecision(17)<<'[';for(int j=0;j<14;++j)s<<(j?",":"")<<a[j];return s.str()+']';}
Joints angles(const StatusFrame& s){Joints q{};for(int j=0;j<14;++j)q[j]=arm_position(s,j);return q;}
struct Request {std::uint64_t id=0;std::string type,error;TricalSegment segment;};
Request parse(const std::string& line){
  Request r;
  try{
    Json p;std::istringstream input(line);boost::property_tree::read_json(input,p);
    r.id=p.get<std::uint64_t>("id");r.type=p.get<std::string>("type");
    if(!r.id)throw std::runtime_error("positive_id_required");
    if(r.type=="segment"){
      r.segment.id=r.id;r.segment.duration_ns=p.get<std::int64_t>("duration_ns");
      r.segment.from=joint_values(p.get_child("q_from"));r.segment.to=joint_values(p.get_child("q_to"));
      r.segment.windup=json_bool(p.get_child("windup"));
    }else if(r.type!="finish")throw std::runtime_error("unknown_request");
  }catch(const std::exception& e){r.error=e.what();}
  return r;
}
class IO {
  std::mutex in_lock,out_lock;std::deque<Request> requests;std::deque<std::string> events;
  std::thread reader,writer;
 public:
  std::atomic<bool> disconnected{false},finished{false};
  IO(){
    reader=std::thread([this]{
      std::string pending;char bytes[4096];
      while(!finished&&!disconnected){
        pollfd p{STDIN_FILENO,POLLIN,0};if(poll(&p,1,20)<=0)continue;
        auto n=read(STDIN_FILENO,bytes,sizeof(bytes));if(n<=0){disconnected=true;break;}
        pending.append(bytes,n);if(pending.size()>32768){disconnected=true;break;}
        std::size_t end;
        while((end=pending.find('\n'))!=std::string::npos){
          auto r=parse(pending.substr(0,end));pending.erase(0,end+1);
          std::lock_guard<std::mutex> lock(in_lock);
          if(requests.size()>=16){disconnected=true;break;}
          requests.push_back(std::move(r));
        }
      }
    });
    writer=std::thread([this]{
      fcntl(STDOUT_FILENO,F_SETFL,fcntl(STDOUT_FILENO,F_GETFL)|O_NONBLOCK);
      while(true){
        std::string line;
        {std::lock_guard<std::mutex> lock(out_lock);if(!events.empty()){line=std::move(events.front());events.pop_front();}}
        if(line.empty()){if(finished)break;std::this_thread::sleep_for(std::chrono::milliseconds(1));continue;}
        line+='\n';std::size_t at=0;const auto deadline=mono()+2000000000LL;
        while(at<line.size()){
          pollfd p{STDOUT_FILENO,POLLOUT,0};if(poll(&p,1,20)>0){
            auto n=write(STDOUT_FILENO,line.data()+at,line.size()-at);
            if(n>0)at+=n;else if(errno!=EAGAIN&&errno!=EINTR){disconnected=true;return;}
          }
          if(mono()>deadline||(finished&&disconnected)){disconnected=true;return;}
        }
      }
    });
  }
  std::deque<Request> take(){
    std::unique_lock<std::mutex> lock(in_lock,std::try_to_lock);std::deque<Request> result;
    if(lock.owns_lock())result.swap(requests);return result;
  }
  void emit(std::string line){
    std::lock_guard<std::mutex> lock(out_lock);
    if(events.size()>=128){disconnected=true;return;}events.push_back(std::move(line));
  }
  ~IO(){finished=true;reader.join();writer.join();}
};
struct Options {
  std::string endpoint,urdf,output,side,background;
  Joints kp{},kd{};
  Eigen::Vector3d com=Eigen::Vector3d::Zero(),gravity{0,0,-9.81};
  double mass=0,windup=.012,gain=1,effort_limit=100;
  bool bare=false,have_kp=false,have_kd=false;
};
std::vector<double> numbers(std::string value){
  std::replace(value.begin(),value.end(),',',' ');std::istringstream s(value);std::vector<double> out;double x;
  while(s>>x){if(!std::isfinite(x))throw std::runtime_error("nonfinite_option");out.push_back(x);}
  if(!s.eof())throw std::runtime_error("invalid_numeric_option");return out;
}
Options options(int argc,char** argv){
  Options o;
  for(int i=1;i<argc;++i){
    std::string key=argv[i];if(key=="--bare-flange"){o.bare=true;continue;}
    if(i+1==argc)throw std::runtime_error("option_value_required");std::string v=argv[++i];
    if(key=="--shm")o.endpoint=v;else if(key=="--urdf")o.urdf=v;else if(key=="--output")o.output=v;
    else if(key=="--tool-side")o.side=v;else if(key=="--hand-background")o.background=v;
    else if(key=="--kp"||key=="--kd"){
      auto a=numbers(v);if(a.size()!=14)throw std::runtime_error("expected_14_gains");
      for(int j=0;j<14;++j){if(a[j]<0||a[j]>(key=="--kp"?500:50))throw std::runtime_error("gain_range");(key=="--kp"?o.kp:o.kd)[j]=a[j];}
      (key=="--kp"?o.have_kp:o.have_kd)=true;
    }else if(key=="--tool-com-m"||key=="--gravity"){
      auto a=numbers(v);if(a.size()!=3)throw std::runtime_error("expected_three_values");
      for(int j=0;j<3;++j)(key=="--gravity"?o.gravity:o.com)[j]=a[j];
    }else {
      auto a=numbers(v);if(a.size()!=1)throw std::runtime_error("scalar_required");
      if(key=="--tool-mass-kg")o.mass=a[0];else if(key=="--max-windup-rad")o.windup=a[0];
      else if(key=="--effort-gain")o.gain=a[0];else if(key=="--effort-limit-nm")o.effort_limit=a[0];else throw std::runtime_error("unknown_option");
    }
  }
  if(o.endpoint.empty()||o.urdf.empty()||o.output.empty()||!o.have_kp||!o.have_kd||o.bare==!o.background.empty()||
      (o.side!="left"&&o.side!="right")||o.mass<0||o.windup<=0||o.effort_limit<=0)throw std::runtime_error("invalid_executor_options");
  return o;
}
void check_start(const StatusFrame& state,const casbot::ArmModel& model){
  for(int i=0;i<29;++i)if(state.joints[i].error_code)throw std::runtime_error("start_drive_fault");
  for(int j=0;j<14;++j){auto a=arm_feedback(state,j);
    if(a.position<model.joints[j].lower||a.position>model.joints[j].upper)throw std::runtime_error("start_outside_urdf");
    if(std::abs(a.velocity)>=.2)throw std::runtime_error("start_not_stationary");
  }
}
StatusFrame fresh(Channel& channel){
  StatusFrame s{};std::optional<std::uint64_t> first;auto deadline=mono()+2000000000LL;
  while(mono()<deadline&&!interrupted){
    if(channel.receive(s)){if(first&&s.frame_counter!=*first)return s;first=s.frame_counter;}
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  throw std::runtime_error("fresh_feedback_required");
}
void sleep_to(std::int64_t next){timespec ts{next/1000000000LL,next%1000000000LL};while(clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&ts,nullptr)==EINTR&&!interrupted){}}
}
int trical_main(int argc,char** argv){
  interrupted=0;std::signal(SIGINT,stop);std::signal(SIGTERM,stop);std::signal(SIGHUP,stop);std::signal(SIGPIPE,SIG_IGN);
  if(argc==2&&std::string(argv[1])=="--help"){
    std::cout<<"shm_trical_executor --shm ENDPOINT --urdf FILE --output DIR --bare-flange|--hand-background FILE --tool-side left|right --kp CSV14 --kd CSV14 [--tool-mass-kg M --tool-com-m x,y,z --max-windup-rad .012]\n";return 0;
  }
  try{
    auto o=options(argc,argv);auto imported=casbot::ArmDynamics::from_urdf(o.urdf);auto model=imported.model();model.gravity_world=o.gravity;
    if(model.joints.size()!=14)throw std::runtime_error("expected_14_joints");
    Joints lower{},upper{},velocity{};
    for(int j=0;j<14;++j){if(model.joints[j].name!=arm_names[j])throw std::runtime_error("arm_order_mismatch");lower[j]=model.joints[j].lower;upper[j]=model.joints[j].upper;velocity[j]=std::min(1.5,model.joints[j].velocity_limit);}
    casbot::ArmDynamics engine(model);auto workspace=engine.workspace();
    const auto tool=o.side+"_trical_tcp";
    if(o.mass>0)workspace.point_geometry(Eigen::VectorXd::Zero(14),tool,o.com);
    const auto parent=getppid();if(parent<=1||prctl(PR_SET_PDEATHSIG,SIGTERM)||getppid()!=parent)throw std::runtime_error("no_supervisor");
    OwnerLease lease(o.endpoint);Channel channel(o.endpoint);auto state=fresh(channel);check_start(state,model);auto q0=angles(state);
    auto second=fresh(channel);check_start(second,model);
    for(int j=0;j<14;++j)if(std::abs(arm_position(second,j)-q0[j])>.005)throw std::runtime_error("start_changed");
    state=second;q0=angles(state);
    CommandFrame hand{};
    if(!o.bare){std::ifstream f(o.background,std::ios::binary);std::vector<char> data{std::istreambuf_iterator<char>(f),{}};hand=decode_command(data.data(),data.size());if(hand.hand_count>2)throw std::runtime_error("invalid_hand_background");}
    std::filesystem::path root(o.output);std::filesystem::create_directories(root.parent_path());
    if(!std::filesystem::create_directory(root))throw std::runtime_error("output_exists");
    Recorder recorder(root);IO io;ClockMap clock;
    auto command=initial_command(state,hand,q0,0,0);Joints hold=q0;ArmMask all{};all.fill(true);
    std::optional<TricalSegment> active;TricalBlock block;
    std::int64_t next=mono(),phase_begin=next,last_receive=next,last_send=next,last_pair=next-1000000000LL,last_publish=next;
    std::int64_t segment_begin=0,first_write=0,first_command=0,max_gap=0,writes=0,skips=0;
    auto last_frame=state.frame_counter;int phase=0;bool phase_sent=false,ack=false,finish_requested=false;std::string failure;
    auto change=[&](int p,const std::string& event){phase=p;phase_begin=mono();phase_sent=false;recorder.event(p,event);};
    auto fail=[&](const std::string& reason){if(failure.empty())failure=reason;if(phase!=4){active.reset();change(4,reason);}};
    auto reply=[&](std::uint64_t id,bool ok,const std::string& why){io.emit("{\"type\":\"reply\",\"id\":"+std::to_string(id)+",\"ok\":"+(ok?"true":"false")+",\"reason\":"+quote(why)+"}");};
    recorder.event(0,"disabling");
    while(true){
      const auto now=mono();Record r;r.at=now;
      try{
        if(now-last_pair>=1000000000LL){r.clock=clock.pair();r.has_pair=true;last_pair=now;}
        r.received=channel.receive(state);r.receive_at=r.received?mono():0;
        if(r.received&&state.frame_counter!=last_frame){last_receive=r.receive_at;last_frame=state.frame_counter;if(phase==3)block.add(r.receive_at,angles(state));}
        if(interrupted)fail("operator_interrupt");else if(io.disconnected)fail("supervisor_disconnected");
        if(recorder.stopped)fail("recording_failed");
        if(phase!=4){
          if(now-last_receive>2000000000LL)fail("feedback_timeout");
          if(now-last_send>2000000000LL)fail("command_queue_timeout");
          for(int j=0;j<29;++j)if(state.joints[j].error_code)fail("drive_error_slot_"+std::to_string(j));
          for(int j=0;j<14;++j)if(std::abs(arm_feedback(state,j).velocity)>velocity[j])fail("observed_velocity_limit");
          if(phase!=3&&now-phase_begin>3000000000LL)fail("transition_timeout");
          if(phase==3&&!drive_ack(state,0x27,all,true))fail("enabled_mode_lost");
        }
        if(phase==4&&phase_sent&&r.received&&drive_ack(state,0x21,all)){ack=true;recorder.event(4,"disable_ack");}
        if(!ack&&phase_sent&&now-phase_begin>=20000000LL&&phase<3){
          auto p=drive_phase_next(phase,state,all);
          if(p==2){
            q0=angles(state);
            for(int j=0;j<14;++j)if(q0[j]<lower[j]||q0[j]>upper[j])throw std::runtime_error("start_outside_urdf_before_enable");
            hold=q0;
          }
          if(p!=phase){change(p,p==1?"switching_mit":p==2?"enabling_at_start":"hold_begin");
            if(p==3){block={};io.emit("{\"type\":\"ready\",\"mono_ns\":"+std::to_string(now)+",\"q0\":"+array(q0)+"}");}
          }
        }
        for(auto& req:io.take()){
          if(!req.error.empty()){reply(req.id,false,req.error);continue;}
          if(req.type=="finish"){
            finish_requested=true;active.reset();if(phase!=4)change(4,"finish");reply(req.id,true,"disabling");continue;
          }
          std::string error=phase!=3?"not_enabled":active?"segment_busy":req.segment.validate(hold,angles(state),lower,upper,velocity,o.windup);
          if(error.empty()){active=req.segment;segment_begin=now;block={};recorder.event(3,"segment_begin_"+std::to_string(req.id));}
          reply(req.id,error.empty(),error.empty()?"accepted":error);
        }
        Joints q=hold,dq{};
        if(active){auto values=active->at(now-segment_begin);q=values.first;dq=values.second;
          if(now-segment_begin>=active->duration_ns){hold=active->to;q=hold;dq={};auto id=active->id;active.reset();block={};
            recorder.event(3,"segment_done_"+std::to_string(id));io.emit("{\"type\":\"segment_done\",\"id\":"+std::to_string(id)+",\"mono_ns\":"+std::to_string(now)+"}");}
        }
        Eigen::Map<const Eigen::Matrix<double,14,1>> qref(q.data());Eigen::VectorXd effort=Eigen::VectorXd::Zero(14);
        if(phase==2||phase==3){effort=workspace.gravity(qref);if(o.mass>0)effort-=workspace.point_geometry(qref,tool,o.com).joints.transpose()*(o.mass*o.gravity);effort*=o.gain;
          if(!effort.allFinite()||effort.cwiseAbs().maxCoeff()>o.effort_limit){fail("feedforward_limit");effort.setZero();}}
        for(int j=0;j<14;++j){arm_target(command,j,q[j],phase==4?0:dq[j],o.kp[j],o.kd[j],effort[j]);auto& a=command.joints[j+12];if(phase>=1)a.mode=-1;a.control_word=(phase==2||phase==3)?15:6;}
        command.frame_counter++;command.timestamp=now/1000000;r.sent=channel.send(command);r.send_at=r.sent?mono():0;
        if(r.sent){if(first_write)max_gap=std::max(max_gap,r.send_at-last_send);last_send=r.send_at;phase_sent=true;++writes;if(!first_write)first_write=r.send_at;if(phase==3&&!first_command)first_command=r.send_at;}else ++skips;
        if(now-last_publish>=20000000LL&&block.count){
          io.emit("{\"type\":\"state\",\"mono_ns\":"+std::to_string(now)+",\"holding\":"+(!active?"true":"false")+",\"active_id\":"+std::to_string(active?active->id:0)+
            ",\"start_ns\":"+std::to_string(block.start_ns)+",\"end_ns\":"+std::to_string(block.end_ns)+",\"count\":"+std::to_string(block.count)+",\"q_mean\":"+array(block.mean())+",\"q_min\":"+array(block.low)+",\"q_max\":"+array(block.high)+"}");
          block={};last_publish=now;
        }
      }catch(const std::exception& e){fail(e.what());}
      r.phase=phase;r.cmd=command;r.state=state;r.segment=clock.segment();if(r.received)r.observation=clock.observation(state.timestamp);recorder.push(std::move(r));
      if(ack)break;
      if(phase==4&&mono()-phase_begin>3000000000LL){if(failure.empty())failure="disable_unconfirmed";break;}
      next+=1000000;if(next<mono())next=mono()+1000000;sleep_to(next);
    }
    recorder.finish();const bool complete=finish_requested&&failure.empty()&&ack&&!recorder.stopped;
    std::ofstream result(root/"result.txt");result<<"execution_completed="<<complete<<"\nreason="<<(failure.empty()?"completed":failure)<<"\ndisable_ack="<<ack<<"\nfirst_command_mono_ns="<<first_command<<"\nfirst_shm_write_mono_ns="<<first_write<<"\nmax_send_gap_ns="<<max_gap<<"\ncommands_sent="<<writes<<"\nsend_skipped="<<skips<<"\nrecording_stopped="<<recorder.stopped<<'\n';
    io.emit("{\"type\":\"finished\",\"ok\":"+std::string(complete?"true":"false")+",\"disable_ack\":"+(ack?"true":"false")+",\"reason\":"+quote(failure)+"}");return complete?0:1;
  }catch(const std::exception& e){std::cerr<<"shm_trical_executor: "<<e.what()<<'\n';return 1;}
}
}
