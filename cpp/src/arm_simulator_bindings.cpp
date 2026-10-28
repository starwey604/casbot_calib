#include "casbot_calib/arm_simulator.hpp"
#include "casbot_calib/arm_guard.hpp"
#include "casbot_calib/trical_mock.hpp"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py=pybind11;
using namespace casbot;
namespace {
using Times=py::array_t<Nanoseconds,py::array::c_style|py::array::forcecast>;
using Values=py::array_t<double,py::array::c_style|py::array::forcecast>;
std::vector<ArmCommand> read_commands(Times times,Values values,const Eigen::VectorXd& q0) {
  const py::ssize_t n=q0.size();
  if(times.ndim()!=1 || values.ndim()!=3 || values.shape(0)!=times.size() || values.shape(1)!=n || values.shape(2)!=5)
    throw std::invalid_argument("Expected times[N], commands[N,J,5]: q,dq,torque_ff,kp,kd");
  auto ts=times.unchecked<1>();auto input=values.unchecked<3>();std::vector<ArmCommand> commands;commands.reserve(times.size());
  for(py::ssize_t row=0;row<times.size();++row) {
    ArmCommand cmd;cmd.send_ns=ts(row);cmd.q.resize(n);cmd.dq.resize(n);cmd.torque.resize(n);cmd.kp.resize(n);cmd.kd.resize(n);
    for(py::ssize_t j=0;j<n;++j) {cmd.q[j]=input(row,j,0);cmd.dq[j]=input(row,j,1);cmd.torque[j]=input(row,j,2);cmd.kp[j]=input(row,j,3);cmd.kd[j]=input(row,j,4);}
    commands.push_back(std::move(cmd));
  }
  return commands;
}
py::tuple pack_result(const ArmSimulationResult& result,py::ssize_t n) {
  py::dict obs,truth;
  auto scalar=[&](const char* key,auto member) {py::array_t<Nanoseconds> array(result.observations.size());auto out=array.mutable_unchecked<1>();
    for(std::size_t i=0;i<result.observations.size();++i) out(i)=result.observations[i].*member;obs[key]=array;};
  scalar("sequence",&ArmSample::sequence);scalar("sample_ns",&ArmSample::sample_ns);scalar("receive_ns",&ArmSample::receive_ns);
  auto vector=[&](const char* key,auto member) {py::array_t<double> array({static_cast<py::ssize_t>(result.observations.size()),n});auto out=array.mutable_unchecked<2>();
    for(std::size_t i=0;i<result.observations.size();++i) for(py::ssize_t j=0;j<n;++j) out(i,j)=(result.observations[i].*member)[j];obs[key]=array;};
  vector("q",&ArmSample::q);vector("dq",&ArmSample::dq);vector("current",&ArmSample::current);
  py::array_t<int> status({static_cast<py::ssize_t>(result.observations.size()),n});auto flags=status.mutable_unchecked<2>();
  for(std::size_t i=0;i<result.observations.size();++i) for(py::ssize_t j=0;j<n;++j) flags(i,j)=result.observations[i].status[j];obs["status"]=status;
  py::array_t<Nanoseconds> time(result.truth.size());auto time_out=time.mutable_unchecked<1>();
  for(std::size_t i=0;i<result.truth.size();++i) time_out(i)=result.truth[i].sample_ns;truth["sample_ns"]=time;
  auto true_vector=[&](const char* key,auto member) {py::array_t<double> array({static_cast<py::ssize_t>(result.truth.size()),n});auto out=array.mutable_unchecked<2>();
    for(std::size_t i=0;i<result.truth.size();++i) for(py::ssize_t j=0;j<n;++j) out(i,j)=(result.truth[i].*member)[j];truth[key]=array;};
  true_vector("q",&ArmTruth::q);true_vector("dq",&ArmTruth::dq);true_vector("ddq",&ArmTruth::ddq);
  true_vector("current",&ArmTruth::current);true_vector("torque_request",&ArmTruth::request);
  return py::make_tuple(obs,truth);
}
}
void bind_arm_simulator(py::module_& m) {
  m.attr("arm_stiction_backend")="coupled_active_set_mass_projection_v2";
  auto config=py::class_<ArmSimulationConfig>(m,"ArmSimulationConfig").def(py::init<>());
#define FIELD(x) config.def_readwrite(#x,&ArmSimulationConfig::x)
  FIELD(integration_ns);FIELD(sample_period_ns);FIELD(command_delay_ns);FIELD(feedback_delay_ns);FIELD(current_tau);
  FIELD(torque_limit);FIELD(position_noise);FIELD(current_noise);FIELD(seed);FIELD(gain);FIELD(drop_probability);FIELD(repeat_probability);
  FIELD(stiction_enabled);FIELD(stick_velocity_tolerance);
#undef FIELD
  auto sensor=py::class_<TricalMockConfig>(m,"TricalMockConfig").def(py::init<>());
#define FIELD(x) sensor.def_readwrite(#x,&TricalMockConfig::x)
  FIELD(calibration);FIELD(minimum);FIELD(maximum);FIELD(bias);FIELD(encoder_offset);FIELD(face_radius);FIELD(noise_std);FIELD(quantum);
  FIELD(drop_probability);FIELD(period_ns);FIELD(delay_ns);FIELD(jitter_ns);FIELD(phase_ns);FIELD(expose_sample_time);FIELD(seed);
#undef FIELD
  py::class_<TricalMockSession>(m,"TricalMockSession")
    .def(py::init<ArmDynamics,ArmSimulationConfig,TricalMockConfig,const Eigen::VectorXd&,const Eigen::VectorXd&,bool>(),
      py::arg("plant"),py::arg("arm"),py::arg("sensor"),py::arg("q0"),py::arg("v0"),py::arg("retain_truth")=false)
    .def_property_readonly("time_ns",&TricalMockSession::time_ns)
    .def("select_target",&TricalMockSession::select_target)
    .def("set_fault",&TricalMockSession::set_fault)
    .def("submit",[](TricalMockSession& session,Nanoseconds time,const Eigen::MatrixXd& values){
      if(values.cols()!=5)throw std::invalid_argument("Expected command[J,5]");
      ArmCommand c;c.send_ns=time;c.q=values.col(0);c.dq=values.col(1);c.torque=values.col(2);c.kp=values.col(3);c.kd=values.col(4);
      session.submit(c);
    })
    .def("advance_to",[](TricalMockSession& session,Nanoseconds end){
      TricalMockResult result;
      {py::gil_scoped_release release;result=session.advance_to(end);}
      const py::ssize_t n=session.joint_count();
      auto state=pack_result(result.simulation,n);py::list packets;
      for(const auto& p:result.indicators){py::dict row;
        row["channel"]=p.channel;row["status"]=p.status;row["sequence"]=p.sequence;row["sample_ns"]=p.sample_ns;
        row["receive_ns"]=p.receive_ns;row["sample_known"]=p.sample_known;row["reading_m"]=p.reading;
        row["target_id"]=p.target_id;row["tool_frame"]=p.tool_frame;packets.append(row);
      }
      return py::make_tuple(state[0],packets,state[1]);
    });
  auto guard=py::class_<ArmGuardConfig>(m,"ArmGuardConfig").def(py::init<>());
#define FIELD(x) guard.def_readwrite(#x,&ArmGuardConfig::x)
  FIELD(joint);FIELD(begin_ns);FIELD(end_ns);FIELD(sustain_ns);FIELD(max_sample_age_ns);
  FIELD(displacement_threshold);FIELD(speed_threshold);FIELD(maximum_displacement);FIELD(maximum_speed);
  FIELD(restore_kp);FIELD(restore_kd);FIELD(baseline_torque);
#undef FIELD
  py::class_<ArmSimulationRunner>(m,"ArmSimulationRunner").def(py::init<ArmDynamics,ArmSimulationConfig>())
    .def("run",[](const ArmSimulationRunner& runner,Times times,Values values,Nanoseconds duration,
      const Eigen::VectorXd& q0,const Eigen::VectorXd& v0) {
      const auto commands=read_commands(times,values,q0);
      const Eigen::VectorXd position=q0,velocity=v0;ArmSimulationResult result;
      {py::gil_scoped_release release;result=runner.run(commands,duration,position,velocity);}
      return pack_result(result,q0.size());
    });
  m.def("run_guarded_arm",[](const ArmDynamics& plant,const ArmSimulationConfig& config,Times times,Values values,
      Nanoseconds duration,const Eigen::VectorXd& q0,const Eigen::VectorXd& v0,const ArmGuardConfig& guard) {
    const auto commands=read_commands(times,values,q0);const Eigen::VectorXd position=q0,velocity=v0;ArmGuardedResult result;
    {py::gil_scoped_release release;result=run_guarded_arm(plant,config,commands,duration,position,velocity,guard);}
    py::tuple data=pack_result(result.simulation,q0.size());py::list events;
    for(const auto& event:result.events) {
      py::dict row;row["reason"]=event.reason;row["first_sample_ns"]=event.first_sample_ns;row["confirmed_sample_ns"]=event.confirmed_sample_ns;
      row["decision_ns"]=event.decision_ns;row["initial_q"]=event.initial_q;row["fault"]=event.fault;events.append(row);
    }
    py::array_t<double> executed({static_cast<py::ssize_t>(result.executed_commands.size()),static_cast<py::ssize_t>(q0.size()),py::ssize_t(5)});
    auto output=executed.mutable_unchecked<3>();
    for(std::size_t row=0;row<result.executed_commands.size();++row) {
      const auto& c=result.executed_commands[row];
      for(py::ssize_t j=0;j<q0.size();++j) {output(row,j,0)=c.q[j];output(row,j,1)=c.dq[j];output(row,j,2)=c.torque[j];output(row,j,3)=c.kp[j];output(row,j,4)=c.kd[j];}
    }
    return py::make_tuple(data[0],data[1],executed,events);
  });
}
