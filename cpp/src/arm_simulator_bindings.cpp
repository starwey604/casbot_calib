#include "casbot_calib/arm_simulator.hpp"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
namespace py=pybind11;
using namespace casbot;
void bind_arm_simulator(py::module_& m) {
  auto config=py::class_<ArmSimulationConfig>(m,"ArmSimulationConfig").def(py::init<>());
#define FIELD(x) config.def_readwrite(#x,&ArmSimulationConfig::x)
  FIELD(integration_ns);FIELD(sample_period_ns);FIELD(command_delay_ns);FIELD(feedback_delay_ns);FIELD(current_tau);
  FIELD(torque_limit);FIELD(position_noise);FIELD(current_noise);FIELD(seed);FIELD(gain);FIELD(drop_probability);FIELD(repeat_probability);
#undef FIELD
  py::class_<ArmSimulationRunner>(m,"ArmSimulationRunner").def(py::init<ArmDynamics,ArmSimulationConfig>())
    .def("run",[](const ArmSimulationRunner& runner,
      py::array_t<Nanoseconds,py::array::c_style|py::array::forcecast> times,
      py::array_t<double,py::array::c_style|py::array::forcecast> values,Nanoseconds duration,
      const Eigen::VectorXd& q0,const Eigen::VectorXd& v0) {
      const py::ssize_t n=q0.size();
      if(times.ndim()!=1 || values.ndim()!=3 || values.shape(0)!=times.size() || values.shape(1)!=n || values.shape(2)!=5)
        throw std::invalid_argument("Expected times[N], commands[N,J,5]: q,dq,torque_ff,kp,kd");
      auto ts=times.unchecked<1>();auto input=values.unchecked<3>();std::vector<ArmCommand> commands;commands.reserve(times.size());
      for(py::ssize_t row=0;row<times.size();++row) {
        ArmCommand cmd;cmd.send_ns=ts(row);cmd.q.resize(n);cmd.dq.resize(n);cmd.torque.resize(n);cmd.kp.resize(n);cmd.kd.resize(n);
        for(py::ssize_t j=0;j<n;++j) {cmd.q[j]=input(row,j,0);cmd.dq[j]=input(row,j,1);cmd.torque[j]=input(row,j,2);cmd.kp[j]=input(row,j,3);cmd.kd[j]=input(row,j,4);}
        commands.push_back(std::move(cmd));
      }
      // Own every input before releasing the GIL.
      const Eigen::VectorXd position=q0,velocity=v0;ArmSimulationResult result;
      {py::gil_scoped_release release;result=runner.run(commands,duration,position,velocity);}
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
    });
}
