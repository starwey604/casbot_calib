#include "casbot_calib/simulator.hpp"
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py=pybind11;
using namespace casbot;
void bind_arm(py::module_& m);
void bind_arm_simulator(py::module_& m);
void bind_collision(py::module_& m);
template<class T> py::array_t<T> column(std::size_t n) { return py::array_t<T>(n); }
PYBIND11_MODULE(_core,m) {
  bind_arm(m);
  bind_arm_simulator(m);
  bind_collision(m);
  m.doc()="Platform-independent virtual joint; arrays use SI units and int64 nanoseconds";
  py::class_<SimulationConfig> config(m,"SimulationConfig"); config.def(py::init<>());
#define FIELD(x) config.def_readwrite(#x,&SimulationConfig::x)
  FIELD(inertia);FIELD(damping);FIELD(coulomb);FIELD(spring);FIELD(spring_zero);FIELD(friction_velocity);FIELD(gain);
  FIELD(gravity_sin);FIELD(gravity_cos);FIELD(current_tau);FIELD(current_filter_tau);FIELD(torque_limit);FIELD(current_limit);
  FIELD(initial_q);FIELD(initial_v);FIELD(initial_string_x);FIELD(initial_string_v);
  FIELD(position_noise);FIELD(velocity_noise);FIELD(current_noise);FIELD(position_quantum);FIELD(current_quantum);
  FIELD(temperature);FIELD(drop_probability);FIELD(repeat_probability);FIELD(integration_ns);FIELD(sample_period_ns);
  FIELD(command_delay_ns);FIELD(feedback_delay_ns);FIELD(jitter_ns);FIELD(current_lag_ns);FIELD(expose_sample_time);
  FIELD(double_inertia);FIELD(contact_enabled);FIELD(motor_inertia);FIELD(coupling_spring);FIELD(coupling_damping);
  FIELD(string_mass);FIELD(string_spring);FIELD(string_damping);FIELD(contact_spring);FIELD(contact_damping);
  FIELD(lever);FIELD(pick_offset);FIELD(depth);FIELD(attack_angle);FIELD(seed);
#undef FIELD
  py::class_<DeviceCapabilities>(m,"DeviceCapabilities").def(py::init<>())
    .def_readonly("independent_torque_sensor",&DeviceCapabilities::independent_torque_sensor)
    .def_readonly("motor_encoder_exposed",&DeviceCapabilities::motor_encoder_exposed)
    .def_readonly("inner_pi_tunable",&DeviceCapabilities::inner_pi_tunable);
  py::class_<SimulationRunner>(m,"SimulationRunner")
    .def(py::init<SimulationConfig>())
    .def_static("integration_backend",&SimulationRunner::integration_backend)
    .def("run",[](const SimulationRunner& runner,
      py::array_t<Nanoseconds,py::array::c_style|py::array::forcecast> times,
      py::array_t<double,py::array::c_style|py::array::forcecast> values, Nanoseconds duration_ns) {
      if(times.ndim()!=1 || values.ndim()!=2 || values.shape(0)!=times.shape(0) || values.shape(1)!=6)
        throw std::invalid_argument("Expected times[N], values[N,6]: mode,q_ref,dq_ref,torque_ff,kp,kd");
      auto ts=times.unchecked<1>();auto vs=values.unchecked<2>();
      std::vector<JointCommand> commands; commands.reserve(times.size());
      for(py::ssize_t n=0;n<times.size();++n) {
        if(vs(n,0)!=0 && vs(n,0)!=1) throw std::invalid_argument("Unknown command mode");
        commands.push_back({ts(n),static_cast<Mode>(static_cast<int>(vs(n,0))),vs(n,1),vs(n,2),vs(n,3),vs(n,4),vs(n,5)});
      }
      SimulationResult result;
      {py::gil_scoped_release release;result=runner.run(commands,duration_ns);}
      py::dict obs,truth;
#define OBS(name,type,field) {auto a=column<type>(result.observations.size());auto v=a.mutable_unchecked<1>();for(std::size_t i=0;i<result.observations.size();++i)v(i)=result.observations[i].field;obs[name]=a;}
      OBS("sequence",Nanoseconds,sequence);OBS("sample_ns",Nanoseconds,sample_ns);
      OBS("current_sample_ns",Nanoseconds,current_sample_ns);OBS("receive_ns",Nanoseconds,receive_ns);
      OBS("status",std::uint32_t,status);OBS("q",double,position);OBS("dq",double,velocity);
      OBS("current",double,current);OBS("estimated_torque",double,estimated_torque);OBS("temperature",double,temperature);
#undef OBS
#define TRUTH(name,type,field) {auto a=column<type>(result.truth.size());auto v=a.mutable_unchecked<1>();for(std::size_t i=0;i<result.truth.size();++i)v(i)=result.truth[i].field;truth[name]=a;}
      TRUTH("time_ns",Nanoseconds,time_ns);TRUTH("q",double,q);TRUTH("dq",double,v);
      TRUTH("ddq",double,acceleration);TRUTH("current",double,current);TRUTH("torque_request",double,torque_request);
      TRUTH("contact_force",double,contact_force);TRUTH("string_x",double,string_x);TRUTH("string_v",double,string_v);
      TRUTH("energy",double,energy);TRUTH("motor_q",double,motor_q);TRUTH("contact",bool,contact);
#undef TRUTH
      return py::make_tuple(obs,truth);
    },py::arg("times_ns"),py::arg("commands"),py::arg("duration_ns"));
}
