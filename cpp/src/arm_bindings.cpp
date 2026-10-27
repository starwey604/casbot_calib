#include "casbot_calib/arm_dynamics.hpp"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py=pybind11;
using namespace casbot;
namespace {
// A workspace retains the GIL in Python; the immutable engine may release it.
template<class T,class... Guards>
void bind_computations(py::class_<T>& cls,const Guards&... guards) {
  cls.def("mass_matrix",&T::mass_matrix,guards...)
    .def("gravity",&T::gravity,guards...)
    .def("velocity_bias",&T::velocity_bias,guards...)
    .def("friction",&T::friction,guards...)
    .def("rotor_torque",&T::rotor_torque,guards...)
    .def("inverse",&T::inverse,guards...)
    .def("forward",&T::forward,guards...)
    .def("components",&T::components,guards...)
    .def("fk",&T::fk,guards...)
    .def("energy",&T::energy,guards...)
    .def("torque_regressor",&T::torque_regressor,guards...)
    .def("inverse_derivatives",&T::inverse_derivatives,guards...)
    .def("forward_derivatives",&T::forward_derivatives,guards...)
    .def("tool_jacobian",&T::tool_jacobian,guards...);
}
using BatchArray=py::array_t<double,py::array::c_style|py::array::forcecast>;
using RowMatrix=Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>;
struct BatchSnapshot {
  RowMatrix q,dq,ddq;
  BatchSnapshot(const ArmDynamics& engine,const BatchArray& positions,const BatchArray& velocities,const BatchArray& accelerations) {
    const auto n=static_cast<py::ssize_t>(engine.model().joints.size());
    if(positions.ndim()!=2 || velocities.ndim()!=2 || accelerations.ndim()!=2 || positions.shape(1)!=n ||
       velocities.shape(0)!=positions.shape(0) || accelerations.shape(0)!=positions.shape(0) ||
       velocities.shape(1)!=n || accelerations.shape(1)!=n)
      throw std::invalid_argument("Expected q,dq,ddq[N,J] in model joint order");
    const auto count=positions.shape(0);
    // Own and validate the whole batch before releasing the GIL.
    q=Eigen::Map<const RowMatrix>(positions.data(),count,n);
    dq=Eigen::Map<const RowMatrix>(velocities.data(),count,n);
    ddq=Eigen::Map<const RowMatrix>(accelerations.data(),count,n);
    if(!q.allFinite() || !dq.allFinite() || !ddq.allFinite())
      throw std::invalid_argument("Batch states must be finite");
  }
};
}
void bind_arm(py::module_& m) {
  py::class_<ArmPose>(m,"ArmPose").def(py::init<>())
    .def_readwrite("rotation",&ArmPose::rotation).def_readwrite("translation",&ArmPose::translation)
    .def("matrix",&ArmPose::matrix);
  py::class_<ArmBody>(m,"ArmBody").def(py::init<>())
    .def_readwrite("mass",&ArmBody::mass).def_readwrite("com",&ArmBody::com).def_readwrite("inertia_com",&ArmBody::inertia_com);
  py::class_<ArmFriction>(m,"ArmFriction").def(py::init<>())
    .def_readwrite("viscous",&ArmFriction::viscous).def_readwrite("coulomb",&ArmFriction::coulomb)
    .def_readwrite("static_friction",&ArmFriction::static_friction)
    .def_readwrite("stribeck_velocity",&ArmFriction::stribeck_velocity)
    .def_readwrite("smoothing_velocity",&ArmFriction::smoothing_velocity);
  py::class_<ArmJoint>(m,"ArmJoint").def(py::init<>())
    .def_readwrite("name",&ArmJoint::name).def_readwrite("link_name",&ArmJoint::link_name)
    .def_readwrite("origin",&ArmJoint::origin).def_readwrite("axis",&ArmJoint::axis)
    .def_readwrite("body",&ArmJoint::body).def_readwrite("friction",&ArmJoint::friction)
    .def_readwrite("rotor_inertia",&ArmJoint::rotor_inertia)
    .def_readwrite("lower",&ArmJoint::lower).def_readwrite("upper",&ArmJoint::upper)
    .def_readwrite("velocity_limit",&ArmJoint::velocity_limit).def_readwrite("effort_limit",&ArmJoint::effort_limit);
  py::class_<ArmModel>(m,"ArmModel").def(py::init<>())
    .def_readwrite("model_id",&ArmModel::model_id).def_readwrite("model_version",&ArmModel::model_version)
    .def_readwrite("base",&ArmModel::base).def_readwrite("tool",&ArmModel::tool)
    .def_readwrite("gravity_world",&ArmModel::gravity_world).def_readwrite("joints",&ArmModel::joints)
    .def_readwrite("payload_enabled",&ArmModel::payload_enabled).def_readwrite("payload",&ArmModel::payload)
    .def_readwrite("parents",&ArmModel::parents);
  py::class_<ArmTerms>(m,"ArmTerms")
    .def_readonly("mass_matrix",&ArmTerms::mass_matrix).def_readonly("inertia",&ArmTerms::inertia)
    .def_readonly("velocity",&ArmTerms::velocity).def_readonly("gravity",&ArmTerms::gravity)
    .def_readonly("friction",&ArmTerms::friction).def_readonly("total",&ArmTerms::total);
  py::class_<ArmEnergy>(m,"ArmEnergy").def_readonly("kinetic",&ArmEnergy::kinetic).def_readonly("potential",&ArmEnergy::potential);
  py::class_<ArmInverseDerivatives>(m,"ArmInverseDerivatives")
    .def_readonly("dtau_dq",&ArmInverseDerivatives::dtau_dq).def_readonly("dtau_dv",&ArmInverseDerivatives::dtau_dv)
    .def_readonly("dtau_da",&ArmInverseDerivatives::dtau_da);
  py::class_<ArmForwardDerivatives>(m,"ArmForwardDerivatives")
    .def_readonly("ddq_dq",&ArmForwardDerivatives::ddq_dq).def_readonly("ddq_dv",&ArmForwardDerivatives::ddq_dv)
    .def_readonly("ddq_dtau",&ArmForwardDerivatives::ddq_dtau);
  auto workspace=py::class_<ArmWorkspace>(m,"ArmWorkspace");
  bind_computations(workspace);
  auto engine_class=py::class_<ArmDynamics>(m,"ArmDynamics");
  bind_computations(engine_class,py::call_guard<py::gil_scoped_release>());
  engine_class.def(py::init<ArmModel>())
    .def_static("from_urdf",&ArmDynamics::from_urdf)
    .def_static("backend_version",&ArmDynamics::backend_version)
    .def_property_readonly("model",&ArmDynamics::model)
    .def("workspace",&ArmDynamics::workspace)
    .def("rigid_parameters",&ArmDynamics::rigid_parameters)
    .def("rigid_parameter_names",&ArmDynamics::rigid_parameter_names)
    .def("evaluate_batch",[](const ArmDynamics& engine,
       py::array_t<double,py::array::c_style|py::array::forcecast> q,
       py::array_t<double,py::array::c_style|py::array::forcecast> dq,
       py::array_t<double,py::array::c_style|py::array::forcecast> ddq) {
      const BatchSnapshot input(engine,q,dq,ddq);
      const py::ssize_t n=input.q.cols();
      const py::ssize_t count=q.shape(0);
      py::array_t<double> masses({count,n,n}),inertia({count,n}),velocity({count,n}),gravity({count,n}),friction({count,n}),total({count,n});
      auto mm=masses.mutable_unchecked<3>();auto ii=inertia.mutable_unchecked<2>();auto vv=velocity.mutable_unchecked<2>();
      auto gg=gravity.mutable_unchecked<2>();auto ff=friction.mutable_unchecked<2>();auto tt=total.mutable_unchecked<2>();
      {
        py::gil_scoped_release release;
        auto runtime=engine.workspace();
        for(py::ssize_t row=0;row<count;++row) {
          const ArmTerms terms=runtime.components(input.q.row(row).transpose(),input.dq.row(row).transpose(),input.ddq.row(row).transpose());
          for(py::ssize_t j=0;j<n;++j) {
            for(py::ssize_t k=0;k<n;++k) mm(row,j,k)=terms.mass_matrix(j,k);
            ii(row,j)=terms.inertia[j];vv(row,j)=terms.velocity[j];gg(row,j)=terms.gravity[j];ff(row,j)=terms.friction[j];tt(row,j)=terms.total[j];
          }
        }
      }
      py::dict result;result["mass_matrix"]=masses;result["inertia"]=inertia;result["velocity"]=velocity;
      result["gravity"]=gravity;result["friction"]=friction;result["total"]=total;return result;
    })
    .def("regressor_batch",[](const ArmDynamics& engine,BatchArray q,BatchArray dq,BatchArray ddq) {
      const BatchSnapshot input(engine,q,dq,ddq);
      const py::ssize_t count=input.q.rows(),n=input.q.cols(),p=10*n;
      py::array_t<double> regressor({count,n,p}),rotor({count,n}),friction({count,n});
      auto yy=regressor.mutable_unchecked<3>();auto rr=rotor.mutable_unchecked<2>();auto ff=friction.mutable_unchecked<2>();
      {
        py::gil_scoped_release release;
        auto runtime=engine.workspace();
        for(py::ssize_t row=0;row<count;++row) {
          const Eigen::VectorXd position=input.q.row(row).transpose(),velocity=input.dq.row(row).transpose(),acceleration=input.ddq.row(row).transpose();
          const auto y=runtime.torque_regressor(position,velocity,acceleration);
          const auto r=runtime.rotor_torque(acceleration);const auto f=runtime.friction(velocity);
          for(py::ssize_t j=0;j<n;++j) {
            for(py::ssize_t k=0;k<p;++k) yy(row,j,k)=y(j,k);
            rr(row,j)=r[j];ff(row,j)=f[j];
          }
        }
      }
      // Model parameter values are deliberately not attached to the design matrix.
      py::dict result;result["regressor"]=regressor;result["rotor_torque"]=rotor;result["friction"]=friction;
      result["parameter_names"]=engine.rigid_parameter_names();return result;
    });
}
