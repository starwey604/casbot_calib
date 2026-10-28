#include "casbot_calib/collision.hpp"
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
namespace py=pybind11;
using namespace casbot;
void bind_collision(py::module_& m) {
  py::class_<CollisionGeometry>(m,"CollisionGeometry").def(py::init<>())
    .def_readwrite("name",&CollisionGeometry::name).def_readwrite("parent",&CollisionGeometry::parent)
    .def_readwrite("attached_to_base",&CollisionGeometry::attached_to_base).def_readwrite("placement",&CollisionGeometry::placement)
    .def_readwrite("vertices",&CollisionGeometry::vertices).def_readwrite("triangles",&CollisionGeometry::triangles);
  py::class_<CollisionChecker>(m,"CollisionChecker").def(py::init<ArmDynamics,const std::vector<CollisionGeometry>&,const std::vector<std::pair<int,int>>&>())
    .def_static("backend_version",&CollisionChecker::backend_version).def("motion_radii",&CollisionChecker::motion_radii)
    .def_property_readonly("pairs",&CollisionChecker::pairs)
    .def("check_batch",[](CollisionChecker& checker,const Eigen::MatrixXd& q) {
      const Eigen::MatrixXd owned=q;CollisionBatch output;
      {py::gil_scoped_release release;output=checker.check_batch(owned);}
      py::dict result;result["distances"]=output.distances;result["worst_pair"]=output.worst_pair;
      result["first_point"]=output.first_point;result["second_point"]=output.second_point;return result;
    });
}
