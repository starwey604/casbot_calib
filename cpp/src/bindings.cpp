#include <pybind11/pybind11.h>
namespace py=pybind11;
void bind_arm(py::module_& m);
void bind_collision(py::module_& m);
void bind_csv(py::module_& m);
PYBIND11_MODULE(_core,m) {
  m.doc()="CASBOT dynamics, collision and TriCal geometry";
  bind_arm(m);
  bind_collision(m);
  bind_csv(m);
}
