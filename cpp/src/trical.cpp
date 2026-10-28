#include "casbot_calib/trical.hpp"
#include <Eigen/LU>
#include <cmath>
#include <stdexcept>

namespace casbot {
namespace {
void validate(const TricalCalibration& c) {
  if(!c.axes.allFinite() || !c.zero.allFinite() || std::abs(c.axes.determinant())<.1)
    throw std::invalid_argument("Invalid or degenerate TriCal axes/zero");
  for(int row=0;row<3;++row)
    if(std::abs(c.axes.row(row).norm()-1.)>1e-10)
      throw std::invalid_argument("TriCal axes must have unit row normals");
}
}
Eigen::Vector3d trical_readings(const ArmPose& sensor_world,const Eigen::Vector3d& ball_world,const TricalCalibration& c) {
  validate(c);const auto& r=sensor_world.rotation;
  if(!ball_world.allFinite() || !sensor_world.translation.allFinite() || !r.allFinite() ||
     (r.transpose()*r-Eigen::Matrix3d::Identity()).norm()>1e-10 || std::abs(r.determinant()-1.)>1e-10)
    throw std::invalid_argument("Invalid TriCal world geometry");
  return c.axes*(r.transpose()*(ball_world-sensor_world.translation))+c.zero;
}
Eigen::Vector3d trical_offset(const Eigen::Vector3d& readings,const TricalCalibration& c) {
  validate(c);
  if(!readings.allFinite())throw std::invalid_argument("TriCal readings must be finite");
  return c.axes.fullPivLu().solve(readings-c.zero);
}
}  // namespace casbot
