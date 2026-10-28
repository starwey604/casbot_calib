#include <casbot_calib/arm_dynamics.hpp>
#include <casbot_calib/integration.hpp>
#include <casbot_calib/trical.hpp>
#include <iostream>
int main() {
  casbot::TricalCalibration calibration;casbot::ArmPose sensor;
  const Eigen::Vector3d displacement(.001,-.002,.003);
  if((casbot::trical_readings(sensor,displacement,calibration)-displacement).norm()>1e-14) return 1;
  casbot::ArmModel model;model.model_id="installed_frame_consumer";
  casbot::ArmJoint joint;joint.name="joint";joint.link_name="link";model.joints.push_back(joint);
  casbot::ArmFrame frame;frame.name="tcp";frame.parent=0;frame.placement.translation=displacement;model.frames.push_back(frame);
  const casbot::ArmDynamics engine(model);
  if((engine.frame_pose(Eigen::VectorXd::Zero(1),"tcp").topRightCorner<3,1>()-displacement).norm()>1e-14) return 2;
  std::cout << casbot::ArmDynamics::backend_version() << '\n'
            << casbot::integration_backend() << '\n';
  return 0;
}
