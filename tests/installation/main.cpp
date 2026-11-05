#include <casbot_calib/arm_dynamics.hpp>
#include <casbot_calib/integration.hpp>
#include <casbot_calib/trical.hpp>
#include <casbot_calib/trical_mock.hpp>
#include <iostream>
#include <filesystem>
int main() {
#ifdef CASBOT_CALIB_MODEL_DIRECTORY
  const auto sources=std::filesystem::path(CASBOT_CALIB_MODEL_DIRECTORY)/"casbot_hl12p1_dvt2"/"source";
  const std::vector<std::pair<std::string,std::size_t>> variants{{"a2",51},{"a2_ff",29},{"b1",40},{"b2",51},{"b3",29}};
  for(const auto& variant:variants) {
    const auto model=casbot::ArmDynamics::from_urdf((sources/variant.first/"robot_description_formatted.urdf").string());
    if(model.model().joints.size()!=variant.second)return 5;
  }
  if(!std::filesystem::is_regular_file(sources/"assets/collision/CASBOTHL12/base_link.obj"))return 6;
  std::cout<<"Installed DVT2 sources: five variants loaded from "<<sources<<'\n';
#endif
#ifdef CASBOT_CALIB_MODEL_DIRECTORY
  const auto candidate=std::filesystem::path(CASBOT_CALIB_MODEL_DIRECTORY)/"casbot_hl12p1_dvt2"/"derived"/"a2ff_stock_hands_candidate_v1";
  const auto reduced=casbot::ArmDynamics::from_urdf((candidate/"model.urdf").string());
  if(reduced.model().joints.size()!=14 || reduced.model().model_id!="casbot_hl12p1_dvt2_a2ff_stock_hands_candidate_v1")return 7;
  if(!std::filesystem::is_regular_file(candidate/"scene.xml") || !std::filesystem::is_regular_file(candidate/"collision.npz"))return 8;
  std::cout<<"Installed DVT2 candidate: 14 axes loaded from "<<candidate<<'\n';
#endif
  casbot::TricalCalibration calibration;casbot::ArmPose sensor;
  const Eigen::Vector3d displacement(.001,-.002,.003);
  if((casbot::trical_readings(sensor,displacement,calibration)-displacement).norm()>1e-14) return 1;
  casbot::ArmModel model;model.model_id="installed_frame_consumer";
  casbot::ArmJoint joint;joint.name="joint";joint.link_name="link";model.joints.push_back(joint);
  casbot::ArmFrame frame;frame.name="tcp";frame.parent=0;frame.placement.translation=displacement;model.frames.push_back(frame);
  const casbot::ArmDynamics engine(model);
  if((engine.frame_pose(Eigen::VectorXd::Zero(1),"tcp").topRightCorner<3,1>()-displacement).norm()>1e-14) return 2;
  if((engine.point_geometry(Eigen::VectorXd::Zero(1),"tcp",Eigen::Vector3d::Zero()).position-displacement).norm()>1e-14)return 4;
  casbot::ArmSimulationConfig drive;drive.gain=Eigen::VectorXd::Ones(1);
  casbot::TricalMockSession mock(engine,drive,casbot::TricalMockConfig{},Eigen::VectorXd::Zero(1),Eigen::VectorXd::Zero(1));
  mock.select_target("tcp","ball",displacement);
  const auto readout=mock.advance_to(10'000'000);
  if(readout.indicators.size()!=3 || !readout.simulation.truth.empty())return 3;
  std::cout << casbot::ArmDynamics::backend_version() << '\n'
            << casbot::integration_backend() << '\n';
  return 0;
}
