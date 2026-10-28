#include "casbot_calib/arm_dynamics.hpp"
#include "casbot_calib/trical.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace casbot;
namespace { void check(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);} }
int main() {
  ArmModel model;model.model_id="analytic_2r_frames";
  for(int j=0;j<2;++j) {
    ArmJoint joint;joint.name="j"+std::to_string(j);joint.link_name="link"+std::to_string(j);
    joint.origin.translation={j==0?0.:.7,0.,0.};model.joints.push_back(joint);
  }
  ArmFrame tcp;tcp.name="tcp";tcp.parent=1;tcp.placement.translation={.45,0.,0.};
  ArmFrame root;root.name="root";model.frames={tcp,root};ArmDynamics engine(model);
  const Eigen::Vector2d q(.4,-.7);const auto p=engine.frame_pose(q,"tcp");
  Eigen::Vector3d expected(.7*std::cos(q[0])+.45*std::cos(q.sum()),.7*std::sin(q[0])+.45*std::sin(q.sum()),0.);
  check((p.topRightCorner<3,1>()-expected).norm()<1e-13,"analytic planar TCP position");
  Eigen::Matrix<double,6,2> jac=Eigen::Matrix<double,6,2>::Zero();
  jac(0,0)=-expected[1];jac(1,0)=expected[0];jac(0,1)=-.45*std::sin(q.sum());jac(1,1)=.45*std::cos(q.sum());jac.row(5).setOnes();
  check((engine.frame_jacobian(q,"tcp")-jac).norm()<1e-13,"analytic named frame Jacobian");
  check(engine.frame_jacobian(q,"root").norm()==0.,"root frame has zero joint Jacobian");
  auto runtime=engine.workspace();runtime.mass_matrix(q);runtime.frame_pose(-q,"root");
  check((runtime.frame_jacobian(q,"tcp")-jac).norm()<1e-13,"interleaved frame Data");
  model.base.translation={.4,-.2,.8};model.base.rotation=Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  ArmDynamics placed(model);
  check((placed.frame_pose(q,"root")-model.base.matrix()).norm()<1e-13,"root frame follows model base once");
  TricalCalibration cal;cal.zero={.001,-.002,.003};ArmPose sensor;
  sensor.translation={1.,2.,3.};sensor.rotation=Eigen::AngleAxisd(1.5707963267948966,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d ball(1.002,2.001,2.997),offset(.001,-.002,-.003);
  const auto reading=trical_readings(sensor,ball,cal);
  check((reading-cal.zero-offset).norm()<1e-14,"independent rotated flat-tip geometry");
  check((trical_offset(reading,cal)-offset).norm()<1e-14,"mastered offsets");
  bool rejected=false;try {engine.frame_pose(q,"missing");}catch(const std::invalid_argument&){rejected=true;}
  check(rejected,"unknown frames fail closed");
  std::cout<<"K1 analytic named frames, Jacobian and TriCal geometry passed\n";
}
