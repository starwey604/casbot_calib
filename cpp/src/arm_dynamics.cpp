// Rigid-body dynamics and kinematics are delegated exclusively to Pinocchio.
#include <pinocchio/fwd.hpp>
#include "casbot_calib/arm_dynamics.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/energy.hpp>
#include <pinocchio/algorithm/regressor.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/config.hpp>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

namespace casbot {
namespace {
void require(bool condition,const std::string& message) {
  if(!condition) throw std::invalid_argument(message);
}
void validate_pose(const ArmPose& pose) {
  require(pose.rotation.allFinite() && pose.translation.allFinite(),"Non-finite arm pose");
  require((pose.rotation.transpose()*pose.rotation-Eigen::Matrix3d::Identity()).norm()<1e-10 &&
          std::abs(pose.rotation.determinant()-1.)<1e-10,"Arm pose requires a proper rotation");
}
void validate_body(const ArmBody& body) {
  require(std::isfinite(body.mass) && body.mass>0. && body.com.allFinite() && body.inertia_com.allFinite(),
          "Arm mass must be positive finite; COM/inertia must be finite");
  require((body.inertia_com-body.inertia_com.transpose()).norm()<1e-12,"Arm COM inertia must be symmetric");
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(body.inertia_com);
  require(eig.info()==Eigen::Success && eig.eigenvalues().minCoeff()>0.,"Arm COM inertia must be positive definite");
  const auto values=eig.eigenvalues();
  require(values[2]<=values[0]+values[1]+1e-12,"Arm inertia violates principal-moment triangle inequality");
}

pinocchio::SE3 to_pin(const ArmPose& pose) { return {pose.rotation,pose.translation}; }
pinocchio::Inertia to_pin(const ArmBody& body) { return {body.mass,body.com,body.inertia_com}; }
Eigen::VectorXd friction_slope(const ArmModel& model,const Eigen::VectorXd& dq) {
  Eigen::VectorXd slope(dq.size());
  for(Eigen::Index j=0;j<dq.size();++j) {
    const auto& f=model.joints[j].friction;const double v=dq[j];
    const double excess=(f.static_friction-f.coulomb)*std::exp(-std::pow(v/f.stribeck_velocity,2));
    const double smooth=std::tanh(v/f.smoothing_velocity);
    slope[j]=f.viscous-2.*v/(f.stribeck_velocity*f.stribeck_velocity)*excess*smooth+
      (f.coulomb+excess)*(1.-smooth*smooth)/f.smoothing_velocity;
  }
  return slope;
}
}  // namespace

struct ArmDynamics::Backend {
  pinocchio::Model model;
  explicit Backend(const ArmModel& source) {
    model.name=source.model_id;model.gravity.linear()=source.gravity_world;
    pinocchio::JointIndex parent=0;
    for(std::size_t j=0;j<source.joints.size();++j) {
      const auto& joint=source.joints[j];
      const int parent_index=source.parents.empty()?static_cast<int>(j)-1:source.parents[j];
      parent=parent_index<0?0:static_cast<pinocchio::JointIndex>(parent_index+1);
      const auto placement=parent==0 ? to_pin(source.base)*to_pin(joint.origin) : to_pin(joint.origin);
      parent=model.addJoint(parent,pinocchio::JointModelRevoluteUnaligned(joint.axis),placement,joint.name);
      model.appendBodyToJoint(parent,to_pin(joint.body));
      model.armature[static_cast<Eigen::Index>(j)]=joint.rotor_inertia;
      model.lowerPositionLimit[static_cast<Eigen::Index>(j)]=joint.lower;
      model.upperPositionLimit[static_cast<Eigen::Index>(j)]=joint.upper;
      model.velocityLimit[static_cast<Eigen::Index>(j)]=joint.velocity_limit;
      model.effortLimit[static_cast<Eigen::Index>(j)]=joint.effort_limit;
    }
    // Pinocchio performs the rigid transform, parallel-axis shift and merge.
    if(source.payload_enabled) model.appendBodyToJoint(parent,to_pin(source.payload),to_pin(source.tool));
  }
};

Eigen::Matrix4d ArmPose::matrix() const {
  return to_pin(*this).toHomogeneousMatrix();
}
std::string ArmDynamics::backend_version() { return std::string("pinocchio-")+PINOCCHIO_VERSION; }
ArmDynamics::ArmDynamics(ArmModel model):model_(std::make_shared<const ArmModel>(std::move(model))) {
  require(!model_->model_id.empty() && (model_->model_version=="serial_rigid_v1" || model_->model_version=="tree_rigid_v1"),"Unsupported or missing arm model identity/version");
  require(!model_->joints.empty() && model_->joints.size()<=64,"Arm requires 1..64 scalar revolute joints");
  if(model_->model_version=="serial_rigid_v1") require(model_->parents.empty(),"Serial model cannot declare tree parents");
  else {
    require(model_->parents.size()==model_->joints.size(),"Tree parents must match joints");
    for(std::size_t j=0;j<model_->parents.size();++j)
      require(model_->parents[j]>=-1 && model_->parents[j]<static_cast<int>(j),"Tree parent must be root or an earlier joint");
    require(!model_->payload_enabled,"Tree URDF inertias already include fixed payloads");
  }
  validate_pose(model_->base);validate_pose(model_->tool);
  require(model_->gravity_world.allFinite(),"Non-finite gravity vector");
  std::set<std::string> names,links;
  for(const auto& j:model_->joints) {
    require(!j.name.empty() && names.insert(j.name).second && !j.link_name.empty() && links.insert(j.link_name).second,
            "Arm joint/link names must be nonempty and unique");
    validate_pose(j.origin);validate_body(j.body);
    require(j.axis.allFinite() && std::abs(j.axis.norm()-1.)<1e-10,"Arm joint axis must be unit length");
    require(std::isfinite(j.rotor_inertia) && j.rotor_inertia>=0.,"Invalid reflected rotor inertia");
    require(std::isfinite(j.lower) && std::isfinite(j.upper) && j.lower<j.upper &&
            std::isfinite(j.velocity_limit) && j.velocity_limit>0. &&
            std::isfinite(j.effort_limit) && j.effort_limit>0.,"Invalid arm joint limits");
    const auto& f=j.friction;
    require(std::isfinite(f.viscous) && f.viscous>=0. && std::isfinite(f.coulomb) && f.coulomb>=0. &&
            std::isfinite(f.static_friction) && f.static_friction>=f.coulomb &&
            std::isfinite(f.stribeck_velocity) && f.stribeck_velocity>0. &&
            std::isfinite(f.smoothing_velocity) && f.smoothing_velocity>0.,"Invalid dissipative arm friction");
  }
  if(model_->payload_enabled) {
    validate_body(model_->payload);
  }

  backend_=std::make_shared<const Backend>(*model_);
}
ArmDynamics ArmDynamics::from_urdf(const std::string& path) {
  pinocchio::Model source;pinocchio::urdf::buildModel(path,source);
  require(source.nq==source.nv && source.nv>0,"URDF requires fixed-base scalar revolute joints");
  ArmModel converted;converted.model_id=source.name;converted.model_version="tree_rigid_v1";
  converted.gravity_world=source.gravity.linear();
  pinocchio::Data data(source);const Eigen::VectorXd neutral=pinocchio::neutral(source);
  for(pinocchio::JointIndex id=1;id<source.njoints;++id) {
    require(source.joints[id].nq()==1 && source.joints[id].nv()==1,"URDF supports scalar revolute joints only");
    source.joints[id].calc(data.joints[id],neutral);
    const Eigen::MatrixXd motion=data.joints[id].S().matrix();
    require(motion.topRows(3).norm()<1e-12 && std::abs(motion.bottomRows(3).norm()-1.)<1e-12,"URDF joint is not revolute");
    ArmJoint joint;joint.name=source.names[id];joint.link_name=source.names[id]+"_body";
    joint.axis=motion.bottomRows(3).col(0);
    joint.origin.rotation=source.jointPlacements[id].rotation();joint.origin.translation=source.jointPlacements[id].translation();
    const auto& inertia=source.inertias[id];joint.body.mass=inertia.mass();joint.body.com=inertia.lever();joint.body.inertia_com=inertia.inertia().matrix();
    const auto index=source.joints[id].idx_v();
    joint.rotor_inertia=source.armature[index];joint.lower=source.lowerPositionLimit[index];joint.upper=source.upperPositionLimit[index];
    joint.velocity_limit=source.velocityLimit[index];joint.effort_limit=source.effortLimit[index];
    converted.parents.push_back(static_cast<int>(source.parents[id])-1);converted.joints.push_back(joint);
  }
  return ArmDynamics(std::move(converted));
}
void ArmDynamics::check_vector(const Eigen::VectorXd& x,const char* label) const {
  require(x.size()==static_cast<Eigen::Index>(model_->joints.size()) && x.allFinite(),std::string("Invalid arm vector: ")+label);
}
struct ArmWorkspace::Impl {
  pinocchio::Data data;
  explicit Impl(const pinocchio::Model& model):data(model) {}
};
ArmWorkspace::ArmWorkspace(const ArmDynamics& engine):engine_(engine),impl_(std::make_unique<Impl>(engine.backend_->model)) {}
ArmWorkspace::~ArmWorkspace()=default;
ArmWorkspace::ArmWorkspace(ArmWorkspace&&) noexcept=default;
ArmWorkspace& ArmWorkspace::operator=(ArmWorkspace&&) noexcept=default;
ArmWorkspace ArmDynamics::workspace() const { return ArmWorkspace(*this); }

Eigen::VectorXd ArmDynamics::rigid_parameters() const {
  Eigen::VectorXd parameters(10*static_cast<Eigen::Index>(model_->joints.size()));
  for(std::size_t j=0;j<model_->joints.size();++j)
    parameters.segment<10>(10*j)=backend_->model.inertias[j+1].toDynamicParameters();
  return parameters;
}
std::vector<std::string> ArmDynamics::rigid_parameter_names() const {
  std::vector<std::string> names;
  for(const auto& joint:model_->joints)
    for(const auto* key:{"mass","hx","hy","hz","Ixx","Ixy","Iyy","Ixz","Iyz","Izz"})
      names.push_back(joint.name+"."+key);
  return names;
}

// Stateless convenience entry points are safe for concurrent readers. Runners and batches
// explicitly retain a workspace instead of hiding mutable Data in a shared engine.
Eigen::MatrixXd ArmDynamics::mass_matrix(const Eigen::VectorXd& q) const {return workspace().mass_matrix(q);}
Eigen::VectorXd ArmDynamics::gravity(const Eigen::VectorXd& q) const {return workspace().gravity(q);}
Eigen::VectorXd ArmDynamics::velocity_bias(const Eigen::VectorXd& q,const Eigen::VectorXd& dq) const {return workspace().velocity_bias(q,dq);}
Eigen::VectorXd ArmDynamics::inverse(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq) const {return workspace().inverse(q,dq,ddq);}
Eigen::VectorXd ArmDynamics::forward(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& torque) const {return workspace().forward(q,dq,torque);}
ArmTerms ArmDynamics::components(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq) const {return workspace().components(q,dq,ddq);}
std::vector<Eigen::Matrix4d> ArmDynamics::fk(const Eigen::VectorXd& q) const {return workspace().fk(q);}
ArmEnergy ArmDynamics::energy(const Eigen::VectorXd& q,const Eigen::VectorXd& dq) const {return workspace().energy(q,dq);}
Eigen::MatrixXd ArmDynamics::torque_regressor(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq) const {return workspace().torque_regressor(q,dq,ddq);}
ArmInverseDerivatives ArmDynamics::inverse_derivatives(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq) const {return workspace().inverse_derivatives(q,dq,ddq);}
ArmForwardDerivatives ArmDynamics::forward_derivatives(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& torque) const {return workspace().forward_derivatives(q,dq,torque);}
Eigen::MatrixXd ArmDynamics::tool_jacobian(const Eigen::VectorXd& q) const {return workspace().tool_jacobian(q);}

Eigen::VectorXd ArmDynamics::rotor_torque(const Eigen::VectorXd& ddq) const {
  check_vector(ddq,"ddq");return backend_->model.armature.cwiseProduct(ddq);
}
Eigen::VectorXd ArmWorkspace::friction(const Eigen::VectorXd& dq) {return engine_.friction(dq);}
Eigen::VectorXd ArmWorkspace::rotor_torque(const Eigen::VectorXd& ddq) {return engine_.rotor_torque(ddq);}
Eigen::MatrixXd ArmWorkspace::mass_matrix(const Eigen::VectorXd& q) {
  engine_.check_vector(q,"q");auto& data=impl_->data;
  pinocchio::crba(engine_.backend_->model,data,q);
  return data.M.selfadjointView<Eigen::Upper>();
}
Eigen::VectorXd ArmWorkspace::gravity(const Eigen::VectorXd& q) {
  engine_.check_vector(q,"q");auto& data=impl_->data;
  return pinocchio::computeGeneralizedGravity(engine_.backend_->model,data,q);
}
Eigen::VectorXd ArmWorkspace::velocity_bias(const Eigen::VectorXd& q,const Eigen::VectorXd& dq) {
  engine_.check_vector(q,"q");engine_.check_vector(dq,"dq");auto& data=impl_->data;
  const Eigen::VectorXd nonlinear=pinocchio::nonLinearEffects(engine_.backend_->model,data,q,dq);
  return nonlinear-pinocchio::computeGeneralizedGravity(engine_.backend_->model,data,q);
}
Eigen::VectorXd ArmDynamics::friction(const Eigen::VectorXd& dq) const {
  check_vector(dq,"dq");Eigen::VectorXd result(dq.size());
  for(Eigen::Index j=0;j<dq.size();++j) {
    const auto& f=model_->joints[j].friction;const double v=dq[j];
    const double magnitude=f.coulomb+(f.static_friction-f.coulomb)*std::exp(-std::pow(v/f.stribeck_velocity,2));
    result[j]=f.viscous*v+magnitude*std::tanh(v/f.smoothing_velocity);
  }
  return result;
}
Eigen::VectorXd ArmWorkspace::inverse(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq) {
  engine_.check_vector(q,"q");engine_.check_vector(dq,"dq");engine_.check_vector(ddq,"ddq");auto& data=impl_->data;
  return pinocchio::rnea(engine_.backend_->model,data,q,dq,ddq)+friction(dq);
}
Eigen::VectorXd ArmWorkspace::forward(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& torque) {
  engine_.check_vector(q,"q");engine_.check_vector(dq,"dq");engine_.check_vector(torque,"torque");auto& data=impl_->data;
  const Eigen::VectorXd applied=torque-friction(dq);
  const Eigen::VectorXd result=pinocchio::aba(engine_.backend_->model,data,q,dq,applied);
  if(!result.allFinite()) throw std::runtime_error("Pinocchio forward dynamics produced non-finite acceleration");
  return result;
}
ArmTerms ArmWorkspace::components(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq) {
  engine_.check_vector(q,"q");engine_.check_vector(dq,"dq");engine_.check_vector(ddq,"ddq");auto& data=impl_->data;
  ArmTerms result;
  pinocchio::crba(engine_.backend_->model,data,q);result.mass_matrix=data.M.selfadjointView<Eigen::Upper>();
  result.inertia=result.mass_matrix*ddq;
  const Eigen::VectorXd nonlinear=pinocchio::nonLinearEffects(engine_.backend_->model,data,q,dq);
  result.gravity=pinocchio::computeGeneralizedGravity(engine_.backend_->model,data,q);
  result.velocity=nonlinear-result.gravity;result.friction=friction(dq);
  result.total=result.inertia+result.velocity+result.gravity+result.friction;
  return result;
}
std::vector<Eigen::Matrix4d> ArmWorkspace::fk(const Eigen::VectorXd& q) {
  engine_.check_vector(q,"q");auto& data=impl_->data;
  pinocchio::forwardKinematics(engine_.backend_->model,data,q);
  std::vector<Eigen::Matrix4d> result;result.reserve(engine_.model_->joints.size()+1);
  for(std::size_t j=1;j<=engine_.model_->joints.size();++j) result.push_back(data.oMi[j].toHomogeneousMatrix());
  if(engine_.model_->model_version=="serial_rigid_v1")
    result.push_back((data.oMi[engine_.model_->joints.size()]*to_pin(engine_.model_->tool)).toHomogeneousMatrix());
  return result;
}
ArmEnergy ArmWorkspace::energy(const Eigen::VectorXd& q,const Eigen::VectorXd& dq) {
  engine_.check_vector(q,"q");engine_.check_vector(dq,"dq");auto& data=impl_->data;
  // CRBA defines generalized kinetic energy including model.armature exactly once.
  pinocchio::crba(engine_.backend_->model,data,q);
  const Eigen::MatrixXd mass=data.M.selfadjointView<Eigen::Upper>();
  const double kinetic=.5*dq.dot(mass*dq);
  const double potential=pinocchio::computePotentialEnergy(engine_.backend_->model,data,q);
  return {kinetic,potential};
}
Eigen::MatrixXd ArmWorkspace::torque_regressor(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq) {
  engine_.check_vector(q,"q");engine_.check_vector(dq,"dq");engine_.check_vector(ddq,"ddq");
  // Pinocchio's body regressor excludes armature and explicit friction.
  return pinocchio::computeJointTorqueRegressor(engine_.backend_->model,impl_->data,q,dq,ddq);
}
ArmInverseDerivatives ArmWorkspace::inverse_derivatives(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq) {
  engine_.check_vector(q,"q");engine_.check_vector(dq,"dq");engine_.check_vector(ddq,"ddq");auto& data=impl_->data;
  data.dtau_dq.setZero();data.dtau_dv.setZero();data.M.setZero();
  pinocchio::computeRNEADerivatives(engine_.backend_->model,data,q,dq,ddq);
  ArmInverseDerivatives out{data.dtau_dq,data.dtau_dv,data.M.selfadjointView<Eigen::Upper>()};
  out.dtau_dv.diagonal()+=friction_slope(*engine_.model_,dq);
  return out;
}
ArmForwardDerivatives ArmWorkspace::forward_derivatives(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& torque) {
  engine_.check_vector(q,"q");engine_.check_vector(dq,"dq");engine_.check_vector(torque,"torque");auto& data=impl_->data;
  const Eigen::VectorXd applied=torque-friction(dq);
  data.ddq_dq.setZero();data.ddq_dv.setZero();data.Minv.setZero();
  pinocchio::computeABADerivatives(engine_.backend_->model,data,q,dq,applied);
  ArmForwardDerivatives out{data.ddq_dq,data.ddq_dv,data.Minv.selfadjointView<Eigen::Upper>()};
  out.ddq_dv-=out.ddq_dtau*friction_slope(*engine_.model_,dq).asDiagonal();
  return out;
}
Eigen::MatrixXd ArmWorkspace::tool_jacobian(const Eigen::VectorXd& q) {
  require(engine_.model_->model_version=="serial_rigid_v1","Tree tools need named frame support; legacy single TCP is not defined");
  engine_.check_vector(q,"q");auto& data=impl_->data;const auto& model=engine_.backend_->model;
  pinocchio::computeJointJacobians(model,data,q);
  return pinocchio::getFrameJacobian(model,data,model.njoints-1,to_pin(engine_.model_->tool),pinocchio::LOCAL_WORLD_ALIGNED);
}
}  // namespace casbot
