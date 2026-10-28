#pragma once
#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

namespace casbot {
// All vectors/torques use SI. Pose maps child coordinates into parent coordinates.
struct ArmPose {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Matrix4d matrix() const;
};
struct ArmBody {
  double mass = 1.;
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
  Eigen::Matrix3d inertia_com = .01*Eigen::Matrix3d::Identity();
};
struct ArmFriction {
  double viscous = 0., coulomb = 0., static_friction = 0.;
  double stribeck_velocity = .05, smoothing_velocity = .01;
};
struct ArmJoint {
  std::string name, link_name;
  ArmPose origin;
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  ArmBody body;
  ArmFriction friction;
  // Output-side reflected rotor inertia, excluded from body.inertia_com.
  double rotor_inertia = 0.;
  double lower = -3.141592653589793, upper = 3.141592653589793;
  double velocity_limit = 10., effort_limit = 20.;
};
// Massless observation frame. parent=-1 means model base, otherwise joint index.
struct ArmFrame {
  std::string name;
  int parent = -1;
  ArmPose placement;  // Frame coordinates -> parent coordinates.
};
struct ArmModel {
  std::string model_id, model_version = "serial_rigid_v1";
  ArmPose base, tool;
  Eigen::Vector3d gravity_world = {0.,0.,-9.81};
  std::vector<ArmJoint> joints;
  bool payload_enabled = false;
  ArmBody payload;  // In tool coordinates, merged once into the last body.
  // tree_rigid_v1 only: -1 is fixed root, otherwise an earlier joint index.
  std::vector<int> parents;
  std::vector<ArmFrame> frames;
};
struct ArmTerms {
  Eigen::MatrixXd mass_matrix;
  Eigen::VectorXd inertia, velocity, gravity, friction, total;
};
struct ArmEnergy { double kinetic, potential; };
struct ArmInverseDerivatives { Eigen::MatrixXd dtau_dq, dtau_dv, dtau_da; };
struct ArmForwardDerivatives { Eigen::MatrixXd ddq_dq, ddq_dv, ddq_dtau; };
struct ArmPointGeometry {
  Eigen::Vector3d position;
  Eigen::MatrixXd placements; // 3 x 6J, right-local joint origin increments [v,w].
  Eigen::MatrixXd joints;    // 3 x J, observed joint-angle derivatives.
};
class ArmWorkspace;

// Immutable validated copy: changing a caller's model cannot change a running device.
// Pinocchio backend; fixed-base scalar revolute chains/trees. No hardware I/O.
class ArmDynamics {
 public:
  explicit ArmDynamics(ArmModel model);
  ArmModel model() const { return *model_; }
  static std::string backend_version();
  static ArmDynamics from_urdf(const std::string& path);
  // A single-owner, reusable Pinocchio Data. Independent workspaces may run concurrently.
  ArmWorkspace workspace() const;
  // Ten body parameters per moving joint, including the merged payload, excluding armature.
  Eigen::VectorXd rigid_parameters() const;
  std::vector<std::string> rigid_parameter_names() const;
  Eigen::MatrixXd mass_matrix(const Eigen::VectorXd& q) const;
  Eigen::VectorXd gravity(const Eigen::VectorXd& q) const;
  Eigen::VectorXd velocity_bias(const Eigen::VectorXd& q,const Eigen::VectorXd& dq) const;
  Eigen::VectorXd friction(const Eigen::VectorXd& dq) const;
  Eigen::VectorXd inverse(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq) const;
  Eigen::VectorXd forward(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& torque) const;
  ArmTerms components(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq) const;
  // One world pose per moving joint. Legacy serial model appends its tool pose.
  std::vector<Eigen::Matrix4d> fk(const Eigen::VectorXd& q) const;
  ArmEnergy energy(const Eigen::VectorXd& q,const Eigen::VectorXd& dq) const;
  Eigen::VectorXd rotor_torque(const Eigen::VectorXd& ddq) const;
  Eigen::MatrixXd torque_regressor(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq) const;
  ArmInverseDerivatives inverse_derivatives(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq) const;
  ArmForwardDerivatives forward_derivatives(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& torque) const;
  // World-aligned TCP velocity Jacobian: rows [linear m/s, angular rad/s].
  Eigen::MatrixXd tool_jacobian(const Eigen::VectorXd& q) const;
  std::vector<std::string> frame_names() const;
  Eigen::Matrix4d frame_pose(const Eigen::VectorXd& q,const std::string& name) const;
  // LOCAL_WORLD_ALIGNED, rows [linear, angular], columns in model joint order.
  Eigen::MatrixXd frame_jacobian(const Eigen::VectorXd& q,const std::string& name) const;
  ArmPointGeometry point_geometry(const Eigen::VectorXd& q,const std::string& name,const Eigen::Vector3d& offset) const;
 private:
  friend class ArmWorkspace;
  std::shared_ptr<const ArmModel> model_;
  struct Backend;
  std::shared_ptr<const Backend> backend_;
  void check_vector(const Eigen::VectorXd& x,const char* label) const;

};

// Keeps its own immutable model handle alive. Not shared between C++ worker threads.
// Python methods retain the GIL; engine batch calls allocate one workspace per batch.
class ArmWorkspace {
 public:
  ~ArmWorkspace();
  ArmWorkspace(ArmWorkspace&&) noexcept;
  ArmWorkspace& operator=(ArmWorkspace&&) noexcept;
  ArmWorkspace(const ArmWorkspace&) = delete;
  ArmWorkspace& operator=(const ArmWorkspace&) = delete;
  Eigen::MatrixXd mass_matrix(const Eigen::VectorXd& q);
  Eigen::VectorXd gravity(const Eigen::VectorXd& q);
  Eigen::VectorXd velocity_bias(const Eigen::VectorXd& q,const Eigen::VectorXd& dq);
  Eigen::VectorXd friction(const Eigen::VectorXd& dq);
  Eigen::VectorXd rotor_torque(const Eigen::VectorXd& ddq);
  Eigen::VectorXd inverse(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq);
  Eigen::VectorXd forward(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& torque);
  ArmTerms components(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq);
  std::vector<Eigen::Matrix4d> fk(const Eigen::VectorXd& q);
  ArmEnergy energy(const Eigen::VectorXd& q,const Eigen::VectorXd& dq);
  Eigen::MatrixXd torque_regressor(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq);
  ArmInverseDerivatives inverse_derivatives(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq);
  ArmForwardDerivatives forward_derivatives(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& torque);
  Eigen::MatrixXd tool_jacobian(const Eigen::VectorXd& q);
  Eigen::Matrix4d frame_pose(const Eigen::VectorXd& q,const std::string& name);
  Eigen::MatrixXd frame_jacobian(const Eigen::VectorXd& q,const std::string& name);
  ArmPointGeometry point_geometry(const Eigen::VectorXd& q,const std::string& name,const Eigen::Vector3d& offset);
 private:
  friend class ArmDynamics;
  explicit ArmWorkspace(const ArmDynamics& engine);
  ArmDynamics engine_;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace casbot
