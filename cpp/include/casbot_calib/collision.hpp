#pragma once
#include "casbot_calib/arm_dynamics.hpp"
#include <utility>
namespace casbot {
struct CollisionGeometry {
  std::string name;
  int parent=-1;  // -1 fixed; otherwise the configured model's joint index.
  bool attached_to_base=true;
  ArmPose placement;
  Eigen::MatrixXd vertices;
  Eigen::MatrixXi triangles;
};
struct CollisionBatch {
  Eigen::MatrixXd distances;  // poses x active pairs; signed surface separation, m.
  Eigen::MatrixXd first_point,second_point;
  Eigen::VectorXi worst_pair;
};
// Single-owner reusable Pinocchio Data and Coal distance solvers. No stepping,
// geometry mutation or Python callbacks. Fixed same-parent pairs are omitted.
class CollisionChecker {
 public:
  CollisionChecker(ArmDynamics model,const std::vector<CollisionGeometry>& geometry,
                   const std::vector<std::pair<int,int>>& excluded_pairs);
  ~CollisionChecker();
  CollisionChecker(CollisionChecker&&) noexcept;
  CollisionChecker& operator=(CollisionChecker&&) noexcept;
  CollisionBatch check_batch(const Eigen::MatrixXd& positions);
  Eigen::MatrixXd motion_radii() const; // pair x joint; ||point displacement|| <= R*abs(delta_q).
  std::vector<std::pair<int,int>> pairs() const;
  static std::string backend_version();
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
