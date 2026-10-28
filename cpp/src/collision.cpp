#include "casbot_calib/collision.hpp"
#include <coal/config.hh>
#include <coal/distance.h>
#include <coal/shape/convex.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <set>
#include <stdexcept>

namespace casbot {
struct CollisionChecker::Impl {
  ArmDynamics model;
  ArmWorkspace workspace;
  std::vector<CollisionGeometry> geometries;
  std::vector<std::shared_ptr<coal::CollisionGeometry>> shapes;
  std::vector<std::pair<int,int>> pairs;
  std::vector<std::unique_ptr<coal::ComputeDistance>> solvers;
  Eigen::MatrixXd radii;
  Eigen::Matrix4d base;
  std::mutex mutex;
  Impl(ArmDynamics source,const std::vector<CollisionGeometry>& geometry,const std::vector<std::pair<int,int>>& excluded)
    :model(std::move(source)),workspace(model.workspace()),geometries(geometry) {
    const auto config=model.model();const int n=static_cast<int>(config.joints.size()),count=static_cast<int>(geometry.size());
    base=config.base.matrix();
    if(count<2)throw std::invalid_argument("At least two collision geometries required");
    Eigen::MatrixXd body_radii=Eigen::MatrixXd::Zero(count,n);
    std::set<std::string> names;
    for(int i=0;i<count;++i) {
      const auto& g=geometry[i];const auto pose=g.placement.matrix();
      if(g.name.empty() || !names.insert(g.name).second || g.parent<-1 || g.parent>=n ||
         g.vertices.cols()!=3 || g.vertices.rows()<4 || !g.vertices.allFinite() ||
         g.triangles.cols()!=3 || g.triangles.rows()<4 || g.triangles.minCoeff()<0 || g.triangles.maxCoeff()>=g.vertices.rows() ||
         !pose.allFinite() || (g.placement.rotation.transpose()*g.placement.rotation-Eigen::Matrix3d::Identity()).norm()>1e-10 ||
         std::abs(g.placement.rotation.determinant()-1.)>1e-10)
        throw std::invalid_argument("Invalid named convex collision geometry");
      auto points=std::make_shared<std::vector<coal::Vec3s>>();points->reserve(g.vertices.rows());
      double radius=0.;
      for(Eigen::Index j=0;j<g.vertices.rows();++j) {
        const Eigen::Vector3d point=g.vertices.row(j).transpose();points->push_back(point);
        radius=std::max(radius,(g.placement.rotation*point+g.placement.translation).norm());
      }
      auto faces=std::make_shared<std::vector<coal::Triangle>>();faces->reserve(g.triangles.rows());
      for(Eigen::Index j=0;j<g.triangles.rows();++j)faces->emplace_back(g.triangles(j,0),g.triangles(j,1),g.triangles(j,2));
      auto shape=std::make_shared<coal::Convex<coal::Triangle>>(points,points->size(),faces,faces->size());
      shape->computeLocalAABB();shapes.push_back(std::move(shape));
      for(int joint=g.parent;joint>=0;) {
        body_radii(i,joint)=radius;
        radius+=config.joints[joint].origin.translation.norm();
        joint=config.parents.empty()?joint-1:config.parents[joint];
      }
    }
    std::set<std::pair<int,int>> disabled;
    for(auto pair:excluded) {
      if(pair.first>pair.second)std::swap(pair.first,pair.second);
      if(pair.first<0 || pair.second>=count || pair.first==pair.second)throw std::invalid_argument("Invalid collision exclusion");
      disabled.insert(pair);
    }
    for(int a=0;a<count;++a)for(int b=a+1;b<count;++b) {
      if(geometry[a].parent==geometry[b].parent || disabled.count({a,b}))continue;
      pairs.emplace_back(a,b);solvers.push_back(std::make_unique<coal::ComputeDistance>(shapes[a].get(),shapes[b].get()));
    }
    if(pairs.empty())throw std::invalid_argument("No active moving collision pairs");
    radii.resize(pairs.size(),n);
    for(std::size_t p=0;p<pairs.size();++p) {
      const auto a=pairs[p].first,b=pairs[p].second;
      radii.row(p)=body_radii.row(a)+body_radii.row(b);
      // A shared ancestor applies the same rigid transform to both shapes.
      // It cannot change their distance, even while other joints move.
      for(int j=0;j<n;++j)if(body_radii(a,j)>0 && body_radii(b,j)>0)radii(p,j)=0.;
    }
  }
};
CollisionChecker::CollisionChecker(ArmDynamics m,const std::vector<CollisionGeometry>& g,const std::vector<std::pair<int,int>>& excluded)
  :impl_(std::make_unique<Impl>(std::move(m),g,excluded)) {}
CollisionChecker::~CollisionChecker()=default;
CollisionChecker::CollisionChecker(CollisionChecker&&) noexcept=default;
CollisionChecker& CollisionChecker::operator=(CollisionChecker&&) noexcept=default;
std::string CollisionChecker::backend_version() {return std::string("coal-")+COAL_VERSION;}
Eigen::MatrixXd CollisionChecker::motion_radii() const {return impl_->radii;}
std::vector<std::pair<int,int>> CollisionChecker::pairs() const {return impl_->pairs;}
CollisionBatch CollisionChecker::check_batch(const Eigen::MatrixXd& positions) {
  auto& s=*impl_;std::lock_guard<std::mutex> lock(s.mutex);const auto n=s.radii.cols();
  if(positions.cols()!=static_cast<Eigen::Index>(n) || !positions.allFinite())throw std::invalid_argument("Expected finite q[N,J]");
  CollisionBatch output;output.distances.resize(positions.rows(),s.pairs.size());output.worst_pair.resize(positions.rows());
  output.first_point.resize(positions.rows(),3);output.second_point.resize(positions.rows(),3);
  std::vector<coal::Transform3s> poses;poses.reserve(s.geometries.size());
  coal::DistanceRequest request(true,true);
  for(Eigen::Index row=0;row<positions.rows();++row) {
    const auto joints=s.workspace.fk(positions.row(row).transpose());poses.clear();
    for(const auto& g:s.geometries) {
      const Eigen::Matrix4d transform=(g.parent<0?(g.attached_to_base?s.base:Eigen::Matrix4d::Identity()):joints[g.parent])*g.placement.matrix();
      poses.emplace_back(Eigen::Matrix3d(transform.topLeftCorner<3,3>()),Eigen::Vector3d(transform.topRightCorner<3,1>()));
    }
    double minimum=std::numeric_limits<double>::infinity();
    for(std::size_t p=0;p<s.pairs.size();++p) {
      coal::DistanceResult result;const auto pair=s.pairs[p];
      const double distance=(*s.solvers[p])(poses[pair.first],poses[pair.second],request,result);
      if(!std::isfinite(distance))throw std::runtime_error("Coal distance query failed; refusing to certify path");
      output.distances(row,p)=distance;
      if(distance<minimum) {minimum=distance;output.worst_pair[row]=p;
        output.first_point.row(row)=result.nearest_points[0].transpose();output.second_point.row(row)=result.nearest_points[1].transpose();}
    }
  }
  return output;
}
}
