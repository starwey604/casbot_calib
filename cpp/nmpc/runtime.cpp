#include "model_data.hpp"
#include "runtime_api.h"
#include "acados_solver_dvt2.h"
#include <Eigen/Geometry>
#include <coal/distance.h>
#include <coal/shape/convex.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <vector>

namespace {
using Vec = Eigen::Vector3d;
using Mat = Eigen::Matrix4d;
using RowMat = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using Joint = Eigen::Matrix<double, 14, 1>;
using Hull = coal::Convex<coal::Triangle>;
constexpr int fixed = -1;

Mat matrix(const double* p) { return Eigen::Map<const RowMat>(p); }
Vec vector(const double* p) { return Eigen::Map<const Vec>(p); }

struct Poses {
    std::array<Mat, 14> joint;
    std::array<Vec, 14> origin, axis;
    std::array<Mat, 2> flange;
    std::array<std::array<Vec, 2>, data::shapes> ends;
    std::array<coal::Transform3s, data::shapes> transforms;

    explicit Poses(const double* q) {
        for (int arm = 0; arm < 2; ++arm) {
            Mat pose = matrix(data::bases + 16*arm);
            for (int joint_id = 7*arm; joint_id < 7*arm+7; ++joint_id) {
                pose *= matrix(data::origins + 16*joint_id);
                Vec local = vector(data::axes + 3*joint_id);
                origin[joint_id] = pose.topRightCorner<3,1>();
                axis[joint_id] = pose.topLeftCorner<3,3>() * local;
                pose.topLeftCorner<3,3>() *= Eigen::AngleAxisd(q[joint_id], local).toRotationMatrix();
                joint[joint_id] = pose;
            }
            flange[arm] = pose * matrix(data::tools + 16*arm);
        }
        for (int i = 0; i < data::shapes; ++i) {
            const auto parent = data::parents[i];
            if (parent != fixed)
                transforms[i] = coal::Transform3s(joint[parent].topLeftCorner<3,3>(),
                                                   joint[parent].topRightCorner<3,1>());
            for (int j = 0; j < 2; ++j)
                ends[i][j] = transforms[i].transform(vector(data::endpoints + 6*i + 3*j));
        }
    }
};

struct Witness {
    double distance;
    Vec first, second;
};

// Closest points of closed segments, including zero-length capsule centerlines.
Witness capsule(const Poses& pose, int i, int j) {
    const Vec p = pose.ends[i][0], q = pose.ends[j][0];
    const Vec u = pose.ends[i][1]-p, v = pose.ends[j][1]-q, w = p-q;
    const double a = u.squaredNorm(), b = u.dot(v), c = v.squaredNorm();
    const double d = u.dot(w), e = v.dot(w);
    auto unit = [](double t) { return std::clamp(t, 0., 1.); };
    double s = 0., t = 0.;
    if (c <= 1e-24) {
        if (a > 1e-24) s = unit(-d/a);
    } else if (a <= 1e-24) {
        t = unit(e/c);
    } else {
        const double det = a*c-b*b;
        if (det > 1e-14*a*c) s = unit((b*e-c*d)/det);
        t = (b*s+e)/c;
        if (t < 0.) { t = 0.; s = unit(-d/a); }
        if (t > 1.) { t = 1.; s = unit((b-d)/a); }
    }
    Vec first = p+s*u, second = q+t*v;
    return {(first-second).norm()-data::radius[i]-data::radius[j], first, second};
}

struct Runtime {
    std::array<std::shared_ptr<Hull>, data::shapes> hulls;
    std::array<std::unique_ptr<coal::ComputeDistance>, data::count> convex_queries;
    dvt2_solver_capsule* solver = nullptr;

    Runtime() {
        for (int i = 0; i < data::shapes; ++i) {
            if (data::vertex_offsets[i] == data::vertex_offsets[i+1]) continue;
            auto points = std::make_shared<std::vector<Vec>>();
            auto faces = std::make_shared<std::vector<coal::Triangle>>();
            for (int v = data::vertex_offsets[i]; v < data::vertex_offsets[i+1]; ++v)
                points->push_back(vector(data::vertices + 3*v));
            for (int f = data::face_offsets[i]; f < data::face_offsets[i+1]; ++f) {
                const int* ids = data::triangles + 3*f;
                faces->emplace_back(ids[0], ids[1], ids[2]);
            }
            hulls[i] = std::make_shared<Hull>(points, points->size(), faces, faces->size());
            hulls[i]->computeLocalAABB();
        }
        for (int i = 0; i < data::count; ++i) {
            if (data::convex[i])
                convex_queries[i] = std::make_unique<coal::ComputeDistance>(
                    hulls[data::pairs[2*i]].get(), hulls[data::pairs[2*i+1]].get());
        }
        solver = dvt2_acados_create_capsule();
        if (dvt2_acados_create(solver)) {
            dvt2_acados_free_capsule(solver);
            throw std::runtime_error("acados initialization failed");
        }
    }
    ~Runtime() {
        dvt2_acados_free(solver);
        dvt2_acados_free_capsule(solver);
    }

    Witness witness(const Poses& pose, int pair) const {
        int i = data::pairs[2*pair], j = data::pairs[2*pair+1];
        if (!data::convex[pair]) return capsule(pose, i, j);
        coal::DistanceResult result;
        double gap = (*convex_queries[pair])(pose.transforms[i], pose.transforms[j],
                                             coal::DistanceRequest(true, true), result);
        return {gap, result.nearest_points[0], result.nearest_points[1]};
    }
};

Joint gradient(const Poses& pose, int pair, const Witness& w) {
    Vec normal = (w.first-w.second).normalized();
    if (data::convex[pair] && w.distance < 0) normal = -normal;
    Joint g = Joint::Zero();
    for (int side = 0; side < 2; ++side) {
        const int link = data::parents[data::pairs[2*pair+side]];
        if (link == fixed) continue;
        const Vec point = side ? w.second : w.first;
        for (int j = 7*(link/7); j <= link; ++j)
            g[j] += (side ? -1 : 1)*normal.dot(pose.axis[j].cross(point-pose.origin[j]));
    }
    return g;
}
}

void* nmpc_create() {
    try { return new Runtime; } catch (...) { return nullptr; }
}

void nmpc_destroy(void* handle) { delete static_cast<Runtime*>(handle); }

int nmpc_query(void* handle, const double* q, double* gaps, double* grads, double* poses) {
    try {
        const Poses k(q);
        auto& r = *static_cast<Runtime*>(handle);
        if (poses) for (int i = 0; i < 2; ++i)
            Eigen::Map<RowMat>(poses+16*i) = k.flange[i];
        if (gaps) for (int i = 0; i < data::count; ++i) {
            const auto w = r.witness(k,i);
            if (!std::isfinite(w.distance)) return -1;
            gaps[i] = w.distance;
            if (grads) Eigen::Map<Joint>(grads+14*i) = gradient(k,i,w);
        }
        return 0;
    } catch (...) { return -1; }
}

int nmpc_solve(void* handle, const double* states, const double* controls,
               const double* targets, double* output) {
    try {
        auto& r = *static_cast<Runtime*>(handle);
        auto config = dvt2_acados_get_nlp_config(r.solver);
        auto dims = dvt2_acados_get_nlp_dims(r.solver);
        auto in = dvt2_acados_get_nlp_in(r.solver);
        auto out = dvt2_acados_get_nlp_out(r.solver);
        std::array<double,DVT2_NY> reference{};
        std::copy_n(targets, 24, reference.begin());
        for (int stage = 0; stage <= DVT2_N; ++stage) {
            const double* x = states+42*stage;
            Joint q = Eigen::Map<const Joint>(x);
            if (stage == DVT2_N)
                q += data::rest[1]*Eigen::Map<const Joint>(x+14)
                   + data::rest[2]*Eigen::Map<const Joint>(x+28);
            Poses k(q.data());
            std::array<Witness,data::count> witnesses;
            std::array<int,data::count> ids;
            std::iota(ids.begin(), ids.end(), 0);
            for (int i : ids) {
                witnesses[i] = r.witness(k,i);
                if (!std::isfinite(witnesses[i].distance)) return -1;
            }
            auto slack = [&](int i) { return witnesses[i].distance-data::margin[i]; };
            std::partial_sort(ids.begin(), ids.begin()+data::active, ids.end(),
                              [&](int a, int b) { return slack(a)<slack(b); });
            std::array<double,DVT2_NP> params{};
            for (int row = 0; row < data::active; ++row) {
                const int i = ids[row];
                Joint g = gradient(k,i,witnesses[i]);
                if (!g.allFinite()) return -1;
                Eigen::Map<Joint>(params.data()+15*row) = g;
                const double buffer = std::clamp(slack(i)-1e-4, 0., 0.001);
                params[15*row+14] = slack(i)-buffer-g.dot(q);
            }
            dvt2_acados_update_params(r.solver, stage, params.data(), DVT2_NP);
            ocp_nlp_cost_model_set(config,dims,in,stage,"yref",reference.data());
            ocp_nlp_out_set(config,dims,out,in,stage,"x",const_cast<double*>(x));
            if (stage < DVT2_N)
                ocp_nlp_out_set(config,dims,out,in,stage,"u",const_cast<double*>(controls+14*stage));
        }
        for (const char* key : {"lbx","ubx"})
            ocp_nlp_constraints_model_set(config,dims,in,out,0,key,const_cast<double*>(states));
        const int status = dvt2_acados_solve(r.solver);
        if (status == 0) for (int i = 0; i < DVT2_N; ++i)
            ocp_nlp_out_get(config,dims,out,i,"u",output+14*i);
        return status;
    } catch (...) { return -1; }
}

namespace {
using State = Eigen::Matrix<double,42,1>;

State at(const State& x, const Joint& j, double t) {
    State y;
    auto v = x.segment<14>(14), a = x.tail<14>();
    y.head<14>() = x.head<14>()+t*v+t*t/2*a+t*t*t/6*j;
    y.segment<14>(14) = v+t*a+t*t/2*j;
    y.tail<14>() = a+t*j;
    return y;
}

// Each cubic point (including all substep probes) lies inside its Bernstein hull.
Joint excursion(const State& a, const State& b, const Joint& center, double dt) {
    Joint q = a.head<14>()-center, r = b.head<14>()-center;
    return q.cwiseAbs().cwiseMax(r.cwiseAbs())
        .cwiseMax((q+dt/3*a.segment<14>(14)).cwiseAbs())
        .cwiseMax((r-dt/3*b.segment<14>(14)).cwiseAbs());
}

bool prove(const Runtime& runtime, const State& x, const Joint& jerk, double dt,
           const std::vector<int>& ids, const double* radii, int depth) {
    State mid = at(x,jerk,dt/2);
    Poses poses(mid.data());
    Joint variation = excursion(x, at(x,jerk,dt), mid.head<14>(), dt);
    std::vector<int> uncertain;
    for (int i : ids) {
        double slack = runtime.witness(poses,i).distance-data::margin[i];
        if (!(slack > 0)) return false;
        if (slack <= variation.dot(Eigen::Map<const Joint>(radii+14*i)))
            uncertain.push_back(i);
    }
    if (uncertain.empty()) return true;
    if (depth >= 8) return false;
    int subdivisions = depth == 0 ? 5 : 2;
    for (int s = 0; s < subdivisions; ++s)
        if (!prove(runtime,at(x,jerk,s*dt/subdivisions),jerk,dt/subdivisions,
                   uncertain,radii,depth+1)) return false;
    return true;
}
}

int nmpc_certify(void* handle, const double* knots, const double* jerks, int segments,
                 double dt, const double* gaps, const double* radii) {
    try {
        auto& r = *static_cast<Runtime*>(handle);
        const Joint center = Eigen::Map<const Joint>(knots);
        std::vector<Joint> variations(segments);
        Joint whole = Joint::Zero();
        for (int s = 0; s < segments; ++s) {
            State x = Eigen::Map<const State>(knots+42*s);
            State y = Eigen::Map<const State>(knots+42*(s+1));
            variations[s] = excursion(x,y,center,dt);
            whole = whole.cwiseMax(variations[s]);
        }
        std::vector<int> active, uncertain;
        for (int i = 0; i < data::count; ++i) {
            if (!(gaps[i] > data::margin[i])) return 0;
            if (gaps[i]-data::margin[i] <= whole.dot(Eigen::Map<const Joint>(radii+14*i)))
                active.push_back(i);
        }
        // The tail often approaches the boundary first. Its intervals can reject a trial early.
        for (int s = segments-1; s >= 0; --s) {
            uncertain.clear();
            for (int i : active) {
                if (gaps[i]-data::margin[i] <= variations[s].dot(Eigen::Map<const Joint>(radii+14*i)))
                    uncertain.push_back(i);
            }
            if (!uncertain.empty() && !prove(r,Eigen::Map<const State>(knots+42*s),
                    Eigen::Map<const Joint>(jerks+14*s),dt,uncertain,radii,0)) return 0;
        }
        return 1;
    } catch (...) { return -1; }
}
