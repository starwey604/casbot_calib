"""Generate the fixed DVT2 SQP-RTI solver and its native geometry runtime."""
import json
import os
from pathlib import Path
import subprocess
import sys

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

from ..model_context import load_context
from ..project_paths import ROOT
from .import_dvt2 import write_model
from .model import stopping_maps

BUILD = ROOT / 'build/nmpc/dvt2_bare_flange'
ACTIVE = 12


def make_ocp(spec, output):
    cfg = spec['config']
    dt = cfg['dt']
    weights = cfg['weights']
    m = AcadosModel()
    m.name = 'dvt2_arm'
    m.x = ca.SX.sym('x', 42)
    m.u = ca.SX.sym('u', 14)
    q, v, a = m.x[:14], m.x[14:28], m.x[28:]
    j = m.u * cfg['jerk_rad_s3']
    m.disc_dyn_expr = ca.vertcat(q + dt*v + dt**2*a/2 + dt**3*j/6,
                                 v + dt*a + dt**2*j/2, a + dt*j)
    outputs, costs = [], []
    for arm in range(2):
        pose = ca.DM(spec['bases'][arm])
        for link in range(7):
            axis = ca.skew(ca.DM(spec['axes'][arm][link]))
            angle = q[7*arm + link]
            rotation = ca.DM.eye(3) + ca.sin(angle)*axis + (1-ca.cos(angle))*(axis @ axis)
            turn = ca.diagcat(rotation, 1)
            pose = pose @ ca.DM(spec['origins'][arm][link]) @ turn
        pose = pose @ ca.DM(spec['tools'][arm])
        outputs.extend([pose[:3, 3], ca.vec(pose[:3, :3])])
        costs.extend([weights['position']]*3 + [weights['orientation']]*9)
    outputs.extend([q - spec['q0'], v, a])
    costs.extend([weights['posture']]*14 + [weights['velocity']]*14 + [weights['acceleration']]*14)
    if weights['singularity']:
        for arm in range(2):
            position, rotation = outputs[2*arm:2*arm+2]
            rot = ca.reshape(rotation, 3, 3)
            dr = ca.jacobian(rotation, q)[:, 7*arm:7*arm+7]
            omega = [ca.inv_skew(ca.reshape(dr[:, i], 3, 3) @ rot.T) for i in range(7)]
            jac = ca.vertcat(ca.jacobian(position, q)[:, 7*arm:7*arm+7], ca.horzcat(*omega))
            gram = jac @ jac.T + .05**2 * ca.SX.eye(6)
            outputs.append(ca.sqrt(.05**2 * ca.trace(ca.solve(gram, ca.SX.eye(6))) / 6))
            costs.append(weights['singularity'])
    m.cost_y_expr_e = ca.vertcat(*outputs)
    m.cost_y_expr = ca.vertcat(m.cost_y_expr_e, m.u)
    # Closest active margins and analytic gradients are relinearized at every shooting node.
    m.p = ca.SX.sym('cuts', 15*ACTIVE)
    cuts = ca.reshape(m.p, 15, ACTIVE)
    tail_jerks, rest = stopping_maps(cfg['stopping_tail_steps'], dt)
    m.con_h_expr = cuts.T @ ca.vertcat(q, 1)
    m.con_h_expr_e = cuts.T @ ca.vertcat(q + rest[1]*v + rest[2]*a, 1)
    ocp = AcadosOcp()
    ocp.name = 'dvt2'
    ocp.model = m
    ocp.parameter_values = np.zeros(15*ACTIVE)
    ocp.cost.cost_type = ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.cost.W_e = np.diag(costs)
    ocp.cost.W = np.diag(costs + [weights['jerk']]*14)
    ocp.cost.yref_e, ocp.cost.yref = np.zeros(len(costs)), np.zeros(len(costs)+14)
    lo, hi = np.array(spec['bounds']).T
    speed = np.array(spec['velocity'])
    accel = np.full(14, cfg['acceleration_rad_s2'])
    c = ocp.constraints
    c.x0 = np.r_[spec['q0'], np.zeros(28)]
    c.idxbx = c.idxbx_e = np.arange(42)
    c.lbx = c.lbx_e = np.r_[lo, -speed, -accel]
    c.ubx = c.ubx_e = np.r_[hi, speed, accel]
    # The analytic suffix jerk is affine in time. Its two endpoints bound every interval.
    c.C_e = np.vstack([np.kron(tail_jerks[i], np.eye(14)) for i in (0, -1)])
    c.lg_e = np.full(28, -cfg['jerk_rad_s3'])
    c.ug_e = -c.lg_e
    c.idxbu = np.arange(14)
    c.lbu, c.ubu = -np.ones(14), np.ones(14)
    c.lh = c.lh_e = np.zeros(ACTIVE)
    c.uh = c.uh_e = np.full(ACTIVE, 1e6)
    opts = ocp.solver_options
    opts.N_horizon = cfg['horizon_steps']
    opts.tf = opts.N_horizon * dt
    opts.nlp_solver_type = 'SQP_RTI'
    opts.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    opts.qp_solver_cond_N = min(5, opts.N_horizon)
    for key, value in spec['qp_options'].items():
        setattr(opts, 'qp_solver_' + key, value)
    opts.integrator_type = 'DISCRETE'
    opts.hessian_approx = 'GAUSS_NEWTON'
    opts.levenberg_marquardt = 1e-5
    ocp.code_gen_options.code_export_directory = str(output / 'generated')
    ocp.code_gen_options.json_file = str(output / 'ocp.json')
    AcadosOcpSolver.generate(ocp)
    acados_lib = Path(os.environ['ACADOS_SOURCE_DIR']) / 'lib'
    env = dict(os.environ, LDFLAGS=f'-Wl,-rpath,{acados_lib}')
    subprocess.run(['make', '-C', str(output/'generated'), 'ocp_shared_lib'], env=env, check=True)


def native_data(spec, profile):
    lines = ['#pragma once', 'namespace data {']

    def array(name, value, integer=False):
        dtype = 'int' if integer else 'double'
        values = ','.join(map(str, np.asarray(value, dtype=int if integer else float).ravel()))
        lines.append(f'inline constexpr {dtype} {name}[] = {{{values}}};')

    _, rest = stopping_maps(spec['config']['stopping_tail_steps'], spec['config']['dt'])
    array('rest', rest)
    for key in ('bases', 'axes', 'origins', 'tools'):
        array(key, spec[key])
    geom = spec['geometries']
    pairs = spec['pairs']
    array('parents', [g['parent'] for g in geom], True)
    array('endpoints', [g['endpoints'] for g in geom])
    array('radius', [g['radius_m'] for g in geom])
    array('pairs', [p['indices'] for p in pairs], True)
    required = set()
    for mode in ('simulation', 'hardware'):
        rows = [pair['modes'][mode] for pair in pairs]
        array(mode+'_margin', [row['pair_clearance_m'] for row in rows])
        array(mode+'_convex', [row['geometry'] == 'convex' for row in rows], True)
        for pair, row in zip(pairs, rows):
            if row['geometry'] == 'convex':
                required.update(pair['indices'])
    vertices, triangles, vertex_offsets, face_offsets = [], [], [0], [0]
    for i, g in enumerate(profile.geometry):
        if i in required:
            placement = g.placement.matrix()
            if g.parent < 0 and g.attached_to_base:
                placement = np.array(spec['bases'][0]) @ placement
            vertices.extend(g.vertices @ placement[:3, :3].T + placement[:3, 3])
            triangles.extend(g.triangles)
        vertex_offsets.append(len(vertices))
        face_offsets.append(len(triangles))
    array('vertices', vertices)
    array('triangles', triangles, True)
    array('vertex_offsets', vertex_offsets, True)
    array('face_offsets', face_offsets, True)
    lines.append(f'constexpr int shapes={len(geom)}, count={len(pairs)}, active={ACTIVE};')
    return '\n'.join(lines + ['}', ''])


def build():
    BUILD.mkdir(parents=True, exist_ok=True)
    spec = write_model()
    profile = load_context().collision_profile()
    make_ocp(spec, BUILD)
    (BUILD / 'model_data.hpp').write_text(native_data(spec, profile))
    cmake = BUILD / 'cmake'
    subprocess.run(['cmake', '-S', str(ROOT/'cpp/nmpc'), '-B', str(cmake),
                    '-DCMAKE_BUILD_TYPE=Release', f'-DMODEL_DIR={BUILD}',
                    f'-DACADOS_DIR={os.environ["ACADOS_SOURCE_DIR"]}',
                    f'-DCMAKE_PREFIX_PATH={sys.prefix}'], check=True)
    subprocess.run(['cmake', '--build', str(cmake), '--parallel', '2'], check=True)
    (BUILD / 'model.json').write_text(json.dumps(spec, allow_nan=False))
    return BUILD / 'libcasbot_nmpc_runtime.so'
