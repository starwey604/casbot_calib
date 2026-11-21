"""Local SQP-RTI proposals, continuous path checks and confirmed reference state."""
import ctypes as ct
import json
from time import perf_counter

import numpy as np

from ..model_context import load_context
from ..project_paths import ROOT
from .model import stopping_maps

BUILD = ROOT / 'build/nmpc/dvt2_bare_flange'
PTR = ct.POINTER(ct.c_double)


def pointer(array):
    return None if array is None else array.ctypes.data_as(PTR)


def advance(x, jerk, t):
    q, v, a = np.split(x, 3, axis=-1)
    return np.concatenate((q + t*v + t*t*a/2 + t**3*jerk/6,
                           v + t*a + t*t*jerk/2, a + t*jerk), axis=-1)


def rollout(x, jerks, dt):
    acc = np.vstack((x[28:], x[28:] + dt*np.cumsum(jerks, axis=0)))
    dv = dt*(acc[:-1] + acc[1:])/2
    vel = np.vstack((x[14:28], x[14:28] + np.cumsum(dv, axis=0)))
    dq = dt*vel[:-1] + dt**2*(2*acc[:-1]+acc[1:])/6
    pos = np.vstack((x[:14], x[:14] + np.cumsum(dq, axis=0)))
    return np.concatenate((pos, vel, acc), axis=1)


def position_hull(x, y, dt):
    """Cubic Bernstein hull in position for an interval with endpoint states x,y."""
    q0, v0, _ = np.split(x, 3, axis=-1)
    q1, v1, _ = np.split(y, 3, axis=-1)
    controls = np.stack([q0, q0+dt*v0/3, q1-dt*v1/3, q1])
    return controls.min(axis=0), controls.max(axis=0)


class Native:
    def __init__(self, spec, mode):
        self.count = len(spec['pairs'])
        self.n = spec['config']['horizon_steps']
        self.lib = ct.CDLL(str(BUILD / 'libcasbot_nmpc_runtime.so'))
        lib = self.lib
        lib.nmpc_create_mode.argtypes = [ct.c_int]
        lib.nmpc_create_mode.restype = ct.c_void_p
        lib.nmpc_physical_limits.argtypes = [ct.c_void_p, PTR, PTR, PTR]
        lib.nmpc_physical_limits.restype = None
        lib.nmpc_destroy.argtypes = [ct.c_void_p]
        lib.nmpc_destroy.restype = None
        lib.nmpc_query.argtypes = lib.nmpc_solve.argtypes = [ct.c_void_p] + [PTR]*4
        lib.nmpc_certify.argtypes = [ct.c_void_p, PTR, PTR, ct.c_int, ct.c_double, PTR, PTR, PTR]
        self.handle = lib.nmpc_create_mode(mode == 'hardware')
        if not self.handle:
            raise RuntimeError('Native NMPC initialization failed')

    def close(self):
        if self.handle:
            self.lib.nmpc_destroy(self.handle)
            self.handle = None

    def query(self, q, *, gradient=False, flange=False):
        q = np.ascontiguousarray(q, dtype=float).reshape(14)
        gaps = np.empty(self.count)
        jac = np.empty((self.count, 14)) if gradient else None
        poses = np.empty((2, 4, 4)) if flange else None
        if self.lib.nmpc_query(self.handle, pointer(q), pointer(gaps), pointer(jac), pointer(poses)):
            raise RuntimeError('Native collision query failed')
        return gaps, jac, poses

    def solve(self, states, controls, poses):
        targets = np.concatenate([np.r_[p[:3, 3], p[:3, :3].ravel(order='F')] for p in poses])
        output = np.empty((self.n, 14))
        arrays = (states, controls, targets)
        inputs = [np.ascontiguousarray(a, dtype=float) for a in arrays]
        status = self.lib.nmpc_solve(self.handle, *(pointer(a) for a in inputs), pointer(output))
        return None if status else output


def motion_radii(spec, profile, mode):
    """Use CAD motion radii for hull pairs and the enclosing capsule radii for proxies."""
    body = np.zeros((len(spec['geometries']), 14))
    origins = np.array(spec['origins']).reshape(14, 4, 4)
    for i, geom in enumerate(spec['geometries']):
        parent = geom['parent']
        radius = geom['radius_m'] + np.linalg.norm(geom['endpoints'], axis=1).max()
        if parent < 0:
            continue
        for j in range(parent, (parent//7)*7-1, -1):
            body[i, j] = radius
            radius += np.linalg.norm(origins[j, :3, 3])
    radii = profile.radii.copy()
    for i, row in enumerate(spec['pairs']):
        if row['modes'][mode]['geometry'] == 'capsule':
            a, b = body[row['indices']]
            radii[i] = np.where((a > 0) & (b > 0), 0, a+b)
    return np.ascontiguousarray(radii)


class Controller:
    def __init__(self, q=None, *, mode='simulation', physical=False):
        self.spec = json.loads((BUILD/'model.json').read_text())
        cfg = self.spec['config']
        ctx = load_context()
        if ctx.identity != self.spec['model_context']:
            raise ValueError('Rebuild NMPC for the selected geometry')
        if mode not in ('simulation', 'hardware'):
            raise ValueError(f'Unknown collision policy {mode}')
        self.native = Native(self.spec, mode)
        self.dt = cfg['dt']
        self.n = cfg['horizon_steps']
        self.tail = cfg['stopping_tail_steps']
        self.jerk_limit = cfg['jerk_rad_s3']
        self.acc_limit = cfg['acceleration_rad_s2']
        self.velocity = np.asarray(self.spec['velocity'])
        self.lower, self.upper = np.asarray(self.spec['bounds']).T
        self.margin = np.array([p['modes'][mode]['pair_clearance_m'] for p in self.spec['pairs']])
        self.radii = motion_radii(self.spec, ctx.collision_profile(), mode)
        self.tail_map, _ = stopping_maps(self.tail, self.dt)
        self.x = np.r_[np.array(self.spec['q0'] if q is None else q).reshape(14), np.zeros(28)]
        self.backup = np.zeros((self.n+self.tail, 14))
        self.pending = None
        self.gaps, _, self.poses = self.native.query(self.x[:14], flange=True)
        self.reference_lower, self.reference_upper = self.lower, self.upper
        self.planning_margin = self.margin
        if physical:
            shm = cfg['shm']
            self.velocity = np.minimum(self.velocity, shm['reference_velocity_rad_s'])
            self.planning_margin = self.margin + np.minimum(shm['tracking_clearance_m'],
                                                             .4*(self.gaps-self.margin).clip(min=0))
            reserve = shm['tracking_joint_reserve_rad']
            self.reference_lower = self.lower + np.minimum(reserve, .4*(self.x[:14]-self.lower).clip(min=0))
            self.reference_upper = self.upper - np.minimum(reserve, .4*(self.upper-self.x[:14]).clip(min=0))
            acc = np.full(14, self.acc_limit)
            self.native.lib.nmpc_physical_limits(self.native.handle,
                pointer(np.r_[self.reference_lower, -self.velocity, -acc]),
                pointer(np.r_[self.reference_upper, self.velocity, acc]), pointer(self.planning_margin))
        if not self.check(self.backup):
            self.close()
            raise ValueError('Initial posture violates joint or collision limits')
        self.mode = 'paused'
        self.check_ms = self.solve_ms = self.total_ms = 0.

    def close(self):
        self.native.close()

    def check(self, jerks):
        states = rollout(self.x, jerks, self.dt)
        v, a = states[:, 14:28], states[:, 28:]
        low, high = position_hull(states[:-1], states[1:], self.dt)
        # All quadratic velocity and cubic position values are in their Bernstein hulls.
        if (not np.isfinite(states).all() or np.any(np.abs(jerks) > self.jerk_limit+1e-9)
                or np.any(np.abs(a) > self.acc_limit+1e-9)
                or np.any(np.abs(v) > self.velocity+1e-9)
                or np.any(np.abs(v[:-1]+self.dt*a[:-1]/2) > self.velocity+1e-9)
                or np.any(low < self.reference_lower-1e-10) or np.any(high > self.reference_upper+1e-10)
                or np.max(np.abs(states[-1, 14:])) > 1e-8):
            return False
        slack = self.gaps - self.planning_margin
        if np.any(slack <= 0):
            return False
        # First certify the entire horizon and suffix against one set of distance witnesses.
        q = self.x[:14]
        delta = np.maximum(q-low.min(axis=0), high.max(axis=0)-q)
        if np.all(slack > self.radii @ delta):
            return True
        status = self.native.lib.nmpc_certify(
            self.native.handle, pointer(np.ascontiguousarray(states)), pointer(np.ascontiguousarray(jerks)),
            len(jerks), self.dt, pointer(self.gaps), pointer(self.radii), pointer(self.planning_margin))
        if status < 0:
            raise RuntimeError('Continuous distance query failed')
        return bool(status)

    def propose(self, targets, *, stop=False):
        if self.pending is not None:
            raise RuntimeError('Previous NMPC command awaits confirmation')
        start = perf_counter()
        targets = np.asarray(targets, dtype=float)
        if targets.shape != (2, 4, 4) or not np.isfinite(targets).all():
            raise ValueError('Expected two finite flange targets')
        candidate = None
        if not stop:
            nominal = rollout(self.x, self.backup[:self.n], self.dt)
            control = self.native.solve(nominal, self.backup[:self.n]/self.jerk_limit, targets)
            if control is not None and np.isfinite(control).all():
                jerk = control*self.jerk_limit
                final = rollout(self.x, jerk, self.dt)[-1]
                suffix = self.tail_map @ final.reshape(3, 14)
                candidate = np.vstack([jerk, suffix])
        solved = perf_counter()
        self.mode = 'stopping' if stop else 'backup'
        plan = self.backup
        if candidate is not None:
            for scale in (1., .5, .25, .125, .0625, .03125):
                trial = self.backup + scale*(candidate-self.backup)
                if self.check(trial):
                    self.mode = 'ok' if scale == 1. else 'damped'
                    plan = trial
                    break
        # The backup is the remaining suffix of the last continuously certified plan.
        self.pending = (advance(self.x, plan[0], self.dt), plan)
        finished = perf_counter()
        self.solve_ms = 1000*(solved-start)
        self.check_ms = 1000*(finished-solved)
        self.total_ms = 1000*(finished-start)
        return self.pending[0].copy()

    def commit(self):
        if self.pending is None:
            raise RuntimeError('No NMPC command to confirm')
        self.x, plan = self.pending
        self.pending = None
        self.backup = np.vstack([plan[1:], np.zeros((1, 14))])
        self.gaps, _, self.poses = self.native.query(self.x[:14], flange=True)
