"""Local SQP-RTI proposals, continuous path checks and confirmed reference state."""
import ctypes as ct
import json
from time import perf_counter

import numpy as np

from ..model_context import load_context
from ..project_paths import ROOT
from .homing import plan_ready_return
from .model import fixture_profile, stopping_maps

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
    def __init__(self, spec, mode, q):
        self.count = len(spec['pairs'])
        self.n = spec['config']['horizon_steps']
        self.lib = ct.CDLL(str(BUILD / 'libcasbot_nmpc_runtime.so'))
        lib = self.lib
        if lib.nmpc_pair_count() != self.count:
            raise ValueError('Rebuild NMPC for the fixed fixture scene')
        lib.nmpc_create_at.argtypes = [ct.c_int, PTR, ct.POINTER(ct.c_int)]
        lib.nmpc_create_at.restype = ct.c_void_p
        lib.nmpc_physical_limits.argtypes = [ct.c_void_p, PTR, PTR, PTR]
        lib.nmpc_physical_limits.restype = None
        lib.nmpc_destroy.argtypes = [ct.c_void_p]
        lib.nmpc_destroy.restype = None
        lib.nmpc_orientation_weight.argtypes = [ct.c_void_p, ct.c_double]
        lib.nmpc_orientation_weight.restype = None
        lib.nmpc_query.argtypes = lib.nmpc_solve.argtypes = [ct.c_void_p] + [PTR]*4
        lib.nmpc_certify.argtypes = [ct.c_void_p, PTR, PTR, ct.c_int, ct.c_double, PTR, PTR, PTR]
        flags = np.empty(self.count, dtype=np.intc)
        self.handle = lib.nmpc_create_at(mode == 'hardware', pointer(q), flags.ctypes.data_as(ct.POINTER(ct.c_int)))
        self.convex = flags.astype(bool)
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


def motion_radii(spec, profile, convex):
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
        if not convex[i]:
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
        profile = fixture_profile(ctx)
        if self.spec.get('scene_geometry_sha256') != profile.metadata['geometry_sha256']:
            raise ValueError('Rebuild NMPC with the 1105_03 fixture')
        initial_q = np.ascontiguousarray(self.spec['q0'] if q is None else q, dtype=float).reshape(14)
        if not np.isfinite(initial_q).all():
            raise ValueError('Non-finite initial joint state')
        self.native = Native(self.spec, mode, initial_q)
        self.dt = cfg['dt']
        self.n = cfg['horizon_steps']
        self.tail = cfg['stopping_tail_steps']
        self.jerk_limit = cfg['jerk_rad_s3']
        self.acc_limit = cfg['acceleration_rad_s2']
        self.velocity = np.asarray(self.spec['velocity'])
        self.lower, self.upper = np.asarray(self.spec['bounds']).T
        self.margin = np.array([p['modes'][mode]['pair_clearance_m'] for p in self.spec['pairs']])
        self.radii = motion_radii(self.spec, profile, self.native.convex)
        self.tail_map, _ = stopping_maps(self.tail, self.dt)
        self.x = np.r_[initial_q, np.zeros(28)]
        self.backup = np.zeros((self.n+self.tail, 14))
        self.pending = None
        self.return_segments = None
        self.gaps, _, self.poses = self.native.query(self.x[:14], flange=True)
        self.reference_lower, self.reference_upper = self.lower, self.upper
        self.planning_margin = self.margin
        if physical:
            shm = cfg['shm']
            self.velocity = np.minimum(self.velocity, shm['reference_velocity_rad_s'])
            q0 = np.asarray(self.spec['q0'], dtype=float)
            q0_gaps = self.native.query(q0)[0]
            # Keep the extra tracking reserve from making ready pose q0 unreachable.
            clearance_ref = np.where(q0_gaps > self.margin, np.minimum(self.gaps, q0_gaps), self.gaps)
            self.planning_margin = self.margin + np.minimum(shm['tracking_clearance_m'],
                                                             .4*(clearance_ref-self.margin).clip(min=0))
            reserve = shm['tracking_joint_reserve_rad']
            lo_ref = np.where(q0 > self.lower, np.minimum(self.x[:14]-self.lower, q0-self.lower), self.x[:14]-self.lower)
            hi_ref = np.where(q0 < self.upper, np.minimum(self.upper-self.x[:14], self.upper-q0), self.upper-self.x[:14])
            self.reference_lower = self.lower + np.minimum(reserve, .4*lo_ref.clip(min=0))
            self.reference_upper = self.upper - np.minimum(reserve, .4*hi_ref.clip(min=0))
            acc = np.full(14, self.acc_limit)
            self.native.lib.nmpc_physical_limits(self.native.handle,
                pointer(np.r_[self.reference_lower, -self.velocity, -acc]),
                pointer(np.r_[self.reference_upper, self.velocity, acc]), pointer(self.planning_margin))
        if not self.check(self.backup):
            self.close()
            raise ValueError('Initial posture violates joint or collision limits')
        self.mode = 'paused'
        self.check_ms = self.solve_ms = self.total_ms = 0.
        self.track_orientation = True

    def set_orientation_tracking(self, enabled):
        if enabled != self.track_orientation:
            weight = self.spec['config']['weights']['orientation'] if enabled else 0.
            self.native.lib.nmpc_orientation_weight(self.native.handle, weight)
            self.track_orientation = enabled

    def close(self):
        self.native.close()

    def _check_from(self, x, gaps, jerks, dt=None):
        dt = self.dt if dt is None else dt
        states = rollout(x, jerks, dt)
        v, a = states[:, 14:28], states[:, 28:]
        low, high = position_hull(states[:-1], states[1:], dt)
        # All quadratic velocity and cubic position values are in their Bernstein hulls.
        if (not np.isfinite(states).all() or np.any(np.abs(jerks) > self.jerk_limit+1e-9)
                or np.any(np.abs(a) > self.acc_limit+1e-9)
                or np.any(np.abs(v) > self.velocity+1e-9)
                or np.any(np.abs(v[:-1]+dt*a[:-1]/2) > self.velocity+1e-9)
                or np.any(low < self.reference_lower-1e-10) or np.any(high > self.reference_upper+1e-10)
                or np.max(np.abs(states[-1, 14:])) > 1e-8):
            return False
        slack = gaps - self.planning_margin
        if np.any(slack <= 0):
            return False
        # First certify the entire horizon and suffix against one set of distance witnesses.
        q = x[:14]
        delta = np.maximum(q-low.min(axis=0), high.max(axis=0)-q)
        if np.all(slack > self.radii @ delta):
            return True
        status = self.native.lib.nmpc_certify(
            self.native.handle, pointer(np.ascontiguousarray(states)), pointer(np.ascontiguousarray(jerks)),
            len(jerks), dt, pointer(np.ascontiguousarray(gaps)), pointer(self.radii), pointer(self.planning_margin))
        if status < 0:
            raise RuntimeError('Continuous distance query failed')
        return bool(status)

    def check(self, jerks, dt=None):
        return self._check_from(self.x, self.gaps, jerks, dt)

    def check_rest_segment(self, q_start, jerks, dt=None):
        q_start = np.ascontiguousarray(q_start, dtype=float).reshape(14)
        gaps = self.native.query(q_start)[0]
        return self._check_from(np.r_[q_start, np.zeros(28)], gaps, jerks, dt)

    def _blend(self, delta, left_scale, right_scale):
        trial = self.backup.copy()
        if left_scale:
            trial[:, :7] += left_scale * delta[:, :7]
        if right_scale:
            trial[:, 7:] += right_scale * delta[:, 7:]
        return trial

    def _select_candidate(self, candidate):
        """Accept full step, or decouple arms so a blocked arm never freezes the other."""
        if self.check(candidate):
            return candidate, 'ok'
        delta = candidate - self.backup
        has_l = float(np.max(np.abs(delta[:, :7]))) > 1e-12
        has_r = float(np.max(np.abs(delta[:, 7:]))) > 1e-12
        if not has_l and not has_r:
            return self.backup, 'backup'
        sub_scales = (.5, .25, .125, .0625, .03125)
        left_full = has_l and self.check(self._blend(delta, 1., 0.))
        right_full = has_r and self.check(self._blend(delta, 0., 1.))
        if left_full and not right_full:
            if has_r:
                for sr in (.5, .25, .125, .0625):
                    trial = self._blend(delta, 1., sr)
                    if self.check(trial):
                        return trial, 'damped'
            return self._blend(delta, 1., 0.), 'damped'
        if right_full and not left_full:
            if has_l:
                for sl in (.5, .25, .125, .0625):
                    trial = self._blend(delta, sl, 1.)
                    if self.check(trial):
                        return trial, 'damped'
            return self._blend(delta, 0., 1.), 'damped'
        if left_full and right_full:
            for s in sub_scales:
                trial = self._blend(delta, s, s)
                if self.check(trial):
                    return trial, 'damped'
            norm_l = float(np.linalg.norm(delta[:, :7]))
            norm_r = float(np.linalg.norm(delta[:, 7:]))
            return (self._blend(delta, 1., 0.) if norm_l >= norm_r else self._blend(delta, 0., 1.)), 'damped'
        # Neither arm passed at scale 1.0: find each arm's own largest safe scale.
        best_l = 0.
        if has_l:
            for s in sub_scales:
                if self.check(self._blend(delta, s, 0.)):
                    best_l = s
                    break
        best_r = 0.
        if has_r:
            for s in sub_scales:
                if self.check(self._blend(delta, 0., s)):
                    best_r = s
                    break
        if best_l > 0. and best_r > 0.:
            for sl, sr in ((best_l, best_r), (min(best_l, best_r), min(best_l, best_r)),
                           (best_l, .5*best_r), (.5*best_l, best_r)):
                trial = self._blend(delta, sl, sr)
                if self.check(trial):
                    return trial, 'damped'
            norm_l = float(np.linalg.norm(best_l * delta[:, :7]))
            norm_r = float(np.linalg.norm(best_r * delta[:, 7:]))
            return (self._blend(delta, best_l, 0.) if norm_l >= norm_r else self._blend(delta, 0., best_r)), 'damped'
        if best_l > 0.:
            return self._blend(delta, best_l, 0.), 'damped'
        if best_r > 0.:
            return self._blend(delta, 0., best_r), 'damped'
        return self.backup, 'backup'

    def _solve_candidate(self, nominal, targets):
        control = self.native.solve(nominal, self.backup[:self.n]/self.jerk_limit, targets)
        if control is None or not np.isfinite(control).all():
            return None
        jerk = control*self.jerk_limit
        final = rollout(self.x, jerk, self.dt)[-1]
        suffix = self.tail_map @ final.reshape(3, 14)
        return np.vstack([jerk, suffix])

    def propose_ready(self):
        if self.return_segments is None:
            if np.max(abs(self.x[14:])) > 1e-10 or np.max(abs(self.backup)) > 1e-10:
                return self.propose(self.poses, stop=True)
            segments = plan_ready_return(self)
            if segments is not None:
                self.return_segments = segments

        plan = self.backup
        self.mode = 'home_blocked'
        if self.return_segments is not None:
            if not self.return_segments:
                self.mode = 'home_done'
                self.pending = (self.x.copy(), self.backup)
                return self.pending[0].copy()
            seg = self.return_segments[0]
            trial = np.zeros_like(self.backup)
            if len(seg) <= len(self.backup):
                trial[:len(seg)] = seg
            else:
                trial[0] = seg[0]
                first = advance(self.x, trial[0], self.dt)
                trial[1:1+self.tail] = self.tail_map @ first.reshape(3, 14)
            if self.check(trial):
                self.mode = 'homing'
                plan = trial
        self.pending = (advance(self.x, plan[0], self.dt), plan)
        return self.pending[0].copy()

    def propose(self, targets, *, stop=False, return_ready=False):
        if self.pending is not None:
            raise RuntimeError('Previous NMPC command awaits confirmation')
        if return_ready:
            return self.propose_ready()
        self.return_segments = None
        start = perf_counter()
        targets = np.asarray(targets, dtype=float)
        if targets.shape != (2, 4, 4) or not np.isfinite(targets).all():
            raise ValueError('Expected two finite flange targets')
        candidate = None
        nominal = None
        if not stop:
            nominal = rollout(self.x, self.backup[:self.n], self.dt)
            candidate = self._solve_candidate(nominal, targets)
        solved = perf_counter()
        self.mode = 'stopping' if stop else 'backup'
        plan = self.backup
        if candidate is not None:
            plan, self.mode = self._select_candidate(candidate)
        elif not stop and nominal is not None:
            # If coupled QP failed, isolate each arm with the other held at its current pose.
            for free_arm in (0, 1):
                iso_targets = self.poses.copy()
                iso_targets[free_arm] = targets[free_arm]
                iso_cand = self._solve_candidate(nominal, iso_targets)
                if iso_cand is not None:
                    single = self.backup.copy()
                    s = slice(7*free_arm, 7*free_arm+7)
                    single[:, s] = iso_cand[:, s]
                    plan, self.mode = self._select_candidate(single)
                    if self.mode != 'backup':
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
        if self.mode == 'homing':
            self.return_segments[0] = self.return_segments[0][1:]
            if not len(self.return_segments[0]):
                self.return_segments.pop(0)
            if not self.return_segments:
                self.return_segments = None
                self.mode = 'home_done'
        elif self.mode == 'home_blocked':
            self.return_segments = None
        self.gaps, _, self.poses = self.native.query(self.x[:14], flange=True)
