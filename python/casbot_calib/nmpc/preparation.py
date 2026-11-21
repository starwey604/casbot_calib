"""The calibration ready motion with a continuous clearance gate for the fixed assembly."""
import numpy as np

from ..collision import CollisionProfile, certify_reference
from ..shm_ready import preparation, handover_fraction
from ..staged_motion import derivative_bounds
from .model import fixture_profile


class ReadyMotion:
    def __init__(self, ctx, start, ctl):
        self.motion, self.certificate = preparation(start, ctx.engine, min(ctl.velocity))
        self.start_hold_end = self.motion.windows('ready_start_hold')[0][1]
        self.certificate['feedforward'] = (
            'ramp gravity at stationary start, gravity during unfolding, '
            'smooth handover to selected feedforward at ready'
        )
        self.curve = self.motion.polynomial()
        full = fixture_profile(ctx)
        fixture = {i for i, name in enumerate(full.names) if name.startswith(('fixed:', 'ball:', 'stem:'))}
        pairs = {(i, j): ctl.margin[k] for k, (i, j) in enumerate(full.pairs) if i in fixture or j in fixture}
        excluded = [(i,j) for i in range(len(full.geometry)) for j in range(i+1,len(full.geometry))
                    if (i,j) not in pairs]
        self.fixture = CollisionProfile(full.engine, full.geometry, excluded, full.metadata)
        if set(self.fixture.pairs) != pairs.keys() or not pairs:
            raise ValueError('准备段工装碰撞对不完整')
        self.fixture.pair_clearance_m = np.array([pairs[p] for p in self.fixture.pairs])
        self.certificate['collision_check'] = 'continuous fixed-assembly clearance'
        self.proof = certify_reference(self.fixture, self.curve, self.motion.duration,
                                       derivative_bounds(self.curve).max(axis=0),
                                       clearance_m=float(min(self.fixture.pair_clearance_m)))
        if not self.proof['passed']:
            raise ValueError(f'准备轨迹未通过工装认证：{self.proof["witness"]}')

    def fields(self, t, ff, cfg):
        state = np.concatenate([self.curve(t, nu=i) for i in range(3)])
        blend = handover_fraction(round(t*1e9), self.certificate)
        # The stationary inverse-dynamics term supports the arm during unfolding.
        u = min(t / self.start_hold_end, 1.)
        gravity_scale = u**3 * (10. + u * (-15. + 6.*u))
        gravity = ff.effort(np.r_[state[:14], np.zeros(28)])
        effort = gravity_scale * gravity
        if blend:
            effort += blend * (ff.effort(state) - effort)
        kp = 20 + blend*(np.asarray(cfg['kp'])-20)
        kd = 2 + blend*(np.asarray(cfg['kd'])-2)
        return state, effort, kp.tolist(), kd.tolist()

    def measured_fixture_slack(self, measured):
        """Return worst measured clearance slack against the fixture (m); warning-only."""
        return float(np.min(self.fixture.check(measured)['distances'][0] - self.fixture.pair_clearance_m))
