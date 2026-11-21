"""Mouse release and proximity feedback for the two flange handles."""
import numpy as np
from scipy.spatial.transform import Rotation


TETHER_M = .10
SETTLE_S = .4


def tether(position, actual):
    delta = position - actual
    return actual + delta * min(1., TETHER_M / max(np.linalg.norm(delta), 1e-12))


class ReleaseSnap:
    def __init__(self):
        self.dragging = False
        self.anchor = None
        self.since = 0.

    def update(self, active, pose, now):
        if active:
            self.dragging = True
            self.anchor = None
        elif self.dragging:
            self.dragging = False
            self.anchor = pose.copy()
            self.since = now
        elif self.anchor is not None:
            motion = np.linalg.norm(pose[:3, 3] - self.anchor[:3, 3])
            rotation = Rotation.from_matrix(pose[:3, :3].T @ self.anchor[:3, :3]).magnitude()
            if motion > .0015 or rotation > np.deg2rad(1.):
                self.anchor = pose.copy()
                self.since = now
            elif now - self.since >= SETTLE_S:
                self.anchor = None
                return True
        return False


class LimitIndicators:
    """Return the nearest joint limit and collision pair for each handle."""
    def __init__(self, ctl):
        parents = np.array([g['parent'] for g in ctl.spec['geometries']])
        pairs = np.array([p['indices'] for p in ctl.spec['pairs']])
        arms = parents[pairs] // 7
        self.by_arm = [np.flatnonzero((arms == arm).any(axis=1)) for arm in range(2)]
        self.clearance_window = np.minimum(.002, .5*np.maximum(0., ctl.gaps-ctl.planning_margin))

    def read(self, ctl, q, gaps):
        span = ctl.upper - ctl.lower
        near = np.minimum(np.minimum(q, ctl.x[:14]) - ctl.lower,
                          ctl.upper - np.maximum(q, ctl.x[:14])) / span
        slack = np.minimum(ctl.gaps, gaps) - ctl.planning_margin
        result = []
        for arm, pairs in enumerate(self.by_arm):
            joint = 7*arm + int(np.argmin(near[7*arm:7*arm+7]))
            active = pairs[slack[pairs] <= self.clearance_window[pairs]]
            pair = int(active[np.argmin(slack[active])]) if active.size else -1
            result.extend([joint if near[joint] <= .05 else -1, pair])
        return result
