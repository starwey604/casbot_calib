"""Continuously certified joint-space homing planner for returning to ready pose."""
from time import perf_counter

import numpy as np


HOMING_VELOCITY_RAD_S = 0.45
HOMING_ACCEL_RAD_S2 = 1.0
HOMING_JERK_RAD_S3 = 4.0


def rest_to_rest_phases(q_start, q_end, ctl):
    """Four equal constant-jerk phases from rest to rest on the joint-space chord."""
    delta = np.asarray(q_end, dtype=float) - np.asarray(q_start, dtype=float)
    distance = float(np.max(np.abs(delta)))
    if distance < 1e-10:
        return None
    v_cap = np.minimum(ctl.velocity, HOMING_VELOCITY_RAD_S)
    a_cap = min(ctl.acc_limit, HOMING_ACCEL_RAD_S2)
    j_cap = min(ctl.jerk_limit, HOMING_JERK_RAD_S3)
    t = max(
        float(np.max(np.abs(delta) / (2. * v_cap))),
        float(np.sqrt(distance / (2. * a_cap))),
        float(np.cbrt(distance / (2. * j_cap))),
    )
    steps = max(1, int(np.ceil(t / ctl.dt)))
    duration = steps * ctl.dt
    phases = np.array([1., -1., -1., 1.])[:, None] * delta / (2. * duration**3)
    return phases, steps, duration


def _point_clear(ctl, q, extra_margin=0.):
    if np.any(q < ctl.reference_lower - 1e-10) or np.any(q > ctl.reference_upper + 1e-10):
        return False
    gaps = ctl.native.query(q)[0]
    return bool(np.all(gaps - ctl.planning_margin > extra_margin))


def chord_clear(ctl, q_a, q_b):
    """Continuously certify a rest-to-rest cubic chord directly via the native verifier."""
    built = rest_to_rest_phases(q_a, q_b, ctl)
    if built is None:
        return _point_clear(ctl, q_a)
    phases, _, duration = built
    return ctl.check_rest_segment(q_a, phases, duration)


def _fast_edge(ctl, q_a, q_b, extra_margin=0.):
    """Fast discrete pre-screen along a short tree edge; continuous certification runs on final paths."""
    delta = q_b - q_a
    n = max(2, int(np.ceil(float(np.max(np.abs(delta))) / .12)))
    for k in range(1, n + 1):
        if not _point_clear(ctl, q_a + (k / n) * delta, extra_margin):
            return False
    return True


def _clip(ctl, q):
    lo = ctl.reference_lower + np.minimum(1e-3, .1 * (ctl.reference_upper - ctl.reference_lower))
    hi = ctl.reference_upper - np.minimum(1e-3, .1 * (ctl.reference_upper - ctl.reference_lower))
    return np.clip(q, lo, hi)


def _boundary_backoffs(ctl, q_start):
    """Generate small repulsive nudges off tight collision or joint-limit boundaries."""
    gaps, jac, _ = ctl.native.query(q_start, gradient=True)
    slack = gaps - ctl.planning_margin
    order = np.argsort(slack)
    direction = np.zeros(14)
    for idx in order[:4]:
        if slack[idx] < .012:
            direction += jac[idx] / (max(float(slack[idx]), 0.) + 5e-4)
    # Also nudge inward from joints near bounds.
    lo_gap = q_start - ctl.reference_lower
    hi_gap = ctl.reference_upper - q_start
    direction += np.where(lo_gap < .04, 1. / (np.maximum(lo_gap, 0.) + .01), 0.)
    direction -= np.where(hi_gap < .04, 1. / (np.maximum(hi_gap, 0.) + .01), 0.)
    candidates = []
    for mask in (np.ones(14), np.r_[np.ones(7), np.zeros(7)], np.r_[np.zeros(7), np.ones(7)]):
        vec = direction * mask
        norm = float(np.linalg.norm(vec))
        if norm < 1e-9:
            continue
        unit = vec / norm
        for step in (.08, .18, .32):
            q_try = _clip(ctl, q_start + step * unit)
            if chord_clear(ctl, q_start, q_try):
                candidates.append(q_try)
                break
    return candidates


def _canonical_hubs(ctl, q_start):
    abduct_cur = q_start.copy()
    abduct_cur[1] = max(abduct_cur[1], .95)
    abduct_cur[8] = min(abduct_cur[8], -.95)

    unbend_first = q_start.copy()
    unbend_first[3] = max(unbend_first[3], -.35)
    unbend_first[10] = max(unbend_first[10], -.35)

    unbend_cur = abduct_cur.copy()
    unbend_cur[3] = max(unbend_cur[3], -.35)
    unbend_cur[10] = max(unbend_cur[10], -.35)
    unbend_cur[[4, 5, 6, 11, 12, 13]] = 0.

    open_straight = np.array([0., .95, 0., 0., 0., 0., 0., 0., -.95, 0., 0., 0., 0., 0.])
    open_wide = np.array([0., 1.35, 0., 0., 0., 0., 0., 0., -1.35, 0., 0., 0., 0., 0.])
    open_side = np.array([0., .95, 0., -.45, 0., 0., 0., 0., -.95, 0., -.45, 0., 0., 0.])
    open_front = np.array([-.45, 1.0, 0., -.35, 0., 0., 0., -.45, -1.0, 0., -.35, 0., 0., 0.])
    return [_clip(ctl, q) for q in (unbend_first, abduct_cur, unbend_cur, open_straight, open_wide, open_side, open_front)]


def _try_waypoints(ctl, waypoints):
    """Check whether every consecutive pair of waypoints is continuously chord-clear."""
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        if not chord_clear(ctl, a, b):
            return False
    return True


def _shortcut(ctl, path):
    """Greedily collapse intermediate waypoints whenever a direct certified chord exists."""
    result = [np.asarray(p, dtype=float) for p in path]
    i = 0
    while i + 2 < len(result):
        jumped = False
        for j in range(len(result) - 1, i + 1, -1):
            if chord_clear(ctl, result[i], result[j]):
                result = result[:i + 1] + result[j:]
                jumped = True
                break
        if not jumped:
            i += 1
    return result


def _birrt(ctl, starts, goal, hubs, deadline):
    """Seeded 14D BiRRT-Connect with single-arm move bias for entangled dual-arm poses."""
    rng = np.random.default_rng(20261121)
    lo = ctl.reference_lower + np.minimum(1e-3, .1 * (ctl.reference_upper - ctl.reference_lower))
    hi = ctl.reference_upper - np.minimum(1e-3, .1 * (ctl.reference_upper - ctl.reference_lower))

    tree_a = [starts[0]]
    parent_a = [-1]
    for q in starts[1:]:
        tree_a.append(q)
        parent_a.append(0)

    tree_b = [goal]
    parent_b = [-1]
    for hub in hubs:
        if chord_clear(ctl, hub, goal):
            tree_b.append(hub)
            parent_b.append(0)

    step_size = .45

    def nearest(tree, q):
        arr = np.asarray(tree)
        return int(np.argmin(np.sum((arr - q) ** 2, axis=1)))

    def steer(q_from, q_to):
        diff = q_to - q_from
        dist = float(np.linalg.norm(diff))
        if dist <= step_size:
            return q_to.copy()
        return q_from + (step_size / dist) * diff

    def trace(tree, parents, idx):
        chain = []
        while idx >= 0:
            chain.append(tree[idx])
            idx = parents[idx]
        chain.reverse()
        return chain

    def extend(tree, parents, target):
        idx = nearest(tree, target)
        q_new = steer(tree[idx], target)
        if not _fast_edge(ctl, tree[idx], q_new):
            return -1
        tree.append(q_new)
        parents.append(idx)
        return len(tree) - 1

    def connect(tree, parents, target):
        last = -1
        for _ in range(16):
            idx = nearest(tree, target)
            q_near = tree[idx]
            if float(np.linalg.norm(target - q_near)) <= step_size:
                if _fast_edge(ctl, q_near, target):
                    tree.append(target.copy())
                    parents.append(idx)
                    return len(tree) - 1, True
                return last, False
            nxt = extend(tree, parents, target)
            if nxt < 0:
                return last, False
            last = nxt
        return last, False

    swapped = False
    for it in range(500):
        if perf_counter() > deadline:
            break
        u = rng.random()
        if u < .20:
            q_rand = tree_b[int(rng.integers(len(tree_b)))]
        elif u < .55:
            # Move only one 7-DOF arm at a time to untangle crossed/near-body arms quickly.
            base = tree_a[int(rng.integers(len(tree_a)))].copy()
            arm = int(rng.integers(2))
            s = slice(7 * arm, 7 * arm + 7)
            base[s] = rng.uniform(lo[s], hi[s])
            q_rand = base
        else:
            q_rand = rng.uniform(lo, hi)

        added = extend(tree_a, parent_a, q_rand)
        if added >= 0:
            joined, reached = connect(tree_b, parent_b, tree_a[added])
            if reached:
                path_a = trace(tree_a, parent_a, added)
                path_b = trace(tree_b, parent_b, joined)
                path_b.reverse()
                full = path_a + path_b[1:]
                if swapped:
                    full.reverse()
                short = _shortcut(ctl, full)
                if _try_waypoints(ctl, short):
                    return short
        tree_a, tree_b = tree_b, tree_a
        parent_a, parent_b = parent_b, parent_a
        swapped = not swapped
    return None


def _certify_waypoints(ctl, waypoints):
    """Build and authoritatively certify rest-to-rest jerk segments for every waypoint edge."""
    segments = []
    for index, (q_a, q_b) in enumerate(zip(waypoints[:-1], waypoints[1:])):
        built = rest_to_rest_phases(q_a, q_b, ctl)
        if built is None:
            continue
        phases, steps, duration = built
        if index == 0:
            ok = ctl.check(phases, duration)
        else:
            ok = ctl.check_rest_segment(q_a, phases, duration)
        if not ok:
            return None
        segments.append(np.repeat(phases, steps, axis=0))
    return segments


def plan_ready_return(ctl, q_goal=None, *, timeout_s=.25):
    """Plan a continuously certified multi-segment rest-to-rest path from ctl.x[:14] to ready pose."""
    q_start = ctl.x[:14].copy()
    q_goal = np.asarray(ctl.spec['q0'] if q_goal is None else q_goal, dtype=float)
    if not ctl.check(ctl.backup):
        return None
    if float(np.max(np.abs(q_goal - q_start))) < 1e-10:
        return []
    if not _point_clear(ctl, q_goal):
        return None

    # Tier 1: Direct joint-space chord (fast path, also honors ctl.check monkeypatches).
    direct = _certify_waypoints(ctl, [q_start, q_goal])
    if direct is not None:
        return direct

    # If the direct chord failed at t=0 because check() rejected even a trivial motion
    # (e.g. start itself invalid or check overridden), avoid wasting search time.
    if not _point_clear(ctl, q_start):
        return None

    deadline = perf_counter() + timeout_s
    hubs = [h for h in _canonical_hubs(ctl, q_start) if _point_clear(ctl, h)]
    goal_hubs = [h for h in hubs if chord_clear(ctl, h, q_goal)]
    backoffs = _boundary_backoffs(ctl, q_start)
    start_prefixes = [[q_start]] + [[q_start, b] for b in backoffs]
    birrt_starts = [q_start] + list(backoffs)

    def accept(waypoints):
        short = _shortcut(ctl, waypoints)
        return _certify_waypoints(ctl, short)

    # Tier 2: Deterministic canonical hubs, 2-hub bridges (abduct -> open -> ready), and one-arm-first.
    for prefix in start_prefixes:
        if perf_counter() > deadline:
            break
        q_s = prefix[-1]
        if len(prefix) > 1 and chord_clear(ctl, q_s, q_goal):
            certified = accept(prefix + [q_goal])
            if certified is not None:
                return certified
        # 1-hub paths via hubs already certified to reach q_goal.
        for ghub in goal_hubs:
            if chord_clear(ctl, q_s, ghub):
                certified = accept(prefix + [ghub, q_goal])
                if certified is not None:
                    return certified
        # 2-hub bridges: move from q_s to a local hub (e.g. abduct_cur), then to a goal hub.
        local_hubs = [h for h in hubs if chord_clear(ctl, q_s, h)]
        if len(prefix) == 1:
            birrt_starts.extend(local_hubs)
        for lhub in local_hubs:
            for ghub in goal_hubs:
                if chord_clear(ctl, lhub, ghub):
                    certified = accept(prefix + [lhub, ghub, q_goal])
                    if certified is not None:
                        return certified
        # Sequential single-arm transitions (Left-first and Right-first).
        seq_candidates = [
            prefix + [np.r_[q_goal[:7], q_s[7:]], q_goal],
            prefix + [np.r_[q_s[:7], q_goal[7:]], q_goal],
        ]
        for ghub in goal_hubs[:2]:
            seq_candidates.extend([
                prefix + [np.r_[ghub[:7], q_s[7:]], ghub, q_goal],
                prefix + [np.r_[q_s[:7], ghub[7:]], ghub, q_goal],
                prefix + [np.r_[ghub[:7], q_s[7:]], np.r_[ghub[:7], q_goal[7:]], q_goal],
                prefix + [np.r_[q_s[:7], ghub[7:]], np.r_[q_goal[:7], ghub[7:]], q_goal],
            ])
        for lhub in local_hubs[:2]:
            for ghub in goal_hubs[:2]:
                seq_candidates.extend([
                    prefix + [lhub, np.r_[ghub[:7], lhub[7:]], ghub, q_goal],
                    prefix + [lhub, np.r_[lhub[:7], ghub[7:]], ghub, q_goal],
                ])
        for waypoints in seq_candidates:
            if _try_waypoints(ctl, waypoints):
                certified = accept(waypoints)
                if certified is not None:
                    return certified

    # Tier 3: Seeded 14D BiRRT-Connect + shortcutting.
    waypoints = _birrt(ctl, birrt_starts, q_goal, goal_hubs or hubs, deadline)
    if waypoints is not None:
        return _certify_waypoints(ctl, waypoints)
    return None
