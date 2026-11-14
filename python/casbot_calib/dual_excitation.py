"""Consistent analytic q/dq/ddq/jerk references; no measured acceleration side channel."""
from time import perf_counter
import numpy as np


def stratified_harmonics(
    n=14,
    *,
    validation=False,
    proximal_train=(1, 2, 3, 5),
    proximal_val=(1, 3, 4, 6),
    distal_train=(2, 4, 6, 9),
    distal_val=(2, 5, 7, 10),
):
    """Assign higher harmonic bands to distal forearm/wrist axes for acceleration SNR."""
    if n <= 0:
        raise ValueError('Positive joint count required')
    prox = np.asarray(proximal_val if validation else proximal_train, dtype=int)
    dist = np.asarray(distal_val if validation else distal_train, dtype=int)
    if prox.ndim != 1 or dist.ndim != 1 or len(prox) == 0 or len(prox) != len(dist) or np.any(prox <= 0) or np.any(dist <= 0):
        raise ValueError('Expected matching positive 1D harmonic tuples')
    out = np.empty((n, len(prox)), dtype=int)
    for j in range(n):
        out[j] = dist if (j % 7) >= 4 else prox
    return out


def _harmonic_weights(count):
    if count == 4:
        w = np.array([1.0, 0.6, 0.35, 0.2], dtype=float)
    else:
        w = 1.0 / (1.0 + np.arange(count, dtype=float)) ** 1.2
    return w / w.sum()


def periodic_reference(
    t,
    center,
    period_s=4.0,
    amplitude=0.1,
    seed=27,
    validation=False,
    active='both',
    *,
    phases=None,
    harmonics_override=None,
    amplitude_scales=None,
):
    t = np.asarray(t)
    center = np.asarray(center)
    n = len(center)
    rng = np.random.default_rng(seed)

    if harmonics_override is None:
        harmonics_1d = np.array([1, 3, 4, 6] if validation else [1, 2, 3, 5])
        harmonics_mat = None
        k_count = len(harmonics_1d)
    else:
        h_arr = np.asarray(harmonics_override, dtype=float)
        if h_arr.ndim == 1:
            harmonics_mat = np.broadcast_to(h_arr[None, :], (n, len(h_arr))).copy()
        elif h_arr.ndim == 2 and h_arr.shape[0] == n:
            harmonics_mat = h_arr.copy()
        else:
            raise ValueError('Invalid harmonics_override shape')
        if harmonics_mat.shape[1] == 0 or not np.isfinite(harmonics_mat).all() or np.any(harmonics_mat <= 0):
            raise ValueError('Positive finite harmonics required')
        k_count = harmonics_mat.shape[1]

    weights = _harmonic_weights(k_count)
    default_phase = rng.uniform(-np.pi, np.pi, (n, k_count))
    amplitude = np.broadcast_to(np.asarray(amplitude, dtype=float), (n,))
    if not np.isfinite(amplitude).all() or np.any(amplitude < 0):
        raise ValueError('Invalid per-joint excitation amplitude')
    jitter = rng.uniform(0.7, 1.2, (n, k_count))

    if phases is None:
        phase = default_phase
    else:
        phase = np.asarray(phases, dtype=float)
        if phase.shape != (n, k_count) or not np.isfinite(phase).all():
            raise ValueError('Invalid excitation phases shape')

    if amplitude_scales is None:
        amplitudes = amplitude[:, None] * jitter * weights
    else:
        scales = np.asarray(amplitude_scales, dtype=float)
        if not np.isfinite(scales).all() or np.any(scales < 0):
            raise ValueError('Invalid excitation amplitude_scales')
        if scales.shape == (n, k_count):
            amplitudes = amplitude[:, None] * scales
        elif scales.shape == (n,):
            amplitudes = (amplitude * scales)[:, None] * jitter * weights
        else:
            raise ValueError('Invalid excitation amplitude_scales shape')

    if active == 'left':
        amplitudes = amplitudes.copy()
        amplitudes[7:] = 0.0
    elif active == 'right':
        amplitudes = amplitudes.copy()
        amplitudes[:7] = 0.0
    elif active != 'both':
        raise ValueError('Unknown active arm group')

    q = np.broadcast_to(center, (len(t), n)).copy()
    v = np.zeros_like(q)
    a = v.copy()
    jerk = v.copy()

    if harmonics_mat is None:
        for k, harmonic in enumerate(harmonics_1d):
            w = 2 * np.pi * harmonic / period_s
            angle = w * t[:, None] + phase[:, k]
            q += amplitudes[:, k] * np.sin(angle)
            v += amplitudes[:, k] * w * np.cos(angle)
            a -= amplitudes[:, k] * w * w * np.sin(angle)
            jerk -= amplitudes[:, k] * w ** 3 * np.cos(angle)
    else:
        for k in range(k_count):
            w = 2.0 * np.pi * harmonics_mat[:, k] / period_s
            angle = t[:, None] * w[None, :] + phase[None, :, k]
            ak = amplitudes[:, k]
            q += ak * np.sin(angle)
            v += (ak * w) * np.cos(angle)
            a -= (ak * w * w) * np.sin(angle)
            jerk -= (ak * w ** 3) * np.cos(angle)
    return q, v, a, jerk


def optimize_excitation_phases(
    engine,
    profile,
    center,
    *,
    period_s=4.0,
    amplitude=0.18,
    harmonics=None,
    active='both',
    validation=False,
    seed=27,
    clearance_m=0.01,
    velocity_limits=None,
    acceleration_limit=8.0,
    jerk_limit=120.0,
    sample_points=48,
    popsize=12,
    max_generations=18,
    rank_rtol=1e-6,
):
    """Synthesize collision-cleared, URDF-feasible phases minimizing log condition number."""
    import cma
    from .rigid_identification import parameter_scales

    started = perf_counter()
    joints = engine.model.joints
    n = len(joints)
    center = np.asarray(center, dtype=float)
    if center.shape != (n,) or not np.isfinite(center).all():
        raise ValueError('Expected finite center matching model joint count')
    if period_s <= 0 or acceleration_limit <= 0 or jerk_limit <= 0 or sample_points < 8 or popsize < 4 or max_generations < 1:
        raise ValueError('Invalid excitation optimization settings')

    if harmonics is None:
        harmonics_mat = stratified_harmonics(n, validation=validation)
    else:
        h_arr = np.asarray(harmonics)
        if h_arr.ndim == 1:
            harmonics_mat = np.broadcast_to(h_arr[None, :], (n, len(h_arr))).copy()
        elif h_arr.ndim == 2 and h_arr.shape[0] == n:
            harmonics_mat = h_arr.copy()
        else:
            raise ValueError('Invalid harmonics specification')
    if harmonics_mat.shape[1] == 0 or not np.isfinite(harmonics_mat).all() or np.any(harmonics_mat <= 0):
        raise ValueError('Positive finite harmonics required')
    k_count = harmonics_mat.shape[1]

    amp_vec = np.broadcast_to(np.asarray(amplitude, dtype=float), (n,)).copy()
    if not np.isfinite(amp_vec).all() or np.any(amp_vec < 0):
        raise ValueError('Invalid excitation amplitude')

    if isinstance(active, (list, tuple, np.ndarray)):
        active_joints = np.asarray(active, dtype=int).ravel()
        if len(active_joints) == 0 or np.any((active_joints < 0) | (active_joints >= n)) or len(set(active_joints.tolist())) != len(active_joints):
            raise ValueError('Invalid active joint index list')
    elif active == 'both':
        active_joints = np.arange(n, dtype=int)
    elif active == 'left':
        active_joints = np.arange(min(7, n), dtype=int)
    elif active == 'right':
        active_joints = np.arange(7, n, dtype=int)
    else:
        raise ValueError('Unknown active arm group')
    active_joints = active_joints[amp_vec[active_joints] > 0.0]
    if len(active_joints) == 0:
        raise ValueError('At least one active joint must have positive excitation amplitude')

    urdf_vel = np.array([float(jt.velocity_limit) for jt in joints], dtype=float)
    if velocity_limits is None:
        vel_max = np.minimum(1.5, urdf_vel)
    else:
        vel_max = np.minimum(np.broadcast_to(np.asarray(velocity_limits, dtype=float), (n,)), urdf_vel)
    if not np.isfinite(vel_max).all() or np.any(vel_max <= 0):
        raise ValueError('Positive finite velocity limits required')

    rng = np.random.default_rng(seed)
    init_phases = rng.uniform(-np.pi, np.pi, (n, k_count))
    jitter = rng.uniform(0.7, 1.2, (n, k_count))
    weights = _harmonic_weights(k_count)
    amplitudes = amp_vec[:, None] * jitter * weights
    inactive_mask = np.ones(n, dtype=bool)
    inactive_mask[active_joints] = False
    amplitudes[inactive_mask] = 0.0

    omega = 2.0 * np.pi * harmonics_mat.astype(float) / float(period_s)
    safety = 0.92
    for j in active_joints:
        jt = joints[j]
        lo, hi = float(jt.lower), float(jt.upper)
        if not (lo < center[j] < hi):
            raise ValueError(f'Excitation center for {jt.name} outside URDF travel')
        raw_margin = min(center[j] - lo, hi - center[j])
        margin_q = min(center[j] - (lo + 0.02), (hi - 0.02) - center[j])
        if margin_q <= 0.0:
            margin_q = 0.9 * raw_margin
        row_a = amplitudes[j]
        total_a = float(row_a.sum())
        if total_a <= 0.0:
            continue
        w_j = omega[j]
        cap = min(
            1.0,
            margin_q / total_a,
            safety * vel_max[j] / float(np.sum(row_a * w_j)),
            safety * acceleration_limit / float(np.sum(row_a * (w_j ** 2))),
            safety * jerk_limit / float(np.sum(row_a * (w_j ** 3))),
        )
        amplitudes[j] = row_a * cap

    amplitude_scales = np.zeros_like(amplitudes)
    nonzero_amp = amp_vec > 0.0
    amplitude_scales[nonzero_amp] = amplitudes[nonzero_amp] / amp_vec[nonzero_amp, None]

    t_grid = np.linspace(0.0, float(period_s), int(sample_points), endpoint=False)
    rigid_scales = parameter_scales(engine.rigid_parameters())
    n_act = len(active_joints)
    rotor_scales = np.full(n_act, 0.02, dtype=float)
    full_scales = np.r_[rigid_scales, rotor_scales]
    act_arange = np.arange(n_act, dtype=int)

    def _expand_phases(flat_list):
        m = len(flat_list)
        ph = np.broadcast_to(init_phases[None, :, :], (m, n, k_count)).copy()
        ph[:, active_joints, :] = np.asarray(flat_list, dtype=float).reshape(m, n_act, k_count)
        return ph

    def _synth_batch(flat_list):
        ph = _expand_phases(flat_list)
        angles = t_grid[None, :, None, None] * omega[None, None, :, :] + ph[:, None, :, :]
        q = center[None, None, :] + np.sum(amplitudes[None, None, :, :] * np.sin(angles), axis=-1)
        dq = np.sum((amplitudes * omega)[None, None, :, :] * np.cos(angles), axis=-1)
        ddq = -np.sum((amplitudes * (omega ** 2))[None, None, :, :] * np.sin(angles), axis=-1)
        return q.reshape(-1, n), dq.reshape(-1, n), ddq.reshape(-1, n)

    x0 = init_phases[active_joints].ravel().copy()
    probe_rng = np.random.default_rng(int(seed) + 10007)
    probe_candidates = [x0, *probe_rng.uniform(-np.pi, np.pi, (3, len(x0)))]
    qp, dqp, ddqp = _synth_batch(probe_candidates)
    reg_p = engine.regressor_batch(qp, dqp, ddqp)['regressor'][:, active_joints, :]
    rot_p = np.zeros((len(qp), n_act, n_act), dtype=float)
    rot_p[:, act_arange, act_arange] = ddqp[:, active_joints]
    w_probe = np.concatenate([reg_p, rot_p], axis=2).reshape(-1, len(full_scales)) * full_scales
    _, s_probe, vh_probe = np.linalg.svd(w_probe, full_matrices=False)
    r_obs = int(np.count_nonzero(s_probe > max(float(s_probe[0]) * rank_rtol, 1e-12)))
    if r_obs == 0:
        raise ValueError('Excitation amplitude produces zero observable dynamic rank')
    v_obs = vh_probe[:r_obs].T

    def _evaluate_population(flat_list):
        m = len(flat_list)
        pts = len(t_grid)
        qa, dqa, ddqa = _synth_batch(flat_list)
        reg_all = engine.regressor_batch(qa, dqa, ddqa)['regressor'][:, active_joints, :].reshape(m, pts, n_act, -1)
        if profile is not None:
            dists_all = profile.check(qa)['distances'].reshape(m, pts, -1)
        else:
            dists_all = None

        losses = []
        conds = []
        dmins = []
        for idx in range(m):
            dmin = float(dists_all[idx].min()) if dists_all is not None else np.inf
            dmins.append(dmin)
            rot_blk = np.zeros((pts, n_act, n_act), dtype=float)
            rot_blk[:, act_arange, act_arange] = ddqa[idx * pts:(idx + 1) * pts, active_joints]
            wm = (np.concatenate([reg_all[idx], rot_blk], axis=2).reshape(pts * n_act, -1) * full_scales) @ v_obs
            sv = np.linalg.svd(wm, compute_uv=False)
            cond = float(sv[0] / max(float(sv[-1]), 1e-15))
            conds.append(cond)
            if dmin < clearance_m:
                losses.append(1e4 + 1e5 * (clearance_m - dmin) + 1e-3 * float(np.log(max(cond, 1.0))))
            else:
                losses.append(float(np.log(cond) - 0.05 * np.mean(np.log(np.maximum(sv, 1e-15)))))
        return losses, conds, dmins

    init_losses, init_conds, init_dmins = _evaluate_population([x0])
    best_x = x0.copy()
    best_loss = init_losses[0]
    best_cond = init_conds[0]
    best_dmin = init_dmins[0]

    opts = {
        'seed': int(seed),
        'popsize': int(popsize),
        'verbose': -9,
        'CMA_diagonal': True,
    }
    es = cma.CMAEvolutionStrategy(x0, 0.8, opts)
    generations_run = 0
    for _ in range(int(max_generations)):
        candidates = es.ask()
        losses, conds, dmins = _evaluate_population(candidates)
        es.tell(candidates, losses)
        generations_run += 1
        k_best = int(np.argmin(losses))
        if losses[k_best] < best_loss:
            best_loss = losses[k_best]
            best_cond = conds[k_best]
            best_dmin = dmins[k_best]
            best_x = np.asarray(candidates[k_best], dtype=float).copy()

    best_phases = init_phases.copy()
    wrapped = (best_x.reshape(n_act, k_count) + np.pi) % (2.0 * np.pi) - np.pi
    best_phases[active_joints] = wrapped
    return dict(
        phases=best_phases,
        amplitudes=amplitudes,
        amplitude_scales=amplitude_scales,
        harmonics=harmonics_mat,
        observable_rank=r_obs,
        initial_condition_number=float(init_conds[0]),
        optimized_condition_number=float(best_cond),
        minimum_distance_m=float(best_dmin),
        clearance_m=float(clearance_m),
        generations=generations_run,
        wall_s=perf_counter() - started,
    )


def direction_reference(t, center, amplitude=0.45, slot_s=6.0, initial_hold_s=3.0):
    """14 sequential + / - / return jogs, C3-continuous at every waypoint."""
    t = np.asarray(t)
    center = np.asarray(center)
    n = len(center)
    q = np.broadcast_to(center, (len(t), n)).copy()
    v = np.zeros_like(q)
    a = v.copy()
    jerk = v.copy()
    if slot_s <= 0 or initial_hold_s < 0 or amplitude <= 0:
        raise ValueError('Invalid direction jog configuration')
    knots = np.array([0.0, 0.2, 0.6, 0.8, 1.0]) * slot_s
    values = np.array([0.0, amplitude, -amplitude, 0.0, 0.0])
    for j in range(n):
        local = t - initial_hold_s - j * slot_s
        for k in range(4):
            mask = (local >= knots[k]) & (local < knots[k + 1])
            span = knots[k + 1] - knots[k]
            u = (local[mask] - knots[k]) / span
            delta = values[k + 1] - values[k]
            q[mask, j] += values[k] + delta * (35 * u ** 4 - 84 * u ** 5 + 70 * u ** 6 - 20 * u ** 7)
            v[mask, j] = delta * (140 * u ** 3 - 420 * u ** 4 + 420 * u ** 5 - 140 * u ** 6) / span
            a[mask, j] = delta * (420 * u ** 2 - 1680 * u ** 3 + 2100 * u ** 4 - 840 * u ** 5) / span ** 2
            jerk[mask, j] = delta * (840 * u - 5040 * u ** 2 + 8400 * u ** 3 - 4200 * u ** 4) / span ** 3
    return q, v, a, jerk


def gravity_reference(t, center, hold_s=2.0, poses=8, seed=41):
    """Separate static holds joined by quintic transitions; only settled holds are fitted."""
    rng = np.random.default_rng(seed)
    center = np.asarray(center)
    targets = center + rng.uniform(-0.25, 0.25, (poses, len(center)))
    q = np.zeros((len(t), len(center)))
    v = q.copy()
    a = q.copy()
    jerk = q.copy()
    for index, time in enumerate(t):
        pose = min(int(time / hold_s), poses - 1)
        local = time - pose * hold_s
        start = center if pose == 0 else targets[pose - 1]
        delta = targets[pose] - start
        transition = 0.5
        if local >= transition:
            q[index] = targets[pose]
            continue
        u = local / transition
        q[index] = start + delta * (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5)
        v[index] = delta * (30 * u * u - 60 * u ** 3 + 30 * u ** 4) / transition
        a[index] = delta * (60 * u - 180 * u * u + 120 * u ** 3) / transition ** 2
        jerk[index] = delta * (60 - 360 * u + 360 * u * u) / transition ** 3
    return q, v, a, jerk
