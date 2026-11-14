"""A balance diagnostics shared across explicitly selected observation clocks."""
import numpy as np


def imu_projection(problem, imu, *, interval=None):
    """Sensitivity to measured base-motion regressors, not a force correction."""
    from .probe_projection import basis, exposure, least_squares
    t = problem['time_s']; j = problem['joint']
    lo, hi = interval if interval is not None else (float(t[0]), float(t[-1]+np.median(np.diff(t))))
    ti, values = imu
    if ti[0] > t[0] or ti[-1] < t[-1]:
        return dict(available=False, next_action='IMU未覆盖这个运动窗；下一段同时保留IMU与控制记录')
    use = (ti >= lo) & (ti < hi); ti = ti[use]; values = values[use]
    if len(ti) < 2 or not np.isfinite(values).all():
        return dict(available=False, next_action='该窗IMU有效值不足；检查记录中的协方差缺失标记和实际发布')
    duration = hi-lo
    harmonics = int(np.floor(problem['preprocessing']['frequency_band_hz'][1]*duration))
    try:
        x = basis(ti, lo, duration, harmonics)
        coefficients, _ = least_squares(x, values, exposure(ti))
    except (ValueError, np.linalg.LinAlgError) as exc:
        return dict(available=False, next_action='IMU不能支持本窗处理带宽：'+str(exc))
    u = t-t.mean(); a = problem['ddq'][:, j]
    nuisance = np.column_stack([problem['dq'][:, j], np.ones(len(t)), u, u*u])
    remove = lambda z: z-nuisance@np.linalg.lstsq(nuisance, z, rcond=1e-10)[0]
    ap = remove(a); y = remove(problem['target']-problem['projected_gravity_nm'])
    j0 = float(ap@y/(ap@ap)); rows = []
    for shift in (-.005, 0., .005):
        projected = basis(t+shift, lo, duration, harmonics)@coefficients
        alpha = basis(t+shift, lo, duration, harmonics, derivative=1)@coefficients[:, :3]
        omega = projected[:, :3]
        # Local base-motion nuisance: omega, alpha, specific force and
        # centrifugal products. All terms remain diagnostics, never subtracted.
        terms = np.column_stack([projected, alpha, *[omega[:, k]*omega[:, l] for k in range(3) for l in range(k, 3)]])
        z = remove(terms); scales = np.linalg.norm(z, axis=0)
        active = scales > 1e-10
        if not np.any(active):
            rows.append(dict(imu_offset_s=shift, inertia_kg_m2=j0, change_relative=0., nuisance_rank=0)); continue
        U, singular, _ = np.linalg.svd(z[:, active]/scales[active], full_matrices=False)
        rank = int(np.sum(singular > singular[0]*1e-8)); U = U[:, :rank]
        aq = ap-U@(U.T@ap); remaining = float(np.linalg.norm(aq)/np.linalg.norm(ap))
        if remaining < 1e-8:
            rows.append(dict(imu_offset_s=shift, unavailable='该窗身体运动与惯量激励共线', acceleration_remaining_fraction=remaining)); continue
        estimate = float(aq@y/(aq@aq))
        rows.append(dict(imu_offset_s=shift, inertia_kg_m2=estimate, change_relative=abs(estimate-j0)/max(abs(j0), 1e-6),
                         nuisance_rank=rank, acceleration_remaining_fraction=remaining))
    valid = [r for r in rows if 'change_relative' in r]
    return dict(available=len(valid)==len(rows), cases=rows,
        maximum_inertia_change_relative=max((r['change_relative'] for r in valid), default=None),
        interpretation='IMU局部干扰项的拟合敏感性；与惯量共线时调整激励频带或姿态，不将额外参数收益当作已修正力矩')


def window_a(obs, elapsed, interval, joint, profile, band, *, imu=None, observation_time_ns=None):
    from .probe_projection import probe_problem, inertia_balance
    from .model_runtime import model_for
    lo, hi = interval
    if not np.isfinite([lo, hi, band]).all() or lo >= hi or band <= 0:
        raise ValueError('窗口须START<END，带宽须为正数')
    use = (elapsed >= lo) & (elapsed < hi)
    if np.sum(use) < 2:
        raise ValueError('指定窗口内没有足够回调行')
    if lo < elapsed[0] or hi > elapsed[-1]:
        raise ValueError('窗口超出实际反馈覆盖范围')
    if not all(np.isfinite(obs[k][use]).all() for k in ('q', 'dq', 'effort_raw')):
        raise ValueError('窗口内q／dq／effort有缺失或非有限值')
    engine, _ = model_for(profile)
    model = engine.model; model.gravity_world = np.asarray(profile['gravity_world']); engine = type(engine)(model)
    if observation_time_ns is None:
        raise ValueError('需要主站观测时间observation_time_ns')
    config=dict(schema_version='casbot_shm_probe_v1',probe={},processing=dict(polynomial_degree=2),cache_period_ns=None)
    config['probe']['duration_s'] = hi-lo
    trim = min(.2, (hi-lo)/10)
    config['processing'].update(fit_band_hz=band, edge_trim_s=trim,
        quadrature_period_s=min(.002, 1/(10*band)))
    direction = 1 if np.mean(obs['dq'][use, joint]) >= 0 else -1
    motion = dict(knots_s=[lo, hi], labels=['probe_measure'], conditions=dict(joint=joint, direction=direction))
    problem = probe_problem(obs, elapsed, motion, engine, config, profile, observation_time_ns=observation_time_ns)
    # Rank is mathematical information, not a data-quality threshold.
    t = problem['time_s']-np.mean(problem['time_s'])
    a = problem['ddq'][:, joint]; v = problem['dq'][:, joint]
    nuisance = np.column_stack([v, np.ones(len(t)), t, t*t])
    x = np.column_stack([a, nuisance]); scale = np.linalg.norm(x, axis=0)
    if np.any(scale == 0) or np.linalg.matrix_rank(x/scale) < x.shape[1]:
        raise ValueError('该窗口的加速度与速度／慢漂移不可分离；改取含加减速的完整单向片段')
    base = inertia_balance(problem)
    a_perp = a-nuisance@np.linalg.lstsq(nuisance, a, rcond=1e-10)[0]
    independent_accel_rms = float(np.sqrt(np.mean(a_perp*a_perp)))
    changes = []
    for ms in (-5., -2., -1., 1., 2., 5.):
        if abs(ms)*.001 >= trim:
            continue
        shifted = probe_problem(obs, elapsed, motion, engine, config, profile, effort_relative_offset_s=ms*.001, observation_time_ns=observation_time_ns)
        fit = inertia_balance(shifted)
        changes.append(dict(extra_effort_advance_ms=ms, **fit,
            inertia_change_relative=float(abs(fit['inertia_kg_m2']-base['inertia_kg_m2'])/max(abs(base['inertia_kg_m2']), 1e-6))))
    q = obs['q'][use]; dq = obs['dq'][use]; held = [j for j in range(14) if j != joint]
    return dict(available=True, interval_s=list(interval), band_hz=band, baseline=base,
        positive_inertia=bool(base['inertia_kg_m2'] > 0),
        body_motion=imu_projection(problem, imu, interval=interval) if imu is not None else dict(available=False, next_action='下一轮同步记录IMU，以比较自平衡运动加入模型前后的惯量变化'),
        signal=dict(target_q_span_rad=float(np.ptp(q[:, joint])),
            target_velocity_min_rad_s=float(np.min(dq[:, joint])), target_velocity_max_rad_s=float(np.max(dq[:, joint])),
            held_maximum_velocity_rad_s=float(np.max(abs(dq[:, held]))),
            acceleration_after_nuisance_rms_rad_s2=independent_accel_rms,
            torque_rms_budget_for_10_percent_J_nm=.1*base['inertia_kg_m2']*independent_accel_rms if base['inertia_kg_m2'] > 0 else None),
        preprocessing=problem['preprocessing'], perturbations=changes,
        maximum_extra_5ms_inertia_change_relative=max(r['inertia_change_relative'] for r in changes if abs(r['extra_effort_advance_ms']) == 5.) if trim > .005 else None,
        interpretation=('固定名义重力下A局部诊断及通道额外错位敏感性；未估计实际滤波/时差' if observation_time_ns is not None else '固定名义重力下原A的局部诊断；额外错位敏感性与10%力矩预算，不是实测缓存年龄或真实误差上界'))
