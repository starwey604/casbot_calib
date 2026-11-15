"""Wave 2A verification & benchmarks: H1 dual-arm 2-opt pose chain, compact 5ms FF, vectorized analysis, and paired hysteresis separation."""
import json
from pathlib import Path
from time import perf_counter
import numpy as np
import pytest

from casbot_calib.dual_model import nominal_dual_arm
from casbot_calib.field_gravity import _approach_observation, write_gravity_candidate
from casbot_calib.model_runtime import model_for
from casbot_calib.pipeline.robot import load_robot_config
from casbot_calib.rigid_identification import fit_rigid, model_from_parameters
from casbot_calib.shm_data import (
    COMMAND_RECORD_DTYPE,
    DIRECTION,
    FEEDBACK_RECORD_DTYPE,
    NAMES,
    OFFSET,
)
from casbot_calib.shm_ready import ready_position
from casbot_calib.stages.dynamics.h1_gravity import (
    TimeMean,
    _trapezoid_time_mean,
    analyse_h1,
    load_h1_recipe,
    plan_h1,
)


def test_h1_recipe_defaults_and_programmatic_inheritance():
    rec_small = load_h1_recipe()
    assert rec_small['mode'] == 'small'
    assert rec_small['small']['parallel_arms'] is True
    assert rec_small['small']['direct_pose_chain'] is True

    rec_full = load_h1_recipe(mode='full')
    assert rec_full['mode'] == 'full'
    assert rec_full['full']['approach_offset_deg'] == pytest.approx(5.0)
    assert rec_full['full']['parallel_arms'] is True
    assert rec_full['full']['direct_pose_chain'] is True

    # Programmatic dict without explicit flags inherits True, and explicit False is respected.
    prog_small = load_h1_recipe(dict(mode='small', small=dict(joints=['left_shoulder_pitch_joint'])))
    assert prog_small['small']['parallel_arms'] is True
    assert prog_small['small']['direct_pose_chain'] is True

    prog_legacy = load_h1_recipe(
        dict(mode='small', small=dict(parallel_arms=False, direct_pose_chain=False))
    )
    assert prog_legacy['small']['parallel_arms'] is False
    assert prog_legacy['small']['direct_pose_chain'] is False


def test_h1_full_offline_planning_benchmark(tmp_path):
    robot_cfg = load_robot_config()
    recipe = load_h1_recipe(mode='full')
    q0 = ready_position()

    plan_dir = tmp_path / 'h1_full_plan'
    t0 = perf_counter()
    meta = plan_h1(q0, robot_cfg, recipe, plan_dir)
    wall_s = perf_counter() - t0

    duration_s = meta['duration_ns'] * 1e-9
    ref_path = plan_dir / 'reference.json'
    ref_bytes = ref_path.stat().st_size
    ref_mb = ref_bytes / (1024.0 * 1024.0)
    gate = json.loads((plan_dir / 'collision_reference.json').read_text(encoding='utf-8'))
    ref_bundle = json.loads(ref_path.read_text(encoding='utf-8'))

    print(
        f"\n[BENCHMARK H1 full] wall_s={wall_s:.3f}s, duration_s={duration_s:.2f}s, "
        f"reference_mb={ref_mb:.3f}MB, min_dist_mm={meta['minimum_reference_distance_m']*1e3:.2f}mm, "
        f"trials={len(meta['trials'])}, effort_rows={len(ref_bundle['effort'])}"
    )

    assert 350.0 <= duration_s <= 450.0, f'H1 full duration {duration_s:.2f}s outside [350, 450]s'
    assert ref_bytes < int(1.8 * 1024 * 1024), f'reference.json is {ref_mb:.3f} MB (>= 1.8 MB)'
    assert gate['passed'] is True
    assert meta['minimum_reference_distance_m'] >= 0.01

    # Verify effort table monotonicity and boundary invariants.
    eff = ref_bundle['effort']
    assert eff[0]['t_ns'] == 0
    assert all(0 <= r['t_ns'] <= meta['duration_ns'] for r in eff)
    assert all(b['t_ns'] > a['t_ns'] for a, b in zip(eff, eff[1:]))

    # Verify train and validation splits were never cross-paired.
    train_trials = [t for t in meta['trials'] if t['conditions']['batch'] == 'train']
    val_trials = [t for t in meta['trials'] if t['conditions']['batch'] == 'validation']
    assert len(train_trials) >= 48  # 24 paired poses * 2 approaches (or more if any split)
    assert len(val_trials) >= 16    # 8 paired poses * 2 approaches
    for t in meta['trials']:
        pose_id = t['conditions']['pose']
        assert '__' in pose_id or pose_id.startswith(('left_', 'right_'))

    for motion in meta['static_motions']:
        conditions = motion['conditions']
        arm = conditions['active_arm_group']
        expected = list(range(14)) if arm == 'both' else list(range(7)) if arm == 'left' else list(range(7, 14))
        assert conditions['approach_joints'] == expected
        k = motion['labels'].index('paired_approach')
        displacement = np.array(motion['states'][k + 1][0]) - motion['states'][k][0]
        wanted = np.zeros(14)
        wanted[expected] = -conditions['approach'] * np.deg2rad(5.0)
        np.testing.assert_allclose(displacement, wanted, atol=1e-14)
    assert wall_s < 8.0, f'H1 full planning took {wall_s:.3f}s (>= 8.0s)'


def test_h1_small_and_smoke_offline_planning_benchmarks(tmp_path):
    robot_cfg = load_robot_config()
    q0 = ready_position()

    # 1. Default 2-joint small mode
    rec_small = load_h1_recipe(mode='small')
    dir_small = tmp_path / 'small_2joint'
    t0 = perf_counter()
    meta_small = plan_h1(q0, robot_cfg, rec_small, dir_small)
    wall_small = perf_counter() - t0
    dur_small = meta_small['duration_ns'] * 1e-9
    gate_small = json.loads((dir_small / 'collision_reference.json').read_text(encoding='utf-8'))

    print(
        f"\n[BENCHMARK H1 small] wall_s={wall_small:.3f}s, duration_s={dur_small:.2f}s, "
        f"trials={len(meta_small['trials'])}, min_dist_mm={meta_small['minimum_reference_distance_m']*1e3:.2f}mm"
    )

    assert gate_small['passed'] is True
    assert meta_small['minimum_reference_distance_m'] >= 0.01
    assert 90.0 <= dur_small <= 135.0, f'H1 small duration {dur_small:.2f}s outside [90, 135]s'
    assert len(meta_small['trials']) == 4  # 1 paired train + 1 paired val * 2 approaches

    # 2. Single-joint small mode (unpaired fallback when only left arm has joints)
    rec_1j = load_h1_recipe(
        dict(mode='small', small=dict(joints=['left_shoulder_pitch_joint']))
    )
    dir_1j = tmp_path / 'small_1joint'
    meta_1j = plan_h1(q0, robot_cfg, rec_1j, dir_1j)
    gate_1j = json.loads((dir_1j / 'collision_reference.json').read_text(encoding='utf-8'))
    assert gate_1j['passed'] is True
    assert len(meta_1j['trials']) == 4

    # 3. Smoke mode
    rec_smoke = load_h1_recipe(mode='smoke')
    dir_smoke = tmp_path / 'smoke_plan'
    meta_smoke = plan_h1(q0, robot_cfg, rec_smoke, dir_smoke)
    gate_smoke = json.loads((dir_smoke / 'collision_reference.json').read_text(encoding='utf-8'))
    assert gate_smoke['passed'] is True
    assert meta_smoke['mode'] == 'smoke'


def _compare_time_mean_cases(obs_ns, seg, ambig, values):
    tm = TimeMean()
    for i in range(len(obs_ns)):
        row = dict(
            observation_time_ns=int(obs_ns[i]),
            clock_segment=int(seg[i]),
            time_ambiguous=bool(ambig[i]),
        )
        tm.add(row, values[i])
    expected_mean = tm.result()
    got = _trapezoid_time_mean(obs_ns, seg, ambig, values)

    assert got['rows'] == tm.rows
    assert got['groups'] == tm.groups
    assert got['first'] == tm.first
    assert got['last'] == tm.last
    assert got['maximum_gap_ns'] == tm.maximum_gap_ns
    assert got['error'] == tm.error
    if tm.low is None:
        assert got['low'] is None and got['high'] is None
    else:
        np.testing.assert_allclose(got['low'], tm.low, rtol=0, atol=1e-15)
        np.testing.assert_allclose(got['high'], tm.high, rtol=0, atol=1e-15)
    if expected_mean is None:
        assert got['mean'] is None
    else:
        assert got['mean'] is not None
        np.testing.assert_allclose(got['mean'], expected_mean, rtol=0, atol=1e-14)


def test_trapezoid_time_mean_exact_equivalence_with_streaming_timemean():
    rng = np.random.default_rng(2026)

    # 1. Empty
    _compare_time_mean_cases(
        np.empty(0, dtype=np.int64),
        np.empty(0, dtype=np.int64),
        np.empty(0, dtype=bool),
        np.empty((0, 14), dtype=np.float64),
    )

    # 2. Single timestamp group with duplicates -> None mean
    _compare_time_mean_cases(
        np.array([5000, 5000, 5000], dtype=np.int64),
        np.zeros(3, dtype=np.int64),
        np.zeros(3, dtype=bool),
        rng.normal(size=(3, 14)),
    )

    # 3. Non-uniform gaps + random duplicate multiplicities across 500 rows
    base_times = np.cumsum(rng.integers(500_000, 7_000_000, size=120))
    repeats = rng.integers(1, 6, size=120)
    obs_ns = np.repeat(base_times, repeats)
    vals = rng.normal(loc=1.5, scale=3.0, size=(len(obs_ns), 28))
    _compare_time_mean_cases(
        obs_ns,
        np.full(len(obs_ns), 2, dtype=np.int64),
        np.zeros(len(obs_ns), dtype=bool),
        vals,
    )

    # 4. Observation time reversal (with duplicates after reversal)
    rev_ns = np.array([10, 20, 20, 15, 20, 35], dtype=np.int64)
    _compare_time_mean_cases(
        rev_ns,
        np.zeros(6, dtype=np.int64),
        np.zeros(6, dtype=bool),
        rng.normal(size=(6, 14)),
    )

    # 5. Clock segment crossing
    _compare_time_mean_cases(
        np.array([10, 20, 30, 40], dtype=np.int64),
        np.array([0, 0, 1, 1], dtype=np.int64),
        np.zeros(4, dtype=bool),
        rng.normal(size=(4, 14)),
    )

    # 6. Ambiguity flag + mixed multi-error priority ordering
    _compare_time_mean_cases(
        np.array([10, 20, 30, 25, 40], dtype=np.int64),
        np.array([0, 1, 1, 1, 1], dtype=np.int64),
        np.array([False, False, True, False, False]),
        rng.normal(size=(5, 14)),
    )
    _compare_time_mean_cases(
        np.array([10, 20, 15, 30], dtype=np.int64),
        np.array([0, 0, 0, 1], dtype=np.int64),
        np.array([False, True, False, False]),
        rng.normal(size=(4, 14)),
    )


def test_vectorized_analyse_h1_on_binary_capture(tmp_path):
    robot_cfg = load_robot_config(dict(initial_hold_s=0.05, final_hold_s=0.05, anchor_transition_s=0.1))
    recipe = load_h1_recipe(dict(
        mode='small',
        ready_pose=False,
        small=dict(
            joints=['left_shoulder_pitch_joint', 'right_shoulder_pitch_joint'],
            target_offset_deg=5.0,
            approach_offset_deg=2.0,
            timing=dict(
                initial_hold_s=0.05,
                minimum_transition_s=0.2,
                minimum_paired_approach_s=0.2,
                settle_s=0.1,
                measure_s=0.15,
                final_hold_s=0.05,
            ),
            planning_envelope=dict(speed_rad_s=0.05, acceleration_rad_s2=0.1, jerk_rad_s3=1.0),
        ),
    ))
    q0 = ready_position()
    plan_dir = tmp_path / 'plan'
    plan_meta = plan_h1(q0, robot_cfg, recipe, plan_dir)

    motion_meta = json.loads((plan_dir / 'motion.json').read_text(encoding='utf-8'))
    from scipy.interpolate import BPoly
    curve = BPoly.from_derivatives(motion_meta['knots_s'], np.asarray(motion_meta['states']))
    engine, _ = model_for(robot_cfg)

    duration_ns = int(plan_meta['duration_ns'])
    n_samples = duration_ns // 1_000_000 + 1
    origin_mono = 1_000_000_000
    offset_ns = 500_000_000  # source_ms * 1e6 - offset_ns == obs_ns

    t_s = np.arange(n_samples, dtype=np.float64) * 1e-3
    q_traj = np.asarray(curve(np.minimum(t_s, motion_meta['knots_s'][-1])), dtype=np.float64)
    dq_traj = np.asarray(curve(np.minimum(t_s, motion_meta['knots_s'][-1]), nu=1), dtype=np.float64)
    tau_traj = engine.gravity_batch(q_traj)

    cap_dir = tmp_path / 'capture' / 'native'
    cap_dir.mkdir(parents=True)

    # Write result.txt, clock_pairs.csv, events.csv, commands.bin, feedback.bin
    (cap_dir / 'result.txt').write_text(
        f'execution_completed=1\nrecording_stopped=0\n'
        f'first_shm_write_mono_ns={origin_mono - 2_000_000}\n'
        f'first_command_mono_ns={origin_mono}\n',
        encoding='utf-8',
    )
    (cap_dir / 'clock_pairs.csv').write_text(
        f'clock_segment,mono_before_ns,wall_ns,mono_after_ns,offset_ns,jump_detected\n'
        f'0,{origin_mono - 5_000_000},{origin_mono + offset_ns},{origin_mono - 4_000_000},{offset_ns},0\n',
        encoding='utf-8',
    )
    (cap_dir / 'events.csv').write_text(
        f'mono_ns,phase,event\n{origin_mono},3,trajectory_begin\n',
        encoding='utf-8',
    )

    cmds = np.zeros(1, dtype=COMMAND_RECORD_DTYPE)
    cmds['send_mono_ns'][0] = origin_mono
    cmds.tofile(cap_dir / 'commands.bin')

    direction = np.asarray(DIRECTION, dtype=np.float64)[None, :]
    offset = np.asarray(OFFSET, dtype=np.float64)[None, :]
    recs = np.zeros(n_samples, dtype=FEEDBACK_RECORD_DTYPE)
    obs_ns = origin_mono + np.arange(n_samples, dtype=np.int64) * 1_000_000
    recs['receive_mono_ns'] = obs_ns + 100_000
    recs['frame']['timestamp'] = (obs_ns + offset_ns) // 1_000_000
    recs['frame']['frame_counter'] = np.arange(1, n_samples + 1, dtype=np.uint32)
    recs['frame']['joints'][:, 12:26]['position'] = np.trunc(direction * (q_traj - offset) * 1e6).astype(np.int32)
    recs['frame']['joints'][:, 12:26]['velocity'] = np.trunc(direction * dq_traj * 1e6).astype(np.int32)
    recs['frame']['joints'][:, 12:26]['torque'] = np.trunc(direction * tau_traj * 100.0).astype(np.int16)
    recs.tofile(cap_dir / 'feedback.bin')

    analysis_dir = tmp_path / 'analysis'
    report = analyse_h1(plan_dir, tmp_path / 'capture', analysis_dir)
    assert report['passed'] and report['fit_succeeded'] and report['analysis_available']
    assert report['feedback_rows'] == n_samples
    windows = json.loads((analysis_dir / 'static_windows.json').read_text(encoding='utf-8'))
    assert len(windows) == len(plan_meta['trials'])
    assert all(w['available'] and w['paired_approach']['available'] for w in windows)


@pytest.mark.parametrize('undithered', [False, True])
def test_paired_hysteresis_separation_beats_naive_averaging_and_handles_deadband(tmp_path, undithered):
    robot_cfg = load_robot_config()
    nominal, source = model_for(robot_cfg)
    model = nominal.model
    model.gravity_world = np.asarray(robot_cfg['gravity_world'], dtype=float)
    nominal = type(nominal)(model)

    # Perturb true rigid parameters substantially across both arms so nominal gravity has ~0.5+ Nm error.
    rng = np.random.default_rng(20261115)
    theta_nom = nominal.rigid_parameters().reshape(14, 10).copy()
    theta_true = theta_nom.copy()
    scales = rng.uniform(0.82, 1.24, size=14)
    for j in range(14):
        theta_true[j] *= scales[j]
        theta_true[j, 1:4] += theta_true[j, 0] * rng.uniform(-0.015, 0.015, size=3)
        theta_true[j, [4, 6, 9]] += theta_true[j, 0] * 0.003
    true_engine = model_from_parameters(nominal, theta_true.ravel())

    # True directional static friction hysteresis (Nm) across all 14 joints.
    c_true = np.array([
        0.60, 0.80, 0.45, 0.70, 0.35, 0.25, 0.20,
        0.65, 0.75, 0.50, 0.60, 0.30, 0.25, 0.20,
    ], dtype=float)

    home = np.deg2rad([0.0, 70.0, 0.0, -60.0, 0.0, 0.0, 0.0, 0.0, -70.0, 0.0, -60.0, 0.0, 0.0, 0.0])
    approach_ref_rad = np.deg2rad(5.0)
    means = []

    # Generate 28 dual-arm training poses and 10 clean validation poses.
    n_train = 28
    n_val = 10
    for split, count in (('train', n_train), ('validation', n_val)):
        for p in range(count):
            q_target = home + rng.uniform(-0.65, 0.65, size=14)
            # Keep within URDF limits with margin
            for j, jt in enumerate(nominal.model.joints):
                q_target[j] = np.clip(q_target[j], jt.lower + 0.15, jt.upper - 0.15)

            # Minus & plus windows stop 1.8°–2.2° apart around q_target!
            half_gap = np.deg2rad(rng.uniform(0.9, 1.1, size=14))
            q_minus = q_target - half_gap
            q_plus = q_target + half_gap

            for sign, q_win in ((-1, q_minus), (1, q_plus)):
                dq_ref = np.full(14, -sign * approach_ref_rad)
                # Normal approach moves ~4.0° in direction -sign
                dq_obs = np.full(14, -sign * np.deg2rad(4.0))
                s_phys = np.full(14, -float(sign))

                # Inject deadband-stuck event on training pose 0, approach +1, joint 0 & joint 1:
                # Joint failed to break stiction on reversal (|dq_obs| = 0.05° < 0.3°), retaining +c_true bias!
                if split == 'train' and p == 0 and sign == 1:
                    dq_obs[0] = -np.deg2rad(0.05)
                    dq_obs[1] = -np.deg2rad(0.05)
                    q_win = q_win.copy()
                    q_win[0] = q_minus[0]
                    q_win[1] = q_minus[1]
                    s_phys[0] = +1.0  # Unreversed residual stiction bias!
                    s_phys[1] = +1.0

                noise = rng.normal(0.0, 0.004, size=14) if split == 'train' else np.zeros(14)
                tau_win = true_engine.gravity(q_win) + s_phys * c_true + noise
                # Reproduce history-dependent pitch bias on unapproached axes,
                # with opposite biases in train and validation.
                if undithered and ((split == 'train' and 1 <= p <= 20) or (split == 'validation' and p < 6)):
                    dq_ref[[0, 7]] = 0.0
                    dq_obs[[0, 7]] = 0.0
                    tau_win[[0, 7]] = true_engine.gravity(q_win)[[0, 7]] + (-0.9 if split == 'train' else 0.9)

                means.append(dict(
                    name=f'static_{split}_{p:02d}_{"minus" if sign < 0 else "plus"}',
                    conditions=dict(
                        batch=split,
                        pose=f'pose_{split}_{p:02d}',
                        approach=sign,
                        active_arm_group='both',
                        active_joints=list(range(14)),
                        target_q_rad=q_target.tolist(),
                    ),
                    q=q_win.tolist(),
                    torque=tau_win.tolist(),
                    available=True,
                    paired_approach=dict(
                        available=True,
                        reference_displacement_rad=dq_ref.tolist(),
                        observed_displacement_rad=dq_obs.tolist(),
                    ),
                ))

    out_dir = tmp_path / 'hyst_fit'
    out_dir.mkdir()
    metadata = dict(joint_names=list(NAMES), source_model_sha256=source['sha256'])
    res = write_gravity_candidate(means, robot_cfg, out_dir, metadata)

    # Evaluate on an independent pure-gravity test set of 40 random poses (exact g(q), zero friction).
    q_holdout = home + rng.uniform(-0.6, 0.6, size=(40, 14))
    for j, jt in enumerate(nominal.model.joints):
        q_holdout[:, j] = np.clip(q_holdout[:, j], jt.lower + 0.1, jt.upper - 0.1)
    tau_holdout_true = true_engine.gravity_batch(q_holdout)
    y_holdout = nominal.regressor_batch(
        q_holdout, np.zeros_like(q_holdout), np.zeros_like(q_holdout)
    )['regressor']

    rmse_sep = float(np.sqrt(np.mean((y_holdout @ np.asarray(res['fit']['parameters']) - tau_holdout_true) ** 2)))
    rmse_nom = float(np.sqrt(np.mean((y_holdout @ nominal.rigid_parameters() - tau_holdout_true) ** 2)))

    # Compare with naive target-q regressor evaluation (evaluating Y at target_q instead of actual q_- / q_+,
    # and without hysteresis separation when deadband or single-window dropout occurs).
    naive_y = []
    naive_t = []
    train_by_pose = {}
    for row in means:
        if row['conditions']['batch'] == 'train':
            train_by_pose.setdefault(row['conditions']['pose'], []).append(row)
    for pose_idx, (_, rows) in enumerate(train_by_pose.items()):
        # Simulate realistic single-window availability on 4 poses + target_q regressor mismatch:
        usable = rows[:1] if pose_idx < 4 else rows
        q_tgt = np.asarray(usable[0]['conditions']['target_q_rad'], dtype=float)[None, :]
        naive_y.append(nominal.regressor_batch(q_tgt, np.zeros_like(q_tgt), np.zeros_like(q_tgt))['regressor'][0])
        naive_t.append(np.mean([r['torque'] for r in usable], axis=0))
    naive_fit = fit_rigid(np.array(naive_y), np.array(naive_t), nominal.rigid_parameters(), robust_scale=0.1)
    rmse_naive = float(np.sqrt(np.mean((y_holdout @ np.asarray(naive_fit['parameters']) - tau_holdout_true) ** 2)))

    # Also test write_gravity_candidate when those same 4 training poses have only 1 window available!
    means_with_dropouts = [
        r for r in means
        if not (r['conditions']['batch'] == 'train'
                and r['conditions']['pose'] in {f'pose_train_{k:02d}' for k in range(4)}
                and r['conditions']['approach'] == 1)
    ]
    out_drop = tmp_path / 'hyst_fit_drop'
    out_drop.mkdir()
    res_drop = write_gravity_candidate(means_with_dropouts, robot_cfg, out_drop, metadata)
    rmse_sep_drop = float(np.sqrt(np.mean((y_holdout @ np.asarray(res_drop['fit']['parameters']) - tau_holdout_true) ** 2)))

    hyst_est = np.asarray(res['static_hysteresis_nm'], dtype=float)
    hyst_mae = float(np.mean(np.abs(hyst_est - c_true)))
    cal_json = json.loads((out_dir / 'calibration.json').read_text(encoding='utf-8'))

    print(
        f"\n[BENCHMARK Hysteresis Separation] nominal_rmse={rmse_nom:.4f} Nm, "
        f"naive_rmse={rmse_naive:.4f} Nm, separated_rmse={rmse_sep:.5f} Nm, "
        f"separated_with_dropouts_rmse={rmse_sep_drop:.5f} Nm, hyst_mae={hyst_mae:.4f} Nm, "
        f"deadband_stuck={res['deadband_stuck_count']}"
    )

    assert res['passed'] is True
    assert res['validation_pairs_per_joint'][0] == (4 if undithered else 10)
    assert res['validation_pairs_per_joint'][7] == (4 if undithered else 10)
    assert res['deadband_stuck_count'] == 2
    assert cal_json['deadband_stuck_count'] == 2
    assert len(cal_json['static_hysteresis_nm']) == 14
    assert rmse_sep < 0.03, f'Pure gravity holdout RMSE {rmse_sep:.5f} Nm >= 0.03 Nm'
    assert rmse_sep_drop < 0.03, f'Pure gravity holdout RMSE with dropouts {rmse_sep_drop:.5f} Nm >= 0.03 Nm'
    assert rmse_sep_drop < 0.25 * rmse_naive
    np.testing.assert_allclose(hyst_est, c_true, atol=0.04)


def test_approach_observation_excludes_unmoved_wrong_direction_and_missing_feedback():
    row = dict(conditions=dict(approach=-1, active_joints=[0, 1, 2, 3]), paired_approach=dict(
        available=True,
        reference_displacement_rad=np.deg2rad([5, 5, 5, 0, 0] + [0] * 9).tolist(),
        observed_displacement_rad=np.deg2rad([4, 0.05, -4, 0, 1] + [0] * 9).tolist(),
    ))
    signs, stuck = _approach_observation(row)
    np.testing.assert_array_equal(signs, [1] + [0] * 13)
    assert np.flatnonzero(stuck).tolist() == [1]
    row['paired_approach']['available'] = False
    assert not _approach_observation(row)[0].any()
    # Older records still use their original planned-direction contract.
    del row['paired_approach']
    np.testing.assert_array_equal(_approach_observation(row)[0], [1] * 4 + [0] * 10)


def test_validation_pairs_require_both_opposite_observed_approaches(tmp_path, monkeypatch):
    from casbot_calib import rigid_identification
    robot_cfg = load_robot_config()
    engine, source = model_for(robot_cfg)
    q = ready_position()
    means = []
    for split in ('train', 'validation'):
        for pose in range(2):
            for approach in (-1, 1):
                row = dict(name=f'{split}_{pose}_{approach}', q=q.tolist(), torque=engine.gravity(q).tolist(),
                           conditions=dict(batch=split, pose=pose, approach=approach, active_joints=list(range(14))),
                           paired_approach=dict(available=True,
                               reference_displacement_rad=np.full(14, -approach * 0.08).tolist(),
                               observed_displacement_rad=np.full(14, -approach * 0.06).tolist()))
                if split == 'validation' and pose == 0 and approach == 1:
                    continue  # A single window must not become a gravity validation pair.
                if split == 'validation' and pose == 1 and approach == 1:
                    row['paired_approach']['observed_displacement_rad'][0] = 0.06
                means.append(row)

    fit_rigid = rigid_identification.fit_rigid

    def check_weights(*args, **kwargs):
        np.testing.assert_array_equal(kwargs['validation_weights'], [0] + [1] * 13)
        return fit_rigid(*args, **kwargs)

    monkeypatch.setattr(rigid_identification, 'fit_rigid', check_weights)
    result = write_gravity_candidate(means, robot_cfg, tmp_path,
                                     dict(joint_names=list(NAMES), source_model_sha256=source['sha256']))
    assert result['validation_poses'] == [1]
    assert result['validation_pairs_per_joint'] == [0] + [1] * 13
    assert result['validation_rmse_nm'][0] is None
    assert np.isfinite(result['overall_validation_rmse_nm'])
