"""Wave 2B verification & benchmarks: common.py feedforward/capture, H2 Stribeck sweep, and H3 CMA-ES + rotor inertia."""
import json
from pathlib import Path
from time import perf_counter
import numpy as np
import pytest
from scipy.interpolate import BPoly

from casbot_calib.dynamic_export import check_reload
from casbot_calib.model_runtime import model_for, urdf_for
from casbot_calib.pipeline.robot import load_robot_config
from casbot_calib.pipeline.session import Session
from casbot_calib.rigid_identification import model_from_parameters
from casbot_calib.shm_data import (
    COMMAND_RECORD_DTYPE,
    DIRECTION,
    FEEDBACK_RECORD_DTYPE,
    NAMES,
    OFFSET,
)
from casbot_calib.shm_ready import preparation, prepend, ready_position
from casbot_calib.staged_motion import Motion
from casbot_calib.stages.dynamics.common import (
    build_feedforward_effort_rows,
    build_gravity_effort_rows,
    read_verified_stage_capture,
    resolve_stage_prior,
)
from casbot_calib.stages.dynamics.h2_friction import (
    analyse_h2,
    load_h2_recipe,
    plan_h2,
    write_h2_summary_markdown,
)
from casbot_calib.stages.dynamics.h3_inertia import (
    analyse_h3,
    load_h3_recipe,
    plan_h3,
    write_h3_summary_markdown,
)


def _reconstruct_bpoly_from_reference(ref_bundle):
    knots_s = np.asarray(ref_bundle['knots_ns'], dtype=np.float64) * 1e-9
    states = np.asarray(ref_bundle['states'], dtype=np.float64)
    return BPoly.from_derivatives(knots_s, states)


def _write_synthetic_binary_capture(capture_dir: Path, ref_bundle: dict, q_fn, dq_fn, effort_nm_fn, robot_config: dict, *, sample_ns=1_000_000):
    """Write a binary SHM capture (feedback.bin + commands.bin + clock_pairs.csv + events.csv + result.txt)."""
    native_dir = capture_dir / 'native'
    native_dir.mkdir(parents=True, exist_ok=True)

    duration_ns = int(ref_bundle['knots_ns'][-1])
    dt_ns = sample_ns
    rel_ns = np.arange(0, duration_ns + dt_ns, dt_ns, dtype=np.int64)
    t_s = rel_ns * 1e-9
    n_Formal = len(rel_ns)

    q_arr = np.asarray(q_fn(t_s), dtype=np.float64)
    dq_arr = np.asarray(dq_fn(t_s), dtype=np.float64)
    eff_nm = np.asarray(effort_nm_fn(t_s, q_arr, dq_arr), dtype=np.float64)

    gain = np.asarray(robot_config['effort_gain'], dtype=np.float64)[None, :]
    zero = np.asarray(robot_config['effort_zero_nm'], dtype=np.float64)[None, :]
    eff_raw = eff_nm / gain + zero

    origin_mono_ns = 10**18 + 500_000_000
    wall_base_ms = 1_700_000_000_000
    clock_offset_ns = wall_base_ms * 1_000_000 - origin_mono_ns

    # Include 5 pre-stream phase-2 frames and 5 post-stream phase-4 frames.
    n_pre = 5
    n_post = 5
    total = n_pre + n_Formal + n_post

    k_idx = np.arange(-n_pre, n_Formal + n_post, dtype=np.int64)
    tick_mono = origin_mono_ns + k_idx * dt_ns
    recv_mono = tick_mono + 5_000
    stamp_ms = wall_base_ms + k_idx * (dt_ns // 1_000_000)

    q_full = np.vstack([
        np.repeat(q_arr[:1], n_pre, axis=0),
        q_arr,
        np.repeat(q_arr[-1:], n_post, axis=0),
    ])
    dq_full = np.vstack([
        np.zeros((n_pre, 14), dtype=np.float64),
        dq_arr,
        np.zeros((n_post, 14), dtype=np.float64),
    ])
    eff_full = np.vstack([
        np.repeat(eff_raw[:1], n_pre, axis=0),
        eff_raw,
        np.repeat(eff_raw[-1:], n_post, axis=0),
    ])

    direction = np.asarray(DIRECTION, dtype=np.float64)[None, :]
    offset = np.asarray(OFFSET, dtype=np.float64)[None, :]
    pos_wire = np.round(((q_full - offset) / direction) * 1e6).astype(np.int32)
    vel_wire = np.round((dq_full / direction) * 1e6).astype(np.int32)
    tau_wire = np.clip(np.round((eff_full / direction) * 100.0), -32767, 32767).astype(np.int16)

    fb = np.zeros(total, dtype=FEEDBACK_RECORD_DTYPE)
    fb['receive_mono_ns'] = recv_mono
    fb['frame']['frame_counter'] = np.arange(1, total + 1, dtype=np.uint64)
    fb['frame']['joint_count'] = 35
    fb['frame']['hand_count'] = 2
    fb['frame']['timestamp'] = stamp_ms.astype(np.uint64)
    fb['frame']['joints'][:, 12:26]['status_word'] = 0x0027
    fb['frame']['joints'][:, 12:26]['mode'] = 4
    fb['frame']['joints'][:, 12:26]['position'] = pos_wire
    fb['frame']['joints'][:, 12:26]['velocity'] = vel_wire
    fb['frame']['joints'][:, 12:26]['torque'] = tau_wire
    fb.tofile(native_dir / 'feedback.bin')

    cmds = np.zeros(total, dtype=COMMAND_RECORD_DTYPE)
    cmd_send = tick_mono + 20_000
    cmd_send[n_pre] = origin_mono_ns  # exact first phase-3 command origin
    cmds['send_mono_ns'] = cmd_send
    cmds['frame']['frame_counter'] = np.arange(1, total + 1, dtype=np.uint64)
    cmds['frame']['joint_count'] = 35
    cmds['frame']['hand_count'] = 2
    cmds['frame']['timestamp'] = stamp_ms.astype(np.uint64)
    cmds.tofile(native_dir / 'commands.bin')

    (native_dir / 'clock_pairs.csv').write_text(
        'mono_before_ns,wall_ns,mono_after_ns,offset_ns,clock_segment,jump_detected\n'
        f'{tick_mono[0] - 200_000},{stamp_ms[0] * 1_000_000 - 200_000},{tick_mono[0] - 190_000},{clock_offset_ns},0,0\n',
        encoding='utf-8',
    )
    p3_event_ns = origin_mono_ns - 1_000
    p4_event_ns = origin_mono_ns + n_Formal * dt_ns + 10_000
    (native_dir / 'events.csv').write_text(
        'mono_ns,phase,event\n'
        f'{tick_mono[0] - 50_000},2,enabling_at_start\n'
        f'{p3_event_ns},3,reference_begin\n'
        f'{p4_event_ns},4,disabling_after_reference\n',
        encoding='utf-8',
    )
    (native_dir / 'result.txt').write_text(
        f'execution_completed=1\n'
        f'recording_stopped=0\n'
        f'disable_ack=1\n'
        f'first_command_mono_ns={origin_mono_ns}\n'
        f'first_shm_write_mono_ns={cmd_send[0]}\n',
        encoding='utf-8',
    )


def test_common_feedforward_modes_prior_acceptance_and_binary_capture(tmp_path):
    robot_cfg = load_robot_config()
    nom_eng, _ = model_for(robot_cfg)
    params = np.asarray(nom_eng.rigid_parameters(), dtype=float)
    eng = model_from_parameters(
        nom_eng,
        params,
        viscous=np.full(14, 0.5),
        coulomb=np.full(14, 0.25),
        static=np.full(14, 0.4),
        stribeck_velocity=np.linspace(0.01, 0.04, 14),
        rotor_inertia=np.linspace(0.01, 0.07, 14),
    )

    q0 = np.zeros(14)
    q_ready = ready_position()
    prep, prep_cert = preparation(q0, eng, robot_cfg['maximum_velocity_rad_s'])
    formal = Motion('ff_test', 'inertia', q_ready)
    q_target = q_ready.copy()
    q_target[0] += 0.25
    q_target[7] += 0.25
    formal.add(1.5, q_target, label='swing')
    formal.add(1.5, q_ready, label='back')
    full_motion = prepend(prep, formal)

    rows_none = build_feedforward_effort_rows(full_motion, eng, robot_cfg, prep_cert, mode='none')
    assert rows_none == [dict(t_ns=0, value=[0.0] * 14)]
    assert build_gravity_effort_rows(full_motion, eng, robot_cfg, prep_cert, enabled=False) == rows_none

    rows_grav = build_feedforward_effort_rows(full_motion, eng, robot_cfg, prep_cert, mode='gravity', step_ns=5_000_000)
    rows_alias = build_gravity_effort_rows(full_motion, eng, robot_cfg, prep_cert, enabled=True)
    assert rows_grav == rows_alias

    rows_gf = build_feedforward_effort_rows(full_motion, eng, robot_cfg, prep_cert, mode='gravity_friction', step_ns=5_000_000)
    total_ms = round(full_motion.duration * 1e3)
    assert 10 < len(rows_gf) < total_ms // 4
    vals = np.array([r['value'] for r in rows_gf], dtype=float)
    assert np.isfinite(vals).all()
    assert float(np.max(np.abs(vals))) <= robot_cfg['feedforward_limit_nm']
    # Handover keeps t=0 strictly zero torque, while swing region differs from pure gravity due to viscous + soft-Coulomb.
    assert np.allclose(rows_gf[0]['value'], 0.0)
    assert not np.allclose(rows_gf[-5]['value'], rows_grav[-5]['value'])

    # Prior resolution: vector stribeck_velocity + rotor_inertia + accepted=False skip.
    out_root = tmp_path / 'sessions'
    sess = Session.create(
        output_root=out_root,
        robot_id='BOT_W2B',
        stages_requested=['h1', 'h2', 'h3'],
        model_context=robot_cfg['model_context'],
        plan_only=False,
    )
    h1_dir = sess.root / 'h1/analysis'
    h2_dir = sess.root / 'h2/analysis'
    h1_dir.mkdir(parents=True)
    h2_dir.mkdir(parents=True)

    vs_vec = np.linspace(0.008, 0.034, 14).tolist()
    ri_vec = np.linspace(0.02, 0.085, 14).tolist()
    h1_cal = dict(
        stage='h1',
        accepted=True,
        model_context=robot_cfg['model_context'],
        parameters=params.tolist(),
        viscous=[0.0] * 14,
        coulomb=[0.0] * 14,
        static_friction=[0.0] * 14,
        stribeck_velocity=vs_vec,
        rotor_inertia=ri_vec,
        gravity_world=list(robot_cfg['gravity_world']),
        effort_gain=list(robot_cfg['effort_gain']),
        effort_zero_nm=list(robot_cfg['effort_zero_nm']),
    )
    h2_rejected = dict(h1_cal, stage='h2', accepted=False, viscous=[9.9] * 14)
    (h1_dir / 'calibration.json').write_text(json.dumps(h1_cal), encoding='utf-8')
    (h2_dir / 'calibration.json').write_text(json.dumps(h2_rejected), encoding='utf-8')
    sess.mark_stage('h1', 'completed', fit_succeeded=True, accepted=True, calibration_json='h1/analysis/calibration.json')
    sess.mark_stage('h2', 'completed', fit_succeeded=True, accepted=False, calibration_json='h2/analysis/calibration.json')

    prior_h3 = resolve_stage_prior(sess, 'h3', robot_cfg)
    assert prior_h3['source'] == 'session_h1'
    np.testing.assert_allclose(prior_h3['stribeck_velocity'], vs_vec)
    np.testing.assert_allclose(prior_h3['rotor_inertia'], ri_vec)
    np.testing.assert_allclose([j.rotor_inertia for j in prior_h3['engine'].model.joints], ri_vec)

    # Binary capture reader test via read_verified_stage_capture.
    cap_dir = tmp_path / 'bin_cap'
    short_motion = Motion('short', 'friction', q_ready).add(0.25, q_ready, label='hold')
    short_bundle = dict(knots_ns=[0, 250_000_000], states=short_motion.metadata()['states'])
    _write_synthetic_binary_capture(
        cap_dir,
        short_bundle,
        lambda t: np.broadcast_to(q_ready, (len(t), 14)),
        lambda t: np.zeros((len(t), 14)),
        lambda _t, q, _dq: eng.gravity_batch(q),
        robot_cfg,
    )
    obs, native, col = read_verified_stage_capture({'calibration_begin_ns': 0}, cap_dir, robot_cfg)
    assert obs['formal_rows'] == 251
    assert col['passed']
    assert native['execution_completed'] == '1'


def test_h2_small_offline_planning_gate(tmp_path):
    robot_cfg = load_robot_config()
    q_start = np.zeros(14)

    rec_small = load_h2_recipe(mode='small')
    meta_small = plan_h2(q_start, robot_cfg, rec_small, tmp_path / 'h2_small')
    assert meta_small['collision_certificate']['passed']
    assert all('brake_ns' in pw for sw in meta_small['sweeps'] for pw in sw['passes'])



@pytest.mark.slow
def test_h2_full_offline_planning_and_duration_gate(tmp_path):
    robot_cfg = load_robot_config()
    q_start = np.zeros(14)
    rec_full = load_h2_recipe(mode='full')
    t0 = perf_counter()
    meta_full = plan_h2(q_start, robot_cfg, rec_full, tmp_path / 'h2_full')
    wall_s = perf_counter() - t0
    duration_s = meta_full['duration_ns'] * 1e-9

    print(f'\n[BENCH H2 full] wall_s={wall_s:.3f}s, duration_s={duration_s:.2f}s, min_dist_mm={meta_full["collision_certificate"]["minimum_distance_m"]*1e3:.2f}')
    assert wall_s < 8.0, f'H2 full planning took {wall_s:.2f}s (>= 8.0s)'
    assert 290.0 <= duration_s <= 360.0, f'H2 full duration {duration_s:.2f}s outside [290, 360]s'
    assert meta_full['collision_certificate']['passed']
    assert meta_full['collision_certificate']['minimum_distance_m'] >= 0.01

    # Verify zero redundant intermediate approaches across speeds of the same (pose_offset, group):
    # 3 pose offsets * 7 bilateral groups = 21 total approaches, NOT 3*7*3.3 = 77!
    ref = json.loads((tmp_path / 'h2_full/reference.json').read_text(encoding='utf-8'))
    # Consecutive sweeps on the same (split, pose_index, joint_index) must run back-to-back with no approach gap.
    j0_sweeps = [sw for sw in meta_full['sweeps'] if sw['joint_index'] == 0]
    assert len(j0_sweeps) == 2 * 4 + 1 * 3
    rev_hold_ns = round(rec_full['full']['timing']['reverse_hold_s'] * 1e9)
    for a, b in zip(j0_sweeps, j0_sweeps[1:]):
        if a['split'] == b['split'] and a['pose_index'] == b['pose_index']:
            gap_ns = b['passes'][0]['ramp_ns'][0] - a['passes'][1]['brake_ns'][1]
            assert gap_ns == rev_hold_ns, f'Unexpected intermediate approach gap {gap_ns} ns between speeds'
    assert len(ref['effort']) > 10


def test_h2_synthetic_stribeck_fit_gate():
    from casbot_calib.stages.dynamics.h2_friction import _fit_axis_stribeck
    # Independent analytic positive/negative torque observations.
    expected = np.array([0.60, 0.35, 0.30, 0.025])  # b, fc, fs-fc, vs
    def pairs(velocity):
        v = np.asarray(velocity)
        torque = (0.60 * v + (0.35 + 0.30 * np.exp(-(v / 0.025)**2)) * np.tanh(v / 0.001))
        return v, -v, torque
    training = pairs([0.025, 0.08, 0.20, 0.35])
    transitions = pairs(np.geomspace(0.003, 0.06, 16))
    validation = pairs([0.05, 0.14, 0.28])
    parameters, prediction, report = _fit_axis_stribeck(
        training, transitions, validation, n_train_speeds=4, min_train_speed=0.025)
    assert report['success'] and report['stribeck_nonlinear_used']
    assert report['rank'] == 2 and report['transition_pairs'] == 16
    np.testing.assert_allclose(parameters, expected, rtol=0.005, atol=1e-4)
    np.testing.assert_allclose(prediction, validation[2], atol=1e-4)
    assert report['validation_rmse_nm'] < report['linear_validation_rmse_nm']


@pytest.mark.slow
def test_h2_stribeck_identification_and_offset_diagnostic_synthetic(tmp_path):
    robot_cfg = load_robot_config()
    nom_eng, _ = model_for(robot_cfg)
    q_ready = ready_position()

    # Use full-mode logarithmic Stribeck speed sweep on bilateral shoulder pitch joints with disjoint train/validation postures for the end-to-end analysis.
    recipe = load_h2_recipe({
        'mode': 'small',
        'ready_pose': False,
        'gravity_feedforward': True,
        'pair_points': 24,
        'onset_speed_rad_s': 0.005,
        'small': {
            'home_deg': [0.0, 35.0, 0.0, -35.0, 0.0, 0.0, 0.0, 0.0, -35.0, 0.0, -35.0, 0.0, 0.0, 0.0],
            'joints': ['left_shoulder_pitch_joint', 'right_shoulder_pitch_joint'],
            'stroke_half_deg': 8.0,
            'pose_offsets_deg': {'train': [-3.0, 3.0], 'validation': [0.0]},
            'speeds_rad_s': {
                'train': [0.025, 0.08, 0.20, 0.35],
                'validation': [0.05, 0.14, 0.28],
            },
            'timing': {
                'initial_hold_s': 0.1,
                'transition_s': 0.8,
                'speed_ramp_s': 0.25,
                'max_plateau_s': 1.2,
                'reverse_hold_s': 0.15,
                'plateau_trim_s': 0.15,
                'final_hold_s': 0.1,
            },
            'planning_envelope': {'speed_rad_s': 0.6, 'acceleration_rad_s2': 1.5, 'jerk_rad_s3': 8.0},
        },
    })

    plan_dir = tmp_path / 'h2_synth/plan'
    plan_meta = plan_h2(q_ready, robot_cfg, recipe, plan_dir)
    ref_bundle = json.loads((plan_dir / 'reference.json').read_text(encoding='utf-8'))
    poly = _reconstruct_bpoly_from_reference(ref_bundle)

    true_b = 0.60
    true_fc = 0.35
    true_fs = 0.65
    true_vs = 0.025
    true_offset = 0.08

    visc_vec = np.zeros(14)
    coul_vec = np.zeros(14)
    stat_vec = np.zeros(14)
    vs_vec = np.full(14, 0.005)
    for j in (0, 7):
        visc_vec[j] = true_b
        coul_vec[j] = true_fc
        stat_vec[j] = true_fs
        vs_vec[j] = true_vs

    truth_eng = model_from_parameters(
        nom_eng,
        nom_eng.rigid_parameters(),
        viscous=visc_vec,
        coulomb=coul_vec,
        static=stat_vec,
        smoothing_velocity=0.001,
        stribeck_velocity=vs_vec,
    )
    # Applygravity_world from robot_cfg so gravity matches plan's prior except for deliberate 0.08 Nm offset.
    m = truth_eng.model
    m.gravity_world = np.asarray(robot_cfg['gravity_world'], dtype=float)
    truth_eng = type(truth_eng)(m)

    offset_bias = np.zeros(14)
    offset_bias[0] = true_offset
    offset_bias[7] = -true_offset
    rng = np.random.default_rng(2026)

    def _eff_fn(t_s, q_arr, dq_arr):
        ddq_arr = np.asarray(poly(t_s, nu=2), dtype=np.float64)
        tot = np.asarray(truth_eng.evaluate_batch(q_arr, dq_arr, ddq_arr)['total'], dtype=np.float64)
        noise = rng.normal(0.0, 0.003, size=tot.shape)
        return tot + offset_bias[None, :] + noise

    cap_dir = tmp_path / 'h2_synth/capture'
    _write_synthetic_binary_capture(
        cap_dir,
        ref_bundle,
        lambda t_s: poly(t_s),
        lambda t_s: poly(t_s, nu=1),
        _eff_fn,
        robot_cfg,
    )

    ana_dir = tmp_path / 'h2_synth/analysis'
    report = analyse_h2(plan_dir, cap_dir, ana_dir)
    write_h2_summary_markdown(tmp_path / 'h2_synth', plan_meta, {'ok': True}, report)

    ax0 = report['axes'][0]
    ax7 = report['axes'][7]
    print(
        f'\n[BENCH H2 Stribeck] recovered b={ax0["viscous_nm_s_rad"]:.4f} (true {true_b}), '
        f'fc={ax0["coulomb_nm"]:.4f} (true {true_fc}), fs={ax0["static_friction_nm"]:.4f} (true {true_fs}), '
        f'vs={ax0["stribeck_velocity_rad_s"]:.4f} (true {true_vs}), '
        f'offset_diag={ax0["torque_offset_diagnostic_nm"]:.4f} Nm, '
        f'val_rmse={ax0["validation_rmse_nm"]:.5f} vs linear_val_rmse={ax0["linear_validation_rmse_nm"]:.5f}'
    )

    for ax in (ax0, ax7):
        assert ax['identified']
        assert abs(ax['viscous_nm_s_rad'] - true_b) < 0.04
        assert abs(ax['coulomb_nm'] - true_fc) < 0.025
        assert abs(ax['static_friction_nm'] - true_fs) < 0.05
        assert abs(ax['stribeck_velocity_rad_s'] - true_vs) < 0.008
        assert abs(ax['torque_offset_diagnostic_nm'] - true_offset) < 0.01
        assert ax['validation_rmse_nm'] < ax['linear_validation_rmse_nm']

    cal = json.loads((ana_dir / 'calibration.json').read_text(encoding='utf-8'))
    assert isinstance(cal['stribeck_velocity'], list) and len(cal['stribeck_velocity']) == 14
    assert abs(cal['stribeck_velocity'][0] - ax0['stribeck_velocity_rad_s']) < 1e-12
    reload_info = json.loads((ana_dir / 'urdf_reload.json').read_text(encoding='utf-8'))
    assert reload_info['passed']
    assert (tmp_path / 'h2_synth/report.md').is_file()


def test_h3_small_cma_planning_gate(tmp_path):
    robot_cfg = load_robot_config()
    q_start = np.zeros(14)

    rec_small = load_h3_recipe(mode='small')
    rec_small['cma'] = dict(max_generations=1, popsize=4, sample_points=16)
    rec_small['small'].update(train_seeds=[401], pose_offsets_deg=dict(train=[0.], validation=[0.]))
    assert rec_small['feedforward_mode'] == 'gravity_friction'
    assert rec_small['cma_optimize'] is True
    assert rec_small['identify_rotor_inertia'] is True

    t0 = perf_counter()
    meta_small = plan_h3(q_start, robot_cfg, rec_small, tmp_path / 'h3_small')
    small_wall_s = perf_counter() - t0
    assert meta_small['collision_certificate']['passed']
    for blk in meta_small['blocks']:
        assert blk['optimized_condition_number'] <= blk['initial_condition_number']



@pytest.mark.slow
def test_h3_full_cma_stratified_planning_gates(tmp_path):
    robot_cfg = load_robot_config()
    q_start = np.zeros(14)
    rec_full = load_h3_recipe(mode='full')
    t1 = perf_counter()
    meta_full = plan_h3(q_start, robot_cfg, rec_full, tmp_path / 'h3_full')
    full_wall_s = perf_counter() - t1
    full_dur_s = meta_full['duration_ns'] * 1e-9

    init_conds = [b['initial_condition_number'] for b in meta_full['blocks']]
    opt_conds = [b['optimized_condition_number'] for b in meta_full['blocks']]
    print(
        f'\n[BENCH H3 full] wall_s={full_wall_s:.3f}s, duration_s={full_dur_s:.2f}s, '
        f'cond_init_mean={np.mean(init_conds):.2f} -> cond_opt_mean={np.mean(opt_conds):.2f}, '
        f'kinematics={meta_full["kinematics_summary"]}'
    )

    assert meta_full['collision_certificate']['passed']
    assert meta_full['collision_certificate']['minimum_distance_m'] >= 0.01
    assert all(o < i for o, i in zip(opt_conds, init_conds))
    assert meta_full['kinematics_summary']['maximum_acceleration_rad_s2'] <= rec_full['full']['maximum_acceleration_rad_s2'] * (1 + 1e-9)
    assert meta_full['kinematics_summary']['maximum_jerk_rad_s3'] <= rec_full['full']['maximum_jerk_rad_s3'] * (1 + 1e-9)

    # Check stratified harmonics in train vs validation blocks:
    b_train = next(b for b in meta_full['blocks'] if b['split'] == 'train')
    b_val = next(b for b in meta_full['blocks'] if b['split'] == 'validation')
    assert b_train['harmonics'][0] == [1, 2, 3, 5]
    assert b_train['harmonics'][4] == [2, 4, 6, 9]
    assert b_val['harmonics'][0] == [1, 3, 4, 6]
    assert b_val['harmonics'][4] == [2, 5, 7, 10]


@pytest.mark.slow
def test_h3_joint_rigid_and_rotor_inertia_identification_synthetic(tmp_path):
    robot_cfg = load_robot_config()
    nom_eng, _ = model_for(robot_cfg)
    source_urdf = urdf_for(robot_cfg)
    q_ready = ready_position()

    # Build a session with an upstream H2 calibration containing Stribeck friction so H3 chains automatically.
    sess = Session.create(
        output_root=tmp_path / 'sessions',
        robot_id='BOT_H3_ROT',
        stages_requested=['h2', 'h3'],
        model_context=robot_cfg['model_context'],
        plan_only=False,
    )
    h2_ana = sess.root / 'h2/analysis'
    h2_ana.mkdir(parents=True)
    prior_viscous = [0.45] * 14
    prior_coulomb = [0.18] * 14
    prior_static = [0.28] * 14
    prior_vs = [0.022] * 14
    h2_cal = dict(
        stage='h2',
        accepted=True,
        model_context=robot_cfg['model_context'],
        parameters=np.asarray(nom_eng.rigid_parameters(), dtype=float).tolist(),
        viscous=prior_viscous,
        coulomb=prior_coulomb,
        static_friction=prior_static,
        smoothing_velocity=0.001,
        stribeck_velocity=prior_vs,
        rotor_inertia=[0.0] * 14,
        gravity_world=list(robot_cfg['gravity_world']),
        effort_gain=list(robot_cfg['effort_gain']),
        effort_zero_nm=list(robot_cfg['effort_zero_nm']),
    )
    (h2_ana / 'calibration.json').write_text(json.dumps(h2_cal), encoding='utf-8')
    sess.mark_stage('h2', 'completed', fit_succeeded=True, accepted=True, calibration_json='h2/analysis/calibration.json')

    active_names = [
        'left_shoulder_pitch_joint',
        'left_shoulder_roll_joint',
        'left_elbow_pitch_joint',
        'left_wrist_yaw_joint',
    ]
    active_idx = [NAMES.index(nm) for nm in active_names]

    recipe = load_h3_recipe({
        'mode': 'small',
        'ready_pose': False,
        'gravity_feedforward': True,
        'feedforward_mode': 'gravity_friction',
        'cma_optimize': True,
        'identify_rotor_inertia': True,
        'harmonics': 12,
        'phase_points': 256,
        'torque_floor_nm': 0.05,
        'ridge': 1e-4,
        'ridge_grid': [1e-5, 1e-4, 1e-3, 1e-2, 5e-2],
        'cma': {'max_generations': 10, 'popsize': 8, 'sample_points': 36},
        'small': {
            'home_deg': [0.0, 45.0, 0.0, -45.0, 0.0, 0.0, 0.0, 0.0, -45.0, 0.0, -45.0, 0.0, 0.0, 0.0],
            'joints': active_names,
            'amplitude_deg': [9.0, 6.0, 7.0, 9.0, 10.0, 7.0, 8.0, 7.0, 5.0, 7.0, 7.0, 8.0, 7.0, 8.0],
            'pose_offsets_deg': {'train': [-3.0, 3.0], 'validation': [0.0]},
            'train_seeds': [401, 402],
            'validation_seeds': [501],
            'period_s': 4.0,
            'periods': 3,
            'discard_periods': 1,
            'timing': {
                'initial_hold_s': 0.1,
                'transition_s': 1.2,
                'excitation_ramp_s': 1.0,
                'final_hold_s': 0.1,
            },
            'planning_envelope': {'speed_rad_s': 0.8, 'acceleration_rad_s2': 2.0, 'jerk_rad_s3': 12.0},
            'maximum_acceleration_rad_s2': 6.0,
            'maximum_jerk_rad_s3': 45.0,
        },
    })

    plan_dir = sess.root / 'h3/plan'
    plan_meta = plan_h3(q_ready, robot_cfg, recipe, plan_dir, session=sess)
    assert plan_meta['prior']['source'] == 'session_h2'
    ref_bundle = json.loads((plan_dir / 'reference.json').read_text(encoding='utf-8'))
    poly = _reconstruct_bpoly_from_reference(ref_bundle)

    # Construct ground truth with realistic 25% link COM rotational inertia perturbations (preserving static gravity!)
    # AND nonzero reflected rotor inertias [0.08, 0.06, 0.045, 0.025] kg*m^2 on active joints!
    truth_model = nom_eng.model
    truth_model.gravity_world = np.asarray(robot_cfg['gravity_world'], dtype=float)
    joints = truth_model.joints
    for j, scale_i in ((0, 1.25), (1, 1.25), (3, 1.35), (4, 1.35)):
        body = joints[j].body
        body.inertia_com = np.asarray(body.inertia_com, dtype=float) * scale_i
        joints[j].body = body
    truth_model.joints = joints
    truth_base = type(nom_eng)(truth_model)

    true_rotor = np.zeros(14, dtype=float)
    true_rotor[0] = 0.080
    true_rotor[1] = 0.060
    true_rotor[3] = 0.045
    true_rotor[4] = 0.025

    truth_eng = model_from_parameters(
        nom_eng,
        truth_base.rigid_parameters(),
        viscous=prior_viscous,
        coulomb=prior_coulomb,
        static=prior_static,
        smoothing_velocity=0.001,
        stribeck_velocity=prior_vs,
        rotor_inertia=true_rotor,
    )

    rng = np.random.default_rng(909)

    def _eff_fn(t_s, q_arr, dq_arr):
        ddq_arr = np.asarray(poly(t_s, nu=2), dtype=np.float64)
        tot = np.asarray(truth_eng.evaluate_batch(q_arr, dq_arr, ddq_arr)['total'], dtype=np.float64)
        return tot + rng.normal(0.0, 0.004, size=tot.shape)

    cap_dir = sess.root / 'h3/capture'
    _write_synthetic_binary_capture(
        cap_dir,
        ref_bundle,
        lambda t_s: poly(t_s),
        lambda t_s: poly(t_s, nu=1),
        _eff_fn,
        robot_cfg,
    )

    ana_dir = sess.root / 'h3/analysis'
    report = analyse_h3(plan_dir, cap_dir, ana_dir, session=sess)
    write_h3_summary_markdown(sess.root / 'h3', plan_meta, {'ok': True}, report)

    cal = json.loads((ana_dir / 'calibration.json').read_text(encoding='utf-8'))
    rt_dyn = json.loads((ana_dir / 'runtime_dynamics.json').read_text(encoding='utf-8'))
    fitted_rotor = np.asarray(cal['rotor_inertia'], dtype=float)

    print(
        f'\n[BENCH H3 Rotor+Rigid] prior_val_rmse={report["prior_validation_rmse_nm"]:.5f} Nm -> '
        f'cand_val_rmse={report["validation_rmse_nm"]:.5f} Nm, '
        f'max_grav_change={report["max_gravity_change_nm"]:.2e} Nm, '
        f'selected_ridge={report["selected_ridge"]}\n'
        f'  true_rotor={true_rotor[active_idx].tolist()}\n'
        f'  fit_rotor ={fitted_rotor[active_idx].round(5).tolist()}'
    )

    assert report['fit_succeeded']
    assert report['max_gravity_change_nm'] <= 1e-4
    assert report['validation_rmse_nm'] < 0.15 * report['prior_validation_rmse_nm']

    for j in active_idx:
        assert abs(fitted_rotor[j] - true_rotor[j]) < 0.015, f'Joint {j} rotor inertia {fitted_rotor[j]} vs {true_rotor[j]}'
    assert np.all(fitted_rotor >= 0.0)
    np.testing.assert_allclose(rt_dyn['rotor_inertia'], fitted_rotor, atol=1e-12)
    np.testing.assert_allclose(rt_dyn['friction']['stribeck_velocity'], prior_vs, atol=1e-12)

    cand_eng = model_from_parameters(
        nom_eng,
        cal['parameters'],
        viscous=cal['viscous'],
        coulomb=cal['coulomb'],
        static=cal['static_friction'],
        smoothing_velocity=cal['smoothing_velocity'],
        stribeck_velocity=cal['stribeck_velocity'],
        rotor_inertia=cal['rotor_inertia'],
    )
    # Verify effective joint-space mass matrix diagonal matches truth much closer than prior!
    m_prior_diag = np.diag(nom_eng.mass_matrix(q_ready))[active_idx]
    m_cand_diag = np.diag(cand_eng.mass_matrix(q_ready))[active_idx]
    m_true_diag = np.diag(truth_eng.mass_matrix(q_ready))[active_idx]
    assert np.linalg.norm(m_cand_diag - m_true_diag) < 0.12 * np.linalg.norm(m_prior_diag - m_true_diag)

    reload_rep = check_reload(ana_dir / 'candidate.urdf', cand_eng, source_urdf)
    assert reload_rep['passed']
    assert (ana_dir / 'inertia_axes.csv').is_file()
    assert (sess.root / 'h3/report.md').is_file()


def test_h3_harmonic_analysis_matches_synthetic_dynamics_and_rejects_clock_jump():
    from casbot_calib.stages.dynamics.h3_inertia import _extract_block_harmonic_data, _build_rotor_regressor
    engine, _ = model_for(load_robot_config())
    ns = np.arange(256, dtype=np.int64) * 7_812_500
    t = ns * 1e-9
    q = np.broadcast_to(ready_position(), (len(t), 14)).copy()
    q[:, 0] += .02 * np.sin(2*np.pi*t)
    v = np.zeros_like(q); v[:, 0] = .02*2*np.pi*np.cos(2*np.pi*t)
    a = np.zeros_like(q); a[:, 0] = -.02*(2*np.pi)**2*np.sin(2*np.pi*t)
    obs = dict(relative_ns=ns, phase=np.full(len(t),3), clock_segment=np.zeros(len(t),int),
               time_ambiguous=np.zeros(len(t),bool), q=q, dq=v,
               effort_nm=engine.evaluate_batch(q,v,a)['total'])
    block = dict(id='analytic', split='train', seed=1, steady_ns=[0,2_000_000_000],
                 periodic_ns=[0,2_000_000_000], period_ns=1_000_000_000)
    reg, ddq, target, diagnostic = _extract_block_harmonic_data(obs, engine, block, 4, 64)
    np.testing.assert_allclose(reg @ engine.rigid_parameters(), target, atol=1e-8)
    expected = -.02*(2*np.pi)**2*np.sin(2*np.pi*np.arange(64)/64)
    np.testing.assert_allclose(ddq[:,0], expected, atol=1e-10)
    rotor = _build_rotor_regressor(ddq[:,[0]], [0])
    np.testing.assert_allclose(rotor[:,0,0], expected, atol=1e-10)
    assert diagnostic['velocity_consistency_rmse_rad_s'] < 1e-10
    obs['clock_segment'][128:] = 1
    with pytest.raises(ValueError, match='时钟'): _extract_block_harmonic_data(obs, engine, block, 4, 64)
