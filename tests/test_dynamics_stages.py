"""Integration & closed-loop physics tests for H1 -> H2 -> H3 SHM calibration stages."""
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
import pytest
import yaml
from casbot_calib.dynamic_export import check_reload, write_inertial_urdf
from casbot_calib.model_runtime import model_for, urdf_for
from casbot_calib.pipeline.robot import load_robot_config
from casbot_calib.pipeline.runner import run_pipeline
from casbot_calib.pipeline.session import Session
from casbot_calib.shm_data import ROOT
from casbot_calib.shm_ready import ready_position
from casbot_calib.stages.dynamics.common import resolve_stage_prior

pytestmark = pytest.mark.skipif(sys.platform != 'linux', reason='POSIX SHM only')
NATIVE = Path(os.environ.get('CASBOT_SHM_TEST_BIN', ROOT / 'install/shm-core/bin')) / 'shm_executor'
if not NATIVE.exists():
    NATIVE = ROOT / 'build/humble/cpp/src/shm/shm_executor'


@pytest.mark.slow
def test_dynamics_three_stage_offline_plan(tmp_path):
    robot_config = yaml.safe_load((ROOT / 'configs/robot/dvt2.yaml').read_text())

    def check_gains(plan_dir):
        meta = json.loads((plan_dir / 'plan.json').read_text())
        reference = json.loads((plan_dir / 'reference.json').read_text())
        for key in ('kp', 'kd'):
            expected = robot_config[key]
            assert meta['robot_config'][key] == expected
            assert reference[key] == expected
            assert reference['pd_schedule'][-1][key] == expected
        assert reference['pd_schedule'][0]['kp'] == [20.0] * 14
        assert reference['pd_schedule'][0]['kd'] == [2.0] * 14

    # 1. Full 3-stage CLI dynamics.sh --plan-only in default small mode.
    out_small = tmp_path / 'small_out'
    res = subprocess.run(
        ['bash', str(ROOT / 'calib/dynamics.sh'), '--plan-only', '--robot', 'SN_DYN_01', '--output-root', str(out_small)],
        capture_output=True, text=True,
    )
    assert res.returncode == 0, res.stderr + res.stdout
    latest = (out_small / 'latest').resolve()
    sess = json.loads((latest / 'session.json').read_text())
    assert list(sess['stages'].keys()) == ['h1', 'h2', 'h3']
    for st in ('h1', 'h2', 'h3'):
        assert sess['stages'][st]['status'] == 'planned'
        assert (latest / st / 'plan/reference.json').is_file()
        assert (latest / st / 'report.md').is_file()
        check_gains(latest / st / 'plan')


def test_cross_session_prior_auto_discovery(tmp_path):
    robot_cfg = load_robot_config()
    nom_eng, _ = model_for(robot_cfg)
    params = np.asarray(nom_eng.rigid_parameters(), dtype=float)

    # Create a completed session S1 with H1 and H2 calibration artifacts.
    out_root = tmp_path / 'calib'
    s1 = Session.create(stages_requested=['h1', 'h2'], model_context=robot_cfg['model_context'], robot_id='BOT_CHAIN', output_root=out_root)
    # Promote s1 to a non-plan-only session with completed h1 & h2 records.
    s1.metadata['plan_only'] = False
    for st, b_val in (('h1', 0.0), ('h2', 0.42)):
        adir = s1.root / st / 'analysis'
        adir.mkdir(parents=True, exist_ok=True)
        cal = dict(
            stage=st,
            model_context=robot_cfg['model_context'],
            parameters=params.tolist(),
            viscous=[b_val] * 14,
            coulomb=[0.11 if st == 'h2' else 0.0] * 14,
            static_friction=[0.15 if st == 'h2' else 0.0] * 14,
            gravity_world=list(robot_cfg['gravity_world']),
            effort_gain=list(robot_cfg['effort_gain']),
            effort_zero_nm=list(robot_cfg['effort_zero_nm']),
        )
        (adir / 'calibration.json').write_text(json.dumps(cal))
        s1.mark_stage(st, 'completed', fit_succeeded=True, calibration_json=f'{st}/analysis/calibration.json')

    # A brand new session S2 for the same robot_id automatically discovers S1's H1 (for H2) and H2 (for H3)!
    s2 = Session.create(stages_requested=['h3'], model_context=robot_cfg['model_context'], robot_id='BOT_CHAIN', output_root=out_root)
    p_h2 = resolve_stage_prior(s2, 'h2', robot_cfg)
    p_h3 = resolve_stage_prior(s2, 'h3', robot_cfg)
    assert p_h2['source'] == 'latest_h1'
    assert p_h3['source'] == 'latest_h2'
    assert np.allclose(p_h3['viscous'], 0.42)
    assert np.allclose(p_h3['coulomb'], 0.11)


@pytest.mark.slow
@pytest.mark.skipif(not NATIVE.exists(), reason='Build SHM physics executables')
def test_three_stage_closed_loop_with_physical_shm_mock(tmp_path):
    robot_cfg_base = load_robot_config()
    source_urdf = urdf_for(robot_cfg_base)
    nom_eng, _ = model_for(robot_cfg_base)

    # Construct a physically valid perturbed ground-truth plant:
    # - Shoulder pitch COM x +8 cm -> near-constant gravity offset observable
    #   in this single-axis, single-training-pose small H1 fixture.
    # - Shoulder pitch (joint 0) COM inertia +0.12*I_3 kg*m^2 -> shifts rotational inertia without altering gravity (H3).
    # - Joint 0 dry + viscous friction (b=0.8, fc=0.25, fs=0.35) with coupled stiction (H2).
    truth_model = nom_eng.model
    truth_model.gravity_world = np.asarray(robot_cfg_base['gravity_world'], dtype=float)
    joints = truth_model.joints
    b0 = joints[0].body
    b0.com = np.asarray(b0.com, dtype=float) + [0.08, 0.0, 0.0]
    b0.inertia_com = np.asarray(b0.inertia_com, dtype=float) + np.eye(3) * 0.12
    joints[0].body = b0
    truth_model.joints = joints
    truth_eng = type(nom_eng)(truth_model)

    truth_urdf = tmp_path / 'truth.urdf'
    write_inertial_urdf(source_urdf, truth_eng, truth_urdf)

    q0 = ready_position()
    base_mock = json.loads((ROOT / 'configs/shm/h1_mock.json').read_text())
    base_mock['urdf'] = str(truth_urdf)
    base_mock['stiction_enabled'] = True
    base_mock['friction_viscous'] = [0.80] + [0.30] * 13
    base_mock['friction_coulomb'] = [0.25] + [0.10] * 13
    base_mock['friction_static'] = [0.35] + [0.15] * 13
    base_mock['initial_q'] = q0.tolist()
    base_mock['initial_fields']['position'] = q0.tolist()
    base_mock['initial_fields']['effort'] = truth_eng.gravity(q0).tolist()
    mock_cfg_path = tmp_path / 'truth_mock.json'
    mock_cfg_path.write_text(json.dumps(base_mock, indent=2))

    fast_robot = tmp_path / 'robot.yaml'
    fast_robot.write_text(yaml.safe_dump(dict(
        initial_hold_s=0.05,
        final_hold_s=0.05,
        anchor_transition_s=0.2,
    )))

    fast_h1 = dict(
        mode='small',
        ready_pose=False,
        small=dict(
            joints=['left_shoulder_pitch_joint'],
            target_offset_deg=4.5,
            # Match the real recipe: 1 degree does not reverse this plant's stiction.
            approach_offset_deg=5.0,
            timing=dict(
                initial_hold_s=0.05,
                minimum_transition_s=0.2,
                minimum_paired_approach_s=0.2,
                settle_s=0.2,
                measure_s=0.25,
                final_hold_s=0.05,
            ),
            planning_envelope=dict(speed_rad_s=0.05, acceleration_rad_s2=0.1, jerk_rad_s3=1.0),
        ),
    )
    fast_h2 = dict(
        mode='small',
        ready_pose=False,
        gravity_feedforward=True,
        pair_points=20,
        onset_speed_rad_s=0.005,
        small=dict(
            home_deg=np.rad2deg(q0).tolist(),
            joints=['left_shoulder_pitch_joint'],
            stroke_half_deg=6.0,
            pose_offsets_deg=dict(train=[-1.5, 1.5], validation=[0.0]),
            speeds_rad_s=dict(train=[0.06, 0.14], validation=[0.10]),
            timing=dict(
                initial_hold_s=0.05,
                transition_s=0.35,
                speed_ramp_s=0.20,
                reverse_hold_s=0.10,
                plateau_trim_s=0.15,
                final_hold_s=0.05,
            ),
            planning_envelope=dict(speed_rad_s=0.4, acceleration_rad_s2=1.5, jerk_rad_s3=10.0),
        ),
    )
    fast_h3 = dict(
        mode='small',
        ready_pose=False,
        gravity_feedforward=True,
        harmonics=8,
        phase_points=128,
        torque_floor_nm=0.05,
        ridge=1e-4,
        small=dict(
            home_deg=np.rad2deg(q0).tolist(),
            joints=['left_shoulder_pitch_joint'],
            amplitude_deg=8.0,
            pose_offsets_deg=dict(train=[-1.5], validation=[0.0]),
            train_seeds=[401],
            validation_seeds=[501],
            period_s=2.4,
            periods=3,
            discard_periods=1,
            timing=dict(
                initial_hold_s=0.05,
                transition_s=0.5,
                excitation_ramp_s=0.7,
                final_hold_s=0.05,
            ),
            planning_envelope=dict(speed_rad_s=0.5, acceleration_rad_s2=2.0, jerk_rad_s3=15.0),
            maximum_acceleration_rad_s2=15.0,
            maximum_jerk_rad_s3=350.0,
        ),
    )

    out_root = tmp_path / 'calib_runs'
    # Run H1 first, then --resume into H1,H2,H3 to verify multi-stage checkpointing + chaining!
    r1 = run_pipeline(
        stages='h1',
        robot_id='DVT2_SYNTH',
        stage_config=fast_h1,
        robot_config=fast_robot,
        output_root=out_root,
        executor=NATIVE,
        mock=True,
        mock_config=mock_cfg_path,
    )
    assert r1['completed']

    r_all = run_pipeline(
        stages='h1,h2,h3',
        resume=True,
        stage_config=dict(h1=fast_h1, h2=fast_h2, h3=fast_h3),
        robot_config=fast_robot,
        output_root=out_root,
        executor=NATIVE,
        mock=True,
        mock_config=mock_cfg_path,
    )
    assert r_all['completed']
    sess_root = r_all['session'].root
    assert sess_root == r1['session'].root

    h1_rep = json.loads((sess_root / 'h1/analysis/report.json').read_text())
    h2_rep = json.loads((sess_root / 'h2/analysis/report.json').read_text())
    h3_rep = json.loads((sess_root / 'h3/analysis/report.json').read_text())

    # 1. H1 static gravity improved overall held-out gravity prediction significantly.
    assert h1_rep['fit_succeeded']
    assert h1_rep['validation_pairs_per_joint'] == [1] + [0] * 13
    assert h1_rep['overall_validation_rmse_nm'] < 0.35 * h1_rep['overall_nominal_validation_rmse_nm']

    # 2. H2 automatically chained session_h1 and recovered viscous & Coulomb friction on joint 0.
    assert h2_rep['fit_succeeded']
    assert h2_rep['prior_source'] == 'session_h1'
    assert h2_rep['validation_rmse_nm'] < 0.25 * h2_rep['zero_friction_validation_rmse_nm']
    ax0_h2 = h2_rep['axes'][0]
    assert ax0_h2['viscous_nm_s_rad'] > 0.35
    assert abs(ax0_h2['coulomb_nm'] - 0.25) < 0.08
    assert ax0_h2['static_friction_nm'] >= ax0_h2['coulomb_nm']

    # 3. H3 automatically chained session_h2, locked H1 gravity subspace, and recovered rotational inertia!
    assert h3_rep['fit_succeeded']
    assert h3_rep['prior_source'] == 'session_h2'
    assert h3_rep['max_gravity_change_nm'] < 1e-5
    assert h3_rep['validation_rmse_nm'] < 0.5 * h3_rep['prior_validation_rmse_nm']

    # 4. Final candidate.urdf + runtime_dynamics.json + session delivery/ exist and pass reload consistency.
    final_urdf = sess_root / 'h3/analysis/candidate.urdf'
    final_runtime = json.loads((sess_root / 'h3/analysis/runtime_dynamics.json').read_text())
    assert final_urdf.is_file()
    assert abs(final_runtime['friction']['viscous'][0] - ax0_h2['viscous_nm_s_rad']) < 1e-12
    reload_meta = json.loads((sess_root / 'h3/analysis/urdf_reload.json').read_text())
    assert reload_meta['passed']
    for st in ('h1', 'h2', 'h3'):
        assert (sess_root / st / 'report.md').is_file()

    delivery_dir = sess_root / 'delivery'
    assert (delivery_dir / 'calibrated.urdf').is_file()
    assert (delivery_dir / 'runtime_dynamics.json').is_file()
    assert (delivery_dir / 'summary.md').is_file()
    bundle = json.loads((delivery_dir / 'calibration_bundle.json').read_text())
    assert bundle['selected_calibration_source'] == 'session_h3'
    assert bundle['provenance'] == {
        'gravity': 'session_h1',
        'friction': 'session_h2',
        'inertia': 'session_h3',
        'rotor_inertia': 'session_h3',
    }
    assert bundle['urdf_reload_passed']

    # 5. CLI --reanalyse archives old analysis/ and rebuilds delivery/ offline.
    re_res = subprocess.run(
        [
            sys.executable, '-m', 'casbot_calib.pipeline.runner',
            '--reanalyse',
            '--output-root', str(out_root),
            '--robot-config', str(fast_robot),
        ],
        capture_output=True, text=True,
    )
    assert re_res.returncode == 0, re_res.stderr + re_res.stdout
    for st in ('h1', 'h2', 'h3'):
        archived = list((sess_root / st).glob('analysis.previous_*'))
        assert len(archived) == 1
        assert (sess_root / st / 'analysis/report.json').is_file()
    bundle_after = json.loads((delivery_dir / 'calibration_bundle.json').read_text())
    assert bundle_after['selected_calibration_source'] == 'session_h3'
    np.testing.assert_allclose(bundle_after['parameters'], bundle['parameters'], rtol=1e-12, atol=1e-12)


def test_validation_metrics_report_regression_and_legacy_rejection_still_falls_back(tmp_path):
    from casbot_calib.pipeline.delivery import build_session_delivery
    from casbot_calib.stages.dynamics.common import evaluate_stage_acceptance

    # Case 1: Active axis degrades significantly even though overall RMSE slightly improved (real site_v1 failure mode).
    acc_axis = evaluate_stage_acceptance(
        0.2821,
        0.1669,
        axis_rows=[('left_shoulder_pitch_joint', 0.4584, 0.5198), ('right_shoulder_pitch_joint', 0.2454, 0.1494)],
    )
    assert acc_axis['accepted']
    assert acc_axis['degraded_axes'] == ['left_shoulder_pitch_joint']

    # Case 2: Overall RMSE degrades.
    acc_overall = evaluate_stage_acceptance(0.15, 0.19, axis_rows=[('left_shoulder_pitch_joint', 0.15, 0.19)])
    assert acc_overall['accepted'] and acc_overall['overall_degraded']
    assert not evaluate_stage_acceptance(0.15, float('nan'))['accepted']

    # Case 3: Downstream prior resolution & session delivery skip accepted=False stages automatically.
    robot_cfg = load_robot_config()
    nom_eng, _ = model_for(robot_cfg)
    nom_params = np.asarray(nom_eng.rigid_parameters(), dtype=float)
    h1_params = nom_params.copy()
    h1_params[0] += 0.25  # Perturbed mass on link 0

    out_root = tmp_path / 'nonreg'
    sess = Session.create(stages_requested=['h1'], model_context=robot_cfg['model_context'], robot_id='BOT_GATE', output_root=out_root)
    sess.metadata['plan_only'] = False

    # H1 accepted, H2 accepted, H3 rejected (accepted=False).
    for st, accepted, b_val, params in (
        ('h1', True, 0.0, h1_params),
        ('h2', True, 0.55, h1_params),
        ('h3', False, 0.55, nom_params * 1.1),
    ):
        adir = sess.root / st / 'analysis'
        adir.mkdir(parents=True, exist_ok=True)
        cal = dict(
            stage=st,
            accepted=accepted,
            acceptance=dict(accepted=accepted, reason='passed' if accepted else 'overall_validation_degraded'),
            model_context=robot_cfg['model_context'],
            parameters=params.tolist(),
            viscous=[b_val] * 14,
            coulomb=[0.12 if st != 'h1' else 0.0] * 14,
            static_friction=[0.18 if st != 'h1' else 0.0] * 14,
            gravity_world=list(robot_cfg['gravity_world']),
            effort_gain=list(robot_cfg['effort_gain']),
            effort_zero_nm=list(robot_cfg['effort_zero_nm']),
            prior_source='session_h1' if st == 'h2' else ('session_h2' if st == 'h3' else 'nominal'),
        )
        (adir / 'calibration.json').write_text(json.dumps(cal))
        rep = dict(
            stage=st,
            mode='small',
            fit_succeeded=True,
            accepted=accepted,
            acceptance=cal['acceptance'],
            overall_nominal_validation_rmse_nm=0.30,
            overall_validation_rmse_nm=0.10,
            zero_friction_validation_rmse_nm=0.25,
            prior_validation_rmse_nm=0.20,
            validation_rmse_nm=0.08 if accepted else 0.26,
        )
        (adir / 'report.json').write_text(json.dumps(rep))
        sess.mark_stage(
            st,
            'completed',
            analysis_dir=f'{st}/analysis',
            calibration_json=f'{st}/analysis/calibration.json',
            fit_succeeded=True,
            accepted=accepted,
        )

    # resolve_stage_prior for H3 in a follow-up check skips rejected H3 / picks accepted H2:
    prior_h3 = resolve_stage_prior(sess, 'h3', robot_cfg)
    assert prior_h3['source'] == 'session_h2'

    # Session delivery skips rejected H3 and delivers accepted H2 (with H1 gravity + H2 friction + nominal rotor)!
    delivery = build_session_delivery(sess, robot_cfg)
    assert delivery['selected_calibration_source'] == 'session_h2'
    bundle = json.loads((sess.root / 'delivery/calibration_bundle.json').read_text())
    assert bundle['selected_calibration_source'] == 'session_h2'
    assert bundle['provenance'] == {
        'gravity': 'session_h1',
        'friction': 'session_h2',
        'inertia': 'session_h1',
        'rotor_inertia': 'nominal',
    }
    np.testing.assert_allclose(bundle['parameters'], h1_params)
    np.testing.assert_allclose(bundle['friction']['viscous'], 0.55)
    assert (sess.root / 'delivery/calibrated.urdf').is_file()
    assert '回退先验' in (sess.root / 'delivery/summary.md').read_text(encoding='utf-8')
