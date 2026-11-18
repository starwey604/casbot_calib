"""Offline and private physical SHM coverage for dynamics verification."""

import json
import os
from pathlib import Path
import shutil
import subprocess

import numpy as np
import pytest
import yaml

from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for
from casbot_calib.pipeline import verify_dynamics as verify
from casbot_calib.pipeline.delivery import build_session_delivery
from casbot_calib.pipeline.robot import load_robot_config
from casbot_calib.pipeline.session import Session
from casbot_calib.rigid_identification import model_from_parameters
from casbot_calib.shm_data import NAMES, ROOT
from casbot_calib.shm_observations import ObservationReader
from casbot_calib.staged_motion import derivative_bounds
from casbot_calib.stages.dynamics.common import export_stage_candidate
from scipy.interpolate import BPoly


@pytest.fixture
def robot():
    return load_robot_config()


def _delivery(root, robot, *, h3_accepted=True):
    session = Session.create(output_root=root, robot_id='DVT2_HV', model_context=robot['model_context'],
                             stages_requested=['h1', 'h2', 'h3'])
    nominal, _ = model_for(robot)
    model = nominal.model
    joints = model.joints
    for i in (0, 7):
        joint = joints[i]
        body = joint.body
        body.com = np.asarray(body.com) + [0.055, 0, 0]
        joint.body = body
    model.joints = joints
    h2_parameters = type(nominal)(model).rigid_parameters()
    for i in (0, 3, 7, 10):
        joint = joints[i]
        body = joint.body
        body.inertia_com = np.asarray(body.inertia_com) + np.eye(3) * 0.045
        joint.body = body
    model.joints = joints
    h3_parameters = type(nominal)(model).rigid_parameters()

    viscous = [0.35, 0.35, 0.22, 0.3, 0.11, 0.11, 0.11] * 2
    coulomb = [0.31, 0.26, 0.16, 0.19, 0.09, 0.09, 0.09] * 2
    static = [value * 1.3 for value in coulomb]
    for stage, parameters, accepted, upstream in (
        ('h1', h2_parameters, True, 'nominal'),
        ('h2', h2_parameters, True, 'session_h1'),
        ('h3', h3_parameters, h3_accepted, 'session_h2'),
    ):
        out = session.root / stage / 'analysis'
        out.mkdir(parents=True)
        cal = dict(stage=stage, accepted=accepted, parameters=parameters, joint_names=list(NAMES),
                   gravity_world=robot['gravity_world'], model_context=robot['model_context'],
                   prior_source=upstream, smoothing_velocity=0.001, stribeck_velocity=0.005)
        if stage != 'h1':
            cal.update(viscous=viscous, coulomb=coulomb, static_friction=static)
        if stage == 'h2':
            h2 = model_from_parameters(nominal, parameters, viscous=viscous, coulomb=coulomb, static=static,
                                       smoothing_velocity=0.001, stribeck_velocity=0.005)
            export_stage_candidate(out, h2, robot)
        write_json(out / 'calibration.json', cal)
        write_json(out / 'report.json', dict(fit_succeeded=True, acceptance=dict(accepted=accepted)))
        session.mark_stage(stage, 'completed', fit_succeeded=True, accepted=accepted)
    build_session_delivery(session, robot)
    return session


def test_resolve_delivery_skips_sessions_that_fell_back(tmp_path, robot):
    complete = _delivery(tmp_path, robot)
    fallback = _delivery(tmp_path, robot, h3_accepted=False)
    bundle = json.loads((fallback.root / 'delivery/calibration_bundle.json').read_text())
    assert bundle['urdf_reload_passed'] and bundle['selected_calibration_source'] == 'session_h2'
    with pytest.raises(ValueError, match='缺少完整通过'):
        verify.resolve_verified_delivery(fallback.root, tmp_path, fallback.robot_id, robot)
    delivery = verify.resolve_verified_delivery(None, tmp_path, complete.robot_id, robot)
    assert delivery.root == complete.root
    assert delivery.h2_urdf.parent == complete.root / 'h2/analysis'


def test_resolve_delivery_uses_h2_from_the_actual_prior_session(tmp_path, robot):
    prior = _delivery(tmp_path, robot, h3_accepted=False)
    session = Session.create(output_root=tmp_path, robot_id=prior.robot_id,
                             model_context=robot['model_context'], stages_requested=['h3'])
    source = prior.root / 'h2/analysis/calibration.json'
    analysis = session.root / 'h3/analysis'
    shutil.copytree(prior.root / 'h3/analysis', analysis)
    path = analysis / 'calibration.json'
    cal = json.loads(path.read_text())
    cal.update(accepted=True, prior_source='latest_h2', prior_source_path=str(source))
    write_json(path, cal)
    write_json(analysis / 'report.json', dict(fit_succeeded=True, acceptance=dict(accepted=True)))
    session.mark_stage('h3', 'completed', fit_succeeded=True, accepted=True)
    build_session_delivery(session, robot)

    delivery = verify.resolve_verified_delivery(None, tmp_path, prior.robot_id, robot)
    assert delivery.root == session.root
    assert delivery.h2_urdf.parent == source.parent


def test_complete_preview_preserves_gains_and_session_links(tmp_path, robot):
    existing = tmp_path / 'prior'
    existing.mkdir()
    for name in ('latest', 'latest_dynamics'):
        (tmp_path / name).symlink_to(existing)
    preview = tmp_path / 'preview'
    assert verify.main(['--plan-only', '--output-root', str(tmp_path), '--output', str(preview)]) == 0
    assert verify.CLIP_ASSET.stat().st_size < 60_000
    reference = json.loads((preview / 'reference.json').read_text())
    plan = json.loads((preview / 'plan.json').read_text())
    curve = BPoly.from_derivatives(np.asarray(reference['knots_ns']) * 1e-9, reference['states'])
    assert plan['collision_certificate']['passed']
    assert plan['collision_certificate']['minimum_distance_m'] >= 0.01
    assert derivative_bounds(curve).max() < 1.5
    assert reference['purpose'] == 'shm_calib_ready_then_collision_checked'
    assert reference['observed_velocity_limit_rad_s'] == 1.5
    assert plan['delivery_session'] is None
    schedule = reference['pd_schedule']
    stamps = [row['t_ns'] for row in schedule]
    for entry in plan['groups'].values():
        begin, end = entry['run_ns']
        assert stamps.index(end) == stamps.index(begin) + 1
        assert all(schedule[stamps.index(begin)][key] == schedule[stamps.index(end)][key] for key in ('kp', 'kd'))
    assert all((tmp_path / name).resolve() == existing for name in ('latest', 'latest_dynamics'))
    assert not (tmp_path / 'latest_verify').exists()
    checker = _executor().with_name('shm_reference_check')
    if checker.is_file():
        subprocess.run([str(checker), str(preview / 'reference.json')], input='', text=True, check=True)


def test_observation_windows_average_duplicates_and_detect_faults(monkeypatch):
    origin = 12345678900
    ms = np.arange(201).repeat(2)
    y = np.tile([1.0, 3.0], 201)[:, None] * np.ones(14)
    seg = np.zeros(len(ms), dtype=int)
    data = dict(observation_time_ns=origin + ms * 1_000_000, elapsed_s=ms * 1e-3,
                receive_ns=origin + np.arange(len(ms)) * 500_000,
                clock_segment=seg, time_ambiguous=np.zeros(len(ms), dtype=bool), q=y, dq=y, effort_raw=y)
    reader = object.__new__(ObservationReader)
    reader.origin = origin
    monkeypatch.setattr(reader, 'arrays', lambda: data)
    _, q, dq, effort = verify._window_samples(reader, [0, 200_000_000])
    assert q.shape == (200, 14)
    for out in (q, dq, effort):
        assert np.all(out == 2.0)
    seg[150:] = 1
    with pytest.raises(ValueError, match='跨越时钟分段'):
        verify._window_samples(reader, [0, 200_000_000])
    seg[:] = 0
    sample = (ms < 100) | (ms > 125)
    monkeypatch.setattr(reader, 'arrays', lambda: {key: value[sample] for key, value in data.items()})
    with pytest.raises(ValueError, match='超过 20 ms'):
        verify._window_samples(reader, [0, 200_000_000])


def test_recommendation_only_on_pass(tmp_path, robot):
    session = _delivery(tmp_path, robot)
    delivery = verify.resolve_verified_delivery(session.root, tmp_path, session.robot_id, robot)
    _, recommended = verify._feedforward_models(robot, delivery)
    row = dict(valid=False, invalid_reason='fixture')
    plan = dict(delivery_session=str(session.root), duration_ns=1, recommended=recommended)
    groups = {name: dict(keyboard=row, drum=row, overall=row) for name in verify.GROUPS}
    report = dict(groups=groups, reasons=[], accepted=True, verdict='PASS')
    verify._write_report(tmp_path, plan, report)
    recommendation = tmp_path / 'analysis/recommended_band_gains.yaml'
    assert min(yaml.safe_load(recommendation.read_text())['friction']['smoothing_velocity']) == 0.015
    verify._write_report(tmp_path, plan, dict(report, accepted=False, verdict='FAIL'))
    assert not recommendation.exists()


def _executor():
    installed = Path(os.environ.get('CASBOT_SHM_TEST_BIN', ROOT / 'install/shm-core/bin')) / 'shm_executor'
    return installed if installed.is_file() else ROOT / 'build/humble/cpp/src/shm/shm_executor'


@pytest.mark.slow
def test_physical_mock_round_trip_with_start_replan(tmp_path, robot, monkeypatch):
    binary = _executor()
    if not binary.with_name('shm_mock_server').is_file():
        pytest.skip('Build native SHM physics binaries')
    session = _delivery(tmp_path, robot)
    actual_snapshot = verify.native_snapshot

    def stale_snapshot(*args, **kwargs):
        q = actual_snapshot(*args, **kwargs)
        q[0] += 0.008  # Rejected by the real executor's 5 mrad preflight check.
        return q

    monkeypatch.setattr(verify, 'native_snapshot', stale_snapshot)
    assert verify.main([str(session.root), '--mock', '--output-root', str(tmp_path), '--executor', str(binary)]) == 0
    capture = session.root / 'verify/capture'
    summary = json.loads((capture / 'result.json').read_text())
    assert summary['attempts'][0]['reason'] == 'start_replan'
    assert 'replanned' in summary['effective_plan']
    verify_dir = session.root / 'verify'
    report_path = verify_dir / 'analysis/report.json'
    report = report_path.read_bytes()
    assert json.loads(report)['accepted']
    assert (tmp_path / 'latest_verify').resolve() == verify_dir
    assert (tmp_path / 'latest_dynamics').resolve() == session.root
    # Reanalysis must resolve the effective plan, even if the rejected plan was removed.
    (verify_dir / 'plan/plan.json').unlink()
    assert verify.main(['--reanalyse', '--output-root', str(tmp_path)]) == 0
    assert report_path.read_bytes() == report
