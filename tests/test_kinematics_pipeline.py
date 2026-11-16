import json
from pathlib import Path
import numpy as np
import pytest

from casbot_calib.pipeline.session import Session
from casbot_calib.pipeline.stage import resolve_stages
from casbot_calib.pipeline.takeover import ensure_ready_channel
from casbot_calib.trical.controller import feedback_means, correction
from casbot_calib.shm_data import FEEDBACK_RECORD_DTYPE, DIRECTION, OFFSET
from casbot_calib.model_context import load_context


def test_family_resume_and_mixed_stages(tmp_path):
    h = Session.create(output_root=tmp_path, model_context={}, stages_requested=['h1'])
    (tmp_path/'latest_dynamics').unlink()  # Sessions from before the family index.
    old = json.loads((h.root/'session.json').read_text()); old.pop('family')
    (h.root/'session.json').write_text(json.dumps(old))
    k = Session.create(output_root=tmp_path, model_context={}, stages_requested=['k1'], family='kinematics')
    assert Session.resume(output_root=tmp_path).root == h.root
    assert Session.resume(output_root=tmp_path, family='kinematics').root == k.root
    with pytest.raises(ValueError, match='不属于'): Session.resume(target=k.root)
    with pytest.raises(ValueError, match='混跑'): resolve_stages('h1,k1')
    a,b,c = resolve_stages('k1,k2,k3')
    assert a.stop_after and not b.stop_after and c.execution_kind == 'offline'
    assert not any(s.requires_hand_background for s in (a,b,c))


def test_private_bare_flange_takeover_without_hand_snapshot(tmp_path):
    value = ensure_ready_channel(tmp_path, Path('/missing/executor'),
                                 endpoint='/casbot_calib_test_bare', require_hand_background=False)
    assert value['hand_background'] is None
    assert not value['takeover_performed']


def test_feedback_bin_window_mean_is_unique_frame_weighted(tmp_path):
    records = np.zeros(5, dtype=FEEDBACK_RECORD_DTYPE)
    records['receive_mono_ns'] = [10,20,30,40,50]
    records['frame']['frame_counter'] = [1,2,2,3,4]
    records['frame']['joints']['position'][:,12:26] = np.array([0,1000,1000,3000,4000])[:,None]
    path = tmp_path/'feedback.bin'; records.tofile(path)
    actual = feedback_means(path, [dict(t_start_ns=10,t_end_ns=40)])[0]
    np.testing.assert_allclose(actual, np.array(OFFSET)+.002*np.array(DIRECTION))


def test_cartesian_correction_respects_observed_windup():
    context = load_context()
    q = context.home
    target = correction(context.engine, q, 'left', [.004,.003,.002], .0015,.012,q)
    assert np.max(abs(target-q)) <= .012+1e-15
    np.testing.assert_array_equal(target[7:],q[7:])


def test_remount_checkpoint_resume_and_offline_no_hardware(monkeypatch, tmp_path):
    from casbot_calib.pipeline import kinematics as runner
    from casbot_calib.pipeline.runner import run_pipeline
    from casbot_calib.stages.kinematics.common import AcquisitionStage
    calls = []
    binary = tmp_path/'shm_executor'; binary.touch()
    binary.with_name('shm_trical_executor').touch()
    monkeypatch.setattr(runner, 'find_executor', lambda _: binary)
    monkeypatch.setattr(runner, 'check_hardware_environment', lambda **kw: calls.append('environment'))
    monkeypatch.setattr(runner, 'ensure_ready_channel', lambda *a, **kw: dict(endpoint='/casbot_calib_test_control'))
    def plan(self, q, robot, recipe, path, **kw):
        path.mkdir(parents=True)
        data = dict(recipe=recipe, mode=recipe['mode'], points=[{}]*4)
        (path/'plan.json').write_text(json.dumps(data))
        return data
    monkeypatch.setattr(AcquisitionStage, 'plan', plan)
    monkeypatch.setattr(AcquisitionStage, 'execute', lambda self,*a,**kw: calls.append(self.name))
    def analyse(self, *a, session=None, **kw):
        # Exact nominal observations isolate orchestration from the separately tested physical loop.
        from casbot_calib.trical.routes import design_points
        from casbot_calib.trical.assembly import ball_center
        rows = []
        engine = load_context().engine
        for row in design_points('small'):
            if row['arm'] != self.side: continue
            pose = engine.frame_pose(row['q_rad'], self.side+'_trical_tcp')
            ball = ball_center(row['ball'])
            rows.append(dict(row, offset_m=(pose[:3,:3].T@(ball-pose[:3,3])).tolist(),
                             ball_center_m=ball.tolist(),reading_span_m=[0.]*3))
        (session.root/self.name/'measurements.json').write_text(json.dumps(rows))
        return dict(accepted=True)
    monkeypatch.setattr(AcquisitionStage, 'analyse', analyse)
    result = run_pipeline(stages='k1,k2,k3', stage_overrides={'mode':'small'},output_root=tmp_path/'out')
    assert result['stopped_for_remount'] and calls.count('k1')==1 and 'k2' not in calls
    resumed = run_pipeline(stages='k1,k2,k3', resume=True, output_root=tmp_path/'out')
    assert resumed['completed'] and calls.count('k1')==1 and calls.count('k2')==1
    root = resumed['session'].root
    assert not (root/'delivery').exists()
    assert json.loads((root/'k3/report.json').read_text())['mode']=='small'
    calls.clear()
    run_pipeline(stages='k1,k2,k3', reanalyse=True, output_root=tmp_path/'out', executor=Path('/absent'))
    assert calls == []


def test_first_ball_insertion_is_two_2_5mm_steps_for_both_grids():
    from casbot_calib.trical.routes import Routes
    for side in ('left','right'):
        routes = Routes(side)
        _, contact = routes.path('A')
        positions = np.array([routes.engine.frame_pose(q,side+'_trical_tcp')[:3,3] for q in contact[:3]])
        np.testing.assert_allclose(np.linalg.norm(np.diff(positions,axis=0),axis=1), [.0025,.0025],atol=1e-7)
        routes.check_local('A', contact[0], contact[1])
        routes.check_local('A', contact[1], contact[2])


def test_capture_analysis_recomputes_q_from_binary_not_target(tmp_path):
    from casbot_calib.trical.controller import analyse_capture
    native = tmp_path/'native'; native.mkdir()
    records = np.zeros(2,dtype=FEEDBACK_RECORD_DTYPE)
    records['receive_mono_ns'] = [10,20]
    records['frame']['frame_counter'] = [1,2]
    records.tofile(native/'feedback.bin')
    (tmp_path/'windows.json').write_text(json.dumps([dict(target_q_rad=[99.]*14,t_start_ns=0,t_end_ns=20)]))
    actual = analyse_capture(tmp_path)[0]
    np.testing.assert_array_equal(actual['q_rad'],OFFSET)
    assert actual['target_q_rad']==[99.]*14


@pytest.mark.parametrize('side', ['left','right'])
def test_fixed_preparation_to_route_home_is_collision_checked(side):
    from casbot_calib.trical.routes import Routes
    routes = Routes(side)
    assert routes.validate([])


def test_windup_clipping_keeps_the_cartesian_step_within_1_5mm():
    context = load_context(); q = context.home; engine = context.engine
    rng = np.random.default_rng(12)
    for _ in range(20):
        hold = q+rng.uniform(-.011,.011,14)
        target = correction(engine,q,'left',[.004,.003,.002],.0015,.012,hold)
        assert np.max(abs(target-q)) <= .012+1e-12
        step = engine.frame_pose(target,'left_trical_tcp')[:3,3]-engine.frame_pose(hold,'left_trical_tcp')[:3,3]
        assert np.linalg.norm(step) <= .0015+1e-12


@pytest.mark.parametrize('overrides', [{'mode': 'invalid'}, {'smoke_enabled': True}])
def test_invalid_recipe_fails_before_hardware(monkeypatch, tmp_path, overrides):
    from casbot_calib.pipeline import kinematics as runner
    from casbot_calib.pipeline.runner import run_pipeline
    def unexpected(*args, **kwargs):
        pytest.fail('hardware lookup ran before recipe validation')
    monkeypatch.setattr(runner, 'find_executor', unexpected)
    monkeypatch.setattr(runner, 'ensure_ready_channel', unexpected)
    with pytest.raises(ValueError, match='运动学'):
        run_pipeline(stages='k1,k2,k3', stage_overrides=overrides, output_root=tmp_path)
