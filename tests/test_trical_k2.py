"""K2 sensor causality, bounded collision exceptions and actual closed-loop failures."""
from copy import deepcopy
import json
import shutil
import numpy as np
import pytest
from casbot_calib import _core
from casbot_calib.kinematics import workflow_config,nominal_kinematics,calibration
from casbot_calib.trical_workflow import run_mock,stable_window
from casbot_calib.trical_analysis import analyse
from casbot_calib.trical_fixture import fixture_profile
from casbot_calib.collision import certify_polyline


def native_session(expose=True,fault=None,retain=False):
    c=workflow_config();engine,_=nominal_kinematics(c);q=np.array(c['workflow']['home_rad'])
    arm=_core.ArmSimulationConfig();arm.gain=np.full(14,2.);arm.current_tau=0.;arm.command_delay_ns=0
    arm.position_noise=0.;arm.current_noise=0.
    sensor=_core.TricalMockConfig();sensor.noise_std=0.;sensor.quantum=0.;sensor.expose_sample_time=expose
    sensor.encoder_offset=np.linspace(-.0002,.0002,14)
    session=_core.TricalMockSession(engine,arm,sensor,q,np.zeros(14),retain)
    ball=engine.frame_pose(q,'left_trical_tcp')[:3,3]
    session.select_target('left_trical_tcp','ball_1',ball)
    command=np.column_stack([q,np.zeros(14),engine.gravity(q+sensor.encoder_offset),np.full(14,100.),np.full(14,10.)])
    session.submit(0,command)
    if fault:session.set_fault(1,fault)
    return session,c,engine,q,sensor


def test_native_sensor_chunk_invariance_delay_and_truth_isolation():
    left,*_=native_session();right,*_=native_session();obs,packets,truth=left.advance_to(50_000_000)
    pieces=[];observations=[]
    for end in (7_000_000,21_000_000,34_000_000,50_000_000):
        a,b,c=right.advance_to(end);observations.append(a);pieces+=b
        assert not len(c['q']) and c['q'].shape==(0,14)
    assert packets==pieces and truth['q'].shape==(0,14)
    np.testing.assert_allclose(obs['q'],np.concatenate([a['q'] for a in observations]),atol=1e-13)
    for p in packets:
        assert p['sample_known'] and 2_500_000<=p['receive_ns']-p['sample_ns']<=3_500_000
        assert p['receive_ns']<=50_000_000
    assert len(packets)==9 and any(abs(p['reading_m'])>1e-6 for p in packets)
    assert not left.advance_to(50_000_000)[1]
    with pytest.raises(ValueError):left.advance_to(49_000_000)


def test_unknown_times_and_native_encoder_observation_convention():
    session,c,e,q,sensor=native_session(expose=False,retain=True)
    obs,packets,truth=session.advance_to(10_000_000)
    np.testing.assert_allclose(obs['q'],q[None,:]+np.zeros_like(obs['q']),atol=1e-13)
    np.testing.assert_allclose(truth['q'],q[None,:]+sensor.encoder_offset+np.zeros_like(truth['q']),atol=1e-13)
    assert all(p['sample_ns']==-1 and not p['sample_known'] for p in packets)
    with pytest.raises(ValueError):session.select_target('missing','ball',[0,0,0])


@pytest.mark.parametrize('fault,status',[('frozen',3),('disconnected',4),('out_of_range',2),('no_contact',1)])
def test_native_faults_including_freeze_from_startup(fault,status):
    session,*_=native_session(fault=fault)
    _,packets,_=session.advance_to(50_000_000);channel=[p for p in packets if p['channel']==1]
    assert channel[-1]['status']==status
    if fault=='frozen':
        assert channel[-1]['sequence']==channel[0]['sequence']
        assert channel[-1]['sample_ns']==channel[0]['sample_ns']
    if fault=='disconnected':assert np.isnan(channel[-1]['reading_m'])


def test_collision_permissions_are_exact_bounded_and_do_not_hide_other_objects():
    c=workflow_config();e,_=nominal_kinematics(c);v=c['workflow']['visits'][0]
    p=fixture_profile(e,c,(v['frame'],v['ball_id']),True)
    negative=np.flatnonzero(p.pair_clearance_m<0)
    assert len(negative)==3
    for j in negative:
        names={p.names[k] for k in p.pairs[j]}
        assert v['ball_id'] in names and any('_tip_' in x for x in names)
        assert p.pair_clearance_m[j]==-.005
    assert any('housing' in x for x in p.names) and 'ball_1_stem' in p.names
    q=np.array(v['nominal_q_rad'])
    assert certify_polyline(p,np.vstack([q,q]),clearance_m=.01)['passed']
    # A housing obstacle at the same target cannot be covered by tip exceptions.
    blocked=deepcopy(c);center=e.frame_pose(q,v['frame'])[:3,3]
    blocked['workflow']['collision_scene']['obstacles'].append(dict(name='unexpected_object',center=center.tolist(),half_extents=[.05]*3,attached_to_base=False))
    other=fixture_profile(e,blocked,(v['frame'],v['ball_id']),True)
    assert not certify_polyline(other,np.vstack([q,q]),clearance_m=.01)['passed']


@pytest.fixture(scope='module')
def measured(tmp_path_factory):
    root=tmp_path_factory.mktemp('trical_k2')/'session';report=run_mock(root,workflow_config(),first_only=True)
    return root,report


def test_closed_loop_integrates_reference_and_reanalyses_without_evaluation(measured,tmp_path):
    root,report=measured;assert report['passed'] and report['measurements']==1
    profile=json.loads((root/'profile.json').read_text());assert 'mock' not in profile
    events=json.loads((root/'events.json').read_text());offsets=[e['offset_sensor_m'] for e in events if e['state']=='read_offset']
    assert len(offsets)>=2 and np.max(abs(np.array(offsets[0])))>3e-5
    assert np.max(abs(np.array(offsets[-1])))<=3e-5
    assert list((root/'journal').glob('*.npz'))
    moved=tmp_path/'hidden';shutil.move(root/'evaluation',moved)
    try:
        analysis=analyse(root,tmp_path/'analysis');assert analysis['passed'] and not analysis['truth_read']
        assert analysis['measured_command_hz']==500.
    finally:shutil.move(moved,root/'evaluation')


@pytest.mark.parametrize('fault',[ 'disconnected','frozen','out_of_range'])
def test_failed_instrument_rejects_measurement_and_recovers(tmp_path,fault):
    report=run_mock(tmp_path/fault,workflow_config(),fault=fault,first_only=True)
    assert not report['passed'] and report['measurements']==0 and report['recovered_to_home']
    assert report['failure'].startswith('indicator_fault_')
    assert json.loads((tmp_path/fault/'measurements.json').read_text())==[]


def test_static_unknown_sample_times_can_be_used_after_declared_settling(tmp_path):
    c=workflow_config();c['sensor']['expose_sample_time']=False
    report=run_mock(tmp_path/'unknown',c,first_only=True)
    assert report['passed']
    assert analyse(tmp_path/'unknown',tmp_path/'analysis')['passed']


def test_missing_all_readings_times_out_without_false_measurement(tmp_path):
    c=workflow_config();c['sensor']['drop_probability']=1.;c['workflow']['measurement_timeout_s']=.4
    report=run_mock(tmp_path/'drop',c,first_only=True)
    assert not report['passed'] and report['failure']=='stable_window_timeout'
    assert report['measurements']==0 and report['recovered_to_home']


def test_preflight_collision_stops_before_movement(tmp_path):
    c=workflow_config();e,_=nominal_kinematics(c);v=c['workflow']['visits'][0]
    center=e.frame_pose(np.array(v['nominal_q_rad']),v['frame'])[:3,3]
    c['workflow']['collision_scene']['obstacles'].append(dict(name='blocked_ball',center=center.tolist(),half_extents=[.07]*3,attached_to_base=False))
    report=run_mock(tmp_path/'blocked',c,first_only=True)
    assert not report['passed'] and report['failure'].startswith('complete_visit_preflight_rejected')
    with np.load(tmp_path/'blocked/raw/commands.npz',allow_pickle=False) as data:assert data['commands'].shape==(0,14,5)


def test_stable_window_does_not_look_at_future_rows(measured):
    from casbot_calib.trical_data import load_observations
    root,_=measured;arrays,_=load_observations(root/'observations')
    measurement=json.loads((root/'measurements.json').read_text())[0];c=json.loads((root/'profile.json').read_text())
    with np.load(root/'raw/row_decision_times.npz',allow_pickle=False) as raw:times=raw['time_ns']
    result=stable_window(arrays,times,measurement['accepted_ns'],c,measurement['stable_after_ns'],measurement['frame'],measurement['ball_id'])
    assert result['indices']==measurement['window']['indices'] and max(times[result['indices']])<=measurement['accepted_ns']


def test_mock_must_fit_declared_encoder_uncertainty_before_collection(tmp_path):
    c=workflow_config();c['mock']['encoder_offset_rad'][0]=.002
    with pytest.raises(ValueError,match='uncertainty'):
        run_mock(tmp_path/'out_of_envelope',c,first_only=True)
    assert not (tmp_path/'out_of_envelope/raw/commands.npz').exists()


def test_collision_identity_covers_generated_mesh_not_only_profile(monkeypatch):
    from casbot_calib import trical_fixture
    c=workflow_config();engine,_=nominal_kinematics(c);before=fixture_profile(engine,c)
    original=trical_fixture.sphere_envelope
    def changed(*args,**kwargs):
        result=original(*args,**kwargs);result.vertices=result.vertices*1.001;return result
    monkeypatch.setattr(trical_fixture,'sphere_envelope',changed)
    after=fixture_profile(engine,c)
    assert before.metadata['geometry_sha256']!=after.metadata['geometry_sha256']


@pytest.mark.parametrize('invalid',['stale','status'])
def test_recovery_does_not_issue_retreat_with_invalid_joint_feedback(invalid):
    from types import SimpleNamespace
    from casbot_calib.trical_workflow import MeasurementController
    c=workflow_config();e,_=nominal_kinematics(c)
    device=SimpleNamespace(now=100_000_000,command_times=[0],latest=dict(q=np.zeros(14),receive_ns=0 if invalid=='stale' else 100_000_000,
        status=np.ones(14,dtype=int) if invalid=='status' else np.zeros(14,dtype=int)))
    controller=MeasurementController(e,c,device)
    assert not controller.recover()
    assert controller.events[-1]['state']=='latched_hold_manual_recovery_required'
