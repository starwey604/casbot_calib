"""Actual CAD route policies over the shared measurement/identification kernel."""
from pathlib import Path
import copy
import numpy as np
import pytest
from casbot_calib.kinematics import workflow_config,nominal_kinematics
from casbot_calib.dvt2_trical import profiles,route_steps,route,motion_gate
from casbot_calib.kinematic_parameters import KinematicParameters
from casbot_calib.trical_device import make_mock

CONFIG=Path(__file__).resolve().parents[1]/'configs/dvt2/kinematics.yaml'


@pytest.fixture(scope='module')
def actual():
    c=workflow_config(CONFIG);e,_=nominal_kinematics(c);return c,e


def test_actual_four_ball_shared_fixture_and_one_installed_tool(actual):
    c,e=actual
    assert [b['id'] for b in c['fixture']['balls']]==list('ABCD')
    assert {(v['frame'].split('_')[0],v['ball_id']) for v in c['workflow']['visits']}=={(s,b) for s in ('left','right') for b in 'ABCD'}
    for side in ('left','right'):
        p=profiles(e,c,side+'_trical_tcp','D' if side=='left' else 'C')
        assert all(v.metadata['active_tool']==side and v.metadata['inactive_tool'] is None for v in p.values())
        contact=p['contact'];released=p['released']
        assert len(contact.metadata['exact_contact_pairs'])==3
        i=np.flatnonzero(contact.pair_clearance_m<0)
        assert len(i)==3 and np.all(released.pair_clearance_m[i]>=.0005)
        ordinary=contact.pair_clearance_m>=.01
        assert np.all(released.pair_clearance_m[ordinary]>=.01)
        assert contact.metadata['rigid_geometry_displacement_budget_m']==.0001


def test_release_phases_and_invalid_tool_orientation_are_not_relaxed(actual):
    c,e=actual;r=route(c,'left_trical_tcp','D');steps=route_steps(c,r)
    assert [phase for _,_,phase in steps][:2]==['contact','contact']
    assert all(phase=='released' for _,_,phase in steps[2:])
    p=profiles(e,c,'left_trical_tcp','D')['contact'];q=np.array(r['contact'][0]['q']);bad=q.copy();bad[6]+=.1
    result=motion_gate(p,q,bad,c)
    assert not result['passed'] and result['reason']=='contact_orientation_rejected'


def test_mock_geometry_budget_covers_both_mounts_and_rejects_undeclared_rotation(actual):
    c,e=actual;device=make_mock(e,c,c['mock']);assert device.now==0
    hidden=copy.deepcopy(c['mock']);parameters=KinematicParameters(e)
    i=next(i for i,d in enumerate(parameters.definitions) if d['name']=='right.mount_1.rx')
    hidden['geometry_scaled'][i]=.01
    with pytest.raises(ValueError,match='rotation exceeds'):make_mock(e,c,hidden)
    hidden=copy.deepcopy(c['mock']);i=next(i for i,d in enumerate(parameters.definitions) if d['name']=='right.mount_1.tx')
    hidden['geometry_scaled'][i]=3.
    with pytest.raises(ValueError,match='displacement budget'):make_mock(e,c,hidden)


def test_observed_gate_covers_interior_waypoints_and_fails_missing_frames(actual,monkeypatch):
    from casbot_calib import dvt2_trical
    c,e=actual;p=profiles(e,c,'left_trical_tcp','D')['free'];q=np.array(c['workflow']['home_rad'])
    samples=np.tile(q,(3,1));samples[1,0]+=.01;captured=[]
    def bound(profile,a,b,allowance,config):
        captured.append(allowance);return dict(passed=True,queries=1)
    monkeypatch.setattr(dvt2_trical,'bounded_chord',bound)
    assert dvt2_trical.observed_gate(p,samples,np.array([0,.001,.002]),c)['passed']
    assert captured[0][0]>=.01+c['workflow']['encoder_allowance_rad']
    assert not dvt2_trical.observed_gate(p,samples,np.array([0,.001,.1]),c)['passed']


def test_new_view_scenes_have_only_the_installed_tcp(actual,tmp_path):
    import xml.etree.ElementTree as ET
    from casbot_calib.kinematic_presentation import scene
    c,e=actual
    for side in ('left','right'):
        path=tmp_path/(side+'.xml');adapter=scene(e,c,path,side)
        assert adapter.robot_model_id==e.model.model_id
        names=[s.get('name') for s in ET.parse(path).getroot().iter('site')]
        assert side+'_trical_tcp' in names
        assert ('right' if side=='left' else 'left')+'_trical_tcp' not in names


def test_metrology_cannot_drift_away_from_the_frozen_collision_fixture(actual):
    from casbot_calib.trical_fixture import fixture_profile
    from casbot_calib.kinematics import validate_config
    c,e=actual;changed=copy.deepcopy(c);changed['fixture']['balls'][0]['center_m'][0]+=.001
    with pytest.raises(ValueError,match='metrology fixture differs'):fixture_profile(e,changed)
    changed=copy.deepcopy(c);changed.pop('model_context')
    with pytest.raises(ValueError,match='Only the K3'):validate_config(changed)


def test_allocated_reference_tube_is_not_double_counted(actual,tmp_path):
    from casbot_calib.trical_workflow import run_mock
    c,e=actual;w=c['workflow']
    assert w['encoder_allowance_rad']+w['tracking_allowance_rad']==pytest.approx(w['geometry_allowance_rad'])
    hidden=copy.deepcopy(c['mock']);hidden['encoder_offset_rad'][0]=.0001
    with pytest.raises(ValueError,match='encoder/noise'):make_mock(e,c,hidden)
    bad=copy.deepcopy(c);bad['workflow']['tracking_allowance_rad']*=2
    with pytest.raises(ValueError,match='allocation'):run_mock(tmp_path/'no_capture',bad)
    assert not (tmp_path/'no_capture').exists()


def test_frozen_proof_cache_checks_source_and_clearance_policy(actual,tmp_path):
    import json
    from types import SimpleNamespace
    from casbot_calib.dvt2_trical import seed_reference_cache
    from casbot_calib.data import write_json
    from casbot_calib.staged_motion import Motion
    from casbot_calib.artifact_io import sha256
    c,e=actual;c=copy.deepcopy(c);c.pop('mock');source=tmp_path/'source';source.mkdir()
    p=profiles(e,c,'left_trical_tcp','A')['free'];home=np.array(c['workflow']['home_rad']);proof=motion_gate(p,home,home,c)
    assert proof['passed']
    write_json(source/'profile.json',c);write_json(source/'report.json',dict(passed=True,scope='unit reference-proof fixture'))
    record=dict(target=dict(frame='left_trical_tcp',ball_id='A'),route_phase='free',
        collision=dict(geometry_sha256=p.metadata['geometry_sha256'],pair_clearances_sha256=p.metadata['pair_clearances_sha256']),
        motion=Motion('hold','trical',home).add(.1,home).metadata(),reference=proof)
    write_json(source/'motions.json',[record])
    def seal():write_json(source/'manifest.json',dict(files={n:sha256(source/n) for n in ('profile.json','report.json','motions.json')}))
    seal();controller=SimpleNamespace(config=c,engine=e,route_cache={})
    assert seed_reference_cache(controller,source)['proofs_consumed']==1
    changed=copy.deepcopy(c);changed['workflow']['geometry_error_budget_m']*=2
    with pytest.raises(ValueError,match='policy changed'):seed_reference_cache(SimpleNamespace(config=changed,engine=e,route_cache={}),source)
    record['motion']['states'][0][1][0]=.1;write_json(source/'motions.json',[record]);seal()
    with pytest.raises(ValueError,match='rest-to-rest'):seed_reference_cache(controller,source)
    record['motion']['states'][0][1][0]=0.
    record['route_phase']='contact';write_json(source/'motions.json',[record]);seal()
    with pytest.raises(ValueError,match='clearance identity changed'):seed_reference_cache(controller,source)


def test_actual_model_geometry_derivatives_match_independent_finite_differences(actual):
    c,e=actual;p=KinematicParameters(e);q=np.array([v['nominal_q_rad'] for v in c['workflow']['visits']]);frames=np.array([v['frame'] for v in c['workflow']['visits']])
    x=np.linspace(-.05,.05,p.size);offsets=np.tile([.0001,-.0002,.00005],(len(q),1));_,jac=p.evaluate(x,q,frames,offsets)
    h=.001
    for j in range(p.size):
        plus=x.copy();minus=x.copy();plus[j]+=h;minus[j]-=h
        finite=(p.evaluate(plus,q,frames,offsets)[0]-p.evaluate(minus,q,frames,offsets)[0])/(2*h)
        np.testing.assert_allclose(jac[:,:,j],finite,rtol=3e-6,atol=3e-10)
