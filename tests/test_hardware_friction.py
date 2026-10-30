"""H3 observed-plateau fitting, stage gating and SDK ramp planning contracts."""
import csv
from pathlib import Path
import shutil
import numpy as np
import pytest
import yaml
from casbot_calib import hardware_field as field,hardware_batch as batch,hardware_friction as h3
from casbot_calib.data import write_json
from casbot_calib.sdk_connection import mock_connection
from casbot_calib.staged_identification import fit_friction_features
from test_hardware_field import make_inputs,make_capture
from test_hardware_batch import FakeBackend

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture
def fixture(tmp_path,monkeypatch):
    preset,snapshot=make_inputs(tmp_path)
    p=yaml.safe_load((ROOT/'configs/hardware_field.yaml').read_text());p['collision_scene']=str(ROOT/'configs/collision_scene.yaml')
    p.update(anchor_transition_s=.1,initial_hold_s=.05,final_hold_s=.05)
    pp=tmp_path/'field.yaml';pp.write_text(yaml.safe_dump(p));binary=tmp_path/'executor';binary.write_text('unit fixture')
    monkeypatch.setattr(field,'executor_binary',lambda:(binary,dict(test=True)))
    plan=tmp_path/'gravity_plan';metadata=field.prepare_field(pp,preset,snapshot,plan)
    capture=tmp_path/'gravity_capture';make_capture(plan,metadata,capture)
    gravity=tmp_path/'gravity';field.analyse_field(plan,capture,gravity)
    p['connection']=mock_connection('/casbot_calib_s2_h3_unit',222,'a'*64)
    pp.write_text(yaml.safe_dump(p))
    c=yaml.safe_load((ROOT/'configs/hardware_friction.yaml').read_text())
    c.update(joints=['left_shoulder_pitch_joint'],pose_offsets_rad=dict(train=[-.01],validation=[.01]),
        speeds_rad_s=dict(train=[.03,.06],validation=[.045]),transition_s=1.,settle_s=.5,inter_job_pause_s=0.)
    c['breakaway']['enabled']=False
    cp=tmp_path/'h3.yaml';cp.write_text(yaml.safe_dump(c))
    return dict(root=tmp_path,snapshot=snapshot,profile=pp,path=cp,config=c,gravity=gravity)


class Backend(FakeBackend):
    def snapshot(self,profile,output,domain_id):
        self.snapshots+=1;shutil.copytree(self.seed_snapshot,output)
        write_json(Path(output)/'request.json',dict(config=field.load_profile(profile),domain_id=domain_id));field.seal(output)
        return dict(passed=True)
    def execute(self,plan,output,domain_id):
        result=super().execute(plan,output,domain_id)
        path=Path(output)/'execution/feedback.csv'
        with path.open() as stream:rows=list(csv.DictReader(stream))
        for row in rows:
            v=float(row['v0']);row['effort_raw0']=str(float(row['effort_raw0'])+.08*v+.03*np.tanh(v/.001))
        with path.open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        field.seal(output);return result


def create(fixture,name='session'):
    root=fixture['root']/name
    h3.plan_friction(fixture['path'],fixture['profile'],fixture['gravity'],root,domain_id=222)
    return root


def test_shared_fit_recovers_analytic_coefficients_and_rejects_single_speed():
    speeds=np.repeat([.02,.05,.09],10);x=np.column_stack([speeds,np.ones_like(speeds)])
    parameters,report=fit_friction_features(x,x@np.array([.08,.03]),x,x@np.array([.08,.03]),1e-6)
    np.testing.assert_allclose(parameters,[.08,.03],atol=1e-7);assert report['passed']
    with pytest.raises(ValueError,match='Multiple'):fit_friction_features(x[:10],np.ones(10),x[:10],np.ones(10),1.)


def test_h3_plan_freezes_gravity_roles_without_device_io(fixture):
    root=create(fixture);_,p=batch.load_batch(root)
    assert p['workflow']=='h3_friction' and p['connection_scope']=='localhost_contract_mock'
    assert len(p['jobs'])==3 and {j['split'] for j in p['jobs']}=={'train','validation'}
    assert p['device_writes_during_planning']==0
    field.check_seal(root/'protocol/gravity')


def test_moving_friction_shared_batch_resume_and_readonly_fit(fixture):
    root=create(fixture);backend=Backend(fixture['snapshot'])
    first=batch.run_batch(root,backend=backend,max_jobs=1)
    assert first['state']=='checkpoint' and not first['moving_friction']['available']
    final=batch.run_batch(root,resume=True,backend=backend)
    assert final['passed'] and not final['formal_hardware_calibration_accepted']
    axis=final['moving_friction']['axes'][0]
    assert axis['rank']==2 and abs(axis['coulomb_nm']-.03)<.003 and abs(axis['viscous_nm_s_rad']-.08)<.025
    replay=batch.analyse_batch(root,fixture['root']/'readonly')
    assert replay['moving_friction']['axes']==final['moving_friction']['axes']
    assert backend.executions==3 and backend.clears==0


@pytest.mark.parametrize('change',[lambda c:c['pose_offsets_rad'].update(validation=c['pose_offsets_rad']['train']),
    lambda c:c['speeds_rad_s'].update(train=[.03]),lambda c:c.update(stroke_half_rad=.001)])
def test_invalid_experiment_design_is_rejected_before_io(fixture,change):
    c=fixture['config'];change(c);fixture['path'].write_text(yaml.safe_dump(c))
    with pytest.raises(ValueError):create(fixture)
    assert not (fixture['root']/'session').exists()


def test_gravity_scale_change_is_rejected(fixture):
    p=yaml.safe_load(fixture['profile'].read_text());p['effort_gain']=2.
    fixture['profile'].write_text(yaml.safe_dump(p))
    with pytest.raises(ValueError,match='scale changed'):create(fixture)


def test_breakaway_plan_uses_external_gravity_and_padded_return(fixture):
    c=fixture['config'];c['breakaway'].update(enabled=True,dwell_s=[.5,1.5],slopes_nm_s=[.12])
    fixture['path'].write_text(yaml.safe_dump(c));root=create(fixture);_,p=batch.load_batch(root)
    b=Backend(fixture['snapshot']);folder=fixture['root']/'one';folder.mkdir()
    b.snapshot(fixture['profile'],folder/'snapshot',222)
    job=next(j for j in p['jobs'] if j['kind']=='breakaway')
    result=h3.prepare_trial(root,p,job,fixture['profile'],folder/'snapshot',folder/'plan')
    params=result['config']['breakaway_parameters']
    assert params['gravity_feedforward_enabled'] and params['breakaway_enabled']
    assert params['breakaway_ramp_begin_ns']-params['breakaway_free_begin_ns']==600000000
    assert result['command_assumptions']['gravity_source']=='external'
    motion=h3.read(folder/'plan/motion.json')
    assert 'return_measured_start' in motion['conditions']['allowance_labels']
    assert max(motion['conditions']['position_allowance_rad'])==.006


def test_callback_gap_is_not_smoothed_across(fixture):
    obs=dict(receive_ns=np.array([0,100000000],dtype=np.int64))
    motion=dict(conditions=dict(joint_index=0,speed=.03))
    with pytest.raises(ValueError,match='gap'):h3.plateau_pairs(obs,None,motion,None,fixture['config'],{})


def test_joint_box_refinement_certifies_conservative_bound_and_rejects_real_collision():
    from casbot_calib.collision import certify_reference_box
    class Profile:
        radii=np.ones((1,1))
        def __init__(self,unsafe):self.unsafe=unsafe
        def check(self,q):return dict(distances=.015-self.unsafe*abs(np.asarray(q)))
    evaluate=lambda t:np.zeros((len(t),1))
    safe=certify_reference_box(Profile(False),evaluate,1.,[0.],[.006])
    assert safe['passed'] and safe['queries']>1 and safe['queries']<20
    unsafe=certify_reference_box(Profile(True),evaluate,1.,[0.],[.006])
    assert not unsafe['passed']
    p=Profile(False);p.pair_clearance_m=np.array([.016])
    assert not certify_reference_box(p,evaluate,1.,[0.],[.001])['passed']
