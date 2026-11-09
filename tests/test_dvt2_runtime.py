"""DVT2 consumers share the existing collectors and never cross model contexts."""
import copy
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml
from casbot_calib import _core, hardware_field as field, hardware_batch as batch
from casbot_calib.model_context import load_context
from casbot_calib.model_runtime import model_for,collision_for,context_for,dynamics_scene
from casbot_calib.sdk_mock import SDKMockSession,mock_config
from casbot_calib import hardware_friction as friction,hardware_inertia as inertia
from casbot_calib.dual_model import physical_model
from casbot_calib.data import write_json
from casbot_calib.urdf_resources import resource_elements,resolved_resource
from casbot_calib.staged_collection import synthetic_plant
from casbot_calib.staged_control import load_candidate

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'configs/dvt2'


@pytest.fixture(scope='module')
def profile():return field.load_profile(CONFIG/'hardware_field.yaml')


def snapshot(path,profile):
    path.mkdir();write_json(path/'result.json',dict(passed=True,snapshot_only=True))
    write_json(path/'request.json',dict(config=profile,domain_id=227))
    (path/'execution').mkdir()
    names=['q'+str(i) for i in range(14)]+['v'+str(i) for i in range(14)]
    with (path/'execution/snapshot.csv').open('w') as stream:
        w=csv.writer(stream);w.writerow(names);w.writerow([*context_for(profile).home,*np.zeros(14)])
    field.seal(path);return path


def test_server_client_and_ff_bind_same_new_model(profile,tmp_path):
    c=mock_config(CONFIG/'sdk_mock.yaml');session=SDKMockSession(CONFIG/'sdk_mock.yaml',tmp_path/'not_started',227,profile=CONFIG/'hardware_field.yaml')
    e,s=model_for(profile);context=context_for(profile)
    assert session.config['model_context']==session.profile['model_context']==context.identity
    assert _core.ArmDynamics.from_urdf(field.params(profile,list(context.joint_names))['gravity_model_path']).model.model_id==e.model.model_id
    assert collision_for(profile).metadata['model_context']==context.identity
    assert collision_for(profile).metadata['scene']==profile['scene']
    assert collision_for(profile).metadata['backend']==_core.CollisionChecker.backend_version()
    assert c['initial_q']==context.home.tolist()
    assert not (tmp_path/'not_started').exists()
    # Compatibility config names and explicit DVT2 paths resolve identically.
    assert SDKMockSession(CONFIG/'sdk_mock.yaml',tmp_path/'bad',227,profile=ROOT/'configs/hardware_field.yaml').profile['model_context']==context.identity
    assert SDKMockSession(ROOT/'configs/sdk_mock.yaml',tmp_path/'bad2',227,profile=CONFIG/'hardware_field.yaml').config['model_context']==context.identity
    changed=copy.deepcopy(profile);changed['scene']['obstacles'][0]['center'][2]=100
    with pytest.raises(ValueError,match='collision scene differs'):collision_for(changed)
    snap=snapshot(tmp_path/'snap',profile)
    assert field.snapshot_binding(snap,field.load_profile(ROOT/'configs/hardware_field.yaml'))


def test_h2_protocol_reloads_new_model_without_extra_selectors(tmp_path,profile,monkeypatch):
    c=yaml.safe_load((ROOT/'configs/hardware_batch.yaml').read_text())
    c['static']['enabled']=c['frequency']['enabled']=False
    c['direction']['joints']=['left_shoulder_pitch_joint','right_shoulder_pitch_joint']
    path=tmp_path/'batch.yaml';path.write_text(yaml.safe_dump(c))
    monkeypatch.setattr(batch,'implementation',lambda:dict(test=True))
    folder=tmp_path/'batch';batch.plan_batch(path,CONFIG/'hardware_field.yaml',folder,domain_id=227)
    _,p=batch.load_batch(folder)
    assert p['profile']['model_context']==profile['model_context']
    assert p['source_model_sha256']==model_for(profile)[1]['sha256']
    assert len(p['jobs'])==2


@pytest.fixture
def gravity(tmp_path,profile):
    """Synthetic mean observations only; no truth enters the shared fit."""
    nominal,source=model_for(profile);true=physical_model(nominal,1.03,0,0)
    rng=np.random.default_rng(661106);means=[]
    for split,count in [('train',20),('validation',8)]:
        for pose in range(count):
            q=context_for(profile).home+rng.uniform(-.2,.2,14)
            for sign in [-1,1]:
                means.append(dict(conditions=dict(batch=split,pose=pose,approach=sign),q=q,torque=true.gravity(q)+sign*.02))
    out=tmp_path/'gravity';out.mkdir()
    meta=dict(joint_names=[j.name for j in nominal.model.joints],source_model_sha256=source['sha256'])
    report=field.write_gravity_candidate(means,profile,out,meta);write_json(out/'report.json',report);field.seal(out)
    assert report['passed'] and report['commissioning_validation_passed']
    assert not report['hardware_calibration_accepted']
    return out


def test_h2_gravity_export_h3_load_and_relocation_use_same_source(gravity,tmp_path,profile):
    _,calibration,_=friction.gravity_source(gravity,profile)
    assert calibration['model_context']==profile['model_context']
    e=friction.gravity_engine(gravity);assert e.model.model_id==context_for(profile).model_id
    new=tmp_path/'relocated';friction.copy_gravity(gravity,new);field.check_seal(new)
    assert (new/'candidate_exploratory.urdf').read_bytes()!=(gravity/'candidate_exploratory.urdf').read_bytes()
    resources=resource_elements(ET.parse(new/'candidate_exploratory.urdf').getroot());assert resources
    assert all(Path(resolved_resource(r.get('filename'),new/'candidate_exploratory.urdf')).exists() for r in resources)
    np.testing.assert_allclose(friction.gravity_engine(new).gravity(context_for(profile).home),e.gravity(context_for(profile).home),atol=1e-12)
    assert friction.gravity_source(gravity,field.load_profile(ROOT/'configs/hardware_field.yaml'))


@pytest.mark.parametrize('joint',[0,7])
def test_h3_and_h4_prepare_real_collector_plans_on_new_model(gravity,tmp_path,profile,joint):
    context=context_for(profile);names=list(context.joint_names);root=tmp_path/'batch';(root/'protocol').mkdir(parents=True)
    friction.copy_gravity(gravity,root/'protocol/gravity')
    c=friction.settings(CONFIG/'hardware_friction.yaml',names)
    snap=snapshot(tmp_path/'snapshot',profile)
    protocol=dict(h3=c,joint_names=names)
    job=dict(id='friction_test',kind='moving_friction',joint=names[joint],hz=500,split='train',pose=0,pose_offset_rad=0.,speed=.04)
    report=friction.prepare_trial(root,protocol,job,CONFIG/'hardware_field.yaml',snap,tmp_path/'h3_plan')
    assert report['passed'] and report['source_model_sha256']==model_for(profile)[1]['sha256']
    ramp=dict(job,id='ramp_test',kind='breakaway',direction=1,dwell_s=1.,slope_nm_s=.12,repeat=0)
    ramp_report=friction.prepare_trial(root,protocol,ramp,CONFIG/'hardware_field.yaml',snap,tmp_path/'ramp_plan')
    assert ramp_report['passed']
    # H4 consumes the same gravity candidate and writes a full FF plan for control.
    h4=inertia.settings(CONFIG/'hardware_inertia.yaml',names)
    prior=root/'protocol/prior';prior.mkdir();friction.copy_gravity(gravity,prior/'gravity')
    calibration=json.loads((gravity/'calibration.json').read_text())
    write_json(prior/'prior.json',dict(parameters=calibration['parameters'],viscous=[.03]*14,coulomb=[.04]*14,static_friction=[.07]*14))
    field.seal(prior)
    h4['joints']=[names[joint]]
    job4=dict(id='inertia_test',kind='inertia',joint='both_arms',hz=500,split='train',pose=0,pose_offset_rad=0.,seed=401)
    report4=inertia.prepare_trial(root,dict(h4=h4,joint_names=names),job4,CONFIG/'hardware_field.yaml',snap,tmp_path/'h4_plan')
    assert report4['passed'] and report4['source_model_sha256']==report['source_model_sha256']
    from scipy.interpolate import BPoly
    control_job=dict(job4,id='control_test',kind='control_validation',feedforward='prior',gain_strategy='original')
    control=tmp_path/'control_plan'
    control_report=inertia.prepare_trial(root,dict(h4=h4,joint_names=names),control_job,CONFIG/'hardware_field.yaml',snap,control)
    values=np.loadtxt(control/'feedforward_plan.csv',delimiter=',',skiprows=1)
    with np.load(control/'reference.npz') as data:curve=BPoly(data['coefficients'],data['breakpoints'])
    t=np.minimum(values[:,0]*1e-9,curve.x[-1]);model=inertia.candidate_engine(root)
    expected=model.evaluate_batch(curve(t),curve(t,nu=1),curve(t,nu=2))['total']
    np.testing.assert_allclose(values[:,1:],expected,rtol=1e-13,atol=1e-13)
    assert control_report['config']['gravity_feedforward']=='none'
    assert not control_report['config']['breakaway_parameters']['gravity_feedforward_enabled']



def test_staged_synthetic_plant_and_candidate_retain_context(profile,tmp_path):
    c=yaml.safe_load((CONFIG/'staged_dual_arm.yaml').read_text());e=synthetic_plant(c)
    assert e.model.model_id==context_for(profile).model_id
    candidate=dict(model_version='tree_full_physical_inertia_v1',model_context=c['model_context'],
        source_sha256=model_for(c)[1]['sha256'],joint_names=list(context_for(c).joint_names),gain_status='synthetic_declared',
        parameters=e.rigid_parameters().tolist(),viscous=[.035]*14,coulomb=[.04]*14,static_friction=[.07]*14)
    path=tmp_path/'candidate.json';write_json(path,candidate)
    restored=load_candidate(path)
    np.testing.assert_allclose(restored.rigid_parameters(),e.rigid_parameters(),atol=1e-13)
    assert restored.model.model_id==e.model.model_id


def test_staged_collection_records_new_source_and_rejects_mixed_engine(tmp_path,profile):
    from casbot_calib.staged_collection import collect_trial
    from casbot_calib.staged_motion import Motion,gate_motion
    from casbot_calib.staged_identification import recordings
    from casbot_calib.vendor_identity import sha256
    from casbot_calib.dual_model import nominal_dual_arm
    config=yaml.safe_load((CONFIG/'staged_dual_arm.yaml').read_text());config.pop('mock')
    engine,_=model_for(config);collision=collision_for(config,scene=profile['scene'])
    home=context_for(config).home;m=Motion('hold','gravity',home,dict(batch='train',pose=0,approach=-1))
    m.add(.05,home,label='static_measure');curve=m.polynomial();root=tmp_path/'collection';plan=root/'plans/hold';plan.mkdir(parents=True)
    write_json(plan/'motion.json',m.metadata());np.savez(plan/'reference.npz',coefficients=curve.c,breakpoints=curve.x)
    gate=gate_motion(collision,m);assert gate['passed'];write_json(plan/'collision_reference.json',gate)
    write_json(plan/'manifest.json',dict(motion_sha256=sha256(plan/'motion.json'),reference_sha256=sha256(plan/'reference.npz'),reference_gate_passed=True))
    write_json(root/'plan_report.json',dict(passed=True,selected_trials=['hold']))
    write_json(root/'session.json',dict(config=config,collision=collision.metadata,joint_names=list(context_for(config).joint_names)))
    wrong=_core.ArmDynamics.from_urdf(str(ROOT/'tests/fixtures/legacy_nominal/casbot_dual_arm_benchmark_nominal.urdf'))
    with pytest.raises(ValueError,match='plant/feedforward model differs'):
        collect_trial(root,'hold',wrong,engine,config,collision)
    obs,report=collect_trial(root,'hold',engine,engine,config,collision)
    assert report['passed'] and report['model_context']==config['model_context']
    name,meta,loaded=next(recordings(root,'gravity'))
    np.testing.assert_array_equal(loaded['q'],obs['q'])
    raw_path=root/'runs/hold/metadata.json';saved=json.loads(raw_path.read_text());saved.pop('model_context');write_json(raw_path,saved)
    with pytest.raises(ValueError,match='model context differs'):next(recordings(root,'gravity'))


def test_p0_cannot_silently_load_a_model_different_from_its_context(tmp_path,profile):
    from casbot_calib.staged_protocol import prepare_protocol
    from casbot_calib.staged_plan import plan_session
    config=dict(schema_version=2,model_context=profile['model_context'])
    for entry in (prepare_protocol,plan_session):
        with pytest.raises(ValueError,match='P0 declared model differs'):
            entry(tmp_path/'unsupported',config)
    assert not (tmp_path/'unsupported').exists()
