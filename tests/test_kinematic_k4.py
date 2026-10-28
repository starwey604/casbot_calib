"""K4 frozen-parameter fitting, repeated-batch statistics, metrology and playback."""
from copy import deepcopy
import json
import os
import numpy as np
import pytest
from casbot_calib.kinematics import workflow_config,nominal_kinematics,geometry_manifest
from casbot_calib.kinematic_parameters import KinematicParameters
from casbot_calib.kinematic_identification import solve
from casbot_calib.kinematic_acceptance import repeatability,load_protocol
from casbot_calib.kinematic_metrology import save_meter,load_meter,evaluate_meter,UNITS
from casbot_calib.kinematic_presentation import delivered_state,delivered_indicators


def model():
    config=workflow_config();engine,_=nominal_kinematics(config);return config,engine,KinematicParameters(engine)


def dataset(p,x,n,seed):
    rng=np.random.default_rng(seed);q=rng.uniform(-.7,.7,(n,14));frames=np.array(['left_trical_tcp','right_trical_tcp']*(n//2));offsets=np.zeros((n,3))
    positions,_=p.evaluate(x,q,frames,offsets)
    return dict(q=q,frames=frames,offsets=offsets,targets=positions+rng.normal(0,2e-7,positions.shape))


def test_repeat_fits_use_frozen_subset_and_reject_invalid_indices():
    _,_,p=model();x=np.zeros(p.size);x[2]=.1;x[22]=.05
    train=dataset(p,x,100,100);heldout=dataset(p,x,40,101)
    first,_,_=solve(p,train,heldout);columns=first['selected_parameters']
    second,_,_=solve(p,dataset(p,x,100,102),heldout,selected_parameters=columns)
    assert second['passed'] and second['selected_parameters']==columns
    assert second['selection_source']=='frozen_reference_training_subset'
    for invalid in [columns+[columns[0]],[-1],columns[1:]]:
        with pytest.raises(ValueError,match='subset'):solve(p,train,heldout,selected_parameters=invalid)
    insufficient=deepcopy(train);insufficient['q'][:]=insufficient['q'][0]
    rejected,_,_=solve(p,insufficient,heldout,selected_parameters=columns)
    assert not rejected['passed'] and rejected['reason']=='frozen_subset_unobservable_or_ill_conditioned'


def test_repeatability_is_sample_std_in_common_gauge_and_rejects_drift():
    _,_,p=model();d=dataset(p,np.zeros(p.size),12,103);v=np.zeros((3,p.size));v[:,0]=[-.001,0,.001]
    r=repeatability(p,v,d['q'],d['frames']);assert r['passed']
    assert r['degrees_of_freedom']==2 and r['parameter_std_scaled'][0]==pytest.approx(.001)
    v[2,0]=2.
    assert not repeatability(p,v,d['q'],d['frames'])['passed']
    with pytest.raises(ValueError,match='three'):repeatability(p,v[:2],d['q'],d['frames'])


def meter_fixture(p):
    rng=np.random.default_rng(104);base=rng.uniform(-.4,.4,(6,14));q=np.repeat(base,3,axis=0)
    frames=np.repeat(['left_trical_tcp']*3+['right_trical_tcp']*3,3);position,_=p.evaluate(np.zeros(p.size),q,frames,np.zeros((18,3)))
    t=np.arange(18,dtype=np.int64)*1_000_000_000+1_000_000
    a=dict(q_rad=q,dq_rad_s=np.zeros_like(q),position_meter_m=position,tool_frame=frames,
        point_id=np.repeat(np.array([f'p{i}' for i in range(6)]),3),sequence=np.arange(18,dtype=np.int64),status=np.zeros(18,dtype=np.uint8),
        joint_sample_ns=t,joint_receive_ns=t+100,meter_sample_ns=t,meter_receive_ns=t+200)
    m=dict(schema='kinematic_external_position_v1',units=UNITS,registration_frozen_before_calibration=True,
        registration_estimated_from_validation=False,registration_source='separate_synthetic_registration',registration_id='independent_fixture_1',
        robot_from_meter=np.eye(4).tolist(),timing='common_clock_static_hold',clock_domain='test_ns',source_batch_id='external_test_batch',synthetic_only=True,
        measurement_std_m=1e-6,joint_names=[j.name for j in p.nominal.model.joints],source_geometry_sha256=geometry_manifest(p.nominal,'nominal')['geometry_sha256'])
    return a,m


def test_external_meter_registered_position_and_repeatability_with_independent_noise(tmp_path):
    _,_,p=model();a,m=meter_fixture(p);a['position_meter_m']+=np.random.default_rng(105).normal(0,1e-6,(18,3))
    rotation=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    transform=np.eye(4);transform[:3,:3]=rotation;transform[:3,3]=[.1,-.2,.3]
    a['position_meter_m']=(a['position_meter_m']-transform[:3,3])@rotation;m['robot_from_meter']=transform.tolist()
    save_meter(tmp_path/'meter',a,m);r=evaluate_meter(tmp_path/'meter',p,np.zeros(p.size))
    assert r['passed'] and r['status']=='measured_synthetic' and r['position']['position_rmse_m']<5e-6
    with pytest.raises(ValueError,match='reuses'):evaluate_meter(tmp_path/'meter',p,np.zeros(p.size),excluded_batch_ids=['external_test_batch'])
    a['position_meter_m']+=.001;save_meter(tmp_path/'bad',a,m)
    assert not evaluate_meter(tmp_path/'bad',p,np.zeros(p.size))['passed']


@pytest.mark.parametrize('fault',['registration','units','clock','motion','old_sequence','unknown_time','same_posture'])
def test_external_meter_rejects_invalid_metrology_before_acceptance(tmp_path,fault):
    _,_,p=model();a,m=meter_fixture(p)
    if fault=='registration':m['registration_estimated_from_validation']=True
    if fault=='units':m['units']=dict(UNITS,q_rad='degree')
    if fault=='clock':m['timing']='unknown_clock'
    if fault=='motion':a['dq_rad_s'][0,0]=.02
    if fault=='old_sequence':a['sequence'][1]=a['sequence'][0]
    if fault=='unknown_time':a['meter_sample_ns'][0]=-1
    if fault=='same_posture':a['q_rad'][:9]=a['q_rad'][0]
    with pytest.raises(ValueError):save_meter(tmp_path/'invalid',a,m)
    assert not (tmp_path/'invalid').exists()


def test_external_meter_file_tampering_is_rejected(tmp_path):
    _,_,p=model();a,m=meter_fixture(p);save_meter(tmp_path/'meter',a,m)
    (tmp_path/'meter/metadata.json').write_text('{}')
    with pytest.raises(ValueError,match='digest'):load_meter(tmp_path/'meter')


def test_playback_uses_only_delivered_joint_and_selected_target_packets():
    data=dict(joints=dict(receive_ns=np.array([10,20],dtype=np.int64),q=np.array([[1.],[2.]])),
        indicators=dict(receive_ns=np.array([10,15,30]),channel=np.array([0,0,0]),tool_frame=np.array(['left','right','left']),
            target_id=np.array(['ball','other','ball']),reading_m=np.array([1.,2.,3.])))
    assert delivered_state(data,19)['q'][0]==1.
    with pytest.raises(ValueError):delivered_state(data,9)
    packets=delivered_indicators(data,20,'left','ball');assert packets[0]['reading_m']==1. and packets[1] is None


def test_protocol_cannot_be_revised_after_captures(tmp_path):
    from hashlib import sha256
    p=tmp_path/'protocol.json';p.write_text('{}')
    (tmp_path/'protocol_digest.json').write_text(json.dumps(dict(sha256=sha256(p.read_bytes()).hexdigest())))
    p.write_text('{"settings":{}}')
    with pytest.raises(ValueError,match='protocol changed'):load_protocol(tmp_path)


def test_playback_does_not_adopt_late_old_joint_feedback(tmp_path):
    from casbot_calib.kinematic_presentation import load_playback
    from hashlib import sha256
    (tmp_path/'raw').mkdir()
    np.savez_compressed(tmp_path/'raw/joint_packets.npz',sequence=np.array([0,2,1,3]),receive_ns=np.array([10,20,30,40]),q=np.array([[0.],[2.],[1.],[3.]]))
    np.savez_compressed(tmp_path/'raw/indicator_packets.npz',channel=np.array([],dtype=int))
    for name in ('motions','measurements','events'):(tmp_path/(name+'.json')).write_text('[]')
    names=['raw/joint_packets.npz','raw/indicator_packets.npz','motions.json','measurements.json','events.json']
    (tmp_path/'manifest.json').write_text(json.dumps(dict(files={name:sha256((tmp_path/name).read_bytes()).hexdigest() for name in names})))
    playback=load_playback(tmp_path)
    assert delivered_state(playback,35)['q'][0]==2.
    assert delivered_state(playback,45)['q'][0]==3.
    (tmp_path/'manifest.json').write_text(json.dumps(dict(files={})))
    with pytest.raises(ValueError,match='omits'):load_playback(tmp_path)


@pytest.mark.skipif(os.environ.get('CASBOT_TEST_RENDER')!='1',reason='Explicit EGL render check')
def test_trical_scene_uses_mujoco_only_for_geometry_and_matches_pinocchio(tmp_path,monkeypatch):
    from casbot_calib.kinematic_presentation import scene
    from casbot_calib.viewer import JointView
    from casbot_calib.telemetry import StateFrame
    import mujoco
    def forbidden(*args,**kwargs):raise AssertionError('Physics/collision called by viewer')
    for name in ('mj_step','mj_forward','mj_inverse','mj_collision'):monkeypatch.setattr(mujoco,name,forbidden)
    config,engine,_=model();adapter=scene(engine,config,tmp_path/'scene.xml');q=np.array(config['workflow']['home_rad'])
    frame=StateFrame(adapter.robot_model_id,'unit_geometric_probe',0,0,0,tuple(j.name for j in engine.model.joints),q,np.zeros(14),np.zeros(14),None)
    with JointView(adapter,tmp_path/'scene.xml',width=400,height=300) as view:
        assert view.render(frame).shape==(300,400,3)
        for name in ('left_trical_tcp','right_trical_tcp'):
            i=mujoco.mj_name2id(view.model,mujoco.mjtObj.mjOBJ_SITE,name)
            np.testing.assert_allclose(view.data.site_xpos[i],engine.frame_pose(q,name)[:3,3],atol=1e-12)
        assert not np.any(view.data.qacc)
