"""K3 exact charts, independent derivatives, rank rejection and held-out fitting."""
from copy import deepcopy
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from casbot_calib import _core
from casbot_calib.kinematics import workflow_config,nominal_kinematics
from casbot_calib.kinematic_parameters import KinematicParameters,extract_chart,mdh_row
from casbot_calib.kinematic_identification import solve,select_parameters
from casbot_calib.kinematic_export import export_urdf,export_table


def parameters():
    config=workflow_config();engine,_=nominal_kinematics(config);return config,engine,KinematicParameters(engine)


def test_nominal_and_nonzero_mdh_native_frames_and_inertias_are_equivalent():
    _,native,p=parameters();rng=np.random.default_rng(91)
    for values in (np.zeros(p.size),rng.uniform(-.1,.1,p.size)):
        original=p.build(values);mdh=p.equivalent_mdh_engine(values)
        for q in rng.uniform(-.5,.5,(6,14)):
            for frame in ('left_trical_tcp','right_trical_tcp','left_palm_base_link','right_palm_base_link'):
                np.testing.assert_allclose(original.frame_pose(q,frame),mdh.frame_pose(q,frame),atol=2e-14)
            np.testing.assert_allclose(original.mass_matrix(q),mdh.mass_matrix(q),atol=2e-13)
            if not np.any(values):np.testing.assert_allclose(original.mass_matrix(q),native.mass_matrix(q),atol=2e-13)


def test_full_geometry_derivative_uses_pinocchio_placement_regressor_and_nonzero_readout():
    _,_,p=parameters();rng=np.random.default_rng(92);x=rng.uniform(-.15,.15,p.size)
    q=rng.uniform(-.5,.5,(4,14));frames=np.array(['left_trical_tcp','right_trical_tcp']*2);offsets=rng.uniform(-.002,.002,(4,3))
    _,jac=p.evaluate(x,q,frames,offsets);h=1e-4
    for i in range(p.size):
        delta=np.zeros(p.size);delta[i]=h
        derivative=(p.evaluate(x+delta,q,frames,offsets)[0]-p.evaluate(x-delta,q,frames,offsets)[0])/(2*h)
        np.testing.assert_allclose(jac[:,:,i],derivative,atol=5e-11,rtol=1e-7)
    left=[i for i,d in enumerate(p.definitions) if d['side']=='left'];right=[i for i,d in enumerate(p.definitions) if d['side']=='right']
    np.testing.assert_allclose(jac[frames=='left_trical_tcp'][:,:,right],0.,atol=0)
    np.testing.assert_allclose(jac[frames=='right_trical_tcp'][:,:,left],0.,atol=0)


def test_point_regressor_native_shapes_errors_and_joint_angle_derivative():
    _,engine,_=parameters();q=np.linspace(-.2,.3,14)[None];d=np.array([[.001,-.002,.0015]])
    result=engine.point_geometry_batch(q,'left_trical_tcp',d)
    assert result['placement_jacobians'].shape==(1,3,84)
    pose=engine.frame_pose(q[0],'left_trical_tcp')
    np.testing.assert_allclose(result['positions'][0],pose[:3,3]+pose[:3,:3]@d[0],atol=1e-15)
    for i in range(14):
        direction=np.eye(14)[i]*1e-6
        plus=engine.point_geometry_batch(q+direction,'left_trical_tcp',d)['positions'][0]
        minus=engine.point_geometry_batch(q-direction,'left_trical_tcp',d)['positions'][0]
        np.testing.assert_allclose(result['joint_jacobians'][0,:,i],(plus-minus)/2e-6,atol=2e-10)
    assert engine.point_geometry_batch(np.empty((0,14)),'left_trical_tcp',np.empty((0,3)))['positions'].shape==(0,3)
    with pytest.raises(ValueError):engine.point_geometry_batch(q,'left_trical_tcp',np.full((1,3),np.nan))
    with pytest.raises(ValueError):engine.point_geometry_batch(q,'unknown',d)


def parallel_fixture(beta):
    model=_core.ArmModel();model.model_id='analytic_parallel_dual';model.model_version='tree_rigid_v1';parents=[];joints=[];frames=[]
    for side,y in [('left',.5),('right',-.5)]:
        begin=len(joints)
        for i in range(7):
            joint=_core.ArmJoint();joint.name=f'{side}_j{i}';joint.link_name=f'{side}_link{i}'
            origin=_core.ArmPose();origin.translation=[0,y,0] if i==0 else [.1,0,.03 if i==1 else 0]
            if i==1:origin.rotation=Rotation.from_rotvec([0,beta,0]).as_matrix()
            joint.origin=origin;joints.append(joint);parents.append(-1 if i==0 else begin+i-1)
        frame=_core.ArmFrame();frame.name=side+'_trical_tcp';frame.parent=begin+6
        pose=_core.ArmPose();pose.translation=[.1,.02,.03];frame.placement=pose;frames.append(frame)
    model.parents=parents;model.joints=joints;model.frames=frames;return _core.ArmDynamics(model)


def test_hayati_parallel_chart_is_continuous_without_large_common_normal_offsets():
    records=[]
    for beta in (-1e-7,0.,1e-7):
        engine=parallel_fixture(beta);p=KinematicParameters(engine)
        assert p.chart['sides']['left']['rows'][1]['convention']=='hayati'
        row=p.chart['sides']['left']['rows'][1]['values'];records.append(row)
        assert abs(row['d'])<1. and abs(row['a'])<1.
        original=p.build(np.zeros(p.size))
        for q in np.random.default_rng(93).uniform(-.3,.3,(4,14)):
            np.testing.assert_allclose(original.frame_pose(q,'left_trical_tcp'),engine.frame_pose(q,'left_trical_tcp'),atol=2e-13)
    assert abs(records[-1]['beta']-records[0]['beta'])<1e-6


def test_parallel_fixture_has_independent_planar_position_oracle():
    engine=parallel_fixture(0.);p=KinematicParameters(engine);q=np.zeros(14);q[:2]=[.4,-.7]
    angle=np.cumsum(q[:7]);expected=np.array([np.sum(.1*np.cos(angle))-.02*np.sin(angle[-1]),
        .5+np.sum(.1*np.sin(angle))+.02*np.cos(angle[-1]),.06])
    np.testing.assert_allclose(p.build(np.zeros(p.size)).frame_pose(q,'left_trical_tcp')[:3,3],expected,atol=2e-14)


def test_root_phase_is_represented_once_and_chart_digest_is_checked():
    _,engine,p=parameters();assert p.size==60
    assert not any(d['row']==0 and d['location']=='row' for d in p.definitions)
    x=np.zeros(p.size);offset=np.linspace(-.0005,.0005,14)
    for i,d in enumerate(p.definitions):
        if d['zero_stage']:
            j=p.chart['sides'][d['side']]['indices'][d['row']];x[i]=offset[j]/d['scale']
    q=np.linspace(-.3,.4,14)
    for frame in ('left_trical_tcp','right_trical_tcp'):
        np.testing.assert_allclose(p.build(x).frame_pose(q,frame),engine.frame_pose(q+offset,frame),atol=1e-13)
    bad=deepcopy(p.chart);bad['sides']['left']['rows'][1]['values']['a']+=.001
    with pytest.raises(ValueError,match='identity'):KinematicParameters(engine,bad)


def mathematical_dataset(p,x,count,seed,noise=0.):
    # Pure solver unit fixture: independent externally measured 3D points, not a
    # claimed fixed-ball collection or an alternative acquisition implementation.
    rng=np.random.default_rng(seed);q=rng.uniform(-.8,.8,(count,14));frames=np.array(['left_trical_tcp','right_trical_tcp']*(count//2))
    offsets=rng.uniform(-.001,.001,(count,3));points,_=p.evaluate(x,q,frames,offsets)
    return dict(q=q,frames=frames,offsets=offsets,targets=points+rng.normal(0,noise,points.shape))


def test_bounded_two_layer_fit_and_validation_do_not_change_parameter_selection():
    _,_,p=parameters();x=np.zeros(p.size);x[3]=.15;x[8]=-.1;x[22]=.15;x[47]=-.1
    train=mathematical_dataset(p,x,100,94,2e-7);validation=mathematical_dataset(p,x,40,95,2e-7)
    report,estimate,_=solve(p,train,validation)
    assert report['passed'] and report['validation']['geometry_layer']['position_rmse_m']<1e-6
    changed=deepcopy(validation);changed['targets']+=.001
    failed,second,_=solve(p,train,changed)
    assert not failed['passed'] and report['selected_parameters']==failed['selected_parameters']
    np.testing.assert_allclose(estimate,second,atol=0.,rtol=0.)


def test_four_measurements_or_repeated_pose_do_not_claim_identifiability():
    _,_,p=parameters();data=mathematical_dataset(p,np.zeros(p.size),4,96)
    report,estimate,_=solve(p,data,data)
    assert not report['identifiable'] and estimate is None
    data=mathematical_dataset(p,np.zeros(p.size),40,97);data['q'][:]=data['q'][0]
    report,estimate,_=solve(p,data,data)
    assert not report['identifiable'] and estimate is None


def test_two_targets_per_arm_reject_geometry_even_with_full_rank_zero_layer():
    _,_,p=parameters();data=mathematical_dataset(p,np.zeros(p.size),100,99)
    for frame in np.unique(data['frames']):
        indices=np.flatnonzero(data['frames']==frame)
        data['targets'][indices]=np.array([[.4,.3,.2],[.5,.4,.3]])[np.arange(len(indices))%2]
    report,estimate,_=solve(p,data,data)
    assert report['zero_stage_observability']['rank']==14
    assert not report['identifiable'] and estimate is None
    assert report['reason']=='insufficient_noncollinear_metrology_targets'


def test_k3_profile_has_three_noncollinear_metric_targets_per_arm():
    from casbot_calib.kinematic_design import k3_config
    from casbot_calib.kinematics import ball_world
    config=k3_config()
    assert config['parameter_policy']['fit_enabled'] is True
    current,manifest=nominal_kinematics(config)
    legacy=deepcopy(config);legacy['parameter_policy']['fit_enabled']=False
    _,old_manifest=nominal_kinematics(legacy)
    assert manifest['geometry_sha256']==old_manifest['geometry_sha256']
    for frame in ('left_trical_tcp','right_trical_tcp'):
        points=np.array([ball_world(config,v['ball_id']) for v in config['workflow']['visits'] if v['frame']==frame])
        assert len(points)==3 and np.linalg.matrix_rank(points-points.mean(axis=0),tol=1e-9)==2
    assert config['control_period_ns']==2_000_000


def test_design_rejects_invalid_uncertainty_frequency_and_condition_settings(tmp_path):
    from casbot_calib.kinematic_design import k3_config
    import yaml
    path=tmp_path/'profile.yaml'
    for key,value in [('sigma_m',0.),('variation_rad',float('nan')),('maximum_condition',1.),('validation_limit_m',-1.),('pool_per_ball',True)]:
        path.write_text(yaml.safe_dump(dict(profile='trical_geometry_v1',design={key:value})))
        with pytest.raises(ValueError):k3_config(path)


def test_frozen_plan_rejects_content_and_parameter_convention_changes(tmp_path):
    from hashlib import sha256
    import json
    from casbot_calib.kinematic_workflow import _verified_plan
    config,_,p=parameters();config.pop('mock',None)
    files={'profile.json':config,'parameterization.json':p.description(),'plan_report.json':{'passed':True}}
    for name,value in files.items():(tmp_path/name).write_text(json.dumps(value))
    def manifest():
        (tmp_path/'manifest.json').write_text(json.dumps({'files':{name:sha256((tmp_path/name).read_bytes()).hexdigest() for name in files}}))
    manifest();_verified_plan(tmp_path)
    (tmp_path/'profile.json').write_text('{}')
    with pytest.raises(ValueError,match='digest'):_verified_plan(tmp_path)
    (tmp_path/'profile.json').write_text(json.dumps(config))
    bad=p.description();bad['parameters'][0]['scale']*=2
    (tmp_path/'parameterization.json').write_text(json.dumps(bad));manifest()
    with pytest.raises(ValueError,match='convention'):_verified_plan(tmp_path)


def test_hidden_geometry_must_fit_declared_collision_family_before_commands():
    from casbot_calib.kinematic_design import k3_config
    from casbot_calib.trical_device import make_mock
    config=k3_config();hidden=config.pop('mock');engine,_=nominal_kinematics(config)
    assert make_mock(engine,config,hidden).now==0
    bad=deepcopy(hidden);bad['geometry_chart_sha256']='incorrect'
    with pytest.raises(ValueError,match='chart'):make_mock(engine,config,bad)
    config['workflow']['geometry_error_budget_m']=1e-9
    with pytest.raises(ValueError,match='displacement budget'):make_mock(engine,config,hidden)


def test_exported_urdf_preserves_calibrated_frames_and_dynamics(tmp_path):
    config,native,p=parameters();x=np.random.default_rng(98).uniform(-.1,.1,p.size)
    result=export_urdf(native,p.build(x),config,tmp_path/'robot.urdf')
    assert result['passed'] and result['max_pose_error']<1e-12 and result['max_mass_matrix_error']<1e-12
    assert result['collision_revalidation_required'] and result['dynamics_revalidation_required']
    export_table(p,x,tmp_path)
    import csv
    with (tmp_path/'mount_table.csv').open(encoding='utf-8-sig',newline='') as stream:mounts=list(csv.DictReader(stream))
    assert len(mounts)==2
    index=next(i for i,d in enumerate(p.definitions) if d['name']=='left.mount_1.rz')
    assert float(mounts[0]['rz_rad'])==x[index]*p.scales[index]
