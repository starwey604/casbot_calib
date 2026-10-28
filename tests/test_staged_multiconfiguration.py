"""R1 geometry/weighting/leakage contracts and an independent cross-center counterexample."""
from copy import deepcopy
from pathlib import Path
import numpy as np
import pytest
import yaml
from casbot_calib.staged_protocol import build_split_manifest, prepare_protocol, load_protocol
from casbot_calib.staged_multiconfiguration import (center_separation, motion_for_slot, motion_coverage,
    final_prior_recipes, plan_references, verify_motion_file)
from casbot_calib.staged_multifit import assemble_training, score_problems, batch_repeatability


@pytest.fixture(scope='module')
def config():
    return yaml.safe_load(Path('configs/staged_dual_arm_p0.yaml').read_text())


def test_centers_require_both_normalized_and_physical_separation_and_never_wrap(config):
    c=config['p0']['center_design'];amp=np.asarray(config['pose_amplitude_rad']);q=np.zeros(14)
    assert not center_separation(q,q+.001,amp,c)['passed']
    only_one=q.copy();only_one[0]=1
    assert not center_separation(q,only_one,amp,c)['passed']
    assert center_separation(q,q+.1,amp,c)['passed']
    a=q.copy();b=q.copy();a[:2]=np.pi;b[:2]=-np.pi
    assert center_separation(a,b,amp,c)['maximum_difference_rad']==pytest.approx(2*np.pi)


def test_resolved_motion_preserves_home_and_periodic_center_not_common_home(config):
    slot=build_split_manifest(config)['trials'][0]
    center=np.asarray(config['home_rad'])+.1
    motion=motion_for_slot(slot,center,config);c=motion.polynomial()
    assert motion.metadata()['initial_final_home']
    begin=motion.windows('periodic')[0][0];t=begin+np.arange(1024)*6/1024
    np.testing.assert_allclose(c(t).mean(axis=0),center,atol=1e-9)
    assert motion_coverage(motion,config)['passed']
    for point in (0,motion.duration):
        for d in (1,2,3):np.testing.assert_allclose(c(point,nu=d),0,atol=1e-9)
    assert motion.conditions['reference_seed']==slot['reference_seed']


def test_only_left_or_right_moves_during_main_test_window(config):
    slot=next(s for s in build_split_manifest(config)['trials'] if s['split']=='test' and s['active_arm_group']=='left' and s['stage']=='inertia')
    motion=motion_for_slot(slot,config['home_rad'],config)
    t=motion.windows('periodic')[0][0]+np.arange(256)*6/256
    q,v,a,_=motion.at(t)
    assert np.ptp(q[:,:7],axis=0).min()>.15
    np.testing.assert_allclose(v[:,7:],0,atol=1e-10);np.testing.assert_allclose(a[:,7:],0,atol=1e-10)
    assert motion_coverage(motion,config)['passed']


def test_final_predecessor_holdout_is_complete_disjoint_and_not_compressed(config):
    motions=final_prior_recipes(config)
    assert len(motions)==548
    assert [sum(m.stage==s for m in motions) for s in ('gravity','friction','breakaway')]==[16,84,448]
    assert len({m.conditions['acquisition_seed'] for m in motions})==548
    assert all(m.conditions['split']=='test' and m.conditions['purpose']=='final_prior_holdout' for m in motions)
    old=np.asarray(config['home_rad'])-.25*np.asarray(config['pose_amplitude_rad'])
    first=next(m for m in motions if m.stage=='breakaway')
    assert not np.allclose(first.states[2][0],old)


@pytest.fixture(scope='module')
def fake_gated_plan(tmp_path_factory,config):
    # This fixture never certifies collision: only the real R1 run can do that.
    import casbot_calib.staged_multiconfiguration as module
    root=tmp_path_factory.mktemp('r1_contract')/'plan'
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(module,'gate_motion',lambda profile,motion,clearance:dict(passed=True,queries=0,scope='TEST FIXTURE ONLY'))
        prepare_protocol(root,config,planner=plan_references)
    return root


def test_r1_plan_binds_every_reference_and_keeps_old_logical_manifest(fake_gated_plan):
    import json
    root=fake_gated_plan;p,c,m=load_protocol(root)
    assert p['schema_version']==2
    plan=json.loads((root/'motion_plan.json').read_text())
    assert len(plan['trials'])==88 and len(plan['final_prior_trials'])==548
    assert len(plan['files'])==636*4
    assert all(row['center_rad'] is None for row in m['trials'])  # immutable R0 identity contract
    assert not (root/'runs').exists()
    for row in plan['trials']:verify_motion_file(root,row['trial_id'],p)


def test_reference_mutation_rejected_even_if_local_digest_is_updated(fake_gated_plan,tmp_path):
    import json,shutil
    from casbot_calib.staged_protocol import file_hash
    root=tmp_path/'plan';root.mkdir()
    for name in ('protocol.json','protocol.json.sha256','session.json','split_manifest.json','motion_plan.json','configuration_coverage.json'):
        shutil.copyfile(fake_gated_plan/name,root/name)
    shutil.copytree(fake_gated_plan/'qualification',root/'qualification')
    p,_,m=load_protocol(root);name=m['trials'][0]['trial_id']
    shutil.copytree(fake_gated_plan/'plans'/name,root/'plans'/name)
    target=root/'plans'/name/'motion.json';target.write_text(target.read_text()+'\n')
    local=root/'plans'/name/'manifest.json';data=json.loads(local.read_text());data['motion_sha256']=file_hash(target)
    local.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='Frozen R1 reference changed'):verify_motion_file(root,name,p)


def test_analysis_rejects_changed_nominal_geometry_before_reading_data(fake_gated_plan,monkeypatch):
    import casbot_calib.staged_multifit as module
    engine,source=module.nominal_dual_arm();source={**source,'sha256':'0'*64}
    monkeypatch.setattr(module,'nominal_dual_arm',lambda:(engine,source))
    with pytest.raises(ValueError,match='nominal model changed'):module.load_problems(fake_gated_plan,'train',np.zeros(14))


def test_collision_rejection_does_not_shrink_motion_to_complete_the_plan(tmp_path,config,monkeypatch):
    import casbot_calib.staged_multiconfiguration as module
    c=deepcopy(config);c['p0']['center_design']['candidate_pool_size']=2
    monkeypatch.setattr(module,'gate_motion',lambda *args:dict(passed=False,reason='controlled_collision'))
    with pytest.raises(ValueError,match='No complete collision-free'):
        prepare_protocol(tmp_path/'rejected',c,planner=plan_references)
    assert (tmp_path/'rejected/planning_failure.json').exists()
    assert not (tmp_path/'rejected/protocol.json').exists()
    assert c['pose_amplitude_rad']==config['pose_amplitude_rad']


def simple_problem(name,*,n=64,split='train',center='c0',target=2.,repeat=0):
    p=dict(regression=np.ones((n,14,1))*2,extra=np.zeros((n,14,28)),target=np.full((n,14),target),
        current_torque=np.full((n,14),target),moving=np.ones((n,14),dtype=bool),fixed_friction=np.zeros((n,14)),rotor=np.zeros((n,14)))
    meta=dict(stage='inertia',conditions=dict(split=split,center_id=center,reference_id=name,acquisition_batch_id=f'{split}_{repeat}',repeat_id=repeat,active_arm_group='both'))
    return dict(meta=meta,problem=p,coverage=dict(range_rad=[1.]*14,speed_rms_rad_s=[.1]*14,acceleration_rms_rad_s2=[.2]*14))


def add_probe(problems,n=64):
    problems['wrist']=dict(meta=dict(stage='inertia_probe',conditions=dict(split='train',center_id='c0',acquisition_batch_id='train_0')),
        problem=dict(joint=4,regression=np.ones((n,1)),extra=np.zeros((n,28)),target=np.ones(n)*.1))
    return problems


def test_weights_balance_trajectories_and_do_not_change_when_frames_are_duplicated(config):
    problems=add_probe(dict(a=simple_problem('a'),b=simple_problem('b',n=128)))
    y,x,target,w=assemble_training(problems,config)
    doubled=deepcopy(problems)
    for row in doubled.values():
        row['problem']={k:np.repeat(v,2,axis=0) if isinstance(v,np.ndarray) else v for k,v in row['problem'].items()}
    yy,xx,tt,ww=assemble_training(doubled,config)
    np.testing.assert_allclose(np.linalg.norm(y[:,0]-target),np.linalg.norm(yy[:,0]-tt),rtol=1e-14)
    for axis in range(14):
        a,b=[r for r in w if r['joint']==axis and r['stage']=='inertia']
        assert a['sqrt_weight']**2*a['rows']==pytest.approx(b['sqrt_weight']**2*b['rows'])


def test_selection_cannot_enter_multi_trajectory_fit(config):
    problems=add_probe(dict(a=simple_problem('a',split='selection')))
    with pytest.raises(ValueError,match='Only complete train'):assemble_training(problems,config)


def test_low_training_error_does_not_hide_cross_center_systematic_error(config):
    # Analytic observation: unmodeled posture-dependent torque is zero at c0,
    # 0.3 Nm at c1. No simulator, fitter, test-dependent tuning or large gravity.
    train=simple_problem('a',center='c0');other=simple_problem('b',split='selection',center='c1',target=2.3)
    rows,_=score_problems(dict(a=train,b=other),None,np.array([1.]),np.zeros(28),np.zeros(14),config)
    assert rows['a']['passed'] and rows['a']['maximum_normalized_rmse']==0
    assert not rows['b']['passed']
    assert rows['b']['maximum_normalized_rmse']==pytest.approx(.3/2.3)


def test_repeatability_requires_both_batch_predictions_and_complete_pairs(config):
    a=simple_problem('a');b=simple_problem('a',repeat=1,target=2.3)
    rows,_=score_problems(dict(a=a,b=b),None,np.array([1.]),np.zeros(28),np.zeros(14),config)
    assert not batch_repeatability(rows)['passed']
    with pytest.raises(ValueError,match='Missing/duplicate'):batch_repeatability({'a':rows['a']})


def test_formal_test_is_blocked_before_mock_construction(tmp_path,config,monkeypatch):
    from casbot_calib.staged_development import collect_final_test
    import casbot_calib.staged_collection as collection
    root=tmp_path/'r0';prepare_protocol(root,config)
    monkeypatch.setattr(collection,'synthetic_plant',lambda *_:pytest.fail('Constructed Mock before candidate freeze'))
    with pytest.raises(FileNotFoundError):collect_final_test(root,config)
    assert not (root/'runs').exists()
