"""The companion adds only its selected channels to the frozen objective."""
from copy import deepcopy

import numpy as np

from casbot_calib.staged_multifit import assemble_training, score_problems
from casbot_calib.staged_stability import frozen_group_mask
from casbot_calib.staged_wrist_periodic import POLICY


def test_wrist_periodic_adds_equal_record_weight_without_held_axes_or_reweighting():
    n=32
    conditions=dict(split='train',center_id='train_c01',reference_id='ref',
                    acquisition_batch_id='train_batch_01',repeat_id=1,active_arm_group='left',
                    active_joints=[4,5,6])
    regression=np.ones((n,14,140));extra=np.zeros((n,14,28))
    current=np.broadcast_to(np.arange(14),(n,14)).copy()
    common=dict(regression=regression,extra=extra,current_torque=current,target=current,
                rotor=np.zeros((n,14)),fixed_friction=np.zeros((n,14)),moving=np.ones((n,14),bool))
    probes=dict(regression=regression[:,4],extra=extra[:,4],target=current[:,4],joint=4)
    def problem(stage,p):
        return dict(meta=dict(stage=stage,conditions=conditions),problem=p,coverage={})
    config=dict(p0=dict(weighting=dict(torque_floor_nm=.2,primary_wrist_sqrt_weight=.1),
                        acceptance=dict(torque_floor_nm=.2,dynamic_axis_nrmse=.05),wrist_periodic_train=POLICY))
    base=dict(primary=problem('inertia',common),probe=problem('inertia_probe',probes))
    y,x,target,weights=assemble_training(base,config)
    wrist=problem('wrist_periodic',deepcopy(common))
    new_y,new_x,new_target,new_weights=assemble_training({**base,'wrist':wrist},config)
    np.testing.assert_array_equal(y,new_y[:len(y)])
    np.testing.assert_array_equal(x,new_x[:len(y)])
    np.testing.assert_array_equal(target,new_target[:len(y)])
    assert new_weights[:len(weights)]==weights
    extra_rows=new_weights[len(weights):]
    assert [r['joint'] for r in extra_rows]==[4,5,6]
    assert np.isclose(sum(r['rows']*(r['sqrt_weight']*.2)**2 for r in extra_rows),1024/32)
    assert not frozen_group_mask(new_weights,'center_id','train_c01')[len(y):].any()
    assert not frozen_group_mask(new_weights,'acquisition_batch_id','train_batch_01')[len(y):].any()
    wrist['problem']['current_torque'][:,[4,5,6]]=0
    scores,_=score_problems({'wrist':wrist},None,np.zeros(140),np.zeros(28),None,config)
    assert scores['wrist']['passed'] and scores['wrist']['maximum_normalized_rmse']==0
    assert scores['wrist']['rmse_nm'][3] is None and scores['wrist']['rmse_nm'][7] is None


def test_right_pitch_does_not_renormalize_original_probes():
    from casbot_calib.staged_right_pitch import RIGHT_POLICY

    n=32
    c=dict(split='train',center_id='train_c00',acquisition_batch_id='train_batch_00')
    y=np.zeros((n,14,140));x=np.zeros((n,14,28));tau=np.full((n,14),.01)
    def row(stage,p):
        return dict(meta=dict(stage=stage,conditions=c),problem=p)
    primary=row('inertia',dict(regression=y,extra=x,target=tau,current_torque=tau,moving=np.ones_like(tau,bool)))
    probe=row('inertia_probe',dict(regression=y[:,4],extra=x[:,4],target=tau[:,4],joint=4))
    pitch=row('right_pitch_probe',dict(regression=y[:,12],extra=x[:,12],target=tau[:,12],joint=12))
    cfg=dict(p0=dict(weighting=dict(torque_floor_nm=.2,primary_wrist_sqrt_weight=.1),right_pitch_train=RIGHT_POLICY))
    old=assemble_training(dict(primary=primary,probe=probe),cfg)
    new=assemble_training(dict(primary=primary,probe=probe,pitch=pitch),cfg)
    for a,b in zip(old[:3],new[:3]):
        np.testing.assert_array_equal(a,b[:len(a)])
    assert old[3]==new[3][:-1]
    weight=new[3][-1]
    assert weight['joint']==12 and weight['stage']=='right_pitch_probe'
    assert np.isclose(weight['rows']*(weight['sqrt_weight']*.2)**2,1024/32)
    assert not frozen_group_mask(new[3],'center_id','train_c00')[-n:].any()
    assert not frozen_group_mask(new[3],'acquisition_batch_id','train_batch_00')[-n:].any()


def test_matched_local_kernel_keeps_edges_and_never_crosses_a_gap():
    from casbot_calib.staged_wrist_local import local_kernel

    valid=np.r_[np.arange(201),np.arange(220,401)]
    centers=np.array([0,5,100,200,220,250,400])
    for sigma in (2,6,16):
        mat,radii,_=local_kernel(valid,centers,sigma)
        np.testing.assert_allclose(mat@np.ones(len(valid)),1.,atol=1e-14,rtol=0)
        np.testing.assert_allclose(mat@valid.astype(float),centers,atol=1e-10,rtol=0)
        assert (radii[[0,3,4,-1]]==0).all()
        assert mat[2,201:].nnz==0 and mat[5,:201].nnz==0


def test_sustained_pitch_replaces_only_its_axis_with_same_record_budget():
    n=32;m=48
    c=dict(split='train',center_id='train_c00',acquisition_batch_id='train_batch_00')
    original=dict(regression=np.ones((n,14,140)),extra=np.zeros((n,14,28)),
                  target=np.full((n,14),.01),current_torque=np.full((n,14),.01),
                  moving=np.ones((n,14),bool))
    def row(stage,p):return dict(meta=dict(stage=stage,conditions=c),problem=p)
    primary=row('inertia',original)
    probe=row('inertia_probe',dict(regression=np.ones((n,140)),extra=np.zeros((n,28)),
                                   target=np.full(n,.01),joint=4))
    cfg=dict(p0=dict(weighting=dict(torque_floor_nm=.2,primary_wrist_sqrt_weight=.1)))
    old=assemble_training(dict(primary=primary,probe=probe),cfg)
    changed=deepcopy(primary)
    changed['problem']['sustained_pitch']=dict(regression=np.full((m,140),2.),
        extra=np.full((m,28),3.),target=np.full(m,.02))
    new=assemble_training(dict(primary=changed,probe=probe),cfg)
    old_offset=new_offset=0
    for a,b in zip(old[3],new[3]):
        if a['joint']==12:
            assert b['rows']==m
            np.testing.assert_allclose(b['rows']*b['sqrt_weight']**2,a['rows']*a['sqrt_weight']**2,rtol=1e-14)
            np.testing.assert_allclose(new[0][new_offset:new_offset+m],2*b['sqrt_weight'])
        else:
            assert a==b
            for x,y in zip(old[:3],new[:3]):
                np.testing.assert_array_equal(x[old_offset:old_offset+a['rows']],y[new_offset:new_offset+b['rows']])
        old_offset+=a['rows'];new_offset+=b['rows']
    assert 'sustained_pitch' not in primary['problem']


def test_sustained_pitch_support_excludes_raw_stops_and_sign_changes():
    from casbot_calib.staged_pitch_sustained import support

    step=250000
    stamps=np.arange(0,6000000000,step,dtype=np.int64)
    v=np.full((len(stamps),14),.1)
    # Two one-second steady cycles after two discarded cycles.
    v[12000:12012,12]=0
    v[16000:,12]=-.1
    raw=dict(sample_ns=stamps,dq=v)
    meta=dict(conditions=dict(period_s=1.),labels=['periodic'],knots_s=[0.,6.])
    valid,centers=support(raw,meta,step)
    assert centers.size>32
    assert not np.any((valid>=11980)&(valid<12032))
    # The sign change lies at the end of the retained two-cycle interval.
    assert valid.max()<15980
    for offset in range(-20,21):
        assert np.all(abs(v[valid+offset,12])>.02)
        assert np.all(v[valid+offset,12]*v[valid,12]>0)


def test_sustained_pitch_explicit_axis_keeps_existing_right_contract():
    from casbot_calib.staged_pitch_sustained import support
    stamps=np.arange(0,6000000000,250000,dtype=np.int64)
    speed=np.full((len(stamps),14),.1)
    speed[12000:12040,5]=0.
    raw=dict(sample_ns=stamps,dq=speed)
    meta=dict(conditions=dict(period_s=1.),labels=['periodic'],knots_s=[0.,6.])
    right,centers=support(raw,meta,250000)
    right_explicit,explicit=support(raw,meta,250000,joint=12)
    np.testing.assert_array_equal(right,right_explicit)
    np.testing.assert_array_equal(centers,explicit)
    left,_=support(raw,meta,250000,joint=5)
    assert not np.any((left>=11980)&(left<12060))
    assert np.any((right>=12000)&(right<12040))


def test_uniform_sustained_wrist_preserves_each_original_axis_budget():
    n=32;m=48;axes=(4,5,6,11,12,13)
    c=dict(split='train',center_id='train_c00',acquisition_batch_id='train_batch_00')
    primary=dict(regression=np.ones((n,14,140)),extra=np.zeros((n,14,28)),
        target=np.full((n,14),.01),current_torque=np.full((n,14),.01),moving=np.ones((n,14),bool))
    def row(stage,p):return dict(meta=dict(stage=stage,conditions=c),problem=p)
    probe=row('inertia_probe',dict(regression=np.ones((n,140)),extra=np.zeros((n,28)),target=np.full(n,.01),joint=4))
    cfg=dict(p0=dict(weighting=dict(torque_floor_nm=.2,primary_wrist_sqrt_weight=.1)))
    old=assemble_training(dict(primary=row('inertia',primary),probe=probe),cfg)
    changed=deepcopy(primary)
    changed['sustained_wrist']={j:dict(regression=np.full((m,140),j),
        extra=np.zeros((m,28)),target=np.full(m,.02)) for j in axes}
    new=assemble_training(dict(primary=row('inertia',changed),probe=probe),cfg)
    offset_old=offset_new=0
    for a,b in zip(old[3],new[3]):
        if a['joint'] in axes and a['stage']=='inertia':
            assert b['rows']==m
            assert b['torque_normalizer_nm']==a['torque_normalizer_nm']
            np.testing.assert_allclose(m*b['sqrt_weight']**2,n*a['sqrt_weight']**2,rtol=1e-14)
            np.testing.assert_allclose(new[0][offset_new:offset_new+m],a['joint']*b['sqrt_weight'])
        else:
            assert a==b
            for x,y in zip(old[:3],new[:3]):
                np.testing.assert_array_equal(x[offset_old:offset_old+a['rows']],y[offset_new:offset_new+b['rows']])
        offset_old+=a['rows'];offset_new+=b['rows']


def test_wrist_policy_fixes_axes_and_bandwidth():
    import pytest
    from casbot_calib.staged_pitch_sustained import POLICY as RIGHT,WRIST_POLICY,validate_wrist_policy
    p0=dict(pitch_sustained_train=RIGHT,wrist_sustained_train=deepcopy(WRIST_POLICY))
    validate_wrist_policy(p0)
    p0['wrist_sustained_train']['sigma_ms']=4.
    with pytest.raises(ValueError):validate_wrist_policy(p0)
