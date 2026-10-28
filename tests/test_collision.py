"""Coal geometry oracles and continuous joint-path checks, independent of Mock dynamics."""
from copy import deepcopy
import numpy as np
import pytest
from casbot_calib import _core
from casbot_calib.arm_model import load_arm_model,planar_two_link_config
from casbot_calib.collision import CollisionProfile,box_geometry,certify_polyline,certify_reference,nominal_profile


def simple_profile():
    engine=_core.ArmDynamics(load_arm_model(planar_two_link_config()))
    obstacle=box_geometry('obstacle',[0.,2.,0.],[.1,.1,.1],False)
    moving=box_geometry('rotating',[2.,0.,0.],[.1,.1,.1]);moving.parent=0
    return CollisionProfile(engine,[obstacle,moving],[],dict(test='analytic boxes'))


def test_signed_cube_separation_and_arc_collision_between_clear_endpoints():
    profile=simple_profile();endpoints=profile.check(np.array([[0.,0.],[np.pi,0.]]))
    assert endpoints['distances'].min()>2.
    collision=profile.check(np.array([[np.pi/2,0.]]))
    assert -.201<collision['distances'][0,0]<-.199
    result=certify_polyline(profile,np.array([[0.,0.],[np.pi,0.]]),clearance_m=.01)
    assert not result['passed'] and result['reason']=='collision_or_clearance'
    assert result['witness']['first']=='obstacle' and result['witness']['second']=='rotating'
    # A short reverse arc stays far away, and is certified using an explicit bound.
    safe=certify_reference(profile,lambda t:np.column_stack([-.2*t,np.zeros(len(t))]),1.,[.2,0.])
    assert safe['passed'] and safe['certified_lower_bound_m']>.01


def test_refinement_never_calls_an_unresolved_interval_safe_and_rejects_bad_geometry():
    profile=simple_profile()
    q=lambda t:np.column_stack([np.zeros(len(t)),np.zeros(len(t))])
    result=certify_reference(profile,q,1.,[1000.,0.],initial_step_s=1.,max_depth=0)
    assert not result['passed'] and result['reason']=='interval_not_certified'
    invalid=box_geometry('bad',[0,0,0],[1,1,1]);invalid.triangles=np.array([[999,0,1]]*4)
    with pytest.raises(ValueError):_core.CollisionChecker(profile.engine,[profile.geometry[0],invalid],[])
    with pytest.raises(ValueError):profile.check([[np.nan,0.]])
    # Batching must retain a collision at an interior waypoint, even when the
    # beginning/end of the block (and their midpoint) are identical and clear.
    hidden=np.zeros((21,2));hidden[3,0]=np.pi/2
    assert not certify_polyline(profile,hidden,block_samples=50)['passed']
    with pytest.raises(ValueError,match='gap'):certify_polyline(profile,hidden,times_s=np.arange(21)*.1,max_gap_s=.002)


def test_nominal_cache_provenance_joint_contract_and_batched_distances():
    profile=nominal_profile();q=np.zeros((3,14));q[1,1]=.3;q[1,8]=-.3
    result=profile.check(q)
    assert result['distances'].shape==(3,len(profile.pairs)) and np.isfinite(result['distances']).all()
    assert profile.metadata['source_geometry_count']==45 and len(profile.geometry)==16
    assert max(row['maximum_vertex_plane_violation_m'] for row in profile.metadata['geometries'])<1e-9
    assert np.all(profile.radii>=0)
    # Mounting a differently oriented joint needs a new geometry attachment check.
    model=profile.engine.model;joints=model.joints;joints[0].axis=np.array([1.,0.,0.]);model.joints=joints
    with pytest.raises(ValueError,match='Kinematics'):nominal_profile(_core.ArmDynamics(model))
