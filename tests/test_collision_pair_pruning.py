"""Pruning is justified by bounds, not by a collision exclusion policy."""
import numpy as np
from casbot_calib import _core
from casbot_calib.arm_model import load_arm_model,planar_two_link_config
from casbot_calib.collision import CollisionProfile,box_geometry,certify_reference_box,certify_chord_box_pruned


def scene():
    engine=_core.ArmDynamics(load_arm_model(planar_two_link_config()))
    fixed=box_geometry('fixed',[0,2,0],[.1]*3,False)
    moving=box_geometry('moving',[2,0,0],[.1]*3);moving.parent=0
    return CollisionProfile(engine,[fixed,moving],[],{})


def test_safe_tube_matches_original_and_hidden_interior_collision_is_retained():
    p=scene();p.pair_clearance_m=np.array([.01]);allow=np.array([.001,.001])
    original=certify_reference_box(p,lambda t:np.column_stack([-.2*t,np.zeros(len(t))]),1.,[.2,0],allow)
    pruned=certify_chord_box_pruned(p,[0,0],[-.2,0],allow)
    assert original['passed'] and pruned['passed'] and pruned['all_pairs_covered']
    for row in pruned['subintervals']:assert row['precertified_pairs']+row['unresolved_pairs']==len(p.pairs)
    # Both endpoints are clear but the arc crosses the obstacle.
    bad=certify_chord_box_pruned(p,[0,0],[np.pi,0],allow)
    assert not bad['passed'] and not bad['all_pairs_covered']


def test_negative_stroke_is_not_silently_excluded():
    engine=_core.ArmDynamics(load_arm_model(planar_two_link_config()))
    fixed=box_geometry('ball_proxy',[0,0,0],[.1]*3,False)
    tip=box_geometry('tip',[.199,0,0],[.1]*3);tip.parent=0
    p=CollisionProfile(engine,[fixed,tip],[],{});p.pair_clearance_m=np.array([-.005])
    safe=certify_chord_box_pruned(p,[0,0],[0,0],[.001,.001])
    assert safe['passed'] and safe['subintervals'][0]['precertified_pairs']==1
    pose=tip.placement;pose.translation=np.array([.19,0,0]);tip.placement=pose
    p=CollisionProfile(engine,[fixed,tip],[],{});p.pair_clearance_m=np.array([-.005])
    assert not certify_chord_box_pruned(p,[0,0],[0,0],[.001,.001])['passed']
