"""Real-hole geometry moves coherently; no robot/tool mutation or relaxed gates."""
from copy import deepcopy
import numpy as np
import pytest
from casbot_calib.trical_assembly import (profile,asset_directory,ball_center,layout_shift,ball_offset)
from casbot_calib.trical_holes import column_layout,side_height_layouts
from casbot_calib.dvt2_bundle import read,load_bundle
from casbot_calib.model_sources import model_directory
from casbot_calib.trical_paths import timed_motion,SETTINGS
from casbot_calib.staged_motion import derivative_bounds

BUNDLE=model_directory()/'casbot_hl12p1_dvt2/derived/bare_flange_1105_03_v1'


def selected_hole():
    grid=read(asset_directory()/'table_holes.json');holes=np.array(grid['world_xy_m'])
    return holes[np.argmin(np.linalg.norm(holes-[.3181,-.0005846],axis=1))]


def test_extracted_grid_and_actual_column_footprint():
    grid=read(asset_directory()/'table_holes.json');holes=np.array(grid['world_xy_m'])
    assert len(holes)==209
    np.testing.assert_allclose(np.diff(np.unique(holes[:,0])),.025,atol=1e-9)
    np.testing.assert_allclose(np.diff(np.unique(holes[:,1])),.025,atol=1e-9)
    levels=read(asset_directory()/'pillar_holes.json')['world_z_m']
    np.testing.assert_allclose(np.diff(levels),.01,atol=1e-9)
    with pytest.raises(ValueError):column_layout([.333333,0.])
    with pytest.raises(ValueError):column_layout([float('nan'),0.])


def test_whole_column_move_keeps_robot_tools_table_and_floor_unchanged():
    layout=column_layout(selected_hole());shift=np.array(layout['translation_m'])
    before=profile(BUNDLE,'left');after=profile(BUNDLE,'left',layout=layout)
    assert before.names==after.names and before.pairs==after.pairs
    assert before.metadata['geometry_sha256']!=after.metadata['geometry_sha256']
    for a,b in zip(before.geometry,after.geometry,strict=True):
        moving=a.name.startswith(('ball:','stem:','fixed:11050002'))
        np.testing.assert_allclose(b.placement.translation-a.placement.translation,shift if moving else np.zeros(3),atol=1e-12)
        np.testing.assert_array_equal(a.vertices,b.vertices)
        np.testing.assert_allclose(a.placement.rotation,b.placement.rotation,atol=1e-12)
    for bid in 'ABCD':np.testing.assert_allclose(ball_center(bid,layout)-ball_center(bid),shift,atol=1e-12)
    home=load_bundle(BUNDLE)[1]['profile']['home_rad']
    assert np.max(abs(after.check(home)['distances']-before.check(home)['distances']))>.005
    assert not any(n.startswith('right:') for n in after.names)


def test_side_hole_changes_move_the_selected_ball_and_stem_only():
    layout=next(x for x in side_height_layouts() if x['ball_offsets_m']['C'][2]==-.02 and x['ball_offsets_m']['D'][2]==.03)
    before=profile(BUNDLE,'left');after=profile(BUNDLE,'left',layout=layout)
    for a,b in zip(before.geometry,after.geometry,strict=True):
        z=-.02 if a.name in {'ball:C','stem:C'} else .03 if a.name in {'ball:D','stem:D'} else 0.
        np.testing.assert_allclose(b.placement.translation-a.placement.translation,[0,0,z],atol=1e-12)
    assert ball_offset('C',layout)[2]==-.02
    assert ball_offset('D',layout)[2]==.03
    bad=deepcopy(layout);bad['ball_offsets_m']['C'][2]=.015
    with pytest.raises(ValueError,match='existing CAD pillar hole'):ball_offset('C',bad)
    bad=deepcopy(layout);bad['translation_m'][2]=.01
    with pytest.raises(ValueError,match='unchanged Z'):layout_shift(bad)


def test_contact_exceptions_are_still_exact_and_bounded_after_relocation():
    p=profile(BUNDLE,'right','C',True,layout=column_layout(selected_hole()))
    negative=np.flatnonzero(p.pair_clearance_m<0)
    assert len(negative)==3
    for i in negative:
        names={p.names[j] for j in p.pairs[i]};assert 'ball:C' in names
        assert p.pair_clearance_m[i]==-.005
    assert set(p.pair_clearance_m)<=set([-.005,.002,.01])
    # Waist/shoulder-roll remains checked, unlike the exploratory review.
    assert any({p.names[i] for i in pair}=={'robot_waist_yaw_link_0','robot_right_shoulder_roll_link_0'} for pair in p.pairs)


def test_reference_time_scaling_is_C3_and_500Hz_aligned():
    a=np.zeros(14);b=a.copy();b[0]=.8;b[3]=-.4
    m=timed_motion('test',[a,a,b,b,a,a],['initial_hold','approach','contact_hold','return','final_hold'])
    curve=m.polynomial();limits=[.35,1.,8.]
    for k,bound in enumerate(limits,1):assert np.max(derivative_bounds(curve,k))<=bound
    for t in m.knots:
        np.testing.assert_allclose(curve(t,nu=1),0.,atol=1e-10)
        np.testing.assert_allclose(curve(t,nu=2),0.,atol=1e-10)
        np.testing.assert_allclose(curve(t,nu=3),0.,atol=1e-10)
    np.testing.assert_allclose(np.array(m.knots)*500,np.round(np.array(m.knots)*500),atol=1e-8)
    assert SETTINGS['control_period_ns']==2_000_000


def test_platform_lift_moves_table_and_attached_parts_but_not_pedestal_or_robot():
    layout=dict(schema_version='casbot_trical_column_layout_v1',id='height_test',translation_m=[0.,0.,0.],platform_lift_m=-.075)
    before=profile(BUNDLE,'right');after=profile(BUNDLE,'right',layout=layout)
    moved={'fixed:11050002','fixed:300X500X130001','fixed:3000001_ASM','fixed:3000001_ASM001'}
    for a,b in zip(before.geometry,after.geometry,strict=True):
        expected=[0,0,-.075] if a.name in moved or a.name.startswith(('ball:','stem:')) else [0,0,0]
        np.testing.assert_allclose(b.placement.translation-a.placement.translation,expected,atol=1e-12)
    for bid in 'ABCD':np.testing.assert_allclose(ball_center(bid,layout)-ball_center(bid),[0,0,-.075],atol=1e-12)
    bad=deepcopy(layout);bad['platform_lift_m']=float('nan')
    from casbot_calib.trical_assembly import platform_lift
    with pytest.raises(ValueError):platform_lift(bad)
