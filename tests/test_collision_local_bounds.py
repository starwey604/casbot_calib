"""Local motion bounds retain actual collisions and bound native vertex motion."""
import numpy as np
import pytest
from casbot_calib.collision import local_motion_profile,certify_chord_box_pruned
from casbot_calib.trical_assembly import profile,asset_directory
from casbot_calib.model_sources import model_directory
from casbot_calib.dvt2_bundle import read,load_bundle

BUNDLE=model_directory()/'casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2'


def test_bound_covers_native_vertex_motion_throughout_declared_box():
    p=profile(BUNDLE,'left','D',True);e,m=load_bundle(BUNDLE);q=np.array(m['profile']['home_rad']);half=np.full(14,.12)
    local,proof=local_motion_profile(p,q,half)
    assert np.all(local.radii<=p.radii+1e-14)
    assert np.min(local.radii[p.radii>0]/p.radii[p.radii>0])<.5
    names={'left:Part__Feature5160__cell3','fixed:11050002'}
    pair=next(i for i,ab in enumerate(p.pairs) if {p.names[j] for j in ab}==names)
    g=next(g for g in p.geometry if g.name=='left:Part__Feature5160__cell3')
    def vertices(x):
        T=e.fk(x)[g.parent]@g.placement.matrix()
        return np.asarray(g.vertices)@T[:3,:3].T+T[:3,3]
    rng=np.random.default_rng(20261106)
    for _ in range(24):
        x=q+rng.uniform(-.9,.9,14)*half;v=vertices(x)
        for j in range(7):
            other=x.copy();other[j]+=1e-5
            motion=np.max(np.linalg.norm(vertices(other)-v,axis=1))
            assert motion<=local.radii[pair,j]*1e-5+2e-12
    assert proof['half_width_rad']==half.tolist()
    # Recomputing on another domain uses native global radii, not a stale local bound.
    nested,_=local_motion_profile(local,q,half*2);fresh,_=local_motion_profile(p,q,half*2)
    np.testing.assert_allclose(nested.radii,fresh.radii,atol=1e-14,rtol=0)
    with pytest.raises(ValueError):local_motion_profile(p,q,np.full(14,np.nan))


def test_shared_ancestors_stay_zero_and_real_box_counterexample_is_not_admitted():
    p=profile(BUNDLE,'left','D',True);_,m=load_bundle(BUNDLE);q=np.array(m['profile']['home_rad'])
    local,_=local_motion_profile(p,q,np.full(14,.02))
    pair=next(i for i,ab in enumerate(p.pairs) if {p.names[j] for j in ab}=={'left:Part__Feature5160__cell3','robot_left_wrist_yaw_link_0'})
    np.testing.assert_array_equal(local.radii[pair,:5],0.)
    row=next(r for r in read(asset_directory(BUNDLE)/'seeds.json')['results'] if (r['arm'],r['ball'])==('left','D'))
    q=np.array(row['contact'][0]['q'])
    gate=certify_chord_box_pruned(p,q,q,np.full(14,.00105),local_motion_bounds=True)
    assert not gate['passed']
    assert gate['local_motion_bound']['method']=='joint_axis_radius_plus_descendant_displacement_v1'
