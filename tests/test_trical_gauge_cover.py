"""The refined envelope preserves source geometry, mechanical model and gates."""
import numpy as np
from scipy.spatial import ConvexHull
from casbot_calib.dvt2_bundle import load_bundle,read
from casbot_calib.trical_assembly import asset_directory,asset_manifest,profile,collision_assets
from casbot_calib.mesh_io import read_stl
from casbot_calib.model_sources import model_directory

BASE=model_directory()/'casbot_hl12p1_dvt2/derived'
OLD=BASE/'bare_flange_1105_03_v1';NEW=BASE/'bare_flange_1105_03_gauge_v2'


def test_native_model_is_unchanged_and_old_bundle_keeps_original_asset_root():
    a,ma=load_bundle(OLD);b,mb=load_bundle(NEW)
    assert asset_directory(OLD)==asset_directory()
    assert asset_directory(NEW).name=='trical_1105_03_gauge_cover_v2'
    q=np.array(ma['profile']['home_rad']);q[0]=-.7;q[8]=-.3
    np.testing.assert_allclose(a.mass_matrix(q),b.mass_matrix(q),atol=1e-13,rtol=0)
    np.testing.assert_allclose(a.gravity(q),b.gravity(q),atol=1e-13,rtol=0)
    for side in ['left','right']:
        np.testing.assert_allclose(a.frame_pose(q,side+'_trical_tcp'),b.frame_pose(q,side+'_trical_tcp'),atol=1e-14,rtol=0)
        p=profile(NEW,side,'C',True)
        assert len(p.metadata['exact_contact_pairs'])==3
        assert set(p.pair_clearance_m)=={.01,.002,-.005}
        assert any({p.names[i] for i in pair}=={'robot_waist_yaw_link_0','robot_'+side+'_shoulder_roll_link_0'} for pair in p.pairs)
    assert ma['bundle_sha256']!=mb['bundle_sha256']


def test_gauge_partition_keeps_cad_solid_and_covers_source_mesh_samples():
    root=asset_directory(NEW);manifest,cache,geometry=collision_assets(root)
    assert manifest['setup']==asset_manifest()['setup']
    assert manifest['axes_sensor']==asset_manifest()['axes_sensor']
    proofs=read(root/'gauge_cover_evidence.json');assert len(proofs)==6
    for proof in proofs:
        assert proof['cells']==6
        assert proof['missing_solid_mm3']<1e-3 and proof['extra_solid_mm3']<1e-3
        row=next(r for r in manifest['parts'] if r['name']==proof['name'])
        vertices,faces=read_stl(root/row['mesh']);samples=np.vstack([vertices,vertices[faces].mean(axis=1)])
        violation=[]
        for cell in cache['parts']:
            if cell['source_name']!=row['name']:continue
            planes=ConvexHull(np.asarray(geometry[cell['name']].vertices)).equations
            violation.append(np.max(samples@planes[:,:3].T+planes[:,3],axis=1))
        # A source sample need only lie in one cell of the solid union.
        assert np.max(np.min(violation,axis=0))<1e-7


def test_original_left_D_review_pose_has_a_real_declared_box_counterexample():
    from itertools import product
    row=next(r for r in read(asset_directory(NEW)/'seeds.json')['results'] if r['arm']=='left' and r['ball']=='D')
    q=np.tile(row['contact'][0]['q'],(128,1));q[:,:7]+=np.array(list(product([-1,1],repeat=7)))*.00105
    p=profile(NEW,'left','D',True);d=p.check(q)['distances']-p.pair_clearance_m
    i,k=np.unravel_index(np.argmin(d),d.shape)
    assert {p.names[j] for j in p.pairs[k]}=={'fixed:11050002','left:Part__Feature5147'}
    assert -.00037<d[i,k]<-.00036
