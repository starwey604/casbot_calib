"""N2 full-source reduction, collision provenance and frozen bundle contracts."""
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import pytest
import yaml
from casbot_calib import _core
from casbot_calib.dvt2_build import build_bundle, configuration
from casbot_calib.dvt2_bundle import load_bundle, robot_profile, trical_scene, viewer_adapter
from casbot_calib.dvt2_validate import verify_reduction
from casbot_calib.model_sources import model_directory
from casbot_calib.trical_fixture import fixture_profile, sphere_envelope

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'configs/dvt2_model_candidate.yaml'
BUNDLE=model_directory()/'casbot_hl12p1_dvt2/derived/a2ff_stock_hands_candidate_v1'


def test_candidate_is_not_the_current_default_and_retains_complete_mass():
    engine,metadata=load_bundle(BUNDLE)
    assert len(engine.model.joints)==14
    assert not metadata['assembly_confirmed'] and not metadata['default_model_changed']
    assert not metadata['hardware_executable'] and not metadata['tool_load_added']
    report=verify_reduction(BUNDLE)
    assert report['passed'] and report['common_source_frames']>200
    assert report['source_total_link_mass_kg']==pytest.approx(50.68212243880201)
    from casbot_calib.dual_model import nominal_dual_arm
    assert nominal_dual_arm()[0].model.model_id!=engine.model.model_id


def test_nonzero_locks_and_rotated_world_match_original_full_source(tmp_path):
    c=yaml.safe_load(CONFIG.read_text())
    c['lock_positions_rad'].update(waist_yaw_joint=.13,head_yaw_joint=-.07,head_pitch_joint=.1)
    c['world_from_source']=dict(translation_m=[.2,-.1,.3],rotation_vector_rad=[.12,-.08,.17])
    c['gravity_world_m_s2']=[.1,-.2,-9.79]
    config=tmp_path/'profile.yaml';config.write_text(yaml.safe_dump(c))
    output=tmp_path/'bundle';build_bundle(config,output)
    assert verify_reduction(output,samples=4)['passed']


@pytest.mark.parametrize('change',['missing_lock','arm_lock','bad_lock','bad_home','nan_home'])
def test_configuration_rejects_ambiguous_or_invalid_locks_and_home(tmp_path,change):
    c=yaml.safe_load(CONFIG.read_text())
    if change=='missing_lock':del c['lock_positions_rad']['waist_yaw_joint']
    elif change=='arm_lock':c['lock_positions_rad']['left_elbow_pitch_joint']=0.
    elif change=='bad_lock':c['lock_positions_rad']['waist_yaw_joint']=100.
    elif change=='bad_home':c['home_rad'][0]=100.
    else:c['home_rad'][0]=float('nan')
    path=tmp_path/'invalid.yaml';path.write_text(yaml.safe_dump(c))
    with pytest.raises(ValueError):configuration(path)


def test_geometry_includes_source_spheres_and_only_exact_housing_exclusions():
    engine,metadata=load_bundle(BUNDLE);profile=robot_profile(BUNDLE)
    rows=profile.metadata['geometries']
    assert len(rows)==profile.metadata['source_geometry_count']==74
    assert sum(r['source_shape']=='Sphere' for r in rows)==16
    assert all(r['maximum_vertex_plane_violation_m']<=1e-9 for r in rows)
    audited=[r for r in profile.metadata['excluded_pairs'] if r['reason'].startswith('explicitly audited')]
    assert {tuple(r['source_links']) for r in audited}=={
        ('left_wrist_roll_link','left_wrist_yaw_link'),('right_wrist_roll_link','right_wrist_yaw_link')}
    assert any('palm' in n for n in profile.names)
    assert any('thumb' in n for n in profile.names)
    assert any({'robot_left_wrist_yaw_link_0','robot_left_palm_base_link_0'}=={profile.names[k] for k in pair} for pair in profile.pairs)
    assert profile.check(metadata['profile']['home_rad'])['distances'].min()>.01
    # All 14 joint origins/axes and fixed frames must bind the injected body cache.
    changed=engine.model;joints=changed.joints;joints[0].axis=np.array([1.,0.,0.]);changed.joints=joints
    with pytest.raises(ValueError,match='geometric models differ'):
        fixture_profile(_core.ArmDynamics(changed),trical_scene(BUNDLE),base_profile=profile)


def test_three_contact_exceptions_do_not_disable_hands_other_balls_or_housings():
    engine,metadata=load_bundle(BUNDLE);config=trical_scene(BUNDLE);base=robot_profile(BUNDLE)
    visit=config['workflow']['visits'][0]
    profile=fixture_profile(engine,config,(visit['frame'],visit['ball_id']),True,base_profile=base)
    selected=np.flatnonzero(profile.pair_clearance_m<0)
    assert len(selected)==3
    for i in selected:
        pair={profile.names[k] for k in profile.pairs[i]}
        assert visit['ball_id'] in pair and any('_tip_' in x for x in pair)
        assert profile.pair_clearance_m[i]==-.005
    assert len(config['fixture']['balls'])==6 and config['scene_only']
    assert metadata['scene']['floor_surface_z_m']==0.
    assert profile.check(visit['nominal_q_rad'])['distances'].shape[1]>2000


def test_finer_ball_polytope_remains_an_outer_envelope():
    from scipy.spatial import ConvexHull
    radii=[]
    for samples in (96,512):
        geometry=sphere_envelope('ball',[0,0,0],.0127,samples=samples)
        vertices=np.asarray(geometry.vertices);hull=ConvexHull(vertices)
        assert np.all(-hull.equations[:,3]>=.0127-1e-14)
        radii.append(np.linalg.norm(vertices,axis=1).max())
    assert radii[1]<radii[0]
    with pytest.raises(ValueError):sphere_envelope('ball',[0,0,0],.01,samples=3)


@pytest.mark.parametrize('file,kind',[('model.urdf','bytes'),('collision.npz','bytes'),
    ('collision.json','exclude'),('trical_scene.json','scene'),('model.json','profile'),('scene.xml','bytes')])
def test_changed_profile_geometry_exclusions_or_scene_is_rejected(tmp_path,file,kind):
    # Small descriptors/cache only: relative mesh paths are not read by native FK.
    import shutil
    root=tmp_path/'copy';shutil.copytree(BUNDLE,root)
    path=root/file
    if kind=='bytes':path.write_bytes(path.read_bytes()+b'\n')
    else:
        payload=json.loads(path.read_text())
        if kind=='profile':payload['profile']['gravity_world_m_s2'][0]=1.
        elif kind=='exclude':payload['excluded_pairs'].clear()
        else:payload['fixture']['balls'][0]['center_m'][0]+=.1
        path.write_text(json.dumps(payload))
    with pytest.raises(ValueError):robot_profile(root)


def test_installed_runtime_does_not_import_python_robotics_or_viewer_libraries(tmp_path):
    script='''
import importlib.abc,sys
class Deny(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, *args):
        if fullname.split('.')[0] in {'pinocchio','coal','mujoco'}:raise AssertionError(fullname)
sys.meta_path.insert(0,Deny())
from casbot_calib.dvt2_bundle import load_bundle,robot_profile,viewer_adapter
p=sys.argv[1];engine,metadata=load_bundle(p)
assert len(viewer_adapter(p).mappings)==14
assert robot_profile(p).check(metadata['profile']['home_rad'])['distances'].min()>.01
'''
    subprocess.run([sys.executable,'-c',script,str(BUNDLE)],cwd=tmp_path,check=True)


def test_unhydrated_lfs_collision_cache_gives_an_actionable_error(tmp_path):
    import shutil
    root=tmp_path/'copy';shutil.copytree(BUNDLE,root)
    (root/'collision.npz').write_text('version https://git-lfs.github.com/spec/v1\noid sha256:example\nsize 1\n')
    with pytest.raises(ValueError,match='git lfs pull'):robot_profile(root)


@pytest.mark.parametrize('radius',[0.,-.01,float('nan')])
def test_nonphysical_sphere_radius_is_rejected(radius):
    with pytest.raises(ValueError,match='positive radius'):sphere_envelope('invalid',[0,0,0],radius)
