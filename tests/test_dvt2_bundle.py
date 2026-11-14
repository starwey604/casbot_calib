"""N2 full-source reduction, collision provenance and frozen bundle contracts."""
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import pytest
import yaml
from casbot_calib import _core
from casbot_calib.dvt2_bundle import load_bundle, robot_profile, trical_scene, viewer_adapter
from casbot_calib.model_sources import model_directory

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'configs/dvt2_model_candidate.yaml'
BUNDLE=model_directory()/'casbot_hl12p1_dvt2/derived/a2ff_stock_hands_candidate_v1'














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


