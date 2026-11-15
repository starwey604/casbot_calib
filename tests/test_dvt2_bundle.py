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
BUNDLE=model_directory()/'casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2'


@pytest.mark.parametrize('file,kind',[('model.urdf','bytes'),('collision.npz','bytes'),
    ('collision.json','exclude'),('trical_scene.json','scene'),('model.json','profile')])
def test_changed_profile_geometry_exclusions_or_scene_is_rejected(tmp_path,file,kind):
    # Small descriptors/cache only: relative mesh paths are not read by native FK.
    import shutil
    root=tmp_path/'copy';shutil.copytree(BUNDLE,root,ignore=shutil.ignore_patterns('visual_meshes','*.xml'))
    path=root/file
    if kind=='bytes':path.write_bytes(path.read_bytes()+b'\n')
    else:
        payload=json.loads(path.read_text())
        if kind=='profile':payload['profile']['gravity_world_m_s2'][0]=1.
        elif kind=='exclude':payload['excluded_pairs'].clear()
        else:payload['axes_sensor']['left'][0][0]+=.1
        path.write_text(json.dumps(payload))
    with pytest.raises(ValueError):robot_profile(root)


def test_unhydrated_lfs_collision_cache_gives_an_actionable_error(tmp_path):
    import shutil
    root=tmp_path/'copy';shutil.copytree(BUNDLE,root,ignore=shutil.ignore_patterns('visual_meshes','*.xml'))
    (root/'collision.npz').write_text('version https://git-lfs.github.com/spec/v1\noid sha256:example\nsize 1\n')
    with pytest.raises(ValueError,match='git lfs pull'):robot_profile(root)


