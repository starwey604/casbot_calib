"""Installed wheels use an explicit project root without moving model resources."""
import json
import os
from pathlib import Path
import subprocess
import sys


def project_paths(env, cwd):
    code = '''
import json
from pathlib import Path
from casbot_calib import project_paths, shm_data, shm_mock
from casbot_calib.model_sources import model_directory
print(json.dumps(dict(root=str(project_paths.ROOT), data=str(shm_data.ROOT),
                     mock=str(shm_mock.ROOT), models=str(model_directory()),
                     package_models=str(Path(project_paths.__file__).parent/'assets/models'),
                     default=str(Path(project_paths.__file__).resolve().parents[2]))))
'''
    return json.loads(subprocess.check_output([sys.executable, '-c', code], env=env, cwd=cwd, text=True))


def test_source_default_is_independent_of_working_directory(tmp_path):
    env = dict(os.environ)
    env.pop('CASBOT_PROJECT_ROOT', None)
    result = project_paths(env, tmp_path)
    assert result['root'] == result['data'] == result['mock'] == result['default']


def test_installed_project_root_keeps_package_resources(tmp_path):
    env = dict(os.environ, CASBOT_PROJECT_ROOT=str(tmp_path))
    result = project_paths(env, tmp_path)
    assert result['root'] == result['data'] == result['mock'] == str(tmp_path)
    packaged = Path(result['package_models'])
    expected = packaged if packaged.is_dir() else tmp_path / 'python/casbot_calib/assets/models'
    assert Path(result['models']) == expected


def test_model_resolution_prefers_packaged_directory_then_project_root(tmp_path, monkeypatch):
    from casbot_calib import model_sources, project_paths
    package = tmp_path / 'package'
    project = tmp_path / 'project'
    monkeypatch.setattr(model_sources, 'files', lambda _: package)
    monkeypatch.setattr(project_paths, 'ROOT', project)
    assert model_sources.model_directory() == project / 'python/casbot_calib/assets/models'
    (package / 'assets/models').mkdir(parents=True)
    assert model_sources.model_directory() == package / 'assets/models'
