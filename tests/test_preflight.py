"""Tests for P4 preflight checks, LFS pointer detection, build.sh prefix unification, and calib/setup.sh --check."""
import fcntl
import os
from pathlib import Path
import shutil
import subprocess
import sys
import pytest
from casbot_calib.dvt2_bundle import load_bundle
from casbot_calib.model_context import BUNDLE_RELATIVE
from casbot_calib.model_sources import model_directory
from casbot_calib.pipeline.preflight import (
    check_hardware_environment,
    check_model_bundle,
    check_native_binaries,
    check_python_runtime,
    run_preflight,
)
from casbot_calib.pipeline.runner import run_pipeline
from casbot_calib.shm_data import ROOT
from casbot_calib.shm_takeover import HandoffError, SERVICES

DEFAULT_BUNDLE = model_directory() / BUNDLE_RELATIVE
LFS_POINTER = 'version https://git-lfs.github.com/spec/v1\noid sha256:0000\nsize 12345\n'


def _clone_bundle_shallow(src, dst):
    dst.mkdir(parents=True)
    for item in src.iterdir():
        if item.is_dir():
            sub = dst / item.name
            sub.mkdir()
            for child in item.iterdir():
                (sub / child.name).symlink_to(child.resolve())
        else:
            shutil.copy2(item, dst / item.name)


def test_default_bundle_and_python_runtime_pass():
    versions = check_python_runtime(require_solver=True)
    assert 'pinocchio' in versions and 'clarabel' in versions
    info = check_model_bundle()
    assert info['model_id'] == 'casbot_hl12p1_dvt2_bare_flange_1105_03_gauge_v2'
    assert info['geometries'] > 14


@pytest.mark.parametrize('rel_target', ['collision.npz', 'visual_meshes/part_005_0.stl'])
def test_unhydrated_lfs_pointer_in_bundle_rejected_clearly(tmp_path, rel_target):
    bundle_copy = tmp_path / 'bundle'
    _clone_bundle_shallow(DEFAULT_BUNDLE, bundle_copy)
    target = bundle_copy / rel_target
    if target.is_symlink():
        target.unlink()
    target.write_text(LFS_POINTER, encoding='utf-8')

    with pytest.raises(ValueError, match='Git LFS 指针'):
        check_model_bundle(bundle_copy)

    if rel_target.startswith('visual_meshes/'):
        with pytest.raises(ValueError, match='git lfs pull'):
            load_bundle(bundle_copy)


def test_native_binaries_check_requires_channel_start_and_mock_server(tmp_path):
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    fake_exec = bin_dir / 'shm_executor'
    fake_exec.write_text('#!/bin/sh\nexit 0\n')
    fake_exec.chmod(0o755)

    # Real /shm_mc requires shm_channel_start alongside shm_executor.
    with pytest.raises(ValueError, match='shm_channel_start'):
        check_native_binaries(executor=fake_exec, mock=False, endpoint='/shm_mc')

    # Mock mode requires shm_mock_server alongside shm_executor.
    with pytest.raises(ValueError, match='shm_mock_server'):
        check_native_binaries(executor=fake_exec, mock=True, endpoint='/shm_mc')

    fake_start = bin_dir / 'shm_channel_start'
    fake_start.write_text('#!/bin/sh\nexit 0\n')
    fake_start.chmod(0o755)
    assert check_native_binaries(executor=fake_exec, mock=False, endpoint='/shm_mc') == fake_exec.resolve()


@pytest.mark.skipif(sys.platform != 'linux', reason='POSIX flock only')
def test_hardware_preflight_checks_shm_lock_and_passwordless_sudo(tmp_path):
    dev_shm = tmp_path / 'dev_shm'
    dev_shm.mkdir()
    shm_file = dev_shm / 'shm_mc'
    shm_file.write_bytes(b'\0' * 64)

    # Simulate GUI holding exclusive flock on shm_mc.
    fd = os.open(shm_file, os.O_RDONLY)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(ValueError, match='占用'):
            check_hardware_environment(endpoint='/shm_mc', dev_shm=dev_shm)
    finally:
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)

    vendor = tmp_path / 'vendor'
    vendor.mkdir()
    for name in ('launcher.bash', 'run_ethercat_station.bash'):
        (vendor / name).touch()

    class ActiveVendorHost:
        def __init__(self):
            self.vendor = vendor

        def services(self):
            return {u: 'active' for u in SERVICES}

        def processes(self):
            return [dict(pid=10, name='hlorin', threads=[dict(tid=10, state='S')])]

    with pytest.raises(HandoffError, match='免密 sudo'):
        check_hardware_environment(
            endpoint='/shm_mc',
            dev_shm=dev_shm,
            host=ActiveVendorHost(),
            sudo_check=lambda: False,
        )

    ok = check_hardware_environment(
        endpoint='/shm_mc',
        dev_shm=dev_shm,
        host=ActiveVendorHost(),
        sudo_check=lambda: True,
    )
    assert ok['takeover_needed'] and not ok['standalone_ready']

    class StandaloneStationHost:
        def __init__(self):
            self.vendor = vendor

        def services(self):
            return {u: 'inactive' for u in SERVICES}

        def processes(self):
            return [dict(pid=20, name='ethercat_station', threads=[dict(tid=20, state='S')])]

    # When already in standalone station window, sudo_check is not even needed.
    ready = check_hardware_environment(
        endpoint='/shm_mc',
        dev_shm=dev_shm,
        host=StandaloneStationHost(),
        sudo_check=lambda: (_ for _ in ()).throw(AssertionError('sudo should not be called')),
    )
    assert ready['standalone_ready'] and not ready['takeover_needed']


def test_preflight_failure_leaves_no_empty_session_dir(tmp_path):
    out_root = tmp_path / 'runs'
    with pytest.raises(ValueError, match='找不到原生执行器'):
        run_pipeline(
            stages='h1',
            output_root=out_root,
            executor=tmp_path / 'missing_executor',
            mock=True,
        )
    assert not out_root.exists()


def test_setup_check_and_build_script_defaults(tmp_path):
    help_res = subprocess.run(['bash', str(ROOT / 'scripts/shm/build.sh'), '--help'], capture_output=True, text=True, check=True)
    assert '默认 build/shm-core' in help_res.stdout
    assert '默认 install/shm-core' in help_res.stdout

    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    for name in ('shm_executor', 'shm_channel_start'):
        p = bin_dir / name
        p.write_text('#!/bin/sh\nexit 0\n')
        p.chmod(0o755)

    env = dict(os.environ, CASBOT_SHM_PYTHON=sys.executable, CASBOT_SHM_EXECUTOR=str(bin_dir / 'shm_executor'))
    res = subprocess.run(['bash', str(ROOT / 'calib/setup.sh'), '--check'], env=env, capture_output=True, text=True)
    assert res.returncode == 0, res.stderr + res.stdout
    assert '预检通过' in res.stdout

    info = run_preflight(plan_only=True)
    assert info['executor'] is None and info['model']['model_id'].startswith('casbot_hl12p1_dvt2')
