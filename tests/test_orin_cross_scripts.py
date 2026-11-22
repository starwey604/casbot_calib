"""Cross infrastructure tests; Docker calls are captured without building images."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts' / 'cross'


@pytest.fixture
def docker_env(tmp_path):
    docker = tmp_path / 'docker'
    docker.write_text('''#!/usr/bin/env python3
import json, os, sys
with open(os.environ['DOCKER_LOG'], 'a') as stream:
    stream.write(json.dumps(sys.argv[1:]) + '\\n')
if sys.argv[1:3] == ['image', 'inspect']:
    sys.exit(int(sys.argv[3] in os.environ.get('MISSING_IMAGES', '').split(',')))
''')
    docker.chmod(0o755)
    log = tmp_path / 'docker.jsonl'
    env = dict(os.environ, PATH=f'{tmp_path}:{os.environ["PATH"]}', DOCKER_LOG=str(log))
    return env, log


def calls(log):
    return [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []


@pytest.mark.parametrize('script', ['ensure_images.sh', 'build.sh', 'verify_openmp.sh'])
def test_help_and_bad_options_do_not_run_docker(docker_env, script):
    env, log = docker_env
    help_result = subprocess.run(['bash', str(SCRIPTS / script), '--help'], env=env, capture_output=True, text=True)
    assert help_result.returncode == 0 and '用法' in help_result.stdout
    bad_result = subprocess.run(['bash', str(SCRIPTS / script), '--unknown'], env=env, capture_output=True)
    assert bad_result.returncode == 2
    assert calls(log) == []


def test_existing_images_are_reused(docker_env):
    env, log = docker_env
    subprocess.run(['bash', str(SCRIPTS / 'ensure_images.sh')], env=env, check=True)
    assert len(calls(log)) == 2
    assert all(call[:2] == ['image', 'inspect'] for call in calls(log))


def test_only_missing_builder_is_built(docker_env):
    env, log = docker_env
    env['MISSING_IMAGES'] = 'casbot-calib/orin-x86-cross-builder:humble'
    subprocess.run(['bash', str(SCRIPTS / 'ensure_images.sh')], env=env, check=True)
    builds = [call for call in calls(log) if call[:2] == ['buildx', 'build']]
    assert len(builds) == 1
    assert builds[0][builds[0].index('--platform') + 1] == 'linux/amd64'
    assert builds[0][builds[0].index('--target') + 1] == 'orin-x86-cross-builder'
    assert builds[0][-1] == str(SCRIPTS)


def test_rebuild_uses_both_architectures(docker_env):
    env, log = docker_env
    subprocess.run(['bash', str(SCRIPTS / 'ensure_images.sh'), '--rebuild'], env=env, check=True)
    builds = calls(log)
    assert [call[call.index('--platform') + 1] for call in builds] == ['linux/arm64', 'linux/amd64']
    assert all('--load' in call for call in builds)


def test_command_arguments_and_user_survive_container_entry(docker_env):
    env, log = docker_env
    command = ['printf', '%s', 'a path with spaces', '$ORIGIN']
    subprocess.run(['bash', str(SCRIPTS / 'run.sh'), *command], env=env, check=True)
    run = calls(log)[-1]
    assert run[-len(command):] == command
    assert run[run.index('--user') + 1] == f'{os.getuid()}:{os.getgid()}'
    assert run[run.index('--workdir') + 1] == str(ROOT)


def test_normalize_absolute_and_directory_symlinks(tmp_path):
    spec = importlib.util.spec_from_file_location('normalize_sysroot', SCRIPTS / 'normalize_sysroot.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    root = tmp_path / 'root'
    (root / 'usr/lib').mkdir(parents=True)
    (root / 'usr/lib/library.so').write_text('target')
    (root / 'lib').symlink_to('/usr/lib')
    (root / 'absolute').symlink_to('/usr/lib/library.so')
    (root / 'missing').symlink_to('/usr/lib/missing.so')
    (root / 'relative').symlink_to('usr/lib/library.so')
    assert module.normalize(root) == 3
    assert (root / 'lib/library.so').read_text() == 'target'
    assert (root / 'absolute').resolve() == root / 'usr/lib/library.so'
    assert os.readlink(root / 'missing') == 'usr/lib/missing.so'
    assert os.readlink(root / 'relative') == 'usr/lib/library.so'
    assert module.normalize(root) == 0


@pytest.mark.parametrize('script', ['cross_build_orin.sh', 'verify_orin_qemu.sh'])
def test_package_entry_help_and_bad_options(docker_env, script):
    env, log = docker_env
    path = ROOT / 'scripts/shm' / script
    result = subprocess.run(['bash', str(path), '--help'], env=env, capture_output=True, text=True)
    assert result.returncode == 0 and 'ARM64' in result.stdout
    result = subprocess.run(['bash', str(path), '--unknown'], env=env, capture_output=True)
    assert result.returncode == 2
    assert calls(log) == []


def load_cross_module(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_wheel_command_uses_target_headers_and_thinlto(tmp_path):
    package = load_cross_module('package_orin')
    command = package.wheel_command(tmp_path / 'build with spaces', tmp_path / 'dist')
    assert '--no-deps' in command and '--no-build-isolation' in command and '--no-index' in command
    assert f'-Cbuild-dir={tmp_path / "build with spaces"}' in command
    assert '-Cbuild.targets=_core' in command
    assert '-Ccmake.define.CMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF' in command
    assert '-Ccmake.define.CMAKE_PREFIX_PATH=/opt/ros/humble' in command
    for name in ('Python', 'Python3'):
        assert f'-Ccmake.define.{name}_NumPy_INCLUDE_DIR=/opt/crossenv/cross/lib/python3.10/site-packages/numpy/core/include' in command


def test_failed_stage_stops_without_success_timing():
    package = load_cross_module('package_orin')
    timings = {}
    def fail():
        raise subprocess.CalledProcessError(1, ['build'])
    with pytest.raises(subprocess.CalledProcessError):
        package.timed(timings, 'wheel_build_pack', fail)
    assert 'wheel_build_pack' not in timings


@pytest.mark.parametrize('header,dynamic,valid', [
    ('Machine: AArch64', '(RUNPATH) [$ORIGIN/.libs:/opt/ros/humble/lib]', True),
    ('Machine: X86-64', '(RUNPATH) [$ORIGIN/.libs]', False),
    ('Machine: AArch64', '(RUNPATH) [/sysroot/arm64/lib]', False),
    ('Machine: AArch64', '(RUNPATH) [/opt/orin-openmp/lib]', False),
    ('Machine: AArch64', '(RUNPATH) [/repo/build/lib]', False),
])
def test_package_elf_architecture_and_runtime_paths(monkeypatch, header, dynamic, valid):
    verify = load_cross_module('verify_package')
    monkeypatch.setattr(verify.subprocess, 'check_output',
                        lambda args, **kwargs: header if '-h' in args else dynamic)
    if valid:
        assert verify.elf_audit(Path('probe')) == dynamic
    else:
        with pytest.raises(RuntimeError):
            verify.elf_audit(Path('probe'))


@pytest.mark.parametrize('name,fixtures,filename,expected', [
    ('test_preview_widgets_without_graphics_or_control', [], 'test_shm_gui.py', 'host GUI extra (imgui_bundle)'),
    ('test_h1_full_offline_planning_gate', [], 'test_wave2a_h1.py', None),
    ('test_hold_enable_and_disable', ['master'], 'test_shm_fieldwork.py', 'live SHM / physical Mock scheduling'),
    ('test_native_rejects_bad_schedule_before_opening_any_channel', ['native'], 'test_shm_ready.py', None),
    ('test_backend_selection_and_environment_restoration', ['native'], 'test_fieldwork_window.py', None),
    ('test_qemu_excludes_scheduling_but_keeps_offline_safety', [], 'test_orin_cross_scripts.py',
     'amd64 Docker build orchestration (tested on host)'),
    ('test_only_zero_write_no_fault_start_mismatch_can_replan', [], 'test_shm_h1.py', 'live SHM / physical Mock scheduling'),
])
def test_qemu_excludes_scheduling_but_keeps_offline_safety(name, fixtures, filename, expected):
    module = load_cross_module('pytest_qemu')
    assert module.exclusion(name, fixtures, filename) == expected


def test_deploy_streams_assets_wheels_and_prebuilt_over_one_connection(tmp_path):
    import shutil
    import socket
    repo = tmp_path / 'source'
    (repo / 'scripts/shm').mkdir(parents=True)
    for name in ('deploy_git.sh', 'runtime_assets.py'):
        shutil.copy2(ROOT / 'scripts/shm' / name, repo / 'scripts/shm' / name)
    def git(*args):
        return subprocess.run(['git', '-C', str(repo), *args], check=True, capture_output=True)
    git('init', '-b', 'main')
    git('config', 'user.name', 'Deployment test')
    git('config', 'user.email', 'test@example.invalid')
    bundle = repo / 'python/casbot_calib/assets/models/casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2'
    (bundle / 'visual_meshes').mkdir(parents=True)
    (bundle / 'collision.npz').write_text('version https://git-lfs.github.com/spec/v1\n')
    (bundle / 'visual_meshes/part.stl').write_text('version https://git-lfs.github.com/spec/v1\n')
    import runpy
    assets = runpy.run_path(str(ROOT / 'scripts/shm/runtime_assets.py'))['FILES']
    for relative in assets:
        (repo / relative).parent.mkdir(parents=True, exist_ok=True)
        (repo / relative).write_text('version https://git-lfs.github.com/spec/v1\n')
    (repo / '.gitignore').write_text('artifacts/\ndist/\n')
    git('add', '.')
    git('commit', '-m', 'Fixture')
    for relative in assets:
        (repo / relative).write_bytes(b'asset-data' * 200)
    (bundle / 'visual_meshes/part.stl').write_bytes(b'mesh-data' * 200)
    wheels = repo / 'artifacts/wheelhouse_aarch64'
    wheels.mkdir(parents=True)
    (wheels / 'target-linux_aarch64.whl').write_bytes(b'arm wheel')
    (wheels / 'host-x86_64.whl').write_bytes(b'host wheel')
    dist = repo / 'dist/orin-aarch64'
    dist.mkdir(parents=True)
    for name in ('manifest.json', 'shm-core-aarch64.tar.gz', 'casbot_calib-0.1.0-cp310-cp310-linux_aarch64.whl'):
        (dist / name).write_bytes(name.encode())
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    lfs = bin_dir / 'git-lfs'
    lfs.write_text('#!/bin/sh\ntest "$1" = install\n')
    lfs.chmod(0o755)
    ssh = bin_dir / 'ssh'
    ssh.write_text('''#!/usr/bin/env python3
import os, subprocess, sys
args=sys.argv[1:]
if '-O' in args: sys.exit(0)
assert args[-2] == 'local-test', args
sys.exit(subprocess.call(['bash', '-c', args[-1]]))
''')
    ssh.chmod(0o755)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    destination = tmp_path / 'destination with spaces'
    env = dict(os.environ, PATH=f'{bin_dir}:{os.environ["PATH"]}')
    command = ['bash', str(repo / 'scripts/shm/deploy_git.sh'), '--host', 'local-test',
               '--destination', str(destination), '--port', str(port), '--with-prebuilt', '--committed-only']
    result = subprocess.run(command, env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    for path in dist.iterdir():
        assert (destination / 'artifacts/prebuilt_aarch64' / path.name).read_bytes() == path.read_bytes()
    assert (destination / 'artifacts/wheelhouse_aarch64/target-linux_aarch64.whl').read_bytes() == b'arm wheel'
    assert not (destination / 'artifacts/wheelhouse_aarch64/host-x86_64.whl').exists()
    for path in (repo / relative for relative in assets):
        assert (destination / path.relative_to(repo)).read_bytes() == path.read_bytes()


def test_python_path_preserves_native_and_selects_installed_mode(tmp_path):
    venv = tmp_path / 'venv with spaces'
    (venv / 'bin').mkdir(parents=True)
    command = ['bash', '-c', 'repo=$1; source "$repo/scripts/shm/python_path.sh" "$2"; printf "%s|%s" "${PYTHONPATH:-}" "${CASBOT_PROJECT_ROOT:-}"',
               '--', str(ROOT), str(venv)]
    (venv / 'bin/python').symlink_to(sys.executable)
    env = dict(os.environ, PYTHONPATH='/extra/path')
    env.pop('CASBOT_SHM_PYTHON', None)
    env.pop('CASBOT_PROJECT_ROOT', None)
    assert subprocess.check_output(command, env=env, text=True) == f'{ROOT}/python:/extra/path|'
    (venv / 'casbot-prebuilt').touch()
    assert subprocess.check_output(command, env=env, text=True) == f'|{ROOT}'


def test_wheel_excludes_large_assets_and_keeps_small_config():
    import tomli
    config = tomli.loads((ROOT / 'pyproject.toml').read_text())['tool']['scikit-build']
    assert config['wheel']['exclude'] == ['casbot_calib/assets/models/**', 'casbot_calib/assets/fieldwork_display/**']
    assert not (ROOT / 'scripts/cross/pack_assets.py').exists()


def test_qemu_dependency_check_uses_target_metadata(tmp_path):
    verify = load_cross_module('verify_package')
    site = tmp_path / 'site'
    info = site / 'example_pkg-1.0.dist-info'
    info.mkdir(parents=True)
    (info / 'METADATA').write_text('Name: Example-Pkg\nVersion: 1.0\n')
    req = tmp_path / 'requirements.txt'
    nested = tmp_path / 'nested.txt'
    req.write_text('-r nested.txt\n')
    nested.write_text('example_pkg==1.0\n')
    assert verify.requirements_ready(site, req)
    nested.write_text('example_pkg==2.0\n')
    assert not verify.requirements_ready(site, req)


def test_smoke_command_reports_elapsed_time_and_propagates_failure(monkeypatch):
    smoke = load_cross_module('qemu_package_smoke')
    ticks = iter([10.0, 10.125, 20.0])
    monkeypatch.setattr(smoke.time, 'perf_counter', lambda: next(ticks))
    calls = []
    monkeypatch.setattr(smoke.subprocess, 'run', lambda args, **kwargs: calls.append((args, kwargs)))
    assert smoke.run(Path('/test/path with spaces'), 4, env={'OMP_NUM_THREADS': '4'}) == 0.125
    assert calls == [(['/test/path with spaces', '4'], dict(check=True, env={'OMP_NUM_THREADS': '4'}))]
    def fail(*args, **kwargs):
        raise subprocess.CalledProcessError(3, args[0])
    monkeypatch.setattr(smoke.subprocess, 'run', fail)
    with pytest.raises(subprocess.CalledProcessError) as error:
        smoke.run('failed-test')
    assert error.value.returncode == 3


@pytest.mark.parametrize('mode', [None, '--check-only', '--no-setup'])
def test_deploy_orin_builds_syncs_and_installs_prebuilt(tmp_path, mode):
    import shutil
    import shlex
    repo = tmp_path / 'source'
    scripts = repo / 'scripts/shm'
    scripts.mkdir(parents=True)
    shutil.copy2(ROOT / 'scripts/shm/deploy_orin.sh', scripts / 'deploy_orin.sh')
    log = tmp_path / 'commands.jsonl'
    capture = """#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
with open(os.environ['DEPLOY_LOG'], 'a') as stream:
    stream.write(json.dumps([Path(sys.argv[0]).name, *sys.argv[1:]]) + '\\n')
"""
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    for path in (scripts / 'cross_build_orin.sh', scripts / 'deploy_git.sh',
                 repo / 'scripts/jetson_network.sh'):
        # The entry calls these through bash; exec into the same Python capture.
        path.write_text("#!/bin/bash\nexec python3 " + shlex.quote(str(bin_dir / path.name)) + ' "$@"\n')
        recorder = bin_dir / path.name
        recorder.write_text(capture)
        recorder.chmod(0o755)
    (bin_dir / 'ssh').write_text(capture)
    (bin_dir / 'ssh').chmod(0o755)
    env = dict(os.environ, PATH=f'{bin_dir}:{os.environ["PATH"]}', DEPLOY_LOG=str(log))
    command = ['bash', str(scripts / 'deploy_orin.sh'), '--host', 'local-test',
               '--interface', 'fake-usb', '--destination', '/tmp/project with spaces',
               '--branch', 'main', '--committed-only', '--no-wheels']
    subprocess.run(command + ([mode] if mode else []), env=env, check=True)
    recorded = calls(log)
    assert recorded[0] == ['jetson_network.sh', 'up', 'fake-usb']
    assert recorded[1][-2:] == ['local-test', 'true']
    assert 'sudo -n date' in recorded[2][-1]
    build = [c for c in recorded if c[0] == 'cross_build_orin.sh']
    assert build == ([] if mode == '--check-only' else [['cross_build_orin.sh', '--no-verify']])
    sync, = [c for c in recorded if c[0] == 'deploy_git.sh']
    expected = ['deploy_git.sh', '--host', 'local-test', '--destination', '/tmp/project with spaces',
                '--branch', 'main', '--committed-only', '--no-wheels']
    assert sync == expected + ([] if mode == '--check-only' else ['--with-prebuilt'])
    setup = [c for c in recorded if c[0] == 'ssh' and 'calib/setup.sh' in c[-1]]
    if mode == '--no-setup':
        assert setup == [] and recorded[-1] == sync
    else:
        assert len(setup) == 1 and recorded[-1] == setup[0]
        assert shlex.split(setup[0][-1]) == ['cd', '/tmp/project with spaces', '&&',
                                           'bash', 'calib/setup.sh',
                                           '--check' if mode == '--check-only' else '--prebuilt']
