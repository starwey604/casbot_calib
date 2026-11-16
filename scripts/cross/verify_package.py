#!/usr/bin/env python3
"""Install both packages separately and check their ARM64 OpenMP loading."""
import argparse
from importlib import metadata
import json
import os
import re
from pathlib import Path
import subprocess
import tempfile
import time
import zipfile
import tarfile
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[2]
DIST = ROOT / 'dist/orin-aarch64'
VENV = ROOT / 'build/orin-qemu-venv'
SITE = VENV / 'lib/python3.10/site-packages'


def run(*args):
    subprocess.run(list(map(str, args)), cwd=ROOT, check=True)


def elf_audit(path):
    header = subprocess.check_output(['llvm-readelf-22', '-h', str(path)], text=True)
    if 'AArch64' not in header:
        raise RuntimeError(f'{path}: expected AArch64')
    dynamic = subprocess.check_output(['llvm-readelf-22', '-d', str(path)], text=True)
    for line in dynamic.splitlines():
        if ('RUNPATH' in line or 'RPATH' in line) and any(
                bad in line for bad in ('/sysroot', '/opt/orin-openmp', '/build/')):
            raise RuntimeError(f'{path}: {line}')
    return dynamic


def prepare(directory, dist=DIST):
    metadata = json.loads((dist / 'manifest.json').read_text())
    wheel, = [dist / item['name'] for item in metadata['artifacts'] if item['name'].endswith('.whl')]
    if not wheel.name.endswith('-cp310-cp310-linux_aarch64.whl'):
        raise RuntimeError(f'Unexpected wheel tag: {wheel.name}')
    with zipfile.ZipFile(wheel) as archive:
        names = archive.namelist()
        assert wheel.stat().st_size < 4 * 1024**2
        assert not any(name.startswith(('casbot_calib/assets/models/', 'casbot_calib/assets/fieldwork_display/')) for name in names)
        for name in ('trical_measurement_defaults.json', 'trical_1105_release_turn_reference.json'):
            assert 'casbot_calib/assets/' + name in names
    with tarfile.open(dist / 'shm-core-aarch64.tar.gz') as archive:
        # Runtime includes the interactive TriCal executor alongside the dynamics tools.
        assert (dist / 'shm-core-aarch64.tar.gz').stat().st_size < 1.5 * 1024**2
        assert not any(member.name.endswith(('_tests', '.a', '.hpp', '.h', '.cmake', '/openmp_probe')) for member in archive)
    cpp = directory / 'cpp'
    cpp.mkdir()
    run('tar', '-xzf', dist / 'shm-core-aarch64.tar.gz', '-C', cpp)
    target = SITE
    run('/opt/crossenv/cross/bin/pip', 'install', '--no-index', '--no-deps', '--no-compile',
        '--disable-pip-version-check', '--upgrade', '--target', target, wheel)
    (VENV / 'casbot-prebuilt').touch()
    package = target / 'casbot_calib'
    tests = Path(metadata['test_prefix'])
    (directory / 'test-prefix.txt').write_text(str(tests))
    extension, = package.glob('_core*.so')
    for path in sorted((cpp / 'bin').iterdir()):
        elf_audit(path)
    for path, relative in ((extension, '$ORIGIN/.libs'),
                           (tests / 'bin/openmp_probe', '$ORIGIN/../lib')):
        dynamic = elf_audit(path)
        if 'libomp.so.5' not in dynamic or relative not in dynamic:
            raise RuntimeError(f'{path}: missing private OpenMP dependency/RUNPATH')
    for runtime, licenses in ((package / '.libs/libomp.so.5', package / '.libs/licenses'),
                              (cpp / 'lib/libomp.so.5', cpp / 'share/licenses/casbot_calib/openmp')):
        dynamic = elf_audit(runtime)
        versions = subprocess.check_output(['llvm-readelf-22', '--version-info', str(runtime)], text=True)
        glibc = [tuple(map(int, value.split('.'))) for value in re.findall(r'GLIBC_([0-9.]+)', versions)]
        if max(glibc) > (2, 35):
            raise RuntimeError(f'{runtime}: GLIBC requirement exceeds Jammy')
        if not any('SONAME' in line and 'libomp.so.5' in line for line in dynamic.splitlines()):
            raise RuntimeError(f'{runtime}: unexpected SONAME')
        for name in ('LICENSE.TXT', 'debian-copyright'):
            if not (licenses / name).is_file():
                raise RuntimeError(f'Missing {licenses / name}')
    for config in (Path(metadata['development_prefix']) / 'lib/cmake/casbot_calib').glob('*.cmake'):
        if any(path in config.read_text() for path in ('/sysroot', '/opt/orin-openmp', '/build/')):
            raise RuntimeError(f'Build path leaked into {config}')
    print('ELF、wheel 标签、RUNPATH、CMake 导出和两份 OpenMP 许可检查通过', flush=True)


def requirements_ready(site, requirements):
    installed = {dist.metadata['Name'].lower().replace('_', '-'): dist.version
                 for dist in metadata.distributions(path=[str(site)])}
    def matches(path):
        for raw in path.read_text().splitlines():
            line = raw.split('#', 1)[0].strip()
            if not line:
                continue
            if line.startswith('-r '):
                if not matches(path.parent / line[3:].strip()):
                    return False
            else:
                name, version = line.split('==', 1)
                if installed.get(name.lower().replace('_', '-')) != version:
                    return False
        return True
    return matches(requirements)


def runtime_command(*args, writable=(), readonly=()):
    command = ['docker', 'run', '--rm', '--platform', 'linux/arm64', '--env', 'QEMU_CPU=cortex-a78ae', '--user', f'{os.getuid()}:{os.getgid()}',
               '--volume', f'{ROOT}:{ROOT}:ro', '--volume', f'{VENV}:{VENV}', '--workdir', str(ROOT)]
    for directory in writable:
        command += ['--volume', f'{directory}:{directory}']
    for directory in readonly:
        command += ['--volume', f'{directory}:{directory}:ro']
    return [*command, 'casbot-calib/orin-arm64-runtime:humble',
            'env', '-u', 'LD_LIBRARY_PATH', '-u', 'PYTHONPATH', 'OPENBLAS_NUM_THREADS=1',
            'OMP_NUM_THREADS=4', 'OMP_DYNAMIC=FALSE', 'OMP_WAIT_POLICY=PASSIVE', 'PYTEST_DISABLE_PLUGIN_AUTOLOAD=1',
            'MPLBACKEND=Agg', f'MPLCONFIGDIR={VENV / "matplotlib"}',
            f'CASBOT_PROJECT_ROOT={ROOT}', f'CASBOT_SHM_PYTHON={VENV / "bin/python"}',
            *map(str, args)]


def main():
    parser = argparse.ArgumentParser(description='完整 ARM64/QEMU 数值栈、OpenMP 和非实时回归。',
                                     usage='%(prog)s [--wheelhouse DIR] [--smoke | --pytest [PYTEST_ARGS...]]')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--smoke', action='store_true', help='完整数值冒烟（默认）')
    mode.add_argument('--pytest', nargs=argparse.REMAINDER, help='同时运行非实时 pytest，可追加筛选参数')
    parser.add_argument('--wheelhouse', type=Path, default=ROOT / 'artifacts/wheelhouse_aarch64')
    parser.add_argument('--dist', type=Path, default=DIST, help='待验证产物目录')
    parser.add_argument('--prepare', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.prepare:
        prepare(args.prepare, args.dist)
        return
    started = time.perf_counter()
    wheelhouse = (ROOT / args.wheelhouse).resolve()
    run('bash', ROOT / 'scripts/cross/ensure_images.sh')
    required = [line.split('==') for line in (ROOT / 'scripts/cross/requirements-qemu.txt').read_text().splitlines() if '==' in line]
    if not all(any(wheelhouse.glob(f"{name.replace('-', '_')}-{version}-*.whl")) for name, version in required):
        run('bash', ROOT / 'scripts/cross/run.sh', 'bash', ROOT / 'scripts/shm/fetch_wheels.sh',
            '--aarch64-only', '--output', wheelhouse,
            '--requirements', ROOT / 'scripts/cross/requirements-qemu.txt')
    VENV.mkdir(parents=True, exist_ok=True)
    if not (VENV / 'bin/python').exists():
        run(*runtime_command('python3', '-m', 'venv', VENV))
    # Read target dist-info with host Python; no target imports or emulated pip startup on hot runs.
    if not requirements_ready(SITE, ROOT / 'scripts/cross/requirements-qemu.txt'):
        run(*runtime_command(VENV / 'bin/python', '-m', 'pip', 'install', '--no-index', '--disable-pip-version-check',
                             '--find-links', wheelhouse, '-r', ROOT / 'scripts/cross/requirements-qemu.txt',
                             readonly=(wheelhouse,)))
    report = ROOT / 'artifacts/orin-qemu' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    report.mkdir(parents=True)
    print(f'验证记录：{report}', flush=True)
    with tempfile.TemporaryDirectory(prefix='orin-package-check-', dir=ROOT / 'build') as temporary:
        run('bash', ROOT / 'scripts/cross/run.sh', 'python3', Path(__file__), '--prepare', temporary, '--dist', args.dist)
        command = [VENV / 'bin/python', ROOT / 'scripts/cross/qemu_package_smoke.py', temporary, report]
        if args.pytest is not None:
            command += ['--pytest', *args.pytest]
        run(*runtime_command(*command, writable=(temporary, report)))
    timing = dict(status="passed", pytest=args.pytest is not None, total_seconds=round(time.perf_counter() - started, 3))
    (report / "verification.json").write_text(json.dumps(timing, indent=2) + "\n")
    print(f"ARM64/QEMU 验证总耗时：{timing['total_seconds']:.3f}s", flush=True)


if __name__ == '__main__':
    main()
