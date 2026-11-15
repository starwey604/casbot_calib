#!/usr/bin/env python3
"""Build the ARM64 wheel and C++ archive, then verify their private runtimes."""
import argparse
import json
import os
import shutil
from pathlib import Path
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
DIST = ROOT / 'dist/orin-aarch64'
PIP = '/opt/crossenv/cross/bin/pip'


def run(*args, **kwargs):
    subprocess.run(list(map(str, args)), cwd=ROOT, check=True, **kwargs)


def capture(*args):
    return subprocess.check_output(list(map(str, args)), cwd=ROOT, text=True).strip()


def timed(times, name, action):
    start = time.perf_counter()
    result = action()
    times[name] = round(time.perf_counter() - start, 3)
    print(f'{name}: {times[name]:.3f}s', flush=True)
    return result


def wheel_command(wheel_build, output, thinlto=True):
    defines = {
        'CMAKE_TOOLCHAIN_FILE': ROOT / 'cmake/toolchains/orin-aarch64-llvm.cmake',
        'CMAKE_PREFIX_PATH': '/opt/ros/humble',
        'CALIB_ORIN_THINLTO': 'ON' if thinlto else 'OFF',
        'Python_NumPy_INCLUDE_DIR': '/opt/crossenv/cross/lib/python3.10/site-packages/numpy/core/include',
        'Python3_NumPy_INCLUDE_DIR': '/opt/crossenv/cross/lib/python3.10/site-packages/numpy/core/include',
        'CMAKE_CXX_COMPILER_LAUNCHER': 'ccache',
        'CMAKE_INTERPROCEDURAL_OPTIMIZATION': 'OFF',  # Keep our explicit ThinLTO flags.
        'CALIB_BUILD_PYTHON': 'ON', 'CALIB_BUILD_SHM': 'OFF', 'BUILD_TESTING': 'OFF',
        'pybind11_DIR': '/opt/crossenv/cross/lib/python3.10/site-packages/pybind11/share/cmake/pybind11',
    }
    return [PIP, 'wheel', '.', '--no-deps', '--no-build-isolation', '--no-index',
            '--disable-pip-version-check', '-w', str(output),
            f'-Cbuild-dir={wheel_build}', '-Cbuild.targets=_core',
            *[f'-Ccmake.define.{key}={value}' for key, value in defines.items()]]


def install_components(cpp, prefix, tests):
    shutil.rmtree(prefix, ignore_errors=True)
    shutil.rmtree(tests, ignore_errors=True)
    for component in ('Core', 'Shm'):
        run('cmake', '--install', cpp, '--component', component, '--prefix', prefix)
    run('cmake', '--install', cpp, '--component', 'CrossTests', '--prefix', tests)
    (tests / 'lib').mkdir()
    for library in (prefix / 'lib').glob('libomp.so*'):
        shutil.copy2(library, tests / 'lib' / library.name)

def build(args):
    root = args.build_root
    cpp, wheel_build = root / 'orin-cross', root / 'orin-wheel'
    prefix = root / 'orin-cross-dev-install'
    tests = root / 'orin-cross-tests'
    dist = args.output_dir
    os.environ['CCACHE_DIR'] = str(root / 'ccache-orin-cross')
    os.environ['CMAKE_BUILD_PARALLEL_LEVEL'] = '2'
    times = {}
    dist.mkdir(parents=True, exist_ok=True)
    timed(times, 'configure', lambda: run('cmake', '--preset', 'orin-cross', '-B', cpp, f'-DCMAKE_INSTALL_PREFIX={prefix}',
          f'-DCALIB_ORIN_THINLTO={"ON" if args.thinlto == "on" else "OFF"}'))
    if args.rebuild:
        for directory in (cpp, wheel_build):
            if (directory / 'build.ninja').exists():
                run('cmake', '--build', directory, '--target', 'clean')
    timed(times, 'cpp_build', lambda: run('cmake', '--build', cpp, '--parallel', '2'))
    timed(times, 'cpp_install', lambda: install_components(cpp, prefix, tests))
    with tempfile.TemporaryDirectory(prefix='wheel-', dir=root) as temporary:
        def build_wheel():
            run(*wheel_command(wheel_build, temporary, args.thinlto == "on"))
            code_wheel, = Path(temporary).glob('*.whl')
            shutil.copy2(code_wheel, dist / code_wheel.name)
            return code_wheel
        wheel = timed(times, 'wheel_build_pack', build_wheel)
    archive = dist / 'shm-core-aarch64.tar.gz'
    def pack_runtime():
        with tempfile.TemporaryDirectory(prefix='orin-runtime-', dir=root) as temporary:
            stage = Path(temporary)
            (stage / 'bin').mkdir()
            for binary in (prefix / 'bin').iterdir():
                if not binary.name.endswith('_tests') and binary.name != 'openmp_probe':
                    installed = stage / 'bin' / binary.name
                    shutil.copy2(binary, installed)
                    run('llvm-strip-22', '--strip-unneeded', installed)
            (stage / 'lib').mkdir()
            for library in (prefix / 'lib').glob('libomp.so*'):
                shutil.copy2(library, stage / 'lib' / library.name)
            shutil.copytree(prefix / 'share/licenses', stage / 'share/licenses')
            run('tar', '--use-compress-program=gzip -9', '-cf', archive, '-C', stage, '.')
    timed(times, 'cpp_pack', pack_runtime)
    metadata = {
        'llvm_package_version': capture('dpkg-query', '-W', '-f=${Version}', 'clang-22'),
        'openmp_package_version': Path('/opt/orin-openmp/version.txt').read_text().strip(),
        'compiler': capture('clang++-22', '--version').splitlines()[0],
        'target': 'aarch64-linux-gnu', 'cpu': 'cortex-a78ae',
        'flags': ['-O3', *(['-flto=thin'] if args.thinlto == 'on' else []), '-fuse-ld=lld-22', '-fopenmp=libomp', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-Wl,--icf=safe'],
        'build_root': str(root), 'rebuild': args.rebuild,
        'test_prefix': str(tests), 'development_prefix': str(prefix),
        'artifacts': [{'name': path.name, 'bytes': path.stat().st_size}
                      for path in (dist / wheel.name, archive)],
        'timings_seconds': times,
    }
    (root / 'orin-package.json').write_text(json.dumps(metadata, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description='交叉构建 ARM64 wheel 与 C++ 包，默认执行 QEMU 随包运行库验证。', usage='%(prog)s [--no-verify] [--rebuild] [--build-root DIR] [--thinlto on|off] [--output-dir DIR]')
    parser.add_argument('--no-verify', action='store_true', help='跳过 QEMU 验证')
    parser.add_argument('--rebuild', action='store_true', help='清理目标文件后全量重编，保留 ccache/ThinLTO 缓存')
    parser.add_argument('--build-root', type=Path, default=ROOT / 'build', help='仓库内构建/缓存根目录；默认 build')
    parser.add_argument('--thinlto', choices=('on', 'off'), default='on', help='ThinLTO 对照；默认 on')
    parser.add_argument('--output-dir', type=Path, default=DIST, help='产物目录；默认 dist/orin-aarch64')
    parser.add_argument('--inside-builder', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    args.build_root = (ROOT / args.build_root).resolve()
    args.output_dir = (ROOT / args.output_dir).resolve()
    if args.inside_builder:
        build(args)
        return
    start = time.perf_counter()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    manifest = args.output_dir / 'manifest.json'
    manifest.unlink(missing_ok=True)
    command = ['bash', ROOT / 'scripts/cross/run.sh', 'python3', Path(__file__),
               '--inside-builder', '--build-root', args.build_root, '--thinlto', args.thinlto, '--output-dir', args.output_dir]
    if args.rebuild:
        command.append('--rebuild')
    run(*command)
    metadata = json.loads((args.build_root / 'orin-package.json').read_text())
    metadata.update(git_head=capture('git', 'rev-parse', 'HEAD'),
                    dirty=bool(capture('git', 'status', '--porcelain')),
                    verification='not_run')
    manifest.write_text(json.dumps(metadata, indent=2) + '\n')
    if not args.no_verify:
        try:
            timed(metadata['timings_seconds'], 'qemu_verify',
                  lambda: run('bash', ROOT / 'scripts/shm/verify_orin_qemu.sh', '--dist', args.output_dir, '--smoke'))
            metadata['verification'] = 'passed'
        except subprocess.CalledProcessError:
            metadata['verification'] = 'failed'
            manifest.write_text(json.dumps(metadata, indent=2) + '\n')
            raise
    metadata['timings_seconds']['total'] = round(time.perf_counter() - start, 3)
    manifest.write_text(json.dumps(metadata, indent=2) + '\n')
    print(f'产物：{args.output_dir}\n总耗时：{metadata["timings_seconds"]["total"]:.3f}s')


if __name__ == '__main__':
    main()
