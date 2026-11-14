"""Environment, model LFS materialization, binary, and SHM preflight checks."""
import argparse
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys
from ..dvt2_bundle import robot_profile
from ..model_context import BUNDLE_RELATIVE
from ..model_sources import model_directory
from ..shm_takeover import HandoffError, Host
from .robot import find_executor
from .takeover import check_channel_unlocked, station_standalone

LFS_PREFIX = b'version https://git-lfs.github.com/spec/v1'


def _is_lfs_pointer(path):
    with Path(path).open('rb') as stream:
        return stream.read(80).startswith(LFS_PREFIX)


def check_python_runtime(*, require_solver=True):
    required = ('numpy', 'scipy', 'yaml', 'pinocchio', 'casbot_calib._core')
    if require_solver:
        required = (*required, 'clarabel')
    versions = {}
    missing = []
    for name in required:
        try:
            mod = importlib.import_module(name)
            versions[name] = getattr(mod, '__version__', 'loaded')
        except ImportError as error:
            missing.append(f'{name} ({error})')
    if missing:
        raise RuntimeError(
            '缺少 Python 数值/动力学运行依赖：'
            + ', '.join(missing)
            + '；请先运行 bash calib/setup.sh'
        )
    return versions


def check_model_bundle(bundle=None):
    root = Path(model_directory() / BUNDLE_RELATIVE if bundle is None else bundle).resolve()
    metadata_path = root / 'model.json'
    if not metadata_path.is_file():
        raise FileNotFoundError(f'找不到默认模型包配置：{metadata_path}')
    if _is_lfs_pointer(metadata_path):
        raise ValueError(f'模型配置仍为 Git LFS 指针：{metadata_path}')

    metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
    required_rel = [
        'model.urdf',
        'collision.json',
        'collision.npz',
        'trical_scene.json',
    ]
    if 'viewer_sha256' in metadata:
        required_rel.append('scene.xml')
    required_rel.extend(metadata.get('viewer_files_sha256', {}).keys())

    missing = []
    unhydrated = []
    for rel in dict.fromkeys(required_rel):
        target = root / rel
        if not target.is_file():
            missing.append(rel)
        elif _is_lfs_pointer(target):
            unhydrated.append(rel)

    if missing:
        raise FileNotFoundError(f'模型包缺少资产文件（{root}）：{", ".join(missing)}')
    if unhydrated:
        raise ValueError(
            f'模型资产仍为未物化的 Git LFS 指针（{root}）：{", ".join(unhydrated)}；'
            '请在本机运行 bash scripts/shm/deploy_git.sh --host nvidia@172.16.0.10 自动同步模型包，或运行 git lfs pull'
        )

    profile = robot_profile(root)
    return dict(bundle=str(root), model_id=profile.metadata['model_id'], geometries=profile.metadata['actual_geometry_count'])


def check_native_binaries(*, executor=None, mock=False, endpoint='/shm_mc'):
    binary = find_executor(executor)
    if not os.access(binary, os.X_OK):
        raise ValueError(f'原生执行器不可执行：{binary}')
    if mock:
        mock_srv = binary.with_name('shm_mock_server')
        if not mock_srv.is_file() or not os.access(mock_srv, os.X_OK):
            raise ValueError(f'找不到物理 Mock 服务：{mock_srv}（请用 bash calib/setup.sh 或 --with-mock 编译）')
    elif not endpoint.startswith('/casbot_calib_test_'):
        starter = binary.with_name('shm_channel_start')
        if not starter.is_file() or not os.access(starter, os.X_OK):
            raise ValueError(f'找不到通道握手工具：{starter}（请先运行 bash calib/setup.sh）')
    return binary


def _default_sudo_check():
    try:
        res = subprocess.run(['sudo', '-n', 'true'], capture_output=True, text=True, timeout=3)
        return res.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def check_hardware_environment(*, endpoint='/shm_mc', dev_shm=Path('/dev/shm'), host=None, sudo_check=None):
    check_channel_unlocked(endpoint, dev_shm=dev_shm)
    if endpoint.startswith('/casbot_calib_test_'):
        return dict(takeover_needed=False, standalone_ready=True)

    host = Host() if host is None else host
    if station_standalone(host):
        return dict(takeover_needed=False, standalone_ready=True)

    for script_name in ('launcher.bash', 'run_ethercat_station.bash'):
        if not (host.vendor / script_name).is_file():
            raise HandoffError(f'找不到厂家控制脚本：{host.vendor / script_name}（确认当前在 Orin 本机运行）')

    checker = _default_sudo_check if sudo_check is None else sudo_check
    if not checker():
        raise HandoffError('当前需要接管 EtherCAT 主站，但免密 sudo（sudo -n）不可用；请先配置免密 sudo 后重试')
    return dict(takeover_needed=True, standalone_ready=False)


def run_preflight(
    *,
    plan_only=False,
    mock=False,
    endpoint='/shm_mc',
    executor=None,
    bundle=None,
    dev_shm=Path('/dev/shm'),
    host=None,
    sudo_check=None,
):
    versions = check_python_runtime(require_solver=True)
    model_info = check_model_bundle(bundle)
    result = dict(
        python=sys.executable,
        packages=versions,
        model=model_info,
        executor=None,
        hardware=None,
    )
    if plan_only:
        return result

    binary = check_native_binaries(executor=executor, mock=mock, endpoint=endpoint)
    result['executor'] = str(binary)
    if not mock:
        result['hardware'] = check_hardware_environment(
            endpoint=endpoint,
            dev_shm=dev_shm,
            host=host,
            sudo_check=sudo_check,
        )
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description='CASBOT SHM 标定环境与模型资产预检')
    p.add_argument('--offline', action='store_true', help='仅检查 Python 依赖、模型 LFS 资产和原生二进制，不检查实机主站/sudo')
    p.add_argument('--executor', type=Path, default=None, help=argparse.SUPPRESS)
    args = p.parse_args(argv)
    try:
        versions = check_python_runtime(require_solver=True)
        model_info = check_model_bundle()
        binary = check_native_binaries(executor=args.executor, mock=False, endpoint='/shm_mc')
        hw_status = '已跳过（离线检查）'
        if not args.offline:
            host = Host()
            if host.vendor.is_dir() or (Path('/dev/shm') / 'shm_mc').exists():
                hw = check_hardware_environment(endpoint='/shm_mc', host=host)
                hw_status = '已处于独立主站窗口' if hw['standalone_ready'] else '待自动接管（免密 sudo 可用）'
            else:
                hw_status = '非 Orin 实机环境（仅离线/Mock 可用）'
        print(f'预检通过：模型={model_info["model_id"]}，执行器={binary}，Pinocchio={versions["pinocchio"]}，实机状态={hw_status}')
        return 0
    except (ValueError, OSError, RuntimeError, HandoffError) as error:
        print(f'预检未通过：{error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
