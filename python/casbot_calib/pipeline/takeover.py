"""Automatic boot-scoped Orin SHM takeover, fresh-master handshake, and hand background cache."""
import fcntl
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import time
from ..data import write_json
from ..shm_data import ROOT
from ..shm_takeover import HandoffError, Host, SERVICES, alive, prepare as prepare_takeover

CHANNEL_BYTES = 16 * 1024 * 1024
STATUS_OFFSET = CHANNEL_BYTES // 2
QUEUE_MAGIC = 0x11223344


def current_boot_id(proc=Path('/proc')):
    path = Path(proc) / 'sys/kernel/random/boot_id'
    return path.read_text(encoding='utf-8').strip() if path.is_file() else 'unknown_boot'


def valid_hand_background(path):
    path = Path(path)
    if not path.is_file():
        return False
    data = path.read_bytes()
    return len(data) == 744 and struct.unpack_from('<H', data, 10)[0] <= 2


def check_channel_unlocked(endpoint, dev_shm=Path('/dev/shm')):
    shm_file = Path(dev_shm) / endpoint.lstrip('/')
    if not shm_file.exists():
        return
    fd = os.open(shm_file, os.O_RDONLY)
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ValueError(f'SHM 通道 {endpoint} 正被其它程序（如 GUI 或另一标定进程）占用，请先关闭后再运行') from error
        fcntl.flock(fd, fcntl.LOCK_UN)
    finally:
        os.close(fd)


def read_queue_headers(endpoint, dev_shm=Path('/dev/shm')):
    shm_file = Path(dev_shm) / endpoint.lstrip('/')
    if not shm_file.is_file():
        return None
    for suffix in ('_front', '_back', '_start'):
        if not (Path(dev_shm) / f'sem.{endpoint.lstrip("/")}{suffix}').exists():
            return None
    fd = os.open(shm_file, os.O_RDONLY)
    try:
        st = os.fstat(fd)
        if st.st_size != CHANNEL_BYTES:
            return None
        raw_cmd = os.pread(fd, 40, 0)
        raw_status = os.pread(fd, 40, STATUS_OFFSET)
        if len(raw_cmd) != 40 or len(raw_status) != 40:
            return None
        cmd = struct.unpack('<iiqqqq', raw_cmd)
        status = struct.unpack('<iiqqqq', raw_status)
        if cmd[0] != QUEUE_MAGIC or cmd[1] != 2 or status[0] != QUEUE_MAGIC or status[1] != 2:
            return None
        return dict(
            inode=(st.st_dev, st.st_ino),
            command=dict(count=cmd[2], write_index=cmd[3], read_index=cmd[4], published_index=cmd[5]),
            status=dict(count=status[2], write_index=status[3], read_index=status[4], published_index=status[5]),
        )
    finally:
        os.close(fd)


def _inspect_hand_background(executor, endpoint, target_dir):
    result = subprocess.run(
        [str(executor), '--inspect', '--shm', endpoint, '--output', str(target_dir)],
        capture_output=True,
        text=True,
        timeout=5,
    )
    candidate = Path(target_dir) / 'command.bin'
    if result.returncode == 0 and valid_hand_background(candidate):
        return candidate
    return None


def _search_prior_hand_background(boot_id, search_roots):
    for base in search_roots:
        base = Path(base)
        if not base.is_dir():
            continue
        cached = base / 'calib/state/boots' / boot_id / 'hand_background.bin'
        if valid_hand_background(cached):
            return cached
        for meta_file in sorted(base.rglob('case.json'), reverse=True):
            try:
                meta = json.loads(meta_file.read_text(encoding='utf-8'))
            except (OSError, ValueError):
                continue
            if meta.get('boot_id') != boot_id:
                continue
            for rel in ('hand_background_source.bin', 'native/hand_background_source.bin', 'native/command.bin', 'command.bin'):
                candidate = meta_file.parent / rel
                if valid_hand_background(candidate):
                    return candidate
    return None


def cached_hand_background(state_root=ROOT / 'artifacts/calib/state', boot_id=None):
    boot = boot_id or current_boot_id()
    path = Path(state_root) / 'boots' / boot / 'hand_background.bin'
    return path if valid_hand_background(path) else None


def station_standalone(host):
    states = host.services()
    if not all(states.get(unit) in ('inactive', 'failed') for unit in SERVICES):
        return False
    live = [p for p in host.processes() if alive(p)]
    if any(p['name'] == 'hlorin' for p in live):
        return False
    return any(p['name'] in ('ethercat_statio', 'ethercat_station') for p in live)


def ensure_ready_channel(
    state_root,
    executor,
    *,
    endpoint='/shm_mc',
    explicit_hand_background=None,
    host=None,
    dev_shm=Path('/dev/shm'),
    search_roots=(ROOT / 'artifacts',),
):
    """Ensure Orin vendor services are handed off, fresh master is shaken, and hand_background is ready."""
    executor = Path(executor).resolve()
    check_channel_unlocked(endpoint, dev_shm=dev_shm)

    if explicit_hand_background is not None:
        bg = Path(explicit_hand_background).resolve()
        if not valid_hand_background(bg):
            raise ValueError(f'无效的手部背景帧（需 744 字节且 hand_count≤2）：{bg}')
    else:
        bg = None

    # Private test endpoints never run systemd takeover.
    if endpoint.startswith('/casbot_calib_test_'):
        if bg is None:
            raise ValueError('私有测试端点未提供手部背景帧')
        return dict(endpoint=endpoint, hand_background=bg, boot_id=current_boot_id(), takeover_performed=False)

    host = Host() if host is None else host
    boot_id = current_boot_id(host.proc)
    boot_dir = Path(state_root).resolve() / 'boots' / boot_id
    boot_dir.mkdir(parents=True, exist_ok=True)
    cached_bg = boot_dir / 'hand_background.bin'

    if bg is not None:
        if bg != cached_bg:
            shutil.copyfile(bg, cached_bg)
    elif not valid_hand_background(cached_bg):
        headers = read_queue_headers(endpoint, dev_shm=dev_shm)
        if headers is not None and headers['command']['published_index'] > 0:
            inspect_dir = boot_dir / f'pre_takeover_inspect_{time.monotonic_ns()}'
            captured = _inspect_hand_background(executor, endpoint, inspect_dir)
            if captured is not None:
                shutil.copyfile(captured, cached_bg)
        if not valid_hand_background(cached_bg):
            found = _search_prior_hand_background(boot_id, search_roots)
            if found is not None and found.resolve() != cached_bg.resolve():
                shutil.copyfile(found, cached_bg)

    takeover_performed = False
    if not station_standalone(host):
        print('检测到当前开机窗口尚未完成主站接管，开始自动接管 EtherCAT 主站...', flush=True)
        takeover_dir = boot_dir / f'takeover_{time.strftime("%Y%m%d_%H%M%S")}_{time.monotonic_ns() % 1000000:06d}'
        code = prepare_takeover(takeover_dir, host=host)
        if code != 0:
            raise HandoffError(f'主站接管失败（退出码 {code}），诊断见 {takeover_dir}')
        takeover_performed = True

    deadline = time.monotonic() + 15.0
    headers = read_queue_headers(endpoint, dev_shm=dev_shm)
    while headers is None and time.monotonic() < deadline:
        time.sleep(0.1)
        headers = read_queue_headers(endpoint, dev_shm=dev_shm)
    if headers is None:
        raise HandoffError(f'主站共享内存 {endpoint} 未就绪，请检查主站启动日志')

    handshake_performed = False
    if headers['command']['published_index'] == 0 and headers['status']['published_index'] == 0:
        starter = executor.with_name('shm_channel_start')
        result = subprocess.run(
            [str(starter), '--fresh-master', endpoint],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if result.returncode != 0:
            raise HandoffError(f'全新主站握手失败：{result.stderr.strip() or result.stdout.strip()}')
        handshake_performed = True
        wait_end = time.monotonic() + 5.0
        while time.monotonic() < wait_end:
            headers = read_queue_headers(endpoint, dev_shm=dev_shm)
            if headers and headers['status']['published_index'] > 0:
                break
            time.sleep(0.05)
        else:
            raise HandoffError('主站握手后 5 秒内未收到状态帧，请检查 EtherCAT 链路与从站供电')

    if not valid_hand_background(cached_bg):
        raise ValueError(
            '当前开机窗口未找到停业务前的手部背景帧（744字节 command.bin）。'
            '若此前手动停过厂家服务而未保存 inspect 快照，请重启 Orin 后直接运行标定脚本。'
        )

    state = dict(
        boot_id=boot_id,
        endpoint=endpoint,
        shm_inode=list(headers['inode']),
        hand_background=str(cached_bg),
        takeover_performed=takeover_performed,
        handshake_performed=handshake_performed,
        updated_wall_ns=time.time_ns(),
    )
    write_json(boot_dir / 'state.json', state)
    return dict(
        endpoint=endpoint,
        hand_background=cached_bg,
        boot_id=boot_id,
        takeover_performed=takeover_performed,
        handshake_performed=handshake_performed,
    )
