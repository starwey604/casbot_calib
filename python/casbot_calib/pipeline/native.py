"""Direct native C++ shm_executor execution, start-state checks, and start-drift replanning."""
from contextlib import nullcontext
import json
from pathlib import Path
import shutil
import signal
import struct
import numpy as np
from ..data import write_json
from ..field_defaults import start_velocity_limit
from ..model_runtime import urdf_for
from ..shm_data import DIRECTION, NAMES, OFFSET, ROOT
from ..shm_fieldwork import (
    summarize_commands,
    summarize_feedback,
    summarize_limits,
    supervise,
    text_result,
    timing_summary,
)
from ..shm_reference import snapshot_positions

REPLAN_REASONS = (
    'Reference start differs from fresh measured position; no commands sent',
    'Start changed during preflight; no commands sent',
)


def status_has_no_fault(status_bin):
    path = Path(status_bin)
    if not path.is_file():
        return False
    raw = path.read_bytes()
    return len(raw) == 696 and not any(struct.unpack_from('<H', raw, 12 + j * 16 + 14)[0] for j in range(29))


def read_start_state(snapshot_root, robot_config):
    """Verify snapshot has finite 14-arm positions, |v| < start_velocity_limit, and zero drive errors."""
    root = Path(snapshot_root).resolve()
    status_path = root / 'native/status.bin' if (root / 'native/status.bin').is_file() else root / 'status.bin'
    if not status_path.is_file():
        raise ValueError(f'找不到状态快照：{root}')
    raw = status_path.read_bytes()
    if len(raw) != 696:
        raise ValueError(f'状态快照长度无效（{len(raw)} != 696）：{status_path}')
    q = np.asarray(snapshot_positions(status_path), dtype=float)
    velocity = np.array([
        DIRECTION[j] * struct.unpack_from('<i', raw, 12 + (j + 12) * 16 + 6)[0] * 1e-6
        for j in range(14)
    ])
    threshold = start_velocity_limit(robot_config)
    if not np.isfinite(q).all() or np.max(np.abs(velocity)) >= threshold:
        raise ValueError(f'需要完整且当前停稳的起点（|v|<{threshold:g}rad/s，实测最大 {float(np.max(np.abs(velocity))):.4f}rad/s）')
    faults = [
        (j, struct.unpack_from('<H', raw, 12 + j * 16 + 14)[0])
        for j in range(29)
        if struct.unpack_from('<H', raw, 12 + j * 16 + 14)[0]
    ]
    if faults:
        detail = ', '.join(f'ID{slot}=0x{err:x}' for slot, err in faults)
        raise ValueError(f'快照检测到驱动故障（{detail}）；不启动标定')
    return q


def _run_supervised(command, output_dir, *, timeout=60.0, peer=None):
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    previous_hup = signal.getsignal(signal.SIGHUP)
    try:
        return supervise(command, output_dir, timeout=timeout, peer=peer, defer_hup_restore=True)
    finally:
        signal.signal(signal.SIGHUP, previous_hup)


def native_snapshot(executor, endpoint, robot_config, output_dir, *, peer=None):
    output_dir = Path(output_dir).resolve()
    urdf = urdf_for(robot_config)
    command = [
        str(Path(executor).resolve()),
        '--snapshot',
        '--shm', endpoint,
        '--urdf', str(Path(urdf).resolve()),
        '--output', str(output_dir / 'native'),
    ]
    meta = _run_supervised(command, output_dir, timeout=15.0, peer=peer)
    native = text_result(output_dir / 'native/result.txt') or {}
    completed = meta['returncode'] == 0 and native.get('execution_completed') == '1'
    write_json(output_dir / 'result.json', dict(process=meta, native=native, execution_completed=completed))
    if not completed:
        reason = native.get('reason') or f'returncode={meta["returncode"]}'
        raise ValueError(f'SHM 只读快照失败（{reason}），记录：{output_dir}')
    return read_start_state(output_dir, robot_config)


def native_execute_once(
    executor,
    endpoint,
    robot_config,
    reference_path,
    hand_background,
    output_dir,
    *,
    duration_s,
    peer=None,
):
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    urdf = urdf_for(robot_config)
    bg_copy = output_dir / 'hand_background_source.bin'
    shutil.copyfile(hand_background, bg_copy)
    command = [
        str(Path(executor).resolve()),
        '--execute',
        '--shm', endpoint,
        '--urdf', str(Path(urdf).resolve()),
        '--reference', str(Path(reference_path).resolve()),
        '--hand-background', str(bg_copy),
        '--output', str(output_dir / 'native'),
    ]
    meta = _run_supervised(command, output_dir, timeout=max(30.0, float(duration_s) + 25.0), peer=peer)
    native_dir = output_dir / 'native'
    native = text_result(native_dir / 'result.txt')
    codec = dict(direction=DIRECTION, offset_rad=OFFSET, arm_names=NAMES)
    feedback = summarize_feedback(native_dir, codec)
    commands = summarize_commands(native_dir, codec)
    timing = timing_summary(native_dir)
    limits = summarize_limits(native_dir)
    completed = (
        bool(native and native.get('execution_completed') == '1' and meta.get('returncode', 0) == 0)
        and not meta.get('supervisor_timeout', False)
        and 'mock_process_failed' not in meta
    )
    code = meta['returncode'] if meta['returncode'] >= 0 else 128 - meta['returncode']
    result = dict(
        execution_completed=completed,
        returncode=code,
        process=meta,
        native=native,
        feedback=feedback,
        commands=commands,
        timing=timing,
        observed_limits=limits,
    )
    write_json(output_dir / 'result.json', result)
    return code, result


def open_mock_context(stage_dir, executor, robot_config, mock_config=None):
    from ..shm_mock import SHMMock
    source = Path(mock_config) if mock_config else ROOT / 'configs/shm/h1_mock.json'
    return SHMMock(Path(stage_dir) / 'mock', executor, source, urdf=urdf_for(robot_config))


def execute_with_start_replan(
    *,
    initial_plan_dir,
    initial_plan_meta,
    replan_fn,
    capture_root,
    executor,
    endpoint,
    robot_config,
    hand_background,
    peer=None,
    max_replans=3,
    execute_once_fn=native_execute_once,
    snapshot_fn=native_snapshot,
):
    """Execute planned reference; replan ONLY on zero-write no-fault preflight start drift."""
    capture_root = Path(capture_root).resolve()
    capture_root.mkdir(parents=True, exist_ok=False)
    effective_plan_dir = Path(initial_plan_dir).resolve()
    effective_plan_meta = initial_plan_meta
    attempts = []
    index = 0

    while True:
        attempt_dir = capture_root / 'attempts' / f'{index:04d}'
        attempt_dir.mkdir(parents=True, exist_ok=False)
        capture_dir = attempt_dir / 'capture'
        duration_s = effective_plan_meta['duration_ns'] * 1e-9
        code, run_result = execute_once_fn(
            executor,
            endpoint,
            robot_config,
            effective_plan_dir / 'reference.json',
            hand_background,
            capture_dir,
            duration_s=duration_s,
            peer=peer,
        )
        native = run_result.get('native') or {}
        reason = native.get('reason', '')
        zero_writes = int(native.get('commands_sent', 0)) == 0 and int(native.get('first_shm_write_mono_ns', 0)) == 0
        no_fault = status_has_no_fault(capture_dir / 'native/status.bin')
        eligible = code != 0 and zero_writes and no_fault and reason in REPLAN_REASONS and index < max_replans

        attempts.append(dict(
            index=index,
            capture=str(capture_dir.relative_to(capture_root)),
            returncode=code,
            reason='start_replan' if eligible else reason,
        ))
        summary = dict(
            execution_completed=(code == 0 and bool(run_result.get('execution_completed'))),
            returncode=code,
            attempts=attempts,
            effective_plan=None if effective_plan_dir == Path(initial_plan_dir).resolve() else str(effective_plan_dir.relative_to(capture_root)),
            effective_capture=str(capture_dir.relative_to(capture_root)),
        )
        write_json(capture_root / 'result.json', summary)
        if not eligible:
            return code, summary, effective_plan_dir, effective_plan_meta, capture_dir

        print('检测到使能前起点漂移（零命令写入、无驱动故障），重新读取起点并重规划...', flush=True)
        refresh_dir = attempt_dir / 'refresh'
        replanned_dir = attempt_dir / 'replanned'
        try:
            fresh_q = snapshot_fn(executor, endpoint, robot_config, refresh_dir, peer=peer)
            effective_plan_meta = replan_fn(fresh_q, replanned_dir)
            effective_plan_dir = replanned_dir
        except (ValueError, OSError) as error:
            summary.update(execution_completed=False, returncode=2, replan_error=str(error))
            write_json(capture_root / 'result.json', summary)
            return 2, summary, effective_plan_dir, effective_plan_meta, capture_dir
        index += 1


def maybe_mock_context(mock, stage_dir, executor, robot_config, mock_config=None):
    return open_mock_context(stage_dir, executor, robot_config, mock_config) if mock else nullcontext(None)
