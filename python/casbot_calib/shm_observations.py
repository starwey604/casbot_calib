"""SHM observation time is mapped master conversion time, never receive/sample time."""
import csv
import json
from pathlib import Path
import numpy as np
from . import _core
from .fieldwork_streaming import csv_rows
from .shm_fieldwork import text_result


def native_directory(root):
    root = Path(root).resolve()
    result = root / 'result.json'
    if result.exists():
        value = json.loads(result.read_text(encoding='utf-8'))
        if isinstance(value.get('effective_capture'), str):
            child = (root / value['effective_capture']).resolve()
            if not child.is_relative_to(root):
                raise ValueError('采集路径越界')
            root = child
    return root / 'native' if (root / 'native').is_dir() else root


def command_origin(root):
    root = native_directory(root)
    result = text_result(root / 'result.txt')
    if not result:
        raise ValueError('缺少原生封存结果')
    origin = int(result.get('first_command_mono_ns', 0))
    if origin <= 0:
        raise ValueError('此记录没有首条执行命令；只读观测不能用于相对执行窗A分析')
    path = root / 'commands.csv.zst'
    if not path.is_file():
        raise ValueError('缺少命令记录文件')
    command = _core.read_commands_csv_zst(path, timing_only=True)
    if command['truncated_tail']:
        raise ValueError('SHM命令记录尾部不完整')
    stream = np.flatnonzero(command['phase'] == 3)
    if not len(stream) or command['send_mono_ns'][stream[0]] != origin:
        raise ValueError('首条执行命令与原生时间原点不一致')
    return origin


def _has_passive_event(path):
    if not path.is_file():
        return False
    with path.open(newline='') as stream:
        return any(row['event'] in ('passive_observe_begin', 'passive_ready')
                   for row in csv.DictReader(stream))


class ObservationReader:
    def __init__(self, root):
        self.root = native_directory(root)
        self.origin = command_origin(self.root)
        self.pairs = list(csv_rows(self.root / 'clock_pairs.csv'))
        self.pair_times = [int(p['mono_after_ns']) for p in self.pairs]
        if any(b <= a for a, b in zip(self.pair_times, self.pair_times[1:])):
            raise ValueError('时钟配对顺序无效')
        self.ambiguous = []
        for a, b in zip(self.pairs, self.pairs[1:]):
            if int(b['jump_detected']):
                self.ambiguous.append((int(a['mono_after_ns']), int(b['mono_after_ns'])))

    def arrays(self):
        records = _core.read_feedback_csv_zst(self.root / 'feedback.csv.zst')
        if records['truncated_tail']:
            raise ValueError('SHM反馈记录尾部不完整')
        passive = self.root / 'passive_commands.csv.zst'
        events = self.root / 'events.csv'
        if (records['passive'].any()
                or (passive.is_file() and len(_core.read_commands_csv_zst(passive, timing_only=True)['send_mono_ns']))
                or _has_passive_event(events)):
            raise ValueError('被动快照不能作为一致SHM观测')

        receive = records['receive_mono_ns']
        observation = records['observation_time_ns']
        source = records['source_timestamp_ms']
        segments = records['clock_segment']
        if np.any(receive[1:] <= receive[:-1]):
            raise ValueError('SHM接收时刻倒退或重复')
        pair = np.searchsorted(self.pair_times, receive, side='right') - 1
        if np.any(pair < 0):
            raise ValueError('反馈早于本机时钟配对')
        offsets = np.array([int(p['offset_ns']) for p in self.pairs], dtype=np.int64)
        pair_segments = np.array([int(p['clock_segment']) for p in self.pairs], dtype=np.int64)
        if (np.any(source > np.iinfo(np.int64).max // 1_000_000)
                or np.any(segments != pair_segments[pair])
                or np.any(source.astype(np.int64) * 1_000_000 - offsets[pair] != observation)):
            raise ValueError('观测时间与原始毫秒stamp/本机配对不一致')
        if not all(np.isfinite(records[key]).all() for key in ('q', 'dq', 'effort_raw')):
            raise ValueError('SHM反馈含非有限q/velocity/effort')

        uncertain = np.zeros(len(receive), dtype=bool)
        for a, b in self.ambiguous:
            uncertain |= (receive > a) & (receive <= b)
        for segment in np.unique(segments):
            indices = np.flatnonzero(segments == segment)
            uncertain[indices[1:]] |= observation[indices[1:]] < observation[indices[:-1]]
        return dict(
            index=np.arange(len(receive), dtype=np.int64),
            receive_ns=receive,
            observation_time_ns=observation,
            source_timestamp_ms=source,
            clock_segment=segments,
            frame_counter=records['frame_counter'],
            phase=records['phase'],
            elapsed_s=(observation - np.int64(self.origin)) * 1e-9,
            time_ambiguous=uncertain,
            q=records['q'],
            dq=records['dq'],
            effort_raw=records['effort_raw'],
        )

    def rows(self):
        data = self.arrays()
        for i in range(len(data['index'])):
            yield dict(
                index=int(data['index'][i]),
                receive_ns=int(data['receive_ns'][i]),
                observation_time_ns=int(data['observation_time_ns'][i]),
                source_timestamp_ms=int(data['source_timestamp_ms'][i]),
                clock_segment=int(data['clock_segment'][i]),
                frame_counter=int(data['frame_counter'][i]),
                phase=int(data['phase'][i]),
                elapsed_s=float(data['elapsed_s'][i]),
                time_ambiguous=bool(data['time_ambiguous'][i]),
                q=data['q'][i],
                dq=data['dq'][i],
                effort_raw=data['effort_raw'][i],
            )

    def window(self, interval, segment=None):
        lo, hi = interval
        if not np.isfinite([lo, hi]).all() or lo >= hi:
            raise ValueError('窗口须START<END')
        data = self.arrays()
        elapsed = data['elapsed_s']
        clock_seg = data['clock_segment']
        ambig = data['time_ambiguous']

        mask = (elapsed >= lo) & (elapsed < hi)
        if segment is not None:
            mask &= (clock_seg == segment)
        sel_idx = np.flatnonzero(mask)
        if np.any(ambig[sel_idx]):
            raise ValueError('窗口涉及墙钟跳变的未定位区间或倒退时间；请选择稳定分段内的窗口')
        unique_segs = np.unique(clock_seg[sel_idx])
        if len(unique_segs) != 1:
            raise ValueError('窗口没有观测或跨越时钟分段；可显式指定--clock-segment')
        chosen = int(unique_segs[0])
        seg_mask = clock_seg == chosen
        raw_rows = int(np.sum(seg_mask))
        valid_idx = np.flatnonzero(seg_mask & (~ambig))
        first_idx = int(valid_idx[0])
        last_idx = int(valid_idx[-1])
        if lo < elapsed[first_idx] or hi > elapsed[last_idx]:
            raise ValueError('窗口超出该时钟段的有效反馈覆盖')
        if len(sel_idx) > 1 and np.any(np.diff(sel_idx) != 1):
            raise ValueError('窗口观测不连续，不跨时钟缺口拼接')
        if sel_idx[0] != first_idx:
            sel_idx = np.r_[first_idx, sel_idx]
        if sel_idx[-1] != last_idx:
            sel_idx = np.r_[sel_idx, last_idx]
        ns = data['observation_time_ns'][sel_idx]
        obs = dict(
            receive_ns=data['receive_ns'][sel_idx],
            observation_time_ns=ns,
            q=data['q'][sel_idx],
            dq=data['dq'][sel_idx],
            effort_raw=data['effort_raw'][sel_idx],
        )
        return obs, (ns - np.int64(self.origin)) * 1e-9, dict(
            clock_segment=chosen,
            raw_rows=raw_rows,
            source_timestamp_quantum_ns=1_000_000,
            time_basis='mapped master conversion time; not motor acquisition',
            raw_receive_times_preserved=True,
        )
