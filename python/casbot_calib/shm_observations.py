"""SHM observation time is mapped master conversion time, never receive/sample time."""
import bisect
import json
from pathlib import Path
import numpy as np
from .fieldwork_streaming import csv_rows
from .shm_data import (
    COMMAND_RECORD_DTYPE,
    DIRECTION,
    FEEDBACK_RECORD_DTYPE,
    OFFSET,
)
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
    cmd_csv = root / 'commands.csv'
    cmd_bin = root / 'commands.bin'
    ev_csv = root / 'events.csv'
    if cmd_csv.is_file():
        first = next((int(row['send_mono_ns']) for row in csv_rows(cmd_csv) if row['phase'] == '3'), None)
    elif cmd_bin.is_file() and COMMAND_RECORD_DTYPE is not None and ev_csv.is_file():
        ev_rows = list(csv_rows(ev_csv))
        t3 = next((int(r['mono_ns']) for r in ev_rows if int(r['phase']) == 3), None)
        if t3 is None:
            first = None
        else:
            n_rec = cmd_bin.stat().st_size // COMMAND_RECORD_DTYPE.itemsize
            sends = (np.memmap(cmd_bin, dtype=COMMAND_RECORD_DTYPE, mode='r', shape=(n_rec,))['send_mono_ns']
                     if n_rec else np.empty(0, dtype=np.int64))
            idx = int(np.searchsorted(sends, t3, side='left'))
            first = int(sends[idx]) if idx < len(sends) else None
    else:
        raise ValueError('缺少命令记录文件')
    if first != origin:
        raise ValueError('首条执行命令与原生时间原点不一致')
    return origin


def _reconstruct_phases(receive_ns, events_path, commands_bin_path):
    n = len(receive_ns)
    if n == 0:
        return np.empty(0, dtype=np.int64)
    if not events_path.is_file():
        return np.full(n, -1, dtype=np.int64)
    ev_rows = [r for r in csv_rows(events_path) if r.get('mono_ns') and r.get('phase') is not None]
    if not ev_rows:
        return np.full(n, -1, dtype=np.int64)
    for r in ev_rows:
        if r.get('event') in ('passive_observe_begin', 'passive_ready'):
            raise ValueError('被动快照不能作为一致SHM观测')

    send_ns = np.empty(0, dtype=np.int64)
    if commands_bin_path.is_file() and COMMAND_RECORD_DTYPE is not None:
        cmd_count = commands_bin_path.stat().st_size // COMMAND_RECORD_DTYPE.itemsize
        if cmd_count > 0:
            send_ns = np.memmap(commands_bin_path, dtype=COMMAND_RECORD_DTYPE, mode='r', shape=(cmd_count,))['send_mono_ns'].astype(np.int64)

    init_phase = int(ev_rows[0]['phase'])
    phases = np.full(n, init_phase, dtype=np.int64)
    cur_phase = init_phase

    for row in ev_rows[1:]:
        ev_phase = int(row['phase'])
        if ev_phase == cur_phase:
            continue
        ev_mono = int(row['mono_ns'])
        if ev_phase == 5:
            start_idx = int(np.searchsorted(receive_ns, ev_mono, side='left'))
        else:
            cand = int(np.searchsorted(receive_ns, ev_mono, side='right')) - 1
            if cand >= 0:
                cand_recv = int(receive_ns[cand])
                s_lo = int(np.searchsorted(send_ns, cand_recv, side='right'))
                s_hi = int(np.searchsorted(send_ns, ev_mono, side='right'))
                has_intervening_send = s_hi > s_lo
                if not has_intervening_send and (ev_mono - cand_recv) < 500_000:
                    start_idx = cand
                else:
                    start_idx = cand + 1
            else:
                start_idx = 0
        phases[start_idx:] = ev_phase
        cur_phase = ev_phase
    return phases


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

    def _arrays_from_bin(self, fb_bin):
        size = fb_bin.stat().st_size
        if size % FEEDBACK_RECORD_DTYPE.itemsize != 0:
            raise ValueError('SHM二进制反馈记录尾部不完整')
        passive_bin = self.root / 'passive_commands.bin'
        if passive_bin.is_file() and passive_bin.stat().st_size > 0:
            raise ValueError('被动快照不能作为一致SHM观测')

        count = size // FEEDBACK_RECORD_DTYPE.itemsize
        if count == 0:
            return dict(
                index=np.empty(0, dtype=np.int64),
                receive_ns=np.empty(0, dtype=np.int64),
                observation_time_ns=np.empty(0, dtype=np.int64),
                source_timestamp_ms=np.empty(0, dtype=np.int64),
                clock_segment=np.empty(0, dtype=np.int64),
                frame_counter=np.empty(0, dtype=np.int64),
                phase=np.empty(0, dtype=np.int64),
                elapsed_s=np.empty(0, dtype=np.float64),
                time_ambiguous=np.empty(0, dtype=bool),
                q=np.empty((0, 14), dtype=np.float64),
                dq=np.empty((0, 14), dtype=np.float64),
                effort_raw=np.empty((0, 14), dtype=np.float64),
            )

        recs = np.asarray(np.memmap(fb_bin, dtype=FEEDBACK_RECORD_DTYPE, mode='r', shape=(count,)))
        receive_ns = recs['receive_mono_ns'].astype(np.int64)
        if count > 1 and np.any(np.diff(receive_ns) <= 0):
            raise ValueError('SHM接收时刻倒退或重复')
        if not self.pair_times:
            raise ValueError('反馈早于本机时钟配对')

        pair_times = np.asarray(self.pair_times, dtype=np.int64)
        pair_idx = np.searchsorted(pair_times, receive_ns, side='right') - 1
        if np.any(pair_idx < 0):
            raise ValueError('反馈早于本机时钟配对')

        pair_offsets = np.array([int(p['offset_ns']) for p in self.pairs], dtype=np.int64)
        pair_segments = np.array([int(p['clock_segment']) for p in self.pairs], dtype=np.int64)
        source_ms = recs['frame']['timestamp'].astype(np.int64)
        clock_segment = pair_segments[pair_idx]
        observation_time_ns = source_ms * np.int64(1_000_000) - pair_offsets[pair_idx]

        uncertain = np.zeros(count, dtype=bool)
        for a, b in self.ambiguous:
            uncertain |= (receive_ns > a) & (receive_ns <= b)

        backward = np.zeros(count, dtype=bool)
        for seg in np.unique(clock_segment):
            seg_idx = np.flatnonzero(clock_segment == seg)
            if len(seg_idx) > 1:
                backward[seg_idx[1:]] = observation_time_ns[seg_idx[1:]] < observation_time_ns[seg_idx[:-1]]

        phases = _reconstruct_phases(receive_ns, self.root / 'events.csv', self.root / 'commands.bin')

        arm_joints = recs['frame']['joints'][:, 12:26]
        direction = np.asarray(DIRECTION, dtype=np.float64)[None, :]
        offset = np.asarray(OFFSET, dtype=np.float64)[None, :]
        q = arm_joints['position'].astype(np.float64)
        q *= direction
        q *= 1e-6
        q += offset
        dq = arm_joints['velocity'].astype(np.float64)
        dq *= direction
        dq *= 1e-6
        effort_raw = arm_joints['torque'].astype(np.float64)
        effort_raw *= direction
        effort_raw *= 0.01
        if not (np.isfinite(q).all() and np.isfinite(dq).all() and np.isfinite(effort_raw).all()):
            raise ValueError('SHM反馈含非有限q/velocity/effort')

        return dict(
            index=np.arange(count, dtype=np.int64),
            receive_ns=receive_ns,
            observation_time_ns=observation_time_ns,
            source_timestamp_ms=source_ms,
            clock_segment=clock_segment,
            frame_counter=recs['frame']['frame_counter'].astype(np.int64),
            phase=phases,
            elapsed_s=(observation_time_ns - np.int64(self.origin)) * 1e-9,
            time_ambiguous=uncertain | backward,
            q=q,
            dq=dq,
            effort_raw=effort_raw,
        )

    def _rows_from_csv(self):
        previous = None
        last_source = {}
        for index, row in enumerate(csv_rows(self.root / 'feedback.csv')):
            receive = int(row['receive_mono_ns'])
            source = int(row['source_timestamp_ms'])
            at = int(row['observation_time_ns'])
            segment = int(row['clock_segment'])
            if int(row['passive']):
                raise ValueError('被动快照不能作为一致SHM观测')
            if previous is not None and receive <= previous:
                raise ValueError('SHM接收时刻倒退或重复')
            previous = receive
            pair = bisect.bisect_right(self.pair_times, receive) - 1
            if pair < 0:
                raise ValueError('反馈早于本机时钟配对')
            p = self.pairs[pair]
            if int(p['clock_segment']) != segment or source * 1_000_000 - int(p['offset_ns']) != at:
                raise ValueError('观测时间与原始毫秒stamp/本机配对不一致')
            uncertain = any(a < receive <= b for a, b in self.ambiguous)
            backward = segment in last_source and at < last_source[segment]
            last_source[segment] = at
            values = np.array([[float(row[prefix + str(j)]) for j in range(14)] for prefix in ('q', 'v', 'effort')])
            if not np.isfinite(values).all():
                raise ValueError('SHM反馈含非有限q/velocity/effort')
            yield dict(
                index=index,
                receive_ns=receive,
                observation_time_ns=at,
                source_timestamp_ms=source,
                clock_segment=segment,
                frame_counter=int(row['frame_counter']),
                phase=int(row.get('phase', -1)),
                elapsed_s=(at - self.origin) * 1e-9,
                time_ambiguous=uncertain or backward,
                q=values[0],
                dq=values[1],
                effort_raw=values[2],
            )

    def arrays(self):
        fb_bin = self.root / 'feedback.bin'
        if fb_bin.is_file() and FEEDBACK_RECORD_DTYPE is not None:
            return self._arrays_from_bin(fb_bin)
        rows = list(self._rows_from_csv())
        count = len(rows)
        if count == 0:
            return dict(
                index=np.empty(0, dtype=np.int64),
                receive_ns=np.empty(0, dtype=np.int64),
                observation_time_ns=np.empty(0, dtype=np.int64),
                source_timestamp_ms=np.empty(0, dtype=np.int64),
                clock_segment=np.empty(0, dtype=np.int64),
                frame_counter=np.empty(0, dtype=np.int64),
                phase=np.empty(0, dtype=np.int64),
                elapsed_s=np.empty(0, dtype=np.float64),
                time_ambiguous=np.empty(0, dtype=bool),
                q=np.empty((0, 14), dtype=np.float64),
                dq=np.empty((0, 14), dtype=np.float64),
                effort_raw=np.empty((0, 14), dtype=np.float64),
            )
        return dict(
            index=np.array([r['index'] for r in rows], dtype=np.int64),
            receive_ns=np.array([r['receive_ns'] for r in rows], dtype=np.int64),
            observation_time_ns=np.array([r['observation_time_ns'] for r in rows], dtype=np.int64),
            source_timestamp_ms=np.array([r['source_timestamp_ms'] for r in rows], dtype=np.int64),
            clock_segment=np.array([r['clock_segment'] for r in rows], dtype=np.int64),
            frame_counter=np.array([r['frame_counter'] for r in rows], dtype=np.int64),
            phase=np.array([r['phase'] for r in rows], dtype=np.int64),
            elapsed_s=np.array([r['elapsed_s'] for r in rows], dtype=np.float64),
            time_ambiguous=np.array([r['time_ambiguous'] for r in rows], dtype=bool),
            q=np.array([r['q'] for r in rows], dtype=np.float64),
            dq=np.array([r['dq'] for r in rows], dtype=np.float64),
            effort_raw=np.array([r['effort_raw'] for r in rows], dtype=np.float64),
        )

    def rows(self):
        fb_bin = self.root / 'feedback.bin'
        if not fb_bin.is_file() or FEEDBACK_RECORD_DTYPE is None:
            yield from self._rows_from_csv()
            return
        data = self._arrays_from_bin(fb_bin)
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
