"""Batch replay: delivered callback samples only, no motion across trial gaps."""
import csv
import json
from pathlib import Path
import numpy as np
from . import hardware_field as field
from .hardware_batch import load_batch, outcomes
from .vendor_identity import sha256
from .vendor_viewer import pose_frame, present_pose_source


class BatchReplaySource:
    presentation_heading = '整批现场记录 / 试次分别按接收时间回放 / 边界不插值'

    def __init__(self, folder, *, max_seconds_per_attempt=4.):
        if not np.isfinite(max_seconds_per_attempt) or max_seconds_per_attempt <= 0:
            raise ValueError('Positive replay time cap required')
        self.root, protocol = load_batch(folder); history, _ = outcomes(self.root, protocol)
        self.config = dict(model_id='casbot_20251015_dual_nominal_unverified',
            joint_names=protocol['joint_names'], stale_after_ns=protocol['profile']['max_feedback_age_ns'],
            viewer_collision_scene=protocol['profile']['scene'])
        self.scope = 'localhost_contract_mock' if protocol['connection_scope'] != 'hardware' else 'hardware_read_only'
        self.segments = []; self.skipped = []; self.fingerprints = {
            'protocol/manifest.json': sha256(self.root / 'protocol/manifest.json')}
        offset = 0; ordinal = 0
        for job in protocol['jobs']:
            for row in history[job['id']]:
                target = self.root / row['attempt'] / 'capture/execution'
                path = target / 'feedback.csv'
                if not path.is_file():
                    self.skipped.append(dict(attempt=row['attempt'], reason='no completed feedback file')); continue
                with path.open() as stream:
                    first = next(csv.DictReader(stream), None)
                if first is None:
                    self.skipped.append(dict(attempt=row['attempt'], reason='empty feedback file')); continue
                with path.open('rb') as stream:
                    size = path.stat().st_size; stream.seek(max(0, size-65536)); tail = stream.read()
                if not tail.endswith(b'\n'):
                    self.skipped.append(dict(attempt=row['attempt'], reason='incomplete trailing feedback row')); continue
                last_ns = int(tail.splitlines()[-1].split(b',')[0]); first_ns = int(first['receive_ns'])
                raw_duration = max(last_ns-first_ns, 1); duration = min(raw_duration, round(max_seconds_per_attempt*1e9))
                native = json.loads((target / 'result.json').read_text()) if (target / 'result.json').exists() else {}
                self.segments.append(dict(job=job, outcome=row, start=offset, end=offset+duration,
                    first_ns=first_ns, raw_duration=raw_duration, time_scale=raw_duration/duration,
                    clock=native.get('clock_id','callback_clock_unverified'), ordinal=ordinal))
                self.fingerprints[row['attempt']+'/manifest.json'] = row['manifest_sha256']
                offset += duration + 250000000; ordinal += 1
        if not self.segments: raise ValueError('No sealed delivered observations to replay')
        self.duration_ns = self.segments[-1]['end']; self.starts = [s['start'] for s in self.segments]
        self._cache_index = None; self._data = None; self._frame = None; self._segment = None
        self.error = None; self._stale = True

    def read(self, presentation_ns):
        index = max(0, int(np.searchsorted(self.starts, presentation_ns, side='right'))-1)
        segment = self.segments[index]; self._segment = segment
        if index != self._cache_index:
            path = self.root / segment['outcome']['attempt'] / 'capture/execution/feedback.csv'
            self._data = field.read_feedback(path, require_effort=False); self._cache_index = index
        at = min(max(presentation_ns-segment['start'], 0), segment['end']-segment['start'])
        target = segment['first_ns'] + min(round(at*segment['time_scale']), segment['raw_duration'])
        row = max(0, int(np.searchsorted(self._data['receive_ns'], target, side='right'))-1)
        self._frame = pose_frame(self.config, dict(q=self._data['q'][row], dq=self._data['dq'][row],
            receive_ns=int(self._data['receive_ns'][row]), receive_sequence=(segment['ordinal']<<32)+row), segment['clock'])
        gap = presentation_ns > segment['end']
        self.error = '试次间隔：不重建未记录的运动' if gap else None
        self._stale = gap or target-self._frame.receive_ns > self.config['stale_after_ns']
        return self._frame

    def is_stale(self): return self._stale

    def presentation_labels(self, presentation_ns, frame):
        segment = self._segment or self.segments[0]; job = segment['job']; outcome = segment['outcome']
        stage = {'direction':'单轴方向检查','frequency':'频率分档','static':'静态正反接近',
                 'moving_friction':'多姿态匀速摩擦','breakaway':'零PD起动力矩斜坡',
                 'inertia':'多姿态加速度激励','control_tuning':'五档PD参数选择','control_validation':'模型与增益四组对照'}[job['kind']]
        return [('当前阶段',stage), ('试次/尝试',job['id']+'/'+outcome['attempt'].split('/')[-1]),
                ('记录状态',outcome['status']), ('请求频率',str(job['hz'])+' Hz，非采样率'),
                ('时间倍率',f"{segment['time_scale']:.2f}x，角度不放大"),
                ('边界/采样时刻',self.error or '仅显示实际接收位置；采样时刻未知')]


def render_batch(folder, output, *, fps=12, max_seconds_per_attempt=4.):
    output = field.output_outside(output, folder)
    source = BatchReplaySource(folder, max_seconds_per_attempt=max_seconds_per_attempt)
    report = present_pose_source(source, output, fps=fps, source_sha256=source.fingerprints)
    for name, checksum in source.fingerprints.items():
        if sha256(source.root/name) != checksum: raise ValueError('Batch inputs changed during replay')
    report.update(segments=len(source.segments), skipped_attempts=source.skipped, device_writes=0,
                  inter_attempt_motion_reconstructed=False, device_sequence_known=False,
                  time_compressed=True, maximum_seconds_per_attempt=max_seconds_per_attempt)
    from .data import write_json
    write_json(output/'batch_replay.json', report); field.seal(output)
    return report
