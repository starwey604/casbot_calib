"""Calibration session directories, robot ID persistence, and stage checkpoints."""
import json
import os
from pathlib import Path
import re
import time
from ..data import write_json
from ..run_marker import run_marker
from ..shm_data import ROOT
from .takeover import current_boot_id

DEFAULT_OUTPUT_ROOT = ROOT / 'artifacts/calib'
SAFE_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$')


def resolve_robot_id(state_root, explicit=None):
    state_root = Path(state_root).resolve()
    id_file = state_root / 'robot_id.txt'
    if explicit is not None:
        value = explicit.strip()
        if not SAFE_ID.match(value):
            raise ValueError(f'无效的机器编号（仅限字母、数字、点、下划线、连字符）：{explicit}')
        state_root.mkdir(parents=True, exist_ok=True)
        id_file.write_text(value + '\n', encoding='utf-8')
        return value
    if id_file.is_file():
        saved = id_file.read_text(encoding='utf-8').strip()
        if SAFE_ID.match(saved):
            return saved
    return 'dvt2'


def _update_latest_link(output_root, target):
    link = Path(output_root) / 'latest'
    tmp = Path(output_root) / f'.latest.{os.getpid()}.tmp'
    try:
        if tmp.is_symlink() or tmp.exists():
            tmp.unlink()
        rel = os.path.relpath(target, output_root)
        tmp.symlink_to(rel)
        tmp.replace(link)
    except OSError:
        pass


class Session:
    def __init__(self, root, metadata, *, output_root=DEFAULT_OUTPUT_ROOT):
        self.root = Path(root).resolve()
        self.output_root = Path(output_root).resolve()
        self.metadata = metadata

    @property
    def robot_id(self):
        return self.metadata['robot_id']

    @property
    def state_root(self):
        return self.output_root / 'state'

    @classmethod
    def create(cls, *, output_root=DEFAULT_OUTPUT_ROOT, robot_id=None, model_context, stages_requested, plan_only=False):
        output_root = Path(output_root).resolve()
        state_root = output_root / 'state'
        resolved_id = resolve_robot_id(state_root, robot_id)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        parent = output_root / resolved_id
        parent.mkdir(parents=True, exist_ok=True)
        candidate = parent / stamp
        suffix = 1
        while candidate.exists():
            suffix += 1
            candidate = parent / f'{stamp}_{suffix:02d}'
        candidate.mkdir(parents=True, exist_ok=False)

        metadata = dict(
            robot_id=resolved_id,
            created_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            boot_id=current_boot_id(),
            model_context=model_context,
            plan_only=bool(plan_only),
            stages_requested=list(stages_requested),
            stages={},
            **run_marker(),
        )
        session = cls(candidate, metadata, output_root=output_root)
        session.save()
        _update_latest_link(output_root, candidate)
        return session

    @classmethod
    def resume(cls, *, output_root=DEFAULT_OUTPUT_ROOT, target=None, model_context=None):
        output_root = Path(output_root).resolve()
        if target is None or str(target) == 'latest':
            latest = output_root / 'latest'
            if not latest.exists():
                raise ValueError(f'没有可续跑的最近会话：{latest}')
            session_dir = latest.resolve()
        else:
            session_dir = Path(target).resolve()
        meta_path = session_dir / 'session.json'
        if not meta_path.is_file():
            raise ValueError(f'无效的标定会话目录（缺少 session.json）：{session_dir}')
        metadata = json.loads(meta_path.read_text(encoding='utf-8'))
        if model_context is not None and metadata.get('model_context') != model_context:
            raise ValueError('待续跑会话的模型版本与当前代码不一致，请新建会话')
        if metadata.get('plan_only'):
            raise ValueError('仅规划（--plan-only）会话不含实测数据，请直接新建标定会话')
        return cls(session_dir, metadata, output_root=output_root)

    def save(self):
        write_json(self.root / 'session.json', self.metadata)

    def stage_completed(self, name):
        return self.metadata.get('stages', {}).get(name, {}).get('status') == 'completed'

    def stage_record(self, name):
        return self.metadata.get('stages', {}).get(name)

    def prepare_stage_dir(self, name):
        stage_dir = self.root / name
        if stage_dir.exists():
            archived = self.root / f'{name}.previous_{time.monotonic_ns()}'
            stage_dir.rename(archived)
        stage_dir.mkdir(parents=True, exist_ok=False)
        return stage_dir

    def mark_stage(self, name, status, **info):
        entry = dict(status=status, updated_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'), **info)
        self.metadata.setdefault('stages', {})[name] = entry
        self.save()
        return entry
