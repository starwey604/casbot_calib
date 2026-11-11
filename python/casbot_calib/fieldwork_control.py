"""F2/F3 commissioning session: one operation owner, immutable evidence, no auto-recovery."""
from pathlib import Path
import time
import uuid

import numpy as np
import yaml

from . import hardware_field as field
from .hardware_batch import atomic_json, batch_lock, event, events, read
from .run_marker import run_marker
from .vendor_identity import sha256

SITE_ITEMS = ('emergency_stop_and_support', 'limits_geometry_gravity_load', 'mode_and_non_target_hold')


def implementation():
    # Use the established H2 execution/code binding; only add the new owners.
    from .hardware_batch import implementation as shared_implementation
    result = shared_implementation()
    directory = Path(__file__).parent
    result['sources'].update({name: sha256(directory/name) for name in
        ('fieldwork_control.py', 'fieldwork_worker.py', 'fieldwork_gui.py')})
    return result


def create_session(profile, output, domain_id, scope, conditions=None):
    config = field.load_profile(profile)
    if type(domain_id) is not int or not 0 <= domain_id <= 232:
        raise ValueError('必须显式输入现场确认的ROS域号')
    connection = config.get('connection')
    actual_scope = 'localhost_contract_mock' if connection else 'hardware_position_stream'
    if scope != actual_scope or (connection and connection['domain_id'] != domain_id):
        raise ValueError('域号/scope与profile连接不一致；不自动切换目标')
    root = Path(output).resolve()
    root.mkdir(parents=True, exist_ok=False)
    protocol = root/'protocol'; protocol.mkdir()
    site = yaml.safe_load(Path(conditions).read_text()) if conditions else None
    if site is not None:
        if not isinstance(site, dict):
            raise ValueError('现场条件须为具名条目的YAML映射')
        # Preserve evidence alongside the declaration, not just boolean gates.
        for item in SITE_ITEMS:
            value = site.get(item, {})
            if not isinstance(value, dict) or value.get('status') != 'site_confirmed' or not value.get('conclusion') or not value.get('evidence'):
                raise ValueError('现场运动前提须有结论和证据：'+item)
            if item == 'mode_and_non_target_hold' and value.get('non_target_behavior') != 'hold_prior_target_output':
                raise ValueError('现有14轴路径仅支持已确认的非目标轴保持；释放、缺省0或未知均不放行')
            if not isinstance(value['evidence'], list):
                raise ValueError('evidence须为证据文件路径列表')
            copied = []
            for index, evidence in enumerate(value['evidence']):
                source = (Path(conditions).resolve().parent/evidence).resolve()
                destination = protocol/'site_evidence'/item/(str(index)+'_'+source.name)
                destination.parent.mkdir(parents=True, exist_ok=True)
                import shutil
                shutil.copyfile(source, destination)
                copied.append(str(destination.relative_to(protocol)))
            value['copied_evidence'] = copied
        atomic_json(protocol/'site_conditions.json', site)
    scene = config.pop('scene')
    config['collision_scene'] = 'scene.yaml'
    (protocol/'scene.yaml').write_text(yaml.safe_dump(scene), encoding='utf-8')
    (protocol/'profile.yaml').write_text(yaml.safe_dump(config), encoding='utf-8')
    engine, source = field.model_for(config)
    metadata = dict(schema_version='casbot_fieldwork_control_v1', **run_marker(), domain_id=domain_id,
        scope=scope, joint_names=[j.name for j in engine.model.joints], source_model_sha256=source['sha256'],
        model_context=config['model_context'], site_conditions_recorded=site is not None,
        implementation=implementation(),
        motion_conditions_scope='isolated_mock_only' if connection else 'operator_site_evidence_pending_physical_acceptance',
        physical_motion_qualified=False, hardware_calibration_accepted=False)
    atomic_json(protocol/'session.json', metadata); field.seal(protocol)
    event(root, 'session_created')
    return Session(root).status()


class Session:
    def __init__(self, root):
        self.root = Path(root).resolve()
        field.check_seal(self.root/'protocol')
        self.metadata = read(self.root/'protocol/session.json')
        if self.metadata.get('schema_version') != 'casbot_fieldwork_control_v1':
            raise ValueError('不是现场GUI控制会话')
        self.profile = self.root/'protocol/profile.yaml'
        self.config = field.load_profile(self.profile)
        self.names = self.metadata['joint_names']
        self.journal = events(self.root)
        self._implementation_matches = None

    def implementation_matches(self):
        if self._implementation_matches is None:
            self._implementation_matches = self.metadata.get('implementation') == implementation()
        return self._implementation_matches

    def blocked_reason(self):
        self.journal = events(self.root)
        if any(e['event'] in ('terminal_fault', 'physical_review_rejected') for e in self.journal):
            return '本会话已终止：不得清故障或resume；现场处置后新建会话'
        starts = [e for e in self.journal if e['event'] == 'execution_started']
        ends = {e['attempt'] for e in self.journal if e['event'] == 'execution_finished'}
        if any(e['attempt'] not in ends for e in starts):
            return '存在未封存的执行，状态不明；本会话禁止继续运动'
        if not self.implementation_matches():
            return '实现/SDK/原生扩展与封存条件不同，或旧会话缺少该绑定；新建会话后再运动'
        return None

    def assert_motion(self):
        reason = self.blocked_reason()
        if reason:
            raise ValueError(reason)
        if self.metadata['scope'] == 'hardware_position_stream' and not self.metadata['site_conditions_recorded']:
            raise ValueError('先记录现场急停/支撑、真实限制/安装、模式与非目标轴保持证据，再新建运动会话')

    def reviews(self, *, verify=False):
        self.journal = events(self.root)
        latest = {}
        for entry in self.journal:
            if entry['event'] == 'direction_reviewed':
                review_root = self.root/entry['review']
                if verify:
                    field.check_seal(review_root)
                    if sha256(review_root/'manifest.json') != entry['manifest_sha256']:
                        raise ValueError('方向review绑定已改变')
                review = read(review_root/'review.json')
                if verify:
                    attempt = self.root/review['attempt']
                    field.check_seal(attempt/'input')
                    field.check_seal(attempt/'capture'); field.check_seal(attempt/'analysis'); field.check_seal(attempt/'plan')
                    if sha256(attempt/'capture/manifest.json') != review['capture_manifest_sha256']:
                        raise ValueError('人工结论对应的采集已改变')
                    request = read(attempt/'input/request.json')
                    if (request['kind'] != 'jog' or request['joints'] != [review['joint']] or
                            request['protocol_sha256'] != sha256(self.root/'protocol/manifest.json') or
                            request['plan_manifest_sha256'] != sha256(attempt/'plan/manifest.json') or
                            not read(attempt/'analysis/report.json')['passed'] or not read(attempt/'capture/result.json')['passed']):
                        raise ValueError('人工方向证据的关节、模型或执行/分析绑定不符')
                latest[review['joint']] = review
        return latest

    def require_all_directions(self):
        self.assert_motion()
        reviews = self.reviews(verify=True)
        if set(reviews) != set(self.names) or not all(r['same_axis'] and r['same_direction'] for r in reviews.values()):
            raise ValueError('短轨迹须先完成GUI全部14轴的有效实物映射与方向确认')
        return reviews

    def status(self):
        reviews = self.reviews()
        accepted = [name for name, row in reviews.items() if row['same_axis'] and row['same_direction']]
        snapshot = None
        for entry in self.journal:
            if entry['event'] == 'snapshot_finished': snapshot = entry['snapshot']
            elif entry['event'] in ('execution_started', 'controlled_read_interrupted'): snapshot = None
        attempts = [e for e in self.journal if e['event'] == 'plan_finished']
        reason = self.blocked_reason()
        return dict(**self.metadata, accepted_joints=accepted, reviews=reviews, blocked_reason=reason,
            short_enabled=not reason and len(accepted) == 14,
            snapshot=snapshot, attempts=attempts,
            site_conditions_ready=self.metadata['scope'] == 'localhost_contract_mock' or self.metadata['site_conditions_recorded'])

    def snapshot(self, cancel_check=None):
        relative = 'snapshots/'+uuid.uuid4().hex[:12]
        result = field.run_executor(self.config, self.root/relative, self.metadata['domain_id'],
                                    snapshot=True, cancel_check=cancel_check)
        if not result['passed']:
            if result.get('supervisor_interruption') in ('operator_cancel', 'frontend_closed') and result.get('fault') == 'interrupted':
                event(self.root, 'controlled_read_interrupted', snapshot=relative)
                raise ValueError('只读起姿已受控中断，请重新读取；不沿用先前快照')
            event(self.root, 'terminal_fault', reason='snapshot_failed', snapshot=relative)
            raise RuntimeError('只读起姿采集失败，保留记录并终止会话')
        event(self.root, 'snapshot_finished', snapshot=relative)
        import csv
        with (self.root/relative/'execution/snapshot.csv').open() as stream:
            received = int(next(csv.DictReader(stream))['receive_ns'])
        return dict(snapshot=relative, q_rad=field._read_snapshot(self.root/relative).tolist(), receive_ns=received)

    def plan(self, kind, snapshot, joints, amplitude_deg, transition_s=2.):
        self.assert_motion()
        if kind == 'short':
            self.require_all_directions()
        elif kind != 'jog' or len(joints) != 1:
            raise ValueError('方向检查一次只能选择一个关节')
        if not joints or len(set(joints)) != len(joints) or not set(joints) <= set(self.names):
            raise ValueError('请选择具名双臂关节')
        if not np.isfinite(amplitude_deg) or not 0 < amplitude_deg <= 1:
            raise ValueError('本工具受限幅度必须在(0,1]度；现场安全幅度需单独确认')
        snapshot_path = (self.root/snapshot).resolve()
        if snapshot_path.parent != self.root/'snapshots':
            raise ValueError('起姿必须来自本会话的只读快照')
        if self.status()['snapshot'] != snapshot:
            raise ValueError('请使用本会话最新且未用于执行的实际起姿')
        if any(e['event'] == 'execution_started' and e.get('snapshot') == snapshot for e in self.journal):
            raise ValueError('该快照已用于执行，请重新读取实际起姿')
        attempt_id = uuid.uuid4().hex[:12]
        relative = 'attempts/'+attempt_id
        root = self.root/relative; root.mkdir(parents=True)
        if kind == 'jog':
            report = field.prepare_jog(self.profile, snapshot_path, root/'plan', joints[0], amplitude_deg)
        else:
            if not np.isfinite(transition_s) or not 1 <= transition_s <= 8:
                raise ValueError('短轨迹单段时长须在1至8秒之间')
            from .staged_motion import Motion
            from .static_field import _move
            start = field._read_snapshot(snapshot_path)
            motion = Motion('fieldwork_short_return', 'position', start, dict(joints=joints, amplitude_deg=amplitude_deg))
            motion.add(self.config['initial_hold_s'], start, label='measured_start_hold')
            for sign in (1, -1, 0):
                q = start.copy()
                for name in joints:
                    q[self.names.index(name)] += sign*np.deg2rad(amplitude_deg)
                _move(motion, q, transition_s, dict(speed_rad_s=.03, acceleration_rad_s2=.08, jerk_rad_s3=.5), 'short_return')
            motion.add(self.config['final_hold_s'], start, label='measured_start_final_hold')
            (root/'plan').mkdir()
            report = field.finish_plan(root/'plan', self.config, motion, [], field.snapshot_binding(snapshot_path, self.config))
        directions = {name: row['capture_manifest_sha256'] for name, row in self.reviews().items()} if kind == 'short' else {}
        atomic_json(root/'input/request.json', dict(kind=kind, snapshot=snapshot, joints=joints,
            amplitude_deg=amplitude_deg, direction_bindings=directions, protocol_sha256=sha256(self.root/'protocol/manifest.json'),
            plan_manifest_sha256=sha256(root/'plan/manifest.json')))
        field.seal(root/'input')
        event(self.root, 'plan_finished', attempt=relative, trial_kind=kind, joints=joints)
        return dict(attempt=relative, duration_s=report['duration_s'], joints=joints,
                    q_start_rad=report['observed_start_rad'], kind=kind)

    def execute(self, relative, cancel_check=None):
        self.assert_motion()
        if cancel_check is None:
            raise ValueError('GUI执行须提供前端存活监督，不能从后台绕过')
        attempt = self.attempt(relative)
        field.check_seal(attempt/'input')
        request = read(attempt/'input/request.json')
        if request['plan_manifest_sha256'] != sha256(attempt/'plan/manifest.json'):
            raise ValueError('计划与操作请求绑定不符')
        if request['protocol_sha256'] != sha256(self.root/'protocol/manifest.json'):
            raise ValueError('计划与会话绑定不符')
        if request['kind'] == 'short':
            reviews = self.require_all_directions()
            if request['direction_bindings'] != {name: r['capture_manifest_sha256'] for name, r in reviews.items()}:
                raise ValueError('方向证据已更新，须重新规划短轨迹')
        if (attempt/'capture').exists() or (attempt/'outcome.json').exists():
            raise ValueError('每份计划只执行一次；重新读起姿并生成新计划')
        event(self.root, 'execution_started', attempt=relative, snapshot=request['snapshot'])
        try:
            result = field.execute_field(attempt/'plan', attempt/'capture', self.metadata['domain_id'], cancel_check=cancel_check)
            interruption = result.get('supervisor_interruption')
            controlled = interruption in ('operator_cancel', 'frontend_closed') and result.get('fault') == 'interrupted'
            if not result['passed']:
                atomic_json(attempt/'outcome.json', dict(passed=False, controlled_interruption=controlled, result=result))
                event(self.root, 'execution_finished', attempt=relative, passed=False)
                if not controlled:
                    event(self.root, 'terminal_fault', reason=result.get('fault') or interruption or result.get('reason'),
                          clear_fault=dict(implemented=False, performed=False))
                return dict(passed=False, controlled_interruption=controlled, reason=interruption or result.get('reason'))
            report = field.analyse_field(attempt/'plan', attempt/'capture', attempt/'analysis')
            atomic_json(attempt/'outcome.json', dict(passed=report['passed'], result=result, analysis_passed=report['passed']))
            event(self.root, 'execution_finished', attempt=relative, passed=report['passed'])
            if not report['passed']:
                event(self.root, 'terminal_fault', reason='observed_motion_rejected', clear_fault=dict(implemented=False, performed=False))
            return dict(passed=report['passed'], attempt=relative, analysis=report)
        except BaseException:
            event(self.root, 'terminal_fault', reason='unsealed_execution', clear_fault=dict(implemented=False, performed=False))
            raise

    def attempt(self, relative):
        path = (self.root/relative).resolve()
        if path.parent != self.root/'attempts' or not path.is_dir():
            raise ValueError('请选择本会话中的计划')
        return path

    def review(self, relative, observer, physical_label, same_axis, same_direction, note=''):
        self.assert_motion()
        attempt = self.attempt(relative)
        field.check_seal(attempt/'input')
        request = read(attempt/'input/request.json')
        if (request['kind'] != 'jog' or not read(attempt/'outcome.json')['passed'] or
                not read(attempt/'analysis/report.json')['passed'] or not read(attempt/'capture/result.json')['passed']):
            raise ValueError('人工方向结论只能绑定成功执行并分析通过的单轴试验')
        if not observer.strip() or not physical_label.strip() or type(same_axis) is not bool or type(same_direction) is not bool:
            raise ValueError('请填写观察者、实物关节，并明确映射和方向结论')
        field.check_seal(attempt/'capture'); field.check_seal(attempt/'analysis'); field.check_seal(attempt/'plan')
        relative_review = 'reviews/'+uuid.uuid4().hex[:12]
        review_root = self.root/relative_review; review_root.mkdir(parents=True)
        row = dict(joint=request['joints'][0], observer=observer, physical_label=physical_label,
            same_axis=same_axis, same_direction=same_direction, note=note, interface='imgui_explicit_review',
            attempt=relative, capture_manifest_sha256=sha256(attempt/'capture/manifest.json'), **run_marker())
        atomic_json(review_root/'review.json', row); field.seal(review_root)
        event(self.root, 'direction_reviewed', review=relative_review, manifest_sha256=sha256(review_root/'manifest.json'))
        if not same_axis or not same_direction:
            event(self.root, 'physical_review_rejected', joint=row['joint'])
        return row

    def export_report(self):
        """Summarize frozen evidence; also available after a terminal fault."""
        reviews = self.reviews(verify=True)
        status = self.status()
        rows = []
        for entry in status['attempts']:
            relative = entry['attempt']
            root = self.root/relative
            request = read(root/'input/request.json')
            outcome = read(root/'outcome.json') if (root/'outcome.json').exists() else None
            result = outcome.get('result', {}) if outcome else {}
            state = ('通过' if outcome['passed'] else '受控中断' if outcome.get('controlled_interruption') else '失败') if outcome else '待执行或未封存'
            rows.append(dict(attempt=relative, kind=request['kind'], joints=request['joints'], state=state,
                reason=result.get('supervisor_interruption') or result.get('fault') or result.get('reason'),
                analysis=str(root/'analysis/report.json') if (root/'analysis/report.json').exists() else None))
        for entry in self.journal:
            if entry['event'] == 'planning_rejected':
                rows.append(dict(attempt='events/'+f"{entry['sequence']:08}.json", kind=entry['inputs'].get('kind', 'plan'),
                    joints=entry['inputs'].get('joints', []), state='参考拒绝/未下发', reason=entry['reason'], analysis=None))
        marker = run_marker()
        report = dict(schema_version='casbot_fieldwork_gui_report_v1', source=str(self.root), **marker,
            session=status, attempts=rows, reviews=reviews, physical_direction_software_verified=False,
            physical_stop_verified=False, hardware_calibration_accepted=False)
        relative = 'reports/'+marker['run_id']
        directory = self.root/relative; directory.mkdir(parents=True)
        atomic_json(directory/'report.json', report)
        def text(value): return str(value).replace('|', '\\|').replace('\n', ' ')
        lines = ['# 现场GUI联调报告', '', f"域号：{status['domain_id']}；范围：{status['scope']}",
            f"方向确认：{len(status['accepted_joints'])}/14；会话：{status['blocked_reason'] or '无终止故障记录'}", '',
            '## 具名人工结论', '', '| SDK关节 | 实物名称 | 观察者 | 映射/方向一致 |', '| --- | --- | --- | --- |']
        for name in self.names:
            row = reviews.get(name, {})
            lines.append('| '+' | '.join(text(value) for value in (name, row.get('physical_label', '未确认'),
                row.get('observer', ''), str(row.get('same_axis', '未知'))+'/'+str(row.get('same_direction', '未知'))))+' |')
        lines += ['', '## 尝试与证据', '', '| 尝试 | 类型/关节 | 状态 | 原因 |', '| --- | --- | --- | --- |']
        for row in rows:
            lines.append('| '+' | '.join(text(value) for value in (row['attempt'], row['kind']+': '+', '.join(row['joints']),
                row['state'], row['reason'] or ''))+' |')
        lines += ['', '逐项分析和原始CDR位于各attempt的analysis、capture中；人工结论位于reviews。',
            '方向由观察者声明；Mock记录仅验证软件链路。当前报告不授予物理停止或正式标定资格。', '']
        (directory/'report.md').write_text('\n'.join(lines), encoding='utf-8')
        return dict(report_dir=relative)


def heartbeat_check(path, *, timeout_s=2., expected_owner=None):
    """Frontend-owned file, checked in the existing SDK process supervisor."""
    from .fieldwork import clock_id
    expected = clock_id()
    def check():
        try:
            record = read(path)
            if expected_owner is not None and record.get('owner_id') != expected_owner:
                return 'frontend_owner_changed'
            if record['clock_id'] != expected:
                return 'frontend_clock_mismatch'
            if record.get('cancel'):
                return record['cancel']
            age = time.monotonic_ns()-record['monotonic_ns']
            if not 0 <= age <= timeout_s*1e9:
                return 'frontend_heartbeat_timeout'
        except (OSError, ValueError, KeyError):
            return 'frontend_heartbeat_unavailable'
        return None
    return check


def operate(root, action, values, cancel_check=None):
    """Single entry point shared by GUI worker and tests; never replays commands."""
    with batch_lock(root):
        session = Session(root)
        if action == 'snapshot': return session.snapshot(cancel_check)
        if action == 'plan':
            try: return session.plan(**values)
            except (ValueError, RuntimeError) as exc:
                event(root, 'planning_rejected', inputs=values, reason=str(exc))
                raise
        if action == 'execute': return session.execute(cancel_check=cancel_check, **values)
        if action == 'review': return session.review(**values)
        if action == 'report': return session.export_report()
        raise ValueError('未知GUI操作')
