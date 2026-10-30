"""H2 immutable batches, field execution, bounded quality retries and reports.

Only explicit run_batch invokes SDK I/O. Terminal execution faults never resume;
the clear_fault API logs a no-op and the whole collection ends.
"""
from contextlib import contextmanager
import copy
import csv
import hashlib
import json
import logging
import os
from pathlib import Path
import shutil
import time
import uuid
import numpy as np
import yaml
from . import hardware_field as field
from .data import write_json, jsonable
from .dual_model import nominal_dual_arm
from .vendor_identity import sha256


def read(path):
    return json.loads(Path(path).read_text())


def digest(value):
    return hashlib.sha256(json.dumps(jsonable(value), sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode()).hexdigest()


def atomic_json(path, value, *, replace=False):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.parent / '.pending'; pending.mkdir(exist_ok=True)
    temp = pending / (uuid.uuid4().hex + '.json')
    with temp.open('x', encoding='utf-8') as stream:
        json.dump(jsonable(value), stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    if replace: os.replace(temp, path)
    else:
        os.link(temp, path); temp.unlink()
    fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try: os.fsync(fd)
    finally: os.close(fd)


@contextmanager
def batch_lock(root):
    import fcntl
    root = Path(root)
    with (root / '.run.lock').open('a+') as stream:
        try: fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError: raise RuntimeError('Another batch operation is active') from None
        try: yield
        finally: fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def events(root):
    directory = Path(root) / 'events'; rows = []; previous = None
    for index, path in enumerate(sorted(directory.glob('*.json')) if directory.exists() else []):
        row = read(path); checksum = row.pop('sha256')
        if path.name != f'{index:08}.json' or row['sequence'] != index or row['previous_sha256'] != previous or digest(row) != checksum:
            raise ValueError('Batch event chain changed')
        row['sha256'] = checksum; rows.append(row); previous = checksum
    return rows


def event(root, kind, **values):
    rows = events(root)
    value = dict(sequence=len(rows), previous_sha256=rows[-1]['sha256'] if rows else None,
                 event=kind, wall_time_ns=time.time_ns(), **values)
    value['sha256'] = digest(value)
    atomic_json(Path(root) / 'events' / f'{len(rows):08}.json', value)
    return value


def implementation():
    from . import _core
    binary, sdk = field.executor_binary()
    base = Path(__file__).parent
    names = ['hardware_batch.py', 'hardware_field.py', 'staged_motion.py', 'collision.py',
             'rigid_identification.py', 'dynamic_export.py']
    return dict(sources={name: sha256(base / name) for name in names},
                executor_sha256=sha256(binary), sdk=sdk, core_sha256=sha256(_core.__file__),
                numpy_version=np.__version__)


def settings(path, joint_names):
    config = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if config.get('schema_version') != 'casbot_hardware_batch_v1':
        raise ValueError('Explicit H2 batch schema required')
    for stage in ('direction', 'frequency', 'static'):
        if type(config[stage]['enabled']) is not bool: raise ValueError('Boolean stage enable required')
    for stage in ('direction', 'frequency'):
        block = config[stage]
        if block['joints'] == 'all': block['joints'] = list(joint_names)
        if not block['joints'] or len(set(block['joints'])) != len(block['joints']) or not set(block['joints']) <= set(joint_names):
            raise ValueError('Unique named arm joints required')
        if not np.isfinite(block['amplitude_deg']) or not 0 < block['amplitude_deg'] <= 5:
            raise ValueError('Commissioning amplitude must be in (0,5] degrees')
    if type(config['direction']['require_review']) is not bool:
        raise ValueError('Boolean direction review policy required')
    if not config['direction']['enabled'] and config['direction']['require_review']:
        raise ValueError('Cannot require a direction review without direction trials')
    frequency = config['frequency']
    if (not frequency['rates_hz'] or len(set(frequency['rates_hz'])) != len(frequency['rates_hz']) or
        any(type(hz) is not int or hz not in (100, 250, 500) for hz in frequency['rates_hz'])):
        raise ValueError('Distinct frequency tiers must be 100/250/500 Hz')
    if type(frequency['repeats']) is not int or not 1 <= frequency['repeats'] <= 10:
        raise ValueError('Frequency repeat count must be 1..10')
    if type(frequency['seed']) is not int or frequency['seed'] < 0:
        raise ValueError('Nonnegative randomization seed required')
    if not np.isfinite(frequency['minimum_publish_ratio']) or not 0 < frequency['minimum_publish_ratio'] <= 1:
        raise ValueError('Invalid measured frequency ratio')
    static = config['static']
    cap = static['max_poses_per_split']
    if cap is not None and (type(cap) is not int or cap < 1): raise ValueError('Positive static pose cap required')
    if type(static['quality_retries']) is not int or not 0 <= static['quality_retries'] <= 10:
        raise ValueError('Static quality retries must be bounded at 0..10')
    if not np.isfinite(static['retry_settle_extra_s']) or static['retry_settle_extra_s'] < 0:
        raise ValueError('Nonnegative retry settling increment required')
    if not np.isfinite(config['inter_job_pause_s']) or config['inter_job_pause_s'] < 0:
        raise ValueError('Nonnegative inter-job pause required')
    return config


def plan_batch(config_path, profile_path, output, *, preset=None, domain_id, connection_scope='hardware'):
    if type(domain_id) is not int or not 0 <= domain_id <= 232: raise ValueError('Explicit ROS domain required')
    if connection_scope not in ('hardware', 'localhost_contract_mock'): raise ValueError('Unknown batch connection scope')
    engine, source = nominal_dual_arm(); names = [j.name for j in engine.model.joints]
    config = settings(config_path, names); profile = field.load_profile(profile_path); jobs = []
    if config['frequency']['enabled'] and config['static']['enabled'] and profile['command_hz'] not in config['frequency']['rates_hz']:
        raise ValueError('Static default frequency must be included in the frequency experiment')
    if config['direction']['enabled']:
        for j, name in enumerate(config['direction']['joints']):
            jobs.append(dict(id=f'direction_{j:02}', kind='direction', joint=name, hz=profile['command_hz'],
                             amplitude_deg=config['direction']['amplitude_deg']))
    if config['frequency']['enabled']:
        block = config['frequency']; frequency = []
        for j, name in enumerate(block['joints']):
            for hz in block['rates_hz']:
                for repeat in range(block['repeats']):
                    frequency.append(dict(id=f'frequency_{j:02}_{hz}_{repeat}', kind='frequency',
                        joint=name, hz=hz, repeat=repeat, amplitude_deg=block['amplitude_deg']))
        rng = np.random.default_rng(block['seed'])
        jobs.extend(frequency[int(i)] for i in rng.permutation(len(frequency)))
    if config['static']['enabled']:
        if preset is None: raise ValueError('Static batch requires a sealed static preset')
        preset = Path(preset).resolve(); field.check_seal(preset)
        report = read(preset / 'report.json')
        if not report['passed'] or report['source_model_sha256'] != source['sha256'] or report['joint_names'] != names:
            raise ValueError('Static preset model/acceptance mismatch')
        if read(preset / 'scene.json') != profile['scene']: raise ValueError('Static and field scenes differ')
        rows = read(preset / 'candidates.json'); selected = {}
        for arm in ('left', 'right'):
            for split in ('train', 'validation'):
                subset = [r for r in rows if r['status'] == 'selected' and r['arm'] == arm and r['split'] == split]
                cap = config['static']['max_poses_per_split']
                if cap is not None: subset = subset[:cap]
                selected.update({r['id']: r for r in subset})
        seen = set()
        for name in report['planned_execution_order']:
            conditions = read(preset / 'plans' / name / 'motion.json')['conditions']; candidate = conditions['candidate_id']
            if candidate in selected and candidate not in seen:
                seen.add(candidate); row = selected[candidate]
                jobs.append(dict(id=f'static_{len(seen)-1:03}', kind='static', candidate_id=candidate,
                                 split=row['split'], arm=row['arm'], hz=profile['command_hz']))
        if not seen: raise ValueError('No static pairs selected')
    if not jobs: raise ValueError('Empty batch')
    root = field.output_outside(output, *([preset] if preset is not None else []))
    root.mkdir(parents=True, exist_ok=False); protocol = root / 'protocol'; protocol.mkdir()
    write_json(protocol / 'profile.json', profile); write_json(protocol / 'settings.json', config)
    (protocol / 'scene.yaml').write_text(yaml.safe_dump(profile['scene']), encoding='utf-8')
    if config['static']['enabled']: shutil.copytree(preset, protocol / 'preset')
    record = dict(schema_version='casbot_hardware_batch_protocol_v1', jobs=jobs, domain_id=domain_id,
                  connection_scope=connection_scope, joint_names=names, source_model_sha256=source['sha256'],
                  implementation=implementation(), config=config, profile=profile,
                  recovery_policy='log_noop_and_terminate_session_no_fault_resume',
                  hardware_calibration_accepted=False, device_writes_during_planning=0)
    write_json(protocol / 'protocol.json', record); field.seal(protocol)
    event(root, 'planned', protocol_sha256=sha256(protocol / 'manifest.json'))
    return summarize(root, 'planned')


def load_batch(root, *, for_run=False):
    root = Path(root).resolve(); field.check_seal(root / 'protocol')
    protocol = read(root / 'protocol/protocol.json')
    if protocol['schema_version'] != 'casbot_hardware_batch_protocol_v1':
        raise ValueError('Unsupported batch protocol')
    journal = events(root)
    if not journal or journal[0]['event'] != 'planned' or journal[0]['protocol_sha256'] != sha256(root / 'protocol/manifest.json'):
        raise ValueError('Frozen protocol differs from its planning event')
    _, source = nominal_dual_arm()
    if source['sha256'] != protocol['source_model_sha256']: raise ValueError('Batch source geometry changed')
    if for_run and protocol['implementation'] != implementation():
        raise ValueError('Batch implementation/SDK changed; start a new session')
    return root, protocol


def outcomes(root, protocol):
    root = Path(root); result = {j['id']: [] for j in protocol['jobs']}; pending = []
    committed = [e for e in events(root) if e['event'] == 'attempt_finished']
    references = {e['attempt']: e['manifest_sha256'] for e in committed}
    for folder in sorted((root / 'attempts').glob('*/*')) if (root / 'attempts').exists() else []:
        if not folder.is_dir(): continue
        relative = str(folder.relative_to(root))
        if not (folder / 'manifest.json').exists() or not (folder / 'outcome.json').exists():
            pending.append(relative); continue
        field.check_seal(folder); row = read(folder / 'outcome.json')
        if (row['job_id'] not in result or folder.parent.name != row['job_id'] or row['attempt'] != relative or
            row['protocol_sha256'] != sha256(root / 'protocol/manifest.json')):
            raise ValueError('Attempt belongs to a different frozen job/protocol')
        if relative in references and references[relative] != sha256(folder / 'manifest.json'):
            raise ValueError('Committed attempt changed')
        row['manifest_sha256'] = sha256(folder / 'manifest.json')
        result[row['job_id']].append(row)
    if set(references) - {r['attempt'] for rows in result.values() for r in rows}:
        raise ValueError('Committed attempt is missing')
    for rows in result.values():
        if sum(r['status'] == 'accepted' for r in rows) > 1: raise ValueError('Duplicate accepted captures for one job')
    return result, pending


def accepted(rows):
    return next((r for r in rows if r['status'] == 'accepted'), None)


def direction_reviews(root, protocol, history):
    result = {}
    for item in events(root):
        if item['event'] != 'direction_review': continue
        folder = Path(root) / item['review']
        field.check_seal(folder)
        if sha256(folder / 'manifest.json') != item['manifest_sha256']: raise ValueError('Direction review changed')
        review = read(folder / 'review.json')
        for row in review['joints']:
            job = next((j for j in protocol['jobs'] if j['kind'] == 'direction' and j['joint'] == row['joint']), None)
            source = accepted(history[job['id']]) if job else None
            if source is None or source['manifest_sha256'] != row['attempt_manifest_sha256']:
                raise ValueError('Direction review is not bound to the accepted observation')
            result[row['joint']] = dict(**row, observer=review['observer'], review=str(folder.relative_to(root)))
    return result


def direction_ready(protocol, reviews):
    return not protocol['config']['direction']['require_review'] or all(
        reviews.get(joint, {}).get('observed_same_direction') is True for joint in protocol['config']['direction']['joints'])


def write_direction_template(root, output):
    root, protocol = load_batch(root); history, _ = outcomes(root, protocol)
    rows = []
    for job in protocol['jobs']:
        if job['kind'] != 'direction': continue
        source = accepted(history[job['id']])
        rows.append(dict(joint=job['joint'], observed_same_direction=None, note='',
                         attempt_manifest_sha256=source['manifest_sha256'] if source else None))
    path = field.output_outside(output, root); path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as stream:
        yaml.safe_dump(dict(observer='', joints=rows), stream, allow_unicode=True, sort_keys=False)
    return dict(passed=True, output=str(path), device_writes=0)


def import_direction_review(root, path):
    root, protocol = load_batch(root)
    with batch_lock(root):
        history, _ = outcomes(root, protocol); review = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
        if not isinstance(review.get('observer'), str) or not review['observer'].strip():
            raise ValueError('Named human observer required')
        seen = set()
        for row in review['joints']:
            joint = row['joint']
            if joint in seen or type(row['observed_same_direction']) is not bool:
                raise ValueError('Unique joints and explicit true/false direction observations required')
            seen.add(joint)
            job = next((j for j in protocol['jobs'] if j['kind'] == 'direction' and j['joint'] == joint), None)
            source = accepted(history[job['id']]) if job else None
            if source is None or row['attempt_manifest_sha256'] != source['manifest_sha256']:
                raise ValueError('Direction observation source digest missing/stale')
        if not seen: raise ValueError('Empty direction review')
        folder = root / 'reviews' / uuid.uuid4().hex; folder.mkdir(parents=True)
        write_json(folder / 'review.json', review); shutil.copyfile(path, folder / 'source.yaml'); field.seal(folder)
        event(root, 'direction_review', review=str(folder.relative_to(root)), manifest_sha256=sha256(folder / 'manifest.json'))
        return summarize(root, 'direction_review_recorded')


class SDKBatchBackend:
    def __init__(self, local_remaps=None):
        self.local_remaps = local_remaps

    def snapshot(self, profile, output, domain_id):
        return field.run_executor(field.load_profile(profile), output, domain_id, snapshot=True, local_remaps=self.local_remaps)

    def execute(self, plan, output, domain_id):
        return field.execute_field(plan, output, domain_id, local_remaps=self.local_remaps)

    def inspect(self, plan, capture, output, *, static=False):
        return field.analyse_field(plan, capture, output, windows_only=static)

    def clear_fault(self, reason):
        logging.getLogger(__name__).error('SDK解除故障尚未实现，仅记录并终止整批采集：%s', reason)
        return dict(implemented=False, performed=False, status='not_implemented', reason=reason)


def complete_attempt(root, folder, job, status, reason, **extra):
    value = dict(job_id=job['id'], attempt=str(folder.relative_to(root)), status=status, reason=reason,
                 protocol_sha256=sha256(root / 'protocol/manifest.json'), **extra)
    write_json(folder / 'outcome.json', value); field.seal(folder)
    event(root, 'attempt_finished', job_id=job['id'], attempt=value['attempt'],
          manifest_sha256=sha256(folder / 'manifest.json'), status=status)
    return value


def terminal_fault(root, backend, reason, *, attempt=None):
    try: recovery = backend.clear_fault(reason)
    except Exception as exc: recovery = dict(implemented=False, performed=False, status='stub_error', error=str(exc))
    folder = Path(root) / 'faults' / uuid.uuid4().hex; folder.mkdir(parents=True)
    write_json(folder / 'clear_fault.json', recovery); field.seal(folder)
    event(root, 'terminal_fault', reason=reason, attempt=attempt, fault_record=str(folder.relative_to(root)),
          manifest_sha256=sha256(folder / 'manifest.json'))
    return summarize(root, 'faulted')


def write_profile(root, folder, protocol, hz):
    profile = copy.deepcopy(protocol['profile']); profile.pop('scene')
    profile['command_hz'] = hz
    profile['collision_scene'] = os.path.relpath(Path(root) / 'protocol/scene.yaml', folder)
    path = folder / 'profile.yaml'; path.write_text(yaml.safe_dump(profile), encoding='utf-8')
    return path


def frequency_anchor(root, snapshot):
    folder = Path(root) / 'frequency_anchor'
    if folder.exists():
        field.check_seal(folder)
        logged = [r for r in events(root) if r['event'] == 'frequency_anchor_frozen']
        if len(logged) != 1 or logged[0]['manifest_sha256'] != sha256(folder / 'manifest.json'):
            raise ValueError('Frequency reference anchor is not frozen')
        return read(folder / 'anchor.json')['q_rad']
    folder.mkdir()
    write_json(folder / 'anchor.json', dict(q_rad=field._read_snapshot(snapshot).tolist(),
        source_snapshot_manifest_sha256=sha256(Path(snapshot) / 'manifest.json')))
    field.seal(folder); event(root, 'frequency_anchor_frozen', manifest_sha256=sha256(folder / 'manifest.json'))
    return read(folder / 'anchor.json')['q_rad']


def run_batch(root, *, resume=False, max_jobs=None, backend=None):
    root, protocol = load_batch(root, for_run=True)
    if max_jobs is not None and (type(max_jobs) is not int or max_jobs < 1): raise ValueError('Positive checkpoint job count required')
    backend = backend or SDKBatchBackend()
    if protocol['connection_scope'] == 'hardware' and getattr(backend, 'local_remaps', None):
        raise ValueError('Hardware protocol cannot consume local remapped test data')
    if protocol['connection_scope'] == 'localhost_contract_mock' and not getattr(backend, 'local_remaps', None):
        raise ValueError('Local contract protocol requires an isolated transport')
    with batch_lock(root):
        journal = events(root)
        if any(r['event'] == 'terminal_fault' for r in journal):
            raise RuntimeError('Terminal fault session cannot resume; no recovery is implemented')
        if not resume and any(r['event'] == 'attempt_started' for r in journal):
            raise FileExistsError('Explicit --resume required for a started batch')
        history, pending = outcomes(root, protocol)
        if pending:
            return terminal_fault(root, backend, 'unclassified_incomplete_attempt_no_safe_resume', attempt=pending[0])
        # A crash between sealing and committing an outcome only requires journal reconciliation.
        committed = {r['attempt'] for r in journal if r['event'] == 'attempt_finished'}
        for rows in history.values():
            for row in rows:
                if row['attempt'] not in committed:
                    event(root, 'attempt_finished', job_id=row['job_id'], attempt=row['attempt'],
                          manifest_sha256=row['manifest_sha256'], status=row['status'], reconciled=True)
        if any(r['status'] == 'fault' for rows in history.values() for r in rows):
            return terminal_fault(root, backend, 'sealed_execution_fault_no_resume')
        if any(r['status'] in ('quality_exhausted', 'rejected_direction') for rows in history.values() for r in rows):
            return summarize(root, 'data_rejected')
        completed_now = 0
        for job in protocol['jobs']:
            rows = history[job['id']]
            if accepted(rows) or any(r['status'] == 'rejected_frequency' for r in rows): continue
            if job['kind'] != 'direction' and not direction_ready(protocol, direction_reviews(root, protocol, history)):
                return summarize(root, 'awaiting_direction_review')
            if job['kind'] == 'static' and protocol['config']['frequency']['enabled']:
                required = [j for j in protocol['jobs'] if j['kind'] == 'frequency' and j['hz'] == protocol['profile']['command_hz']]
                if any(not accepted(history[j['id']]) for j in required):
                    return summarize(root, 'default_frequency_rejected')
            while True:
                parent = root / 'attempts' / job['id']; parent.mkdir(parents=True, exist_ok=True)
                index = len(list(parent.glob('attempt_*'))); folder = parent / f'attempt_{index:03}'
                folder.mkdir(); event(root, 'attempt_started', job_id=job['id'], attempt=str(folder.relative_to(root)))
                quality_failures = sum(r['status'] == 'quality_retry' for r in rows)
                profile = write_profile(root, folder, protocol, job['hz']); normal_return = False
                status = 'fault'; reason = ''; analysis = None
                try:
                    snapshot = backend.snapshot(profile, folder / 'snapshot', protocol['domain_id'])
                    if not snapshot['passed']:
                        status = 'interrupted' if snapshot.get('supervisor_interruption') == 'user_interrupt' else 'fault'
                        reason = 'snapshot_failed'
                    else:
                        if job['kind'] == 'static':
                            field.prepare_field(profile, root / 'protocol/preset', folder / 'snapshot', folder / 'plan',
                                candidate_ids=[job['candidate_id']], settle_extra_s=quality_failures * protocol['config']['static']['retry_settle_extra_s'])
                        else:
                            anchor = frequency_anchor(root, folder / 'snapshot') if job['kind'] == 'frequency' else None
                            field.prepare_jog(profile, folder / 'snapshot', folder / 'plan', job['joint'], job['amplitude_deg'], anchor_rad=anchor)
                        executed = backend.execute(folder / 'plan', folder / 'capture', protocol['domain_id'])
                        if not executed['passed']:
                            status = 'interrupted' if executed.get('supervisor_interruption') == 'user_interrupt' else 'fault'
                            reason = 'execution_failed:' + str(executed.get('fault') or executed.get('reason'))
                        else:
                            normal_return = True
                            analysis = backend.inspect(folder / 'plan', folder / 'capture', folder / 'inspection', static=job['kind'] == 'static')
                            status = 'accepted' if analysis['passed'] else 'rejected_direction'
                            reason = 'observation_checks'
                            if job['kind'] == 'frequency':
                                measured = analysis.get('comparison_command_publish_frequency', {}).get('hz')
                                expected = read(folder / 'plan/motion.json')['conditions']['comparison_reference_sha256']
                                passed = bool(analysis['passed'] and measured is not None and
                                              analysis.get('comparison_reference_sha256') == expected and
                                              measured >= job['hz'] * protocol['config']['frequency']['minimum_publish_ratio'])
                                status = 'accepted' if passed else 'rejected_frequency'
                except field.StaticWindowError as exc:
                    reason = str(exc)
                    if normal_return and job['kind'] == 'static':
                        status = 'quality_retry' if quality_failures < protocol['config']['static']['quality_retries'] else 'quality_exhausted'
                        reject = folder / 'inspection'; reject.mkdir(exist_ok=True)
                        write_json(reject / 'report.json', dict(passed=False, retryable=status == 'quality_retry', reason=reason))
                        field.seal(reject)
                    else: status = 'fault'
                except KeyboardInterrupt:
                    status = 'interrupted'; reason = 'user_interrupt_outside_command_loop'
                except Exception as exc:
                    status = 'fault'; reason = type(exc).__name__ + ':' + str(exc)
                row = complete_attempt(root, folder, job, status, reason,
                                       normal_return_confirmed=normal_return,
                                       settle_extra_s=quality_failures * protocol['config']['static']['retry_settle_extra_s'])
                rows.append(dict(row, manifest_sha256=sha256(folder / 'manifest.json')))
                if status == 'fault':
                    return terminal_fault(root, backend, reason, attempt=str(folder.relative_to(root)))
                if status == 'interrupted': return summarize(root, 'interrupted')
                if status in ('quality_exhausted', 'rejected_direction'): return summarize(root, 'data_rejected')
                if status != 'quality_retry': break
            completed_now += 1
            atomic_json(root / 'progress.json', dict(last_job=job['id'], completed_in_invocation=completed_now), replace=True)
            if max_jobs is not None and completed_now >= max_jobs: return summarize(root, 'checkpoint')
            try: time.sleep(protocol['config']['inter_job_pause_s'])
            except KeyboardInterrupt: return summarize(root, 'interrupted_at_checkpoint')
        return summarize(root, 'completed')


def summarize(root, state, *, output=None):
    root, protocol = load_batch(root); history, pending = outcomes(root, protocol)
    reviews = direction_reviews(root, protocol, history)
    jobs = []; frequency = []; static_sources = []; complete = True
    for job in protocol['jobs']:
        rows = history[job['id']]; source = accepted(rows)
        last = source or (rows[-1] if rows else None)
        if source is None and not (last and last['status'] == 'rejected_frequency'): complete = False
        row = dict(**job, status=last['status'] if last else 'pending', attempts=len(rows),
                   source_attempt=source['attempt'] if source else None)
        if source and job['kind'] == 'static':
            windows_path = root / source['attempt'] / 'inspection/static_windows.json'
            windows = read(windows_path)
            if len(windows) != 2 or any(w['conditions']['pose'] != job['candidate_id'] or w['conditions']['batch'] != job['split'] for w in windows):
                raise ValueError('Static accepted windows differ from frozen role')
            static_sources.append(dict(job=job, attempt=source['attempt'], manifest_sha256=source['manifest_sha256'], windows=windows))
        if last and job['kind'] == 'frequency' and last['status'] in ('accepted', 'rejected_frequency'):
            analysis = read(root / last['attempt'] / 'inspection/report.json')
            row.update(measured_hz=analysis.get('comparison_command_publish_frequency', {}).get('hz'),
                       lag_ms=analysis.get('aggregate_response_lag_ms'),
                       comparison_reference_sha256=analysis.get('comparison_reference_sha256'))
            frequency.append(dict(row))
        jobs.append(row)
    # One absolute measured anchor and one continuous reference per probed joint, independent of tier.
    reference_consistent = all(isinstance(r['comparison_reference_sha256'], str) and len(r['comparison_reference_sha256']) == 64 for r in frequency) and all(
        len({r['comparison_reference_sha256'] for r in frequency if r['joint'] == joint}) <= 1 for joint in protocol['config']['frequency']['joints'])
    journal = events(root); faulted = any(e['event'] == 'terminal_fault' for e in journal)
    directions_passed = direction_ready(protocol, reviews)
    default_ready = all(row['status'] == 'accepted' for row in jobs if row['kind'] == 'frequency' and row['hz'] == protocol['profile']['command_hz'])
    all_frequencies = all(row['status'] == 'accepted' for row in jobs if row['kind'] == 'frequency')
    passed = bool(complete and not faulted and not pending and directions_passed and reference_consistent and default_ready and
                  all(row['status'] == 'accepted' for row in jobs if row['kind'] != 'frequency'))
    report = dict(schema_version='casbot_hardware_batch_report_v1', passed=passed, state='faulted' if faulted else state,
        collection_complete=complete, pending_attempts=pending, jobs=jobs, frequency=frequency,
        frequency_references_consistent=reference_consistent, direction_reviews=reviews,
        all_frequency_profiles_passed=all_frequencies, default_frequency_passed=default_ready,
        configured_direction_scope=protocol['config']['direction']['joints'], direction_gate_satisfied=directions_passed,
        direction_review_complete=bool(protocol['config']['direction']['enabled'] and all(
            reviews.get(joint, {}).get('observed_same_direction') is True for joint in protocol['config']['direction']['joints'])),
        full14_direction_scope=bool(protocol['config']['direction']['enabled'] and set(protocol['config']['direction']['joints']) == set(protocol['joint_names'])),
        physical_direction_verified_by_software=False, formal_hardware_calibration_accepted=False,
        fault_recovery_implemented=False, connection_scope=protocol['connection_scope'],
        protocol_manifest_sha256=sha256(root / 'protocol/manifest.json'),
        final_event_sha256=journal[-1]['sha256'] if journal else None,
        frequency_scope='client publication/callback and aggregate trajectory shift; no device adoption/acquisition claim',
        default_command_hz=protocol['profile']['command_hz'], operating_frequency_recommendation=None)
    if output is None:
        parent = root / 'reports'; parent.mkdir(exist_ok=True)
        destination = parent / f'report_{len(list(parent.glob("report_*"))):04}'
    else:
        destination = field.output_outside(output, root)
    destination.mkdir(parents=True, exist_ok=False)
    if passed and protocol['config']['static']['enabled'] and static_sources:
        means = [w for source in static_sources for w in source['windows']]
        gravity = destination / 'gravity'; gravity.mkdir()
        metadata = dict(joint_names=protocol['joint_names'], source_model_sha256=protocol['source_model_sha256'])
        model = field.write_gravity_candidate(means, protocol['profile'], gravity, metadata)
        write_json(gravity / 'static_windows.json', means); write_json(gravity / 'accepted_sources.json', static_sources)
        write_json(gravity / 'report.json', model); field.seal(gravity)
        report['gravity'] = dict(available=True, report='gravity/report.json', fit_succeeded=model['fit_succeeded'],
                                 commissioning_validation_passed=model['commissioning_validation_passed'])
        report['passed'] = bool(report['passed'] and model['passed'] and model['commissioning_validation_passed'])
    else: report['gravity'] = dict(available=False)
    write_json(destination / 'report.json', report)
    with (destination / 'jobs.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=['id', 'kind', 'joint', 'hz', 'repeat', 'candidate_id', 'split', 'arm', 'status', 'attempts', 'source_attempt'], extrasaction='ignore')
        writer.writeheader(); writer.writerows(jobs)
    if frequency:
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        fig = Figure(figsize=(10, 4)); FigureCanvasAgg(fig); ax = fig.subplots()
        x = np.arange(len(frequency)); ax.plot(x, [r['hz'] for r in frequency], 'o--', label='Requested Hz')
        ax.plot(x, [r['measured_hz'] for r in frequency], 'x-', label='Measured client Hz')
        ax.set(xlabel='Frozen randomized execution order', ylabel='Hz (not motor sampling)'); ax.legend(); ax.grid(alpha=.2)
        fig.tight_layout(); fig.savefig(destination / 'frequency.png', dpi=120)
    field.seal(destination)
    if output is None:
        atomic_json(root / 'latest.json', dict(report=str(destination.relative_to(root)), manifest_sha256=sha256(destination / 'manifest.json')), replace=True)
    return report


def analyse_batch(root, output):
    return summarize(root, 'read_only_analysis', output=output)
