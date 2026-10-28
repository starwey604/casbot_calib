"""P0 preregistration and role-scoped inputs, independent of plant truth.

R0 schedules identities, not collision-certified motion. Files are sealed against
accidental changes; hashes are provenance checks, not signatures/access control.
"""
from copy import deepcopy
from datetime import datetime, timezone
from hashlib import sha256
import json
from pathlib import Path
import re
import subprocess
import numpy as np
import yaml
from .data import write_json

PROTOCOL = 'staged_dynamics_p0_v1'
SPLITS = ('train', 'selection', 'test')
PURPOSES = {'fit': 'train', 'stability': 'train', 'select': 'selection', 'final_test': 'test'}
IDENTITY = ('split', 'center_id', 'reference_seed', 'acquisition_batch_id',
            'repeat_id', 'active_arm_group', 'acquisition_seed', 'reference_id')


def file_hash(path):
    digest = sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def digest(value):
    return sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                             allow_nan=False).encode()).hexdigest()


def _read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def _local(root, relative):
    path = Path(relative)
    if path.is_absolute() or '..' in path.parts or 'evaluation' in path.parts:
        raise ValueError('P0 inputs must be local public paths')
    target = Path(root) / path
    if not target.resolve().is_relative_to(Path(root).resolve()) or any(
            p.is_symlink() for p in [target, *target.parents] if p != Path(root).parent):
        raise ValueError('P0 inputs may not escape through symlinks')
    return target


def _seal(root, name, payload):
    """Exclusive creation: never replace a prior decision or input snapshot."""
    path = Path(root) / name
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    with Path(str(path) + '.sha256').open('x', encoding='ascii') as stream:
        stream.write(file_hash(path) + '\n')
    return payload


def _unseal(root, name):
    path = _local(root, name)
    if file_hash(path) != Path(str(path) + '.sha256').read_text().strip():
        raise ValueError('P0 sealed file changed: ' + name)
    return _read(path)


def validate_config(config):
    if config.get('schema_version') != 2 or config.get('mode') != 'mock_only':
        raise ValueError('P0 requires mock configuration schema 2')
    p0 = config.get('p0', {})
    if p0.get('protocol') != PROTOCOL or p0.get('schema_version') != 1:
        raise ValueError('Unsupported P0 protocol')
    # This version has an explicit experimental size. Changing the design needs
    # a new version rather than silently accepting a smaller evidence domain.
    expected = dict(train=(3, 2, ['both']), selection=(2, 2, ['both']), test=(2, 1, ['both', 'left', 'right']))
    if set(p0['roles']) != set(SPLITS):
        raise ValueError('Exactly train/selection/test roles required')
    for split, (centers, phases, groups) in expected.items():
        role = p0['roles'][split]
        if (role['centers'], role['phase_seeds_per_center'], role['active_arm_groups']) != (centers, phases, groups) or role['batches'] != 2:
            raise ValueError('P0 v1 role size mismatch: ' + split)
    seeds = [p0['roles'][s][k] for s in SPLITS for k in ('reference_seed_base', 'acquisition_seed_base', 'order_seed')]
    if any(type(s) is not int or s < 0 for s in seeds) or len(set(seeds)) != len(seeds):
        raise ValueError('Disjoint nonnegative integer role seeds required')
    if p0['selection']['ridge_candidates'] != [1e-5, 1e-4, 1e-3]:
        raise ValueError('P0 v1 requires the declared regularization path')
    if p0['selection']['score'] != 'mean_trajectory_axis_nrmse' or p0['selection']['tie_break'] != 'stronger_regularization' or p0['selection']['tie_absolute_nrmse'] < 0:
        raise ValueError('Unsupported selection score/tie rule')
    if p0['wrist']['joints'] != [4, 11] or p0['wrist']['directions'] != [-1, 1]:
        raise ValueError('P0 v1 requires both wrist yaw axes and directions')
    fq = p0['frequency_qualification']
    if fq['noise_periods'] < 4 or fq['noise_multiplier'] <= 0 or fq['minimum_bin_rms_nm'] <= 0:
        raise ValueError('Invalid independent noise qualification')
    if config['control_period_ns'] != 2000000 or config['inertia_sample_period_ns'] != 250000:
        raise ValueError('P0 v1 requires 500 Hz commands / 4 kHz inertia observations')
    if config['dynamic_periods'] < 4 or not np.isfinite(config['dynamic_period_s']) or config['dynamic_period_s'] <= 0:
        raise ValueError('Need at least two complete steady periods after two discarded periods')
    if p0['preprocessing'] != dict(harmonics=40, phase_points=1024, discard_periods=2,
                                  friction_order='nonlinear_features_before_projection'):
        raise ValueError('Unsupported P0 matched-band preprocessing')
    for key in ('home_rad', 'pose_amplitude_rad'):
        if np.asarray(config[key]).shape != (14,) or not np.isfinite(config[key]).all():
            raise ValueError('Finite 14-axis geometry required')
    # Reject non-finite settings even when they are not used until a later round.
    digest(config)
    return p0


def build_split_manifest(config):
    p0 = validate_config(config)
    config_sha256 = digest({k: v for k, v in config.items() if k != 'mock'})
    rows = []
    for split in SPLITS:
        role = p0['roles'][split]
        for center in range(role['centers']):
            center_id = f'{split}_c{center:02}'
            references = []
            for phase in range(role['phase_seeds_per_center']):
                for group in role['active_arm_groups']:
                    references.append(('inertia', group, None, None, phase))
            # Same role-separated centers, both wrist yaw axes and directions.
            for joint in p0['wrist']['joints']:
                for direction in p0['wrist']['directions']:
                    references.append(('inertia_probe', 'left' if joint < 7 else 'right', joint, direction, 0))
            for batch in range(role['batches']):
                for ref_index, (stage, group, joint, direction, phase) in enumerate(references):
                    index = center * 100 + ref_index
                    reference_id = f'{center_id}_{ref_index:02}'
                    row = dict(trial_id=f'{reference_id}_b{batch:02}', stage=stage, split=split,
                        center_id=center_id, reference_id=reference_id,
                        reference_seed=role['reference_seed_base'] + index,
                        acquisition_batch_id=f'{split}_batch_{batch:02}', repeat_id=batch,
                        acquisition_seed=role['acquisition_seed_base'] + center * 1000 + batch * 100 + ref_index,
                        active_arm_group=group, phase_id=phase,
                        period_s=config['dynamic_period_s'] if stage == 'inertia' else None,
                        excitation_harmonics=([1, 2, 3, 5] if split == 'train' else [1, 3, 4, 6]) if stage == 'inertia' else [],
                        center_rad=None, reference_sha256=None,
                        config_sha256=config_sha256,
                        reference_status='pending_R1_collision_certified_motion')
                    if joint is not None:
                        row.update(joint=joint, direction=direction)
                    rows.append(row)
    order = {}
    for split in SPLITS:
        rng = np.random.default_rng(p0['roles'][split]['order_seed'])
        blocks = []
        for stage in ('inertia_probe', 'inertia'):
            for batch in range(2):
                block = [r['trial_id'] for r in rows if r['split'] == split and r['stage'] == stage and r['repeat_id'] == batch]
                blocks.extend([block[i] for i in rng.permutation(len(block))])
        order[split] = blocks
    manifest = dict(schema_version=1, protocol=PROTOCOL, trials=rows, planned_execution_order=order,
        actual_execution_order_status='not_collected', centers_status='logical slots only; R1 must select and certify seven separated centers',
        final_prior_holdout=p0['final_prior_holdout'],
        final_prior_holdout_status='budget frozen; reference design and independent acquisition pending R1',
        test_acquisition_requires='sealed candidate_freeze.json; no test observations before candidate selection')
    validate_split_manifest(manifest, config)
    return manifest


def validate_split_manifest(manifest, config):
    validate_config(config)
    if manifest.get('schema_version') != 1 or manifest.get('protocol') != PROTOCOL:
        raise ValueError('Unsupported split manifest')
    rows = manifest['trials']; ids = [r['trial_id'] for r in rows]
    if len(ids) != len(set(ids)) or len({r['acquisition_seed'] for r in rows}) != len(rows):
        raise ValueError('Repeated trial identity or acquisition seed')
    seed_references = {}
    for row in rows:
        if row.get('config_sha256') != digest({k: v for k, v in config.items() if k != 'mock'}):
            raise ValueError('Trial public configuration digest changed')
        if any(k not in row for k in IDENTITY) or any(type(row[k]) is not int or row[k] < 0 for k in ('reference_seed', 'acquisition_seed', 'repeat_id')):
            raise ValueError('Invalid trial identity schema')
        if any(not isinstance(row[k], str) or re.fullmatch(r'[A-Za-z0-9_+-]+', row[k]) is None for k in ('trial_id', 'center_id', 'reference_id', 'acquisition_batch_id')):
            raise ValueError('Invalid trial identity/path')
        if seed_references.setdefault(row['reference_seed'], row['reference_id']) != row['reference_id']:
            raise ValueError('Reference seed reused by another design')
    role_centers = {}; role_seeds = {}
    for split in SPLITS:
        role = config['p0']['roles'][split]; selected = [r for r in rows if r['split'] == split]
        centers = {r['center_id'] for r in selected}; role_centers[split] = centers
        role_seeds[split] = {r['reference_seed'] for r in selected} | {r['acquisition_seed'] for r in selected}
        if len(centers) != role['centers']:
            raise ValueError('Missing role center: ' + split)
        if sorted(manifest['planned_execution_order'][split]) != sorted(r['trial_id'] for r in selected):
            raise ValueError('Execution order must contain every role trial exactly once')
        per_center = role['phase_seeds_per_center'] * len(role['active_arm_groups'])
        for center in centers:
            group = [r for r in selected if r['center_id'] == center]
            refs = {r['reference_id'] for r in group}
            if len(refs) != per_center + len(config['p0']['wrist']['joints']) * len(config['p0']['wrist']['directions']):
                raise ValueError('Missing reference design in center')
            for ref in refs:
                repeats = [r for r in group if r['reference_id'] == ref]
                if len(repeats) != 2 or {r['repeat_id'] for r in repeats} != {0, 1} or {r['acquisition_batch_id'] for r in repeats} != {f'{split}_batch_00', f'{split}_batch_01'}:
                    raise ValueError('Missing or repeated acquisition batch')
                for key in ('stage', 'reference_seed', 'active_arm_group', 'joint', 'direction', 'phase_id', 'period_s', 'excitation_harmonics'):
                    if repeats[0].get(key) != repeats[1].get(key):
                        raise ValueError('Repeated reference design changed')
            for batch in range(2):
                periodic = [r for r in group if r['repeat_id'] == batch and r['stage'] == 'inertia']
                if sorted((r['phase_id'], r['active_arm_group']) for r in periodic) != sorted(
                        (phase, arm) for phase in range(role['phase_seeds_per_center']) for arm in role['active_arm_groups']):
                    raise ValueError('Incorrect phase/arm-group coverage')
                probes = [r for r in group if r['repeat_id'] == batch and r['stage'] == 'inertia_probe']
                if sorted((r['joint'], r['direction']) for r in probes) != sorted(
                        (j, d) for j in config['p0']['wrist']['joints'] for d in config['p0']['wrist']['directions']):
                    raise ValueError('Incorrect wrist coverage')
    if any(r['split'] not in SPLITS or r['stage'] not in ('inertia', 'inertia_probe') for r in rows):
        raise ValueError('Unknown split or stage')
    for a, b in (('train', 'selection'), ('train', 'test'), ('selection', 'test')):
        if role_centers[a] & role_centers[b] or role_seeds[a] & role_seeds[b]:
            raise ValueError('Role leakage: shared center or seed')
    return rows


def prepare_protocol(folder, config, *, planner=None):
    """Seal R0 identities, optionally with R1 certified references before sealing."""
    from . import _core
    from .collision import nominal_profile
    from .dual_model import nominal_dual_arm
    from .staged_frequency_contract import qualify_frequency
    p0 = validate_config(config); manifest = build_split_manifest(config)
    root = Path(folder)
    if root.exists():
        raise FileExistsError(root)
    profile = nominal_profile(scene=yaml.safe_load(Path(config['collision_scene']).read_text()))
    nominal, source = nominal_dual_arm()
    root.mkdir(parents=True, exist_ok=False)
    public = {k: deepcopy(v) for k, v in config.items() if k != 'mock'}
    write_json(root / 'session.json', dict(config=public, collision=profile.metadata,
        joint_names=[j.name for j in nominal.model.joints], synthetic_only=True, hardware_executable=False,
        units=dict(q='rad', dq='rad/s', current='A', time='int64 ns', torque='Nm output side'),
        sample_clock='synthetic acquisition integer ns; receive time separately recorded, not interchangeable'))
    if 'mock' in config:
        (root / 'evaluation').mkdir()
        write_json(root / 'evaluation/plant_config.json', config['mock'])
    write_json(root / 'split_manifest.json', manifest)
    qualification = qualify_frequency(root / 'qualification', public)
    planning_report = None; extra_inputs = []
    if planner is not None:
        planning_report, extra_inputs = planner(root, public, manifest, profile)
    repo = Path(__file__).resolve().parents[2]
    source_paths = [p for directory in ('python/casbot_calib', 'cpp', 'cmake', 'scripts/humble')
                    for p in (repo / directory).rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    source_paths += [repo / p for p in ('CMakeLists.txt', 'pyproject.toml', 'CMakePresets.json')]
    try:
        commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        commit = None
    protocol = dict(schema_version=2 if planner is not None else 1, protocol=PROTOCOL, experiment_version=p0['experiment_version'],
        created_utc=datetime.now(timezone.utc).isoformat(), git_commit=commit,
        input_sha256={p: file_hash(root / p) for p in ['session.json', 'split_manifest.json', 'qualification/report.json', 'qualification/noise.npz', *extra_inputs]},
        source_sha256={str(p.relative_to(repo)): file_hash(p) for p in sorted(source_paths)},
        native_binary_sha256=file_hash(_core.__file__), model=source,
        geometry_sha256=profile.metadata['geometry_sha256'],
        policy_sha256=digest(public['p0']), synthetic_only=True, hardware_executable=False)
    _seal(root, 'protocol.json', protocol)
    report = dict(schema_version=2, protocol=PROTOCOL, passed=False, contract_passed=qualification['passed'],
        completed_round='R0', status='contract_only_not_collision_planned_or_collected',
        counts={s: {stage: sum(r['split'] == s and r['stage'] == stage for r in manifest['trials'])
                    for stage in ('inertia', 'inertia_probe')} for s in SPLITS},
        reference_collision_status='not_executed', p0_acceptance_passed=False,
        selected_trials=[], hardware_executable=False,
        pending=['R1 certified centers, trajectories and acquisition', 'R2 stability and selection',
                 'R3 component/frequency attribution', 'R4 fresh final test and full acceptance'])
    if planning_report is not None:
        report = planning_report
    write_json(root / 'plan_report.json', report)
    (root / 'README.md').write_text('# 动力学 P0 R0 数据契约\n\n'
        '已冻结实验身份与选择规则；未选择实际构型、未生成碰撞合格轨迹、未采集、未验收动力学。\n\n'
        '32条主要惯量周期轨迹与56条腕部专项；最终前序留出另有16条重力、84条摩擦、448条起动专项的预算。'
        '既有前序开发实验和控制轨迹尚未计入此增量预算；完整时长与碰撞拒绝开销留待R1。\n\n'
        '[角色清单](split_manifest.json) · [冻结协议](protocol.json) · [频点资格](qualification/report.json) · [状态](plan_report.json)\n', encoding='utf-8')
    if planning_report is not None:
        (root / 'README.md').write_text('# 动力学 P0 R1 参考规划\n\n'
            '七个实际中心、32条主要惯量轨迹、56条腕部专项及548条最终前序留出已通过连续参考碰撞门禁。'
            '参考通过不代表实际观测、动力学候选或三个P0通过；test仅规划，须候选冻结后才采集。\n\n'
            '[实际参考清单](motion_plan.json) · [规划覆盖](configuration_coverage.json) · [冻结协议](protocol.json) · [状态](plan_report.json)\n',encoding='utf-8')
    return report


def load_protocol(root):
    protocol = _unseal(root, 'protocol.json')
    if protocol.get('schema_version') not in (1, 2) or protocol.get('protocol') != PROTOCOL:
        raise ValueError('Unsupported P0 protocol seal')
    required = {'session.json', 'split_manifest.json', 'qualification/report.json', 'qualification/noise.npz'}
    if protocol['schema_version'] == 2:
        required |= {'motion_plan.json', 'configuration_coverage.json'}
        if 'development_reference.json' in protocol['input_sha256']:
            required.add('development_reference.json')
    if set(protocol['input_sha256']) != required:
        raise ValueError('Incomplete P0 protocol input seal')
    for path, expected in protocol['input_sha256'].items():
        if file_hash(_local(root, path)) != expected:
            raise ValueError('P0 input digest changed: ' + path)
    config = _read(Path(root) / 'session.json')['config']
    if 'mock' in config or digest(config['p0']) != protocol['policy_sha256']:
        raise ValueError('Changed policy or plant truth in public configuration')
    manifest = _read(Path(root) / 'split_manifest.json')
    validate_split_manifest(manifest, config)
    if protocol['schema_version']==2:
        from .staged_multiconfiguration import validate_resolved_plan
        validate_resolved_plan(_read(Path(root)/'motion_plan.json'),manifest,config)
    return protocol, config, manifest


def authorize_role(split, purpose):
    if purpose not in PURPOSES or PURPOSES[purpose] != split:
        raise ValueError('Role leakage: purpose ' + str(purpose) + ' cannot consume ' + str(split))


def seal_role_records(root, split):
    """R1 acquisition completion hook; all declared repeats, no best-run filtering."""
    from .staged_collection import load_trial
    from .staged_identification import read_record
    protocol, _, manifest = load_protocol(root)
    if split not in SPLITS:
        raise ValueError('Unknown role')
    freeze_hash = file_hash(Path(root) / 'candidate_freeze.json') if split == 'test' and load_frozen_candidate(root) else None
    session = _read(Path(root) / 'session.json'); records = []; references = {}
    for slot in (r for r in manifest['trials'] if r['split'] == split):
        name = slot['trial_id']; meta, _ = load_trial(root, name)
        if meta['stage'] != slot['stage'] or any(meta['conditions'].get(k) != slot[k] for k in IDENTITY):
            raise ValueError('Acquisition identity differs from preregistered role')
        record, _ = read_record(root, name, meta, session['joint_names'])
        if record.get('protocol_sha256') != file_hash(Path(root) / 'protocol.json'):
            raise ValueError('Raw recording lacks the frozen P0 protocol identity')
        if record.get('native_binary_sha256') != protocol['native_binary_sha256'] or record.get('sample_period_ns') != 250000 or record.get('control_period_ns') != 2000000:
            raise ValueError('P0 native kernel or 500 Hz / 4 kHz acquisition contract changed')
        if split == 'test' and record.get('candidate_freeze_sha256') != freeze_hash:
            raise ValueError('Test recording was not acquired under the frozen candidate')
        reference_hash = file_hash(Path(root) / 'plans' / name / 'reference.npz')
        if references.setdefault(slot['reference_id'], reference_hash) != reference_hash:
            raise ValueError('Same-reference repetitions changed trajectory')
        paths = [f'plans/{name}/{p}' for p in ('motion.json', 'reference.npz', 'collision_reference.json', 'manifest.json')]
        paths += [f'runs/{name}/{p}' for p in ('observations.npz', 'commands.npz', 'feedforward.json', 'collision_actual.json', 'metadata.json')]
        if not _read(_local(root, f'runs/{name}/collision_actual.json'))['passed']:
            raise ValueError('Observed collision gate failed')
        records.append(dict(trial_id=name, files={p: file_hash(_local(root, p)) for p in paths}))
    return _seal(root, f'role_inputs/{split}.json', dict(schema_version=1, split=split,
        protocol_sha256=file_hash(Path(root) / 'protocol.json'), candidate_freeze_sha256=freeze_hash, records=records))


def role_recordings(root, stage, *, split, purpose):
    """Never opens another role's observations, even to filter them afterward."""
    from .staged_collection import load_trial
    from .staged_identification import read_record
    authorize_role(split, purpose)
    _, _, manifest = load_protocol(root)
    frozen = load_frozen_candidate(root) if split == 'test' else None
    index = _unseal(root, f'role_inputs/{split}.json')
    expected = [r['trial_id'] for r in manifest['trials'] if r['split'] == split]
    if index['split'] != split or index['protocol_sha256'] != file_hash(Path(root) / 'protocol.json') or [r['trial_id'] for r in index['records']] != expected:
        raise ValueError('Missing, repeated or changed role records')
    if frozen is not None and index['candidate_freeze_sha256'] != file_hash(Path(root) / 'candidate_freeze.json'):
        raise ValueError('Test was acquired for another candidate freeze')
    names = _read(Path(root) / 'session.json')['joint_names']
    # Verify every record in this role before yielding any data.
    for row in index['records']:
        name = row['trial_id']
        required = {f'{d}/{name}/{p}' for d, files in (
            ('plans', ('motion.json', 'reference.npz', 'collision_reference.json', 'manifest.json')),
            ('runs', ('observations.npz', 'commands.npz', 'feedforward.json', 'collision_actual.json', 'metadata.json'))) for p in files}
        if set(row['files']) != required:
            raise ValueError('Incomplete role record file seal')
        for path, expected_hash in row['files'].items():
            if file_hash(_local(root, path)) != expected_hash:
                raise ValueError('Role recording changed: ' + path)
    slots = {r['trial_id']: r for r in manifest['trials']}
    for row in index['records']:
        name = row['trial_id']; meta, _ = load_trial(root, name)
        if any(meta['conditions'].get(k) != slots[name][k] for k in IDENTITY):
            raise ValueError('Role identity changed')
        if meta['stage'] == stage:
            _, obs = read_record(root, name, meta, names)
            yield name, meta, obs


def freeze_candidate(root, candidate, selection_report):
    """R2/R3 hook: select only eligible candidates using the sealed selection role."""
    _, config, _ = load_protocol(root)
    root = Path(root)
    test_ids=[r['trial_id'] for r in _read(root/'split_manifest.json')['trials'] if r['split']=='test']
    if (root/'motion_plan.json').exists():test_ids += [r['trial_id'] for r in _read(root/'motion_plan.json')['final_prior_trials']]
    if (root / 'role_inputs/test.json').exists() or any((root/'runs'/name).exists() for name in test_ids):
        raise ValueError('Test observations already exist; upgrade protocol and use a fresh test batch')
    report = _read(_local(root, selection_report))
    if set(report) != {'schema_version', 'protocol_sha256', 'input_manifests', 'candidates'} or report['schema_version'] != 1:
        raise ValueError('Invalid selection-only report contract; test metrics are forbidden')
    if report['protocol_sha256'] != file_hash(root / 'protocol.json') or set(report['input_manifests']) != {'train', 'selection'}:
        raise ValueError('Selection must consume exactly train and selection inputs')
    for split in ('train', 'selection'):
        # Opens only the requested role; raw validity must still hold at freeze.
        list(role_recordings(root, 'inertia', split=split, purpose='fit' if split == 'train' else 'select'))
        if report['input_manifests'][split] != file_hash(root / f'role_inputs/{split}.json'):
            raise ValueError('Selection input manifest changed')
    policy = config['p0']['selection']; candidates = report['candidates']
    if sorted(r['ridge'] for r in candidates) != policy['ridge_candidates']:
        raise ValueError('Incomplete regularization candidates')
    eligible = []; evidence_hashes = {}
    for row in candidates:
        if set(row) != {'ridge', 'candidate', 'candidate_sha256', 'selection_axis_nrmse', 'evidence'}:
            raise ValueError('Invalid candidate selection evidence')
        if file_hash(_local(root, row['candidate'])) != row['candidate_sha256']:
            raise ValueError('Candidate changed after selection')
        from .staged_control import load_candidate
        load_candidate(_local(root, row['candidate']))
        scores = np.asarray(row['selection_axis_nrmse'], dtype=float)
        count = sum(r['split'] == 'selection' and r['stage'] == 'inertia' for r in _read(root / 'split_manifest.json')['trials'])
        if scores.shape != (count, 14) or not np.isfinite(scores).all() or np.any(scores < 0):
            raise ValueError('Need all selection trajectories and axes, equally weighted')
        if set(row['evidence']) != {'stability', 'prior_development', 'attribution'}:
            raise ValueError('Missing stability, prior-development or attribution evidence')
        accepted = True
        for kind, path in row['evidence'].items():
            evidence = _read(_local(root, path)); evidence_hashes[path] = file_hash(_local(root, path))
            if evidence.get('schema_version') != 1 or evidence.get('kind') != kind or evidence.get('protocol_sha256') != report['protocol_sha256'] or evidence.get('candidate_sha256') != row['candidate_sha256']:
                raise ValueError('Stale or mismatched candidate eligibility evidence')
            expected_inputs=report['input_manifests'] if kind=='attribution' else {'train':report['input_manifests']['train']}
            if evidence.get('input_manifests') != expected_inputs:
                raise ValueError('Eligibility evidence consumed another split or batch; stability/prior gates must be train-only')
            for source, expected in evidence.get('source_files', {}).items():
                if file_hash(_local(root, source)) != expected:
                    raise ValueError('Eligibility source changed')
                evidence_hashes[source] = expected
            accepted &= evidence.get('completed') is True and evidence.get('passed') is True
            if kind == 'attribution':
                accepted &= evidence.get('prediction_passed') is True and evidence.get('qualified_band_non_degradation_passed') is True
        if accepted:
            eligible.append((float(scores.mean()), row))
    if not eligible:
        raise ValueError('No acceptable candidate; failed fits cannot be selected')
    best = min(score for score, _ in eligible)
    winner = max((row for score, row in eligible if score <= best + policy['tie_absolute_nrmse']), key=lambda r: r['ridge'])
    if str(candidate) != winner['candidate']:
        raise ValueError('Candidate is not the selection-only winner')
    freeze = dict(schema_version=1, protocol_sha256=file_hash(root / 'protocol.json'),
        input_sha256={selection_report: file_hash(_local(root, selection_report)), str(candidate): winner['candidate_sha256'],
                      **{f'role_inputs/{s}.json': report['input_manifests'][s] for s in ('train', 'selection')}, **evidence_hashes},
        candidate=str(candidate), ridge=winner['ridge'], selection_score=float(np.mean(winner['selection_axis_nrmse'])),
        selection_rule='equal trajectory/axis mean; absolute tie tolerance; stronger regularization wins',
        test_consumed=False, final_test_passed=False, p0_acceptance_passed=False,
        publishable=False, allowed_use='frozen candidate for independent R4 test; no deployment acceptance',
        attribution=_read(_local(root,winner['evidence']['attribution'])).get('selection_attribution'))
    return _seal(root, 'candidate_freeze.json', freeze)


def load_frozen_candidate(root):
    load_protocol(root)
    frozen = _unseal(root, 'candidate_freeze.json')
    if frozen['protocol_sha256'] != file_hash(Path(root) / 'protocol.json') or frozen['test_consumed']:
        raise ValueError('Invalid final-test freeze')
    for path, expected in frozen['input_sha256'].items():
        if file_hash(_local(root, path)) != expected:
            raise ValueError('Frozen candidate/selection inputs changed')
    return frozen


def p0_status(config):
    return dict(protocol=PROTOCOL if config.get('schema_version') == 2 else 'legacy_staged_v1',
        status='pending_R1_R4' if config.get('schema_version') == 2 else 'not_executed',
        acceptance_passed=False, reason='No complete P0 acceptance evidence; historical stage gates do not qualify P0')


def require_legacy_execution(config):
    if config.get('schema_version', 1) != 1 or 'p0' in config:
        raise ValueError('P0 R0 is contract-only: use plan-staged; R1-R4 execution is not implemented')
