"""Development-only extension of 004: reuse original roles and add eight yaw probes."""
from copy import deepcopy
from pathlib import Path
from shutil import copyfile


from .data import write_json
from .staged_attribution import formal_test_untouched
from .staged_multiconfiguration import _write_motion, motion_coverage, motion_for_slot
from .staged_motion import gate_motion
from .staged_protocol import _local, _read, _unseal, file_hash, load_protocol, prepare_protocol

VERSION = 'synthetic_dvt2_p0_004_wrist_supplement_v1'
ANGLES = {'train_c00': [.3, 0.], 'train_c01': [-.3, .3]}


def validate_supplement(p0):
    if (p0['experiment_version'] != VERSION or p0['planning_round'] != 'R1'
            or p0['wrist_supplement'] != {'held_pitch_roll_rad': ANGLES}):
        raise ValueError('Unsupported 004 wrist supplement declaration')


def plan_supplement(source, design, output):
    """Create a new session; original acquisition and trajectory files are never edited."""
    source, design, output = (Path(p).resolve() for p in (source, design, output))
    if output.is_relative_to(source):
        raise ValueError('Supplement must be outside the original session')
    protocol, public, old = load_protocol(source)
    if protocol['experiment_version'] != 'synthetic_dvt2_p0_004' or not _read(source/'workflow.json')['passed']:
        raise ValueError('An accepted original 004 R1 session is required')
    formal_test_untouched(source)
    proposal = _read(design)
    if Path(proposal['root']).resolve() != source or proposal['added_train_records'] != 8:
        raise ValueError('Wrist design belongs to another session or acquisition budget')
    old_plan = _read(source/'motion_plan.json')
    coverage = _read(source/'configuration_coverage.json')
    held_poses = {}
    for center, angles in ANGLES.items():
        pose = coverage['centers'][center].copy()
        pose[5:7] = angles
        held_poses[center] = pose
    if sorted(proposal['selected_poses'].values()) != sorted(held_poses.values()):
        raise ValueError('Wrist supplement angles differ from the evaluated design')
    old_digest = file_hash(source/'protocol.json')
    records = {}
    for split in ('train', 'selection'):
        role = _unseal(source, f'role_inputs/{split}.json')
        expected = [r['trial_id'] for r in old['trials'] if r['split'] == split]
        if (role['protocol_sha256'] != old_digest or role['split'] != split
                or [r['trial_id'] for r in role['records']] != expected):
            raise ValueError('Original development observations differ from the 004 role seal')
        records.update((row['trial_id'], row['files']) for row in role['records'])

    config = deepcopy(public)
    config['mock'] = _read(source/'evaluation/plant_config.json')
    config['p0'].update(experiment_version=VERSION, wrist_supplement={'held_pitch_roll_rad': ANGLES})

    def planner(root, public_config, manifest, profile):
        from . import _core
        if (file_hash(_core.__file__) != protocol['native_binary_sha256']
                or profile.metadata != _read(source/'session.json')['collision']):
            raise ValueError('Supplement requires the original 004 native model and collision profile')
        previous_slots = {r['trial_id']: r for r in old['trials']}
        previous_refs = {r['trial_id']: r for r in old_plan['trials']}
        resolved = {}
        new_slots = [r for r in manifest['trials'] if r.get('supplemental_wrist')]
        # Check all four references before copying or allowing acquisition.
        for slot in new_slots:
            name = slot['trial_id']
            center = coverage['centers'][slot['center_id']]
            motion = motion_for_slot(slot, center, public_config)
            score = motion_coverage(motion, public_config)
            gate = gate_motion(profile, motion, profile.metadata['scene']['clearance_m'])
            if not (score['passed'] and gate['passed']):
                raise ValueError('Supplement reference failed the original motion gate: ' + name)
            old_plan['files'].update(_write_motion(root, motion, gate))
            resolved[name] = {**slot, 'center_rad': center, 'probe_pose_rad': held_poses[slot['center_id']],
                              'reference_sha256': file_hash(root/'plans'/name/'reference.npz'),
                              'reference_status': 'continuous_reference_gate_passed'}
            coverage['trials'].append(dict(trial_id=name, **score))
        for slot in manifest['trials']:
            name = slot['trial_id']
            if name in resolved:
                continue
            if {k:v for k,v in slot.items() if k != 'config_sha256'} != {
                    k:v for k,v in previous_slots[name].items() if k != 'config_sha256'}:
                raise ValueError('Original role changed: ' + name)
            resolved[name] = {**previous_refs[name], 'config_sha256': slot['config_sha256']}
        # Include unchanged formal reference files only, never any formal observation.
        original_files = {p:d for p,d in old_plan['files'].items() if p.split('/')[1] not in {r['trial_id'] for r in new_slots}}
        observed_files = {}
        for row in records.values():
            for path, digest in row.items():
                if path.startswith('runs/'):
                    observed_files[path] = digest
                elif original_files[path] != digest:
                    raise ValueError('Source record/reference identity differs')
        for path, expected in (original_files | observed_files).items():
            target = root/path
            target.parent.mkdir(parents=True, exist_ok=True)
            copyfile(_local(source, path), target)
            if file_hash(target) != expected:
                raise ValueError('Copied 004 evidence differs from original seal: ' + path)

        old_plan['trials'] = [resolved[r['trial_id']] for r in manifest['trials']]
        old_plan['planned_execution_order'] = manifest['planned_execution_order']
        report = old_plan['report']
        report.update(selected_trials=list(resolved), wrist_trials=report['wrist_trials']+len(new_slots),
                      reused_development_trials=len(records), new_development_trials=len(new_slots),
                      status='supplement_references_certified_no_new_observations',
                      total_virtual_duration_s=report['total_virtual_duration_s'] + sum(
                          _read(root/'plans'/r['trial_id']/'motion.json')['duration_s'] for r in new_slots))
        report['selected_trials'] = [r['trial_id'] for r in manifest['trials']] + old_plan['final_prior_execution_order']
        write_json(root/'motion_plan.json', old_plan)
        write_json(root/'configuration_coverage.json', coverage)
        copyfile(source/'development_reference.json', root/'development_reference.json')
        write_json(root/'wrist_supplement_reuse.json', dict(source=str(source), source_protocol_sha256=old_digest,
            design=str(design), records=records, new_trial_ids=[r['trial_id'] for r in new_slots],
            scope='byte-identical 004 development copies; formal test observations excluded'))
        return report, ['motion_plan.json', 'configuration_coverage.json', 'development_reference.json', 'wrist_supplement_reuse.json']

    report = prepare_protocol(output, config, planner=planner)
    (output/'README.md').write_text('# 004 左腕补采开发协议\n\n'
        '保留原 36 条训练和 24 条选参记录及其采集协议；仅新增 8 条 train 探针。'
        '本协议只用于 R1/R2 开发，不采集正式测试。\n', encoding='utf-8')
    return config, report
