"""Attach a sealed, train-only narrow-band wrist companion without relabeling captures."""
from copy import deepcopy
from pathlib import Path
from shutil import copyfile

from .data import write_json
from .staged_attribution import formal_test_untouched
from .staged_collection import load_trial
from .staged_development import collection_lock, frozen_predecessors
from .staged_identification import read_record
from .staged_protocol import (_local, _read, _seal, _unseal, file_hash, load_protocol,
                              prepare_protocol, seal_role_records)
from .staged_wrist_supplement import VERSION

AXES = [4, 5, 6]
BINS = [7, 11, 17]
POLICY = dict(active_joints=AXES, harmonics=BINS, normalization_nm=.2,
              baseline_probe_count=32, squared_weight_per_axis=1/3)


def validate_policy(p0):
    if (p0['experiment_version'] != VERSION or 'wrist_supplement' not in p0
            or p0['wrist_periodic_train'] != POLICY):
        raise ValueError('Unsupported wrist-periodic development policy')


def companion(root, *, binding_name='wrist_periodic_input.json', stage='wrist_periodic'):
    """Resolve the bound capture; never interpret it as a legacy fit session."""
    binding = _read(Path(root)/binding_name)
    capture = Path(binding['capture'])
    if (file_hash(capture/'capture_protocol.json') != binding['capture_protocol_sha256']
            or file_hash(capture/'train_capture_manifest.json') != binding['train_capture_manifest_sha256']):
        raise ValueError('Wrist companion protocol or manifest changed')
    frozen = _unseal(capture,'capture_protocol.json')
    role = _unseal(capture,'train_capture_manifest.json')
    if (role['split'] != 'train' or role['stage'] != stage or frozen['stage'] != stage
            or role['capture_protocol_sha256'] != binding['capture_protocol_sha256']
            or role['trials'] != frozen['order'] or role['trials'] != binding['trials']):
        raise ValueError('Wrist capture lacks its declared train binding')
    for name, digest in (frozen['files'] | role['files']).items():
        if file_hash(_local(capture,name)) != digest:
            raise ValueError('Wrist capture evidence changed: '+name)
    return capture, role['trials']


def wrist_records(root):
    capture, order = companion(root)
    names = _read(Path(root)/'session.json')['joint_names']
    for name in order:
        meta,_ = load_trial(capture,name)
        conditions = meta['conditions']
        if (meta['stage'] != 'wrist_periodic' or conditions['split'] != 'train'
                or conditions['active_joints'] != AXES or conditions['harmonics'] != BINS):
            raise ValueError('Wrist record is outside its train-only excitation contract')
        _,obs = read_record(capture,name,meta,names)
        yield name,meta,obs


def plan_integration(source, capture, output, *, right_pitch=None, left_support=None, sustained_pitch_support=None,
                     wrist_primary_support=None):
    source,capture,output = (Path(p).resolve() for p in (source,capture,output))
    if any(output.is_relative_to(p) for p in (source,capture)):
        raise ValueError('Use a new integration directory outside the input sessions')
    parent,public,manifest = load_protocol(source)
    formal_test_untouched(source)
    adding_right=right_pitch is not None
    adding_wrist=wrist_primary_support is not None
    upgrading=sustained_pitch_support is not None or adding_wrist
    if (parent['experiment_version'] != VERSION or ('wrist_periodic_train' in public['p0'])!=(adding_right or upgrading)
            or ('right_pitch_train' in public['p0'])!=upgrading or (adding_right and upgrading)
            or ('pitch_sustained_train' in public['p0'])!=adding_wrist
            or 'wrist_sustained_train' in public['p0']
            or (adding_wrist and sustained_pitch_support is not None)
            or (not upgrading and not _read(source/'workflow.json')['passed'])
            or not _read(capture/'report.json')['passed']):
        raise ValueError('Expected accepted yaw development and completed four-record wrist capture')
    frozen = _unseal(capture,'capture_protocol.json')
    source_hash = file_hash(source/'protocol.json')
    # A preprocessing-only child can retain a rejected parent's immutable data;
    # it receives no acceptance from that parent and must refit/reverify R1.
    parent_yaw=(Path(frozen['source_session']) if upgrading else
                Path(_read(source/'wrist_supplement_reuse.json')['source']) if adding_right else source)
    yaw_hash=file_hash(parent_yaw/'protocol.json')
    if (Path(frozen['source_session']) != parent_yaw or frozen['source_protocol_sha256'] != yaw_hash
            or frozen['source_train_manifest_sha256'] != file_hash(parent_yaw/'role_inputs/train.json')
            or frozen['source_predecessor_freeze_sha256'] != file_hash(parent_yaw/'predecessor_freeze.json')
            or frozen['native_binary_sha256'] != parent['native_binary_sha256']):
        raise ValueError('Wrist capture is bound to different development inputs')
    order = frozen['order']
    expected = [f'train_c{i:02}_wrist_mux_b{j:02}' for j in range(2) for i in range(2)]
    if order != expected:
        raise ValueError('Incomplete two-center, two-batch wrist schedule')
    binding = dict(capture=str(capture), trials=order, capture_protocol_sha256=file_hash(capture/'capture_protocol.json'),
                   train_capture_manifest_sha256=file_hash(capture/'train_capture_manifest.json'))
    records = {}
    for split in ('train','selection'):
        role = _unseal(source,f'role_inputs/{split}.json')
        if (role['split'] != split or role['protocol_sha256'] != source_hash
                or [r['trial_id'] for r in role['records']] != [r['trial_id'] for r in manifest['trials'] if r['split']==split]):
            raise ValueError('Source development roles changed')
        records.update((r['trial_id'],r['files']) for r in role['records'])
    old_reuse = _read(source/'wrist_supplement_reuse.json')
    acquisition = {name:old_reuse.get('acquisition_protocols',{}).get(name,old_reuse['source_protocol_sha256'])
                   if name in old_reuse['records'] else source_hash for name in records}
    config=deepcopy(public)
    config['p0']['wrist_periodic_train']=deepcopy(POLICY)
    if adding_right:
        from .staged_right_pitch import bind_right_pitch, RIGHT_POLICY
        right_binding=bind_right_pitch(parent_yaw,right_pitch,left_support)
        if binding!=_read(source/'wrist_periodic_input.json'):
            raise ValueError('Left companion differs from the existing development session')
        config['p0']['right_pitch_train']=deepcopy(RIGHT_POLICY)
    if upgrading:
        from .staged_pitch_sustained import POLICY as PITCH_POLICY
        from .staged_right_pitch import right_companion, left_sample_indices
        right_companion(source);left_sample_indices(source)
        if binding!=_read(source/'wrist_periodic_input.json'):
            raise ValueError('Preprocessing child must preserve the existing left companion')
        right_binding=_read(source/'right_pitch_input.json')
        if adding_wrist:
            from .staged_pitch_sustained import WRIST_POLICY
            config['p0']['wrist_sustained_train']=deepcopy(WRIST_POLICY)
        else:
            config['p0']['pitch_sustained_train']=deepcopy(PITCH_POLICY)
    config['mock']=_read(source/'evaluation/plant_config.json')
    plan=_read(source/'motion_plan.json')

    def planner(root,new_config,new_manifest,profile):
        from . import _core
        if (profile.metadata != _read(source/'session.json')['collision']
                or file_hash(_core.__file__) != frozen['native_binary_sha256']):
            raise ValueError('Wrist analysis requires the capture model and native kernel')
        old = [{k:v for k,v in r.items() if k!='config_sha256'} for r in manifest['trials']]
        new = [{k:v for k,v in r.items() if k!='config_sha256'} for r in new_manifest['trials']]
        if old!=new or new_manifest['planned_execution_order']!=manifest['planned_execution_order']:
            raise ValueError('Wrist integration changed existing roles')
        for row,slot in zip(plan['trials'],new_manifest['trials']):
            if row['trial_id'] != slot['trial_id']:
                raise ValueError('Original references differ from role order')
            row['config_sha256']=slot['config_sha256']
        paths=dict(plan['files'])
        for row in records.values():
            for name,digest in row.items():
                if paths.setdefault(name,digest)!=digest:
                    raise ValueError('Original role and reference seals disagree')
        for name,digest in paths.items():
            dest=root/name;dest.parent.mkdir(parents=True,exist_ok=True)
            copyfile(_local(source,name),dest)
            if file_hash(dest)!=digest:
                raise ValueError('Imported record changed: '+name)
        plan['report'].update(status='development_only_wrist_periodic_companion_bound',
                              wrist_periodic_train_records=len(order), reused_development_trials=len(records))
        write_json(root/'motion_plan.json',plan)
        for name in ('development_reference.json','configuration_coverage.json'):
            copyfile(source/name,root/name)
        write_json(root/'wrist_supplement_reuse.json',dict(source=str(source),source_protocol_sha256=source_hash,
            records=records,acquisition_protocols=acquisition))
        write_json(root/'wrist_periodic_input.json',binding)
        inputs=['motion_plan.json','configuration_coverage.json','development_reference.json',
                'wrist_supplement_reuse.json','wrist_periodic_input.json']
        if adding_right or upgrading:
            write_json(root/'right_pitch_input.json',right_binding)
            inputs.append('right_pitch_input.json')
        if upgrading:
            copyfile(source/'right_pitch_sample_indices.npz' if adding_wrist else sustained_pitch_support,
                     root/'right_pitch_sample_indices.npz')
            inputs.append('right_pitch_sample_indices.npz')
        if adding_wrist:
            copyfile(wrist_primary_support,root/'wrist_primary_sample_indices.npz')
            inputs.append('wrist_primary_sample_indices.npz')
        return plan['report'],inputs

    prepare_protocol(output,config,planner=planner)
    # Validate the four acquired identities and their disjoint acquisition seeds before any fit.
    previous_seeds={r['acquisition_seed'] for r in manifest['trials']}
    seen=set()
    for name,meta,_ in wrist_records(output):
        c=meta['conditions'];center,batch=name.split('_wrist_mux_b')
        if (c['center_id']!=center or c['repeat_id']!=int(batch)
                or c['acquisition_batch_id']!=f'train_batch_{batch}' or c['period_s']!=public['dynamic_period_s']
                or c['acquisition_seed'] in previous_seeds or c['acquisition_seed'] in seen):
            raise ValueError('Wrist record does not belong to its registered independent repeat')
        seen.add(c['acquisition_seed'])
    if adding_right or upgrading:
        from .staged_right_pitch import right_records, left_sample_indices
        if set(left_sample_indices(output))!=set(order):
            raise ValueError('Frozen left raw support is incomplete')
        for _,meta,_ in right_records(output):
            seed=meta['conditions']['acquisition_seed']
            if seed in previous_seeds or seed in seen:
                raise ValueError('Right pitch seed repeats a development observation')
            seen.add(seed)
    for split in ('train','selection'):
        seal_role_records(output,split)
    prior=frozen_predecessors(source)
    target=output/'development_analysis'
    from shutil import copytree
    copytree(prior,target)
    _seal(output,'predecessor_freeze.json',dict(schema_version=1,protocol_sha256=file_hash(output/'protocol.json'),
        analysis_folder=target.name,files={str(p.relative_to(output)):file_hash(p) for p in target.rglob('*') if p.is_file()},
        scope='unchanged frozen development predecessors from yaw supplement'))
    return output


def fit_integration(root):
    from .staged_multifit import identify_development
    root=Path(root).resolve();_,config,manifest=load_protocol(root)
    validate_policy(config['p0']);formal_test_untouched(root)
    output=root/'identification_development'
    with collection_lock(root):
        report=identify_development(root,output,frozen_predecessors(root))
        _seal(root,'development_fit_freeze.json',dict(schema_version=1,files={
            str(p.relative_to(root)):file_hash(p) for p in output.rglob('*') if p.is_file()}))
        result=dict(round='R1',passed=report['development_passed'],checkpoint_passed=report['development_passed'],
            completed_through='R1_development',inertia_folder=output.name,reused_development_trials=sum(r['split']!='test' for r in manifest['trials']),
            wrist_periodic_train_records=len(_read(root/'wrist_periodic_input.json')['trials']),
            collected_trials=0,formal_test_consumed=False,synthetic_only=True,hardware_executable=False,
            p0_acceptance_passed=False)
        if 'right_pitch_train' in config['p0']:
            result['right_pitch_train_records']=len(_read(root/'right_pitch_input.json')['trials'])
        for name in ('checkpoint.json','workflow.json'):
            write_json(root/name,result)
    return result
