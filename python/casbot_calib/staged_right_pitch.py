"""Read-only binding for the eight approved right-pitch training probes."""
from pathlib import Path

import numpy as np

from .staged_collection import load_trial
from .staged_identification import read_record
from .staged_protocol import _read, _unseal, file_hash
from .staged_wrist_periodic import companion

STAGE='right_pitch_probe'
JOINT=12
POSES={'train_c00':.30,'train_c01':.15}
RIGHT_POLICY=dict(active_joint=JOINT,normalization_nm=.2,baseline_probe_count=32,left_sigma_ms=1.5)


def validate_right_policy(p0):
    if 'wrist_periodic_train' not in p0 or p0['right_pitch_train']!=RIGHT_POLICY:
        raise ValueError('Unsupported right-pitch companion policy')


def bind_right_pitch(yaw, capture, left_support):
    yaw,capture,left_support=(Path(p).resolve() for p in (yaw,capture,left_support))
    frozen=_unseal(capture,'capture_protocol.json')
    expected=[f'{center}_right_pitch_{direction}_b{batch:02}'
              for batch in range(2) for center in POSES for direction in ('neg','pos')]
    if (frozen['stage']!=STAGE or frozen['role']!='train' or frozen['order']!=expected
            or Path(frozen['source_session'])!=yaw or frozen['source_protocol_sha256']!=file_hash(yaw/'protocol.json')
            or frozen['source_train_manifest_sha256']!=file_hash(yaw/'role_inputs/train.json')
            or frozen['source_predecessor_freeze_sha256']!=file_hash(yaw/'predecessor_freeze.json')
            or frozen['native_binary_sha256']!=_unseal(yaw,'protocol.json')['native_binary_sha256']
            or not _read(capture/'report.json')['passed']):
        raise ValueError('Expected the approved positive-roll captures bound to the yaw source')
    return dict(capture=str(capture),trials=expected,capture_protocol_sha256=file_hash(capture/'capture_protocol.json'),
                train_capture_manifest_sha256=file_hash(capture/'train_capture_manifest.json'),
                left_support=str(left_support),left_support_sha256=file_hash(left_support))


def left_sample_indices(root):
    binding=_read(Path(root)/'right_pitch_input.json')
    source=Path(binding['left_support'])
    if file_hash(source)!=binding['left_support_sha256']:
        raise ValueError('Fixed left-wrist raw support changed')
    with np.load(source,allow_pickle=False) as data:
        return {k:data[k] for k in data.files}


def right_companion(root):
    return companion(root,binding_name='right_pitch_input.json',stage=STAGE)


def right_records(root):
    capture,order=right_companion(root)
    session=_read(Path(root)/'session.json')
    for name in order:
        meta,_=load_trial(capture,name)
        c=meta['conditions'];center=c['center_id'];batch=c['repeat_id']
        sign='pos' if c['direction']==1 else 'neg'
        if (meta['stage']!=STAGE or c['split']!='train' or center not in POSES
                or c['center_rad'][13]!=POSES[center] or c['joint']!=JOINT
                or c['direction'] not in (-1,1) or batch not in (0,1)
                or c['active_arm_group']!='right' or c['acquisition_batch_id']!=f'train_batch_{batch:02}'
                or name!=f'{center}_right_pitch_{sign}_b{batch:02}'):
            raise ValueError('Right pitch record is outside the approved roles')
        record,obs=read_record(capture,name,meta,session['joint_names'])
        if (record['sample_period_ns']!=session['config']['inertia_sample_period_ns']
                or record['control_period_ns']!=session['config']['control_period_ns']):
            raise ValueError('Right pitch acquisition clock changed')
        yield name,meta,obs


def rejected_r1_evidence(root):
    """Seal an explicit rejection for diagnostics, never as development eligibility."""
    from .staged_protocol import load_protocol
    root=Path(root);_,config,_=load_protocol(root)
    workflow=_read(root/'workflow.json')
    path=root/'identification_development/report.json';report=_read(path)
    if ('right_pitch_train' not in config['p0'] or workflow.get('round')!='R1'
            or workflow.get('passed') is not False or report.get('development_passed') is not False
            or report['protocol_sha256']!=file_hash(root/'protocol.json')
            or report['input_manifests']!={s:file_hash(root/f'role_inputs/{s}.json') for s in ('train','selection')}):
        raise ValueError('No bound rejected right-pitch R1 evidence')
    return dict(workflow_sha256=file_hash(root/'workflow.json'),report_sha256=file_hash(path))
