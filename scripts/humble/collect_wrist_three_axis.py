#!/usr/bin/env python3
"""Collect the selected four train-only wrist references with the native Mock."""
import argparse
import json
from pathlib import Path
from shutil import copyfile

import numpy as np

from casbot_calib import _core
from casbot_calib.data import write_json
from casbot_calib.model_runtime import collision_for
from casbot_calib.staged_attribution import formal_test_untouched
from casbot_calib.staged_collection import load_trial, synthetic_plant
from casbot_calib.staged_development import _collect_order, collection_lock, frozen_predecessors, predecessor_model
from casbot_calib.staged_identification import read_record
from casbot_calib.staged_protocol import _read, _seal, _unseal, file_hash, load_protocol
from casbot_calib.staged_wrist_supplement import VERSION

MODE = 'mock_wrist_periodic_capture'
AXES = [4, 5, 6]
BINS = [7, 11, 17]
REFERENCE_KEYS = {'motion.json': 'motion_sha256', 'reference.npz': 'reference_sha256',
                  'collision_reference.json': 'collision_sha256'}


def read_certified_reference(original):
    ref = _read(original/'manifest.json')
    if any(file_hash(original/name)!=ref[key] for name,key in REFERENCE_KEYS.items()):
        raise ValueError('Offline reference bundle differs from its continuous proof')
    meta = _read(original/'motion.json')
    gate = _read(original/'collision_reference.json')
    if not gate['passed'] or not ref['reference_gate_passed'] or gate['motion']!=meta['name']:
        raise ValueError('Offline reference lacks its continuous path gate')
    return meta,gate


def copy_reference(root, original, name, meta, gate):
    target=root/'plans'/name
    target.mkdir(parents=True)
    copyfile(original/'reference.npz',target/'reference.npz')
    write_json(target/'motion.json',meta)
    write_json(target/'collision_reference.json',{**gate,'motion':name})
    write_json(target/'manifest.json',dict(reference_gate_passed=True,
        **{key:file_hash(target/filename) for filename,key in REFERENCE_KEYS.items()}))
    return {str(f.relative_to(root)):file_hash(f) for f in target.iterdir()}


def freeze_capture(source,design,root,public,protocol,profile,order,files,mode,stage):
    # A capture-only companion uses the native collector, without masquerading
    # as a P0 fitting session or another acquisition role.
    config={k:v for k,v in public.items() if k!='p0'}
    config.update(schema_version=1,mode=mode)
    write_json(root/'session.json',dict(config=config,collision=profile.metadata,
        joint_names=_read(source/'session.json')['joint_names'],synthetic_only=True,hardware_executable=False))
    write_json(root/'plan_report.json',dict(passed=True,selected_trials=order,stage=stage))
    files.update({n:file_hash(root/n) for n in ('session.json','plan_report.json')})
    return _seal(root,'capture_protocol.json',dict(schema_version=1,source_session=str(source),
        source_protocol_sha256=file_hash(source/'protocol.json'),
        source_train_manifest_sha256=file_hash(source/'role_inputs/train.json'),
        source_predecessor_freeze_sha256=file_hash(source/'predecessor_freeze.json'),
        native_binary_sha256=protocol['native_binary_sha256'],design_report_sha256=file_hash(design/'report.json'),
        order=order,files=files,role='train',stage=stage,formal_test_consumed=False,hardware_executable=False))


def prepare(source, design, root, public, protocol, manifest, profile):
    proposal = _read(design/'report.json')
    if proposal['chosen']['label'] != 'two_centers_mux_equal_record' or proposal['chosen']['new_records'] != 4:
        raise ValueError('Expected the selected four-record wrist design')
    coverage = _read(source/'configuration_coverage.json')['centers']
    expected = {'train_c00': (0., .3), 'train_c01': (-.3, .3)}
    plans = {p['group']: p for p in proposal['plans']}
    if len(proposal['plans']) != 2 or set(plans) != set(expected):
        raise ValueError('Expected two distinct registered train groups')
    base = public['p0']['roles']['train']
    order = []
    files = {}
    root.mkdir(parents=True, exist_ok=False)
    for batch in range(2):
        for group, angles in expected.items():
            p = plans[group]
            original = design/'plans'/p['name']
            meta,gate = read_certified_reference(original)
            pose = coverage[group].copy();pose[5:7] = angles
            c = meta['conditions']
            if (p['center_rad'] != pose or meta['states'][2][0] != pose
                    or c['active_joints'] != AXES or c['harmonics'] != BINS
                    or c['period_s'] != public['dynamic_period_s']
                    or meta['duration_s'] != p['duration_s']):
                raise ValueError('Offline wrist reference does not match selected pose/frequencies')
            center = int(group[-2:])
            name = f'{group}_wrist_mux_b{batch:02}'
            seed = base['acquisition_seed_base'] + center*1000 + batch*100 + 8
            ref_seed = base['reference_seed_base'] + center*100 + 8
            if any(r['acquisition_seed'] == seed or r['reference_seed'] == ref_seed for r in manifest['trials']):
                raise ValueError('Wrist supplement must use unused seeds')
            order.append(name)
            meta.update(name=name, stage='wrist_periodic')
            c.update(split='train', center_id=group, reference_id=f'{group}_wrist_mux',
                     reference_seed=ref_seed, acquisition_batch_id=f'train_batch_{batch:02}',
                     acquisition_seed=seed, repeat_id=batch, active_arm_group='left',
                     center_rad=pose, sample_period_ns=public['inertia_sample_period_ns'])
            files.update(copy_reference(root,original,name,meta,gate))
    return freeze_capture(source,design,root,public,protocol,profile,order,files,MODE,'wrist_periodic')


def summarize(root, name, config, policy, joint_names):
    meta,curve = load_trial(root,name)
    record,obs = read_record(root,name,meta,joint_names)
    if (record['control_period_ns'] != config['control_period_ns']
            or record['sample_period_ns'] != config['inertia_sample_period_ns']):
        raise ValueError('Wrist capture sample/command period changed')
    label = meta['labels'].index('periodic')
    period = meta['conditions']['period_s']
    begin = round((meta['knots_s'][label]+2*period)*1e9)
    per_cycle = round(period*1e9)//record['sample_period_ns']
    indexes = np.searchsorted(obs['sample_ns'],begin)+np.arange(2*per_cycle)
    q = obs['q'][indexes];t=obs['sample_ns'][indexes]*1e-9
    position_error=q-curve(t)
    q = q.reshape(2,per_cycle,14).mean(axis=0)
    amplitude = 2*abs(np.fft.rfft(q,axis=0))/per_cycle
    amps = amplitude[BINS,AXES]
    omega=2*np.pi*np.asarray(BINS)/period
    speed=amps*omega/np.sqrt(2)
    acceleration=amps*omega**2/np.sqrt(2)
    qualified = ((speed >= policy['minimum_speed_bin_rms_rad_s']) &
                 (acceleration >= policy['minimum_acceleration_bin_rms_rad_s2']))
    torque = obs['current'][indexes][:,AXES]*config['drive']['gain_nm_a']
    torque = torque.reshape(2,per_cycle,3).mean(axis=0)
    torque_rms = np.sqrt(2)*abs(np.fft.rfft(torque,axis=0))[BINS,np.arange(3)]/per_cycle
    return dict(trial_id=name, conditions=meta['conditions'], actual_path_passed=record['actual_interpolant_gate_passed'],
        joint_position_fundamental_amplitude_rad=amps, speed_bin_rms_rad_s=speed,
        acceleration_bin_rms_rad_s2=acceleration, torque_bin_rms_nm=torque_rms,
        tracking_peak_rad=np.max(abs(position_error),axis=0),
        all_axes_kinematic_signal_present=bool(qualified.all()))


def capture(source, design, root, resume=False, *, prepare_capture=prepare, summarize_trial=summarize,
            stage='wrist_periodic'):
    source, design, root=(Path(p).resolve() for p in (source,design,root))
    if root.is_relative_to(source) or root.is_relative_to(design):
        raise ValueError('Capture must use a separate output')
    protocol,public,manifest=load_protocol(source)
    if protocol['experiment_version']!=VERSION or not _read(source/'workflow.json')['passed']:
        raise ValueError('Expected accepted yaw-supplement R1 development inputs')
    formal_test_untouched(source)
    native=file_hash(_core.__file__)
    profile=collision_for(public,scene=_read(source/'session.json')['collision']['scene'])
    if native != protocol['native_binary_sha256'] or profile.metadata != _read(source/'session.json')['collision']:
        raise ValueError('Collection requires the same DVT2 native model and collision scene')
    prior=frozen_predecessors(source)
    if resume:
        frozen=_unseal(root,'capture_protocol.json')
        if (frozen['stage']!=stage or frozen['source_session']!=str(source) or frozen['source_protocol_sha256']!=file_hash(source/'protocol.json')
                or frozen['source_train_manifest_sha256']!=file_hash(source/'role_inputs/train.json')
                or frozen['source_predecessor_freeze_sha256']!=file_hash(source/'predecessor_freeze.json')
                or frozen['design_report_sha256']!=file_hash(design/'report.json')
                or any(file_hash(root/p)!=digest for p,digest in frozen['files'].items())):
            raise ValueError('Capture plan/source changed')
    else:
        frozen=prepare_capture(source,design,root,public,protocol,manifest,profile)
    session=_read(root/'session.json');config=session['config']
    plant=synthetic_plant({**config,'mock':_read(source/'evaluation/plant_config.json')})
    feedforward=predecessor_model(prior,public)
    with collection_lock(root):
        _collect_order(root,frozen['order'],plant,feedforward,config,profile,resume)
        journal=[json.loads(line) for line in (root/'execution_order.jsonl').read_text().splitlines()]
        if [e['trial_id'] for e in journal if e['event']=='completed'] != frozen['order']:
            raise ValueError('Capture journal does not match frozen order')
        rows=[summarize_trial(root,n,config,public['p0']['frequency_qualification'],session['joint_names']) for n in frozen['order']]
        formal_test_untouched(source)
        files={str(p.relative_to(root)):file_hash(p) for n in frozen['order'] for p in (root/'runs'/n).iterdir() if p.is_file()}
        manifest=dict(schema_version=1, split='train', stage=stage,
            capture_protocol_sha256=file_hash(root/'capture_protocol.json'), files=files,
            trials=frozen['order'], formal_test_consumed=False)
        if (root/'train_capture_manifest.json').exists():
            if _unseal(root,'train_capture_manifest.json') != manifest:
                raise ValueError('Completed capture evidence changed')
        else:
            _seal(root,'train_capture_manifest.json',manifest)
        report=dict(passed=all(r['all_axes_kinematic_signal_present'] for r in rows),
            records=len(rows), trials=rows, scope='capture and signal diagnostics only; no new fit or R1/R2 acceptance',
            formal_test_consumed=False, hardware_executable=False)
        write_json(root/'report.json',report)
        return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path);parser.add_argument('--design',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path);parser.add_argument('--resume',action='store_true')
    args=parser.parse_args()
    report=capture(args.source,args.design,args.output,args.resume)
    print(f"{report['records']} train records collected; signal check {'PASS' if report['passed'] else 'FAIL'}",flush=True)
    raise SystemExit(0 if report['passed'] else 1)
