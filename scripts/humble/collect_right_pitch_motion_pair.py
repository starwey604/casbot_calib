#!/usr/bin/env python3
"""Capture the sealed c02 A/B diagnostic with the native local Mock only."""
import argparse
import json
from pathlib import Path

from collect_wrist_three_axis import copy_reference, read_certified_reference
from casbot_calib import _core
from casbot_calib.data import write_json
from casbot_calib.model_runtime import collision_for
from casbot_calib.staged_attribution import formal_test_untouched
from casbot_calib.staged_collection import synthetic_plant
from casbot_calib.staged_development import _collect_order, collection_lock, frozen_predecessors, predecessor_model
from casbot_calib.staged_protocol import _read, _seal, file_hash, load_protocol

STAGE = 'right_pitch_pair_diagnostic'
PITCH, SHOULDER, ROLL = 12, 7, 13
CHECKS = dict(minimum_signed_speed_rad_s=.02, maximum_held_speed_rad_s=.001,
    maximum_hold_position_error_rad=.001, maximum_paired_wrist_position_difference_rad=.001,
    maximum_paired_filtered_speed_and_acceleration_relative_rms=.10)
SEED_BASE = 9_104_200


def capture(source, design, output):
    source,design,output=(p.resolve() for p in (source,design,output))
    if output.exists() or any(output.is_relative_to(p) for p in (source,design)):
        raise ValueError('Use a fresh diagnostic directory outside the source and reference')
    parent,public,manifest=load_protocol(source)
    formal_test_untouched(source)
    session=_read(source/'session.json')
    profile=collision_for(public,scene=session['collision']['scene'])
    native=file_hash(_core.__file__)
    if native!=parent['native_binary_sha256'] or profile.metadata!=session['collision']:
        raise ValueError('Use the recorded native DVT2 model and scene')
    report=_read(design/'report.json')
    review=_read(design/'verification.json')
    if (report['model_context']!=public['model_context'] or report['selected_axis']!=SHOULDER
            or report['proposed_record_count']!=8 or len(report['plans'])!=4
            or not review['reference_files_reconstructed'] or review['pair_controlled_axis']!=SHOULDER
            or report['hardware_executable'] or report['observations_collected']):
        raise ValueError('Expected the approved four offline diagnostic references')
    plans={row['name']:row for row in report['plans']}
    sequences=report['suggested_order_by_batch']
    if (len(sequences)!=2 or any(len(b)!=4 or set(b)!=plans.keys() for b in sequences)
            or {(p['direction'],tuple(p['active_joints'])) for p in plans.values()}
            !={(d,j) for d in (-1,1) for j in ((PITCH,),(PITCH,SHOULDER))}
            or any(sequences[0][k:k+2] != sequences[1][k:k+2][::-1] for k in (0,2))):
        raise ValueError('Expected balanced same-direction A/B pairs with order reversal')
    prior=frozen_predecessors(source)
    feedforward=predecessor_model(prior,public)
    config={k:v for k,v in public.items() if k!='p0'}
    config.update(schema_version=1,mode='mock_right_pitch_pair_diagnostic')
    # Only the native capture uses the same synthetic-plant declaration as the
    # original 004 run. The separate analysis process cannot read evaluation.
    plant=synthetic_plant({**config,'mock':_read(source/'evaluation/plant_config.json')})
    output.mkdir(parents=True,exist_ok=False)
    order=[]; files={}
    used={r['acquisition_seed'] for r in manifest['trials']}
    for batch,names in enumerate(sequences):
        for pos,original_name in enumerate(names):
            original=design/'plans'/original_name
            meta,gate=read_certified_reference(original);p=plans[original_name];c=meta['conditions']
            seed=SEED_BASE+4*batch+pos
            if (seed in used or c['active_joints']!=p['active_joints'] or c['direction']!=p['direction']
                    or c['center_rad']!=report['center_rad'] or c['joint']!=PITCH
                    or c['role']!='offline_diagnostic' or c['split'] is not None
                    or gate['diagnostic_derivative_bounds']!=p['derivative_bounds']):
                raise ValueError('Reference identity changed before capture')
            name=f'{original_name}_diag_b{batch:02}'
            meta.update(name=name,stage=STAGE)
            c.update(role='diagnostic',acquisition_seed=seed,reference_id=original_name,
                diagnostic_batch=batch,repeat_id=batch)
            files.update(copy_reference(output,original,name,meta,gate));order.append(name)
    write_json(output/'session.json',dict(config=config,collision=profile.metadata,
        joint_names=session['joint_names'],synthetic_only=True,hardware_executable=False))
    write_json(output/'plan_report.json',dict(passed=True,selected_trials=order,stage=STAGE))
    files.update({n:file_hash(output/n) for n in ('session.json','plan_report.json')})
    _seal(output,'capture_protocol.json',dict(schema_version=1,role='diagnostic',stage=STAGE,order=order,files=files,
        source=str(source),source_protocol_sha256=file_hash(source/'protocol.json'),
        source_predecessor_freeze_sha256=file_hash(source/'predecessor_freeze.json'),
        design=str(design),design_sha256=file_hash(design/'report.json'),design_verification_sha256=file_hash(design/'verification.json'),
        native_binary_sha256=native,comparison_checks=CHECKS,hardware_executable=False))
    with collection_lock(output):
        _collect_order(output,order,plant,feedforward,config,profile,False)
        journal=[json.loads(line) for line in (output/'execution_order.jsonl').read_text().splitlines()]
        if [r['trial_id'] for r in journal if r['event']=='completed']!=order:
            raise ValueError('Acquisition journal differs from the frozen order')
        files={str(p.relative_to(output)):file_hash(p) for name in order for p in (output/'runs'/name).iterdir() if p.is_file()}
        _seal(output,'diagnostic_capture_manifest.json',dict(schema_version=1,role='diagnostic',trials=order,files=files,
            capture_protocol_sha256=file_hash(output/'capture_protocol.json'),formal_test_consumed=False))
    formal_test_untouched(source)
    print(f'{len(order)} diagnostic Mock records captured',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ('source','design','output'):p.add_argument(arg,type=Path)
    capture(**vars(p.parse_args()))
