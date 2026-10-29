#!/usr/bin/env python3
"""Convert audited legacy development and fresh onset observations into S4."""
import argparse
import copy
import json
from pathlib import Path
import yaml
from verify_vendor_s3 import bundle
from casbot_calib.data import write_json
from casbot_calib.dynamics_data import import_dynamics
from casbot_calib.dynamics_qualification import qualify_dynamics
from casbot_calib.vendor_identity import sha256
from casbot_calib.vendor_staged import reference,freeze_staged


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--native',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    source=args.source.resolve();native=args.native.resolve();session=json.loads((source/'session.json').read_text())
    if session['config']['schema_version']!=1:raise ValueError('Legacy development only, not P0 final test')
    from casbot_calib.dual_model import nominal_dual_arm
    engine,model=nominal_dual_arm();names=[j.name for j in engine.model.joints]
    settings=copy.deepcopy(session['config']);settings['mode']='qualified_observations_only';settings.pop('schema_version')
    settings['drive']['torque_noise_nm']=settings['drive']['current_noise_a']*settings['drive']['gain_nm_a']
    records=[];checks=[]
    selected=[]
    for parent in (source,native):
        plan=json.loads((parent/'plan_report.json').read_text())
        if not plan['passed']:raise ValueError('Rejected source references')
        for name in plan['selected_trials']:
            motion=json.loads((parent/'plans'/name/'motion.json').read_text());stage=motion['stage']
            if (parent==source and stage in ('gravity','friction','inertia','inertia_probe')) or (parent==native and stage=='breakaway'):
                selected.append((parent,name,motion))
    for parent,name,motion in selected:
        from casbot_calib.staged_identification import read_record
        from casbot_calib.staged_collection import load_trial
        load_trial(parent,name)  # recheck the full reference/geometry manifest
        record,_=read_record(parent,name,motion,names)
        if motion['stage']=='breakaway' and record['sample_period_ns']!=250000:raise ValueError('Fresh raw4k onset required')
        payload=parent/'runs'/name/'observations.npz';stage=motion['stage'];experiment={}
        if stage=='inertia':
            indices=[i for i,label in enumerate(motion['labels']) if label=='periodic']
            experiment=dict(kind='periodic',begin_ns=round(motion['knots_s'][indices[0]]*1e9),end_ns=round(motion['knots_s'][indices[-1]+1]*1e9),
                period_s=motion['conditions']['period_s'],discard_periods=2,harmonics=40,phase_points=1024)
        fields=dict(position='q',velocity='dq',signal='current',receive='receive_ns',device_sequence='sequence',status='status')
        profile,contract,config=bundle(root/'evidence'/name,payload,fields,names,model['sha256'],experiment=experiment)
        config['requested_band_hz']=30. if stage=='inertia_probe' else 40/6
        profile.write_text(yaml.safe_dump(config))
        import_dynamics(payload,profile,root/'observations'/name,source_type='host')
        q=qualify_dynamics(root/'observations'/name,root/'qualifications'/name)
        conditions=motion['conditions'];split=('train' if conditions['repeat']==0 else 'validation') if stage=='breakaway' else conditions['batch']
        if stage=='inertia' and split in ('left_validation','right_validation'):split='validation'
        row=dict(name=name,stage=stage,split=split,time_origin_ns=0,qualification=reference(root/'qualifications'/name/'manifest.json'),
                 motion=reference(parent/'plans'/name/'motion.json'),reference_collision=reference(parent/'plans'/name/'collision_reference.json'),
                 actual_collision=reference(parent/'runs'/name/'collision_actual.json'),
                 source_kind='fresh_native_4k' if parent==native else 'reused_legacy_development_only')
        if stage=='breakaway':row['guard']=reference(parent/'runs'/name/'guard_events.json')
        records.append(row);checks.append(q['stage_data_eligibility'][stage]['observations_eligible'])
        print(f'S3导入 {name}: {checks[-1]}',flush=True)
    spec=dict(schema_version='casbot_vendor_s4_spec_v1',scope='synthetic_observations',settings=settings,records=records,
              preflight=dict(mapping=reference(source/'direction_check.json'),timing=reference(source/'driver_timing.json')),
              limitations=['Legacy train/validation are reused software development inputs, never a new formal final test',
                           'Fresh onset varies pose/dwell/slope by split; it does not identify their separate effects',
                           'Synthetic scale reference and nominal geometry, no hardware qualification'])
    spec_path=root/'spec.yaml';spec_path.write_text(yaml.safe_dump(spec,sort_keys=False))
    result=freeze_staged(spec_path,root/'protocol')
    write_json(root/'preparation.json',dict(passed=all(checks) and result['passed'],records=len(records),hardware_executable=False,
        source_session_sha256=sha256(source/'session.json'),native_session_sha256=sha256(native/'session.json')))
    return 0 if all(checks) and result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
