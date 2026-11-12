#!/usr/bin/env python3
"""Collect only the eight positive-roll right-pitch probes with the native Mock."""
import argparse
from pathlib import Path

import numpy as np

from collect_wrist_three_axis import capture, copy_reference, freeze_capture, read_certified_reference
from diagnose_right_wrist_pitch import PITCH, ROLL
from casbot_calib.staged_collection import load_trial
from casbot_calib.staged_identification import read_record
from casbot_calib.staged_protocol import _local, _read

STAGE='right_pitch_probe'
POSES={'train_c00':.30,'train_c01':.15}
HELD=[j for j in range(14) if j!=PITCH]


def prepare(source,design,root,public,protocol,manifest,profile):
    report=_read(design/'report.json')
    choice=report['selected_positive']
    if (dict(choice['keys'])!=POSES or choice['added_records']!=8
            or report['axis']!=PITCH or report['held_roll_axis']!=ROLL):
        raise ValueError('Expected the approved eight positive-roll right-pitch references')
    positive=[p for p in report['plans'] if p['role']=='train_candidate']
    plans={(p['center'],p['direction']):p for p in positive}
    if len(positive)!=4 or set(plans)!={(c,d) for c in POSES for d in (-1,1)}:
        raise ValueError('Expected exactly one positive-roll plan for each center and direction')
    coverage=_read(source/'configuration_coverage.json')['centers']
    base=public['p0']['roles']['train']
    prepared=[]
    for batch in range(2):
        for (center,direction),plan in sorted(plans.items()):
            pose=coverage[center].copy();pose[ROLL]=POSES[center]
            original=_local(design,f"plans/{plan['name']}")
            meta,gate=read_certified_reference(original)
            c=meta['conditions']
            if (plan['pose_rad']!=pose or c['center_rad']!=pose or plan['roll_rad']!=POSES[center]
                    or c['joint']!=PITCH or c['direction']!=direction or c['role']!='train_candidate'
                    or c['frequency_hz']!=public['wrist_probe']['frequency_hz']
                    or c['sample_period_ns']!=public['inertia_sample_period_ns']):
                raise ValueError('Reference changed from approved right-pitch design')
            index=int(center[-2:])
            offset=9+(direction==1)  # 8 belongs to the existing three-axis records
            ref_seed=base['reference_seed_base']+index*100+offset
            seed=base['acquisition_seed_base']+index*1000+batch*100+offset
            if any(r['acquisition_seed']==seed or r['reference_seed']==ref_seed for r in manifest['trials']):
                raise ValueError('New pitch probes require independent unused seeds')
            ref_id=f'{center}_right_pitch_{"pos" if direction>0 else "neg"}'
            name=f'{ref_id}_b{batch:02}'
            meta.update(name=name,stage=STAGE)
            c.update(split='train',center_id=center,reference_id=ref_id,reference_seed=ref_seed,
                acquisition_batch_id=f'train_batch_{batch:02}',acquisition_seed=seed,repeat_id=batch,active_arm_group='right')
            prepared.append((original,name,meta,gate))
    root.mkdir(parents=True,exist_ok=False)
    files={}
    for original,name,meta,gate in prepared:
        files.update(copy_reference(root,original,name,meta,gate))
    order=[item[1] for item in prepared]
    return freeze_capture(source,design,root,public,protocol,profile,order,files,'mock_right_pitch_capture',STAGE)


def harmonic_rms(t, values, frequency):
    t=t-t.mean()
    angle=2*np.pi*frequency*t
    X=np.column_stack([np.ones_like(t),t,t*t,np.sin(angle),np.cos(angle)])
    c=np.linalg.lstsq(X,values,rcond=None)[0]
    return float(np.linalg.norm(c[-2:])/np.sqrt(2.))


def summarize(root,name,config,policy,joint_names):
    meta,curve=load_trial(root,name)
    record,obs=read_record(root,name,meta,joint_names)
    c=meta['conditions'];sign=c['direction']
    if (record['control_period_ns']!=config['control_period_ns']
            or record['sample_period_ns']!=config['inertia_sample_period_ns']):
        raise ValueError('Capture clock changed')
    slots=np.flatnonzero(np.array(meta['labels'])=='probe_measure')
    begin,end=curve.x[slots[0]],curve.x[slots[-1]+1]
    t=obs['sample_ns']*1e-9
    measure=(t>=begin)&(t<end)
    trim=(t>=begin+.2)&(t<end-.2)
    q=obs['q'][measure]; v=obs['dq'][measure]
    minimum=float(np.min(sign*v[:,PITCH]))
    held_velocity=float(np.max(abs(v[:,HELD])))
    tau=obs['current'][trim,PITCH]*config['drive']['gain_nm_a']
    frequency=c['frequency_hz']
    torque_bin=harmonic_rms(t[trim],tau,frequency)
    speed_bin=harmonic_rms(t[trim],obs['dq'][trim,PITCH],frequency)
    hold_error=np.max(abs(q[:,HELD]-np.asarray(c['center_rad'])[HELD]),axis=0)
    checks=dict(sustained_speed=minimum>.02,other_axes_held=held_velocity<=.001,
        torque_signal_present=torque_bin>=policy['minimum_bin_rms_nm'],
        speed_signal_present=speed_bin>=policy['minimum_speed_bin_rms_rad_s'])
    return dict(trial_id=name,conditions=c,actual_path_passed=record['actual_interpolant_gate_passed'],
        measurement_samples=int(measure.sum()),minimum_signed_pitch_speed_rad_s=minimum,
        maximum_other_axis_speed_rad_s=held_velocity,maximum_other_axis_position_error_rad=float(hold_error.max()),
        maximum_other_axis_position_span_rad=float(np.max(np.ptp(q[:,HELD],axis=0))),
        observed_roll_mean_rad=float(q[:,ROLL].mean()),
        roll_maximum_position_error_rad=float(np.max(abs(q[:,ROLL]-c['center_rad'][ROLL]))),
        pitch_speed_bin_rms_rad_s=speed_bin,pitch_torque_bin_rms_nm=torque_bin,
        torque_signal_floor_nm=policy['minimum_bin_rms_nm'],frequency_hz=frequency,
        torque_signal_scope='total synthetic actuator output at own bin after removing quadratic trend; not isolated inertia torque',
        checks=checks,all_axes_kinematic_signal_present=all(checks.values()))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path);p.add_argument('--design',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--resume',action='store_true')
    a=p.parse_args()
    report=capture(a.source,a.design,a.output,a.resume,
                   prepare_capture=prepare,summarize_trial=summarize,stage=STAGE)
    print(f"{report['records']} right-pitch train records; checks {'PASS' if report['passed'] else 'REJECT'}",flush=True)
    raise SystemExit(0 if report['passed'] else 1)
