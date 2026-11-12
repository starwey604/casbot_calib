#!/usr/bin/env python3
"""Design paired non-reversing right-pitch probes from sealed train rows."""
import argparse
from itertools import product
from pathlib import Path

import numpy as np

from design_wrist_supplement import goal_noise_gain, mass_rows, protect_observations, reference_design
from diagnose_right_wrist_pitch import PITCH, ROLL
from diagnose_wrist_periodic_counterfactual import RIDGE, sealed_r2
from casbot_calib.data import write_json
from casbot_calib.model_runtime import collision_for, model_for
from casbot_calib.staged_attribution import arrays, formal_test_untouched
from casbot_calib.staged_motion import gate_motion, wrist_probe_motion
from casbot_calib.staged_multiconfiguration import _write_motion, motion_coverage
from casbot_calib.staged_multifit import assemble_training, load_problems
from casbot_calib.staged_protocol import _read, _unseal, file_hash, load_protocol

CENTERS = ('train_c00','train_c01')
ANGLES = (.15,.30,.45,.60)


def plan(root, analysis, diagnosis, output):
    root,analysis,diagnosis,output=(Path(p).resolve() for p in (root,analysis,diagnosis,output))
    if output.exists() or any(output.is_relative_to(p) for p in (root,analysis,diagnosis.parent)):
        raise ValueError('Use a fresh output outside sealed inputs')
    _,config,manifest=load_protocol(root)
    formal_test_untouched(root)
    protect_observations(root,{(r['trial_id'],) for r in manifest['trials'] if r['split']=='train'})
    r2=sealed_r2(root,analysis)
    setup=_unseal(analysis,'training_setup.json')
    if setup['files']['comparison_basis.npz']!=file_hash(analysis/'comparison_basis.npz'):
        raise ValueError('Original comparison coordinates changed')
    coord=arrays(analysis/'coordinates.npz');basis=arrays(analysis/'comparison_basis.npz')
    rank=setup['common_rank']
    project=(coord['scales'][:,None]*basis['basis'])/basis['singular_values'][:rank]
    training=load_problems(root,'train',coord['static'])
    y,x,_,weights=assemble_training(training,config)
    if weights!=setup['weights']:
        raise ValueError('Original train weights changed')
    design=np.column_stack([y,x])@project
    del y,x,training
    centers=sorted({w['center_id'] for w in weights})
    batches=sorted({w['acquisition_batch_id'] for w in weights})
    info={c:np.zeros((rank,rank)) for c in centers}
    per_batch={b:np.zeros((rank,rank)) for b in batches}
    begin=0
    for w in weights:
        end=begin+w['rows'];a=design[begin:end]
        gram=a.T@a
        info[w['center_id']]+=gram
        per_batch[w['acquisition_batch_id']]+=gram
        begin=end
    del design
    full=sum(info.values())
    if np.max(abs(full-np.eye(rank)))>1e-8:
        raise ValueError('Original train Gram differs')
    omitted={c:full-g for c,g in info.items()}
    nominal,_=model_for(config);profile=collision_for(config)
    probes=arrays(analysis/'operator_probe_set.npz')['q']
    baseline=arrays(analysis/'variants'/f'ridge_{RIDGE:g}_full'/'solution.npz')['mass'][:,PITCH,PITCH]
    index=_read(diagnosis)['worst_probe']['index']
    goals=mass_rows(nominal,probes,PITCH)@project[:140]/np.maximum(
        baseline,r2['settings']['mass_diagonal_floor_kg_m2'])[:,None]
    reference=goal_noise_gain(full,goals)
    fixed_poses=_read(root/'configuration_coverage.json')['centers']
    count=sum(w['stage']=='inertia_probe' for w in weights)
    normalizer=float(np.median([w['torque_normalizer_nm'] for w in weights if w['stage']=='inertia' and w['joint']==PITCH]))
    # The original 1 s transition exceeds the dedicated 1 rad/s envelope at c00.
    # Lengthen only approach/return; keep the signed measurement waveform fixed.
    probe_config={**config,'transition_s':1.5}
    options={}
    for c,angle,side in product(CENTERS,ANGLES,(1,-1)):
        pose=np.array(fixed_poses[c]);pose[ROLL]=side*angle
        key=(c,side*angle)
        group=f'{c}_right_pitch_roll{side*angle:+.2f}'
        motions=[wrist_probe_motion(f'{group}_dir{direction:+d}',config['home_rad'],pose,probe_config,PITCH,direction,
            dict(center_id=c,center_rad=pose.tolist(),role='train_candidate' if side>0 else 'negative_roll_control'))
            for direction in (-1,1)]
        gram=sum(reference_design(m,config,nominal,project,normalizer,count) for m in motions)
        options[key]=(gram,motions)

    def summarize(g):
        ratios=goal_noise_gain(g,goals)/reference
        return dict(maximum=float(ratios.max()), maximum_probe_index=int(ratios.argmax()),
                    at_previous_worst=float(ratios[index]))

    def evaluate(keys):
        by_center={c:summarize(g+len(batches)*sum(options[k][0] for k in keys if k[0]!=c))
                   for c,g in omitted.items()}
        by_batch={b:summarize(full-g+(len(batches)-1)*sum(options[k][0] for k in keys))
                  for b,g in per_batch.items()}
        return dict(keys=keys,added_records=2*len(batches)*len(keys),by_omitted_center=by_center,
                    by_omitted_batch=by_batch,worst=max(v['maximum'] for v in by_center.values()))

    gates={}
    def admissible(keys):
        # Every positive candidate must admit an exact negative-roll control.
        for c,angle in keys:
            for key in ((c,abs(angle)),(c,-abs(angle))):
                if key not in gates:
                    evaluated=[]
                    for m in options[key][1]:
                        gate=gate_motion(profile,m,profile.metadata['scene']['clearance_m'])
                        coverage=motion_coverage(m,config)
                        measure=np.array(m.labels)=='probe_measure'
                        speed=float(np.min(m.polynomial().derivative().c[:,measure,PITCH]*m.conditions['direction']))
                        evaluated.append((gate,coverage,speed))
                    gates[key]=evaluated
                if not all(g['passed'] and v['passed'] and s>.02 for g,v,s in gates[key]):
                    return False
        return True

    singles=sorted((evaluate(((c,a),)) for c,a in product(CENTERS,ANGLES)),key=lambda x:x['worst'])
    pairs=sorted((evaluate(tuple(zip(CENTERS,a))) for a in product(ANGLES,repeat=2)),key=lambda x:x['worst'])
    def choose(rows):
        return next((r for r in rows if admissible(r['keys'])),None)
    single=choose(singles);pair=choose(pairs)
    if pair is None:
        raise ValueError('No two-center positive design and matched negative control pass both gates')
    negative=tuple((c,-angle) for c,angle in pair['keys'])
    control=evaluate(negative)
    output.mkdir(parents=True,exist_ok=False)
    plans=[]
    for c,a in (*pair['keys'],*negative):
        for m,(gate,coverage,signed_speed) in zip(options[c,a][1],gates[c,a]):
            _write_motion(output,m,gate)
            plans.append(dict(name=m.name,center=c,roll_rad=a,role=m.conditions['role'],
                direction=m.conditions['direction'],pose_rad=m.conditions['center_rad'],duration_s=m.duration,
                minimum_sampled_distance_m=gate['minimum_distance_m'],
                derivative_bounds=coverage['peak_derivative_bounds'],
                minimum_continuous_signed_speed_rad_s=signed_speed,
                acceleration_rms_rad_s2=coverage['acceleration_rms_rad_s2'][PITCH]))
    formal_test_untouched(root)
    report=dict(scope='offline reference design, train observations only; no fit or collection',
        fixed_operator_probes=len(probes),previous_worst_probe_index=index,axis=PITCH,held_roll_axis=ROLL,
        groups=centers,batches=batches,probe=config['wrist_probe'],transition_s=probe_config['transition_s'],pitch_normalizer_nm=normalizer,
        existing_probe_count=count,weights='freeze existing rows; each proposed record uses one existing probe budget',
        baseline=evaluate(()),best_single=single,selected_positive=pair,matched_negative=control,
        candidate_pairs=pairs,candidate_singles=singles,plans=plans,
        formal_test_consumed=False,selection_observations_used=False,truth_read=False,hardware_executable=False)
    write_json(output/'report.json',report)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('root','analysis','diagnosis','output'):
        parser.add_argument(name,type=Path)
    report=plan(**vars(parser.parse_args()))
    print(report['baseline'], report['selected_positive'],report['matched_negative'],flush=True)
