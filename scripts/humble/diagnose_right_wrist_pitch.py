#!/usr/bin/env python3
"""Locate right-pitch inertia information lost with train_c02, without refits."""
import argparse
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.linalg import null_space, solve

from design_wrist_supplement import goal_noise_gain, mass_rows, protect_observations
from diagnose_wrist_periodic_counterfactual import OMIT, RIDGE, SUBSETS, sealed_r2
from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for
from casbot_calib.staged_attribution import arrays, formal_test_untouched
from casbot_calib.staged_multifit import assemble_training, load_problems
from casbot_calib.staged_protocol import _read, _unseal, file_hash, load_protocol

PITCH, ROLL = 12, 13


def harmonic_design(angles):
    return np.column_stack([np.ones(len(angles)),np.cos(angles),np.sin(angles),np.cos(2*angles),np.sin(2*angles)])


def inspect(root, analysis, local, output):
    root,analysis,local,output=(Path(p).resolve() for p in (root,analysis,local,output))
    if output.exists() or any(output.is_relative_to(p) for p in (root,analysis,local)):
        raise ValueError('Use fresh diagnostic output outside training artifacts')
    protocol,config,manifest=load_protocol(root)
    formal_test_untouched(root)
    protect_observations(root,{(r['trial_id'],) for r in manifest['trials'] if r['split']=='train'})
    r2=sealed_r2(root,analysis)
    setup=_unseal(analysis,'training_setup.json')
    if setup['files']['comparison_basis.npz']!=file_hash(analysis/'comparison_basis.npz'):
        raise ValueError('Changed frozen R2 comparison basis')
    coord=arrays(analysis/'coordinates.npz');basis=arrays(analysis/'comparison_basis.npz')
    probes=arrays(analysis/'operator_probe_set.npz')
    reported=_read(local/'report.json')
    if set(r['worst_joint_index'] for r in reported['matched_bandwidths'].values())!={PITCH}:
        raise ValueError('This diagnosis expects a right-pitch bottleneck')
    indexes={r['worst_probe_index'] for r in reported['matched_bandwidths'].values()}
    if len(indexes)!=1:
        raise ValueError('The common worst right-pitch probe must be fixed before analysis')
    index=indexes.pop()
    pose=probes['q'][index]
    original={s:arrays(analysis/'variants'/f'ridge_{RIDGE:g}_{s}'/'solution.npz') for s in SUBSETS}
    worst_mass=[float(original[s]['mass'][index,PITCH,PITCH]) for s in SUBSETS]
    for sigma in reported['matched_bandwidths']:
        for s in SUBSETS:
            actual=arrays(local/f'sigma_{sigma}_ms' / f'{s}.npz')['mass'][:,PITCH,PITCH]
            if np.max(abs(actual-original[s]['mass'][:,PITCH,PITCH]))>1e-9:
                raise ValueError('Yaw-only right-pitch operators do not represent the reported bottleneck')

    rank=setup['common_rank']
    # Whiten the full training Gram in the fixed gravity-constrained observable
    # coordinates. All omits/restorations keep the original row weights.
    project=(coord['scales'][:,None]*basis['basis'])/basis['singular_values'][:rank]
    train=load_problems(root,'train',coord['static'])
    y,x,_,weights=assemble_training(train,config)
    if weights!=setup['weights']:
        raise ValueError('Original measured train weights did not reproduce')
    design=np.column_stack([y,x])@project
    delta=original[SUBSETS[1]]['vector']-original[SUBSETS[0]]['vector']
    actual_torque=y@delta[:140]+x@delta[140:]
    del y,x
    grams=defaultdict(lambda:np.zeros((rank,rank)))
    actual=defaultdict(float)
    coverage=[]
    begin=0
    for w in weights:
        end=begin+w['rows'];a=design[begin:end];e=actual_torque[begin:end]
        p=train[w['trial_id']]['problem']
        j=w['joint'];c=w['center_id'];stage=w['stage']
        use=p['moving'][:,j] if stage=='inertia' else slice(None)
        roll=p['q'][use,ROLL]
        if len(roll)!=len(a):
            raise ValueError('Measured roll samples do not match frozen regression rows')
        positive=roll>=0
        for label,m in [('negative',~positive),('nonnegative',positive)]:
            aa=a[m];ee=e[m]
            grams[c,stage,j,label]+=aa.T@aa
            actual[c,stage,j,label]+=float(ee@ee)
        if j in (11,PITCH,ROLL):
            ddq=p['ddq'][use,PITCH]
            coverage.append(dict(trial=w['trial_id'],center=c,stage=stage,torque_axis=j,
                minimum_roll_rad=float(roll.min()),maximum_roll_rad=float(roll.max()),
                nonnegative_fraction=float(positive.mean()),
                pitch_acceleration_rms_rad_s2=float(np.sqrt(np.mean(ddq**2))),
                nonnegative_pitch_acceleration_rms_rad_s2=float(np.sqrt(np.mean(ddq[positive]**2))) if positive.any() else None))
        begin=end
    full=sum(grams.values())
    if np.max(abs(full-np.eye(rank)))>1e-8:
        raise ValueError('Full weighted Gram does not reproduce the common basis')
    del design, train
    nominal,_=model_for(config)
    # Identify the roll dependence of M[pitch,pitch] as a linear functional,
    # then verify it on withheld angles and at all fixed R2 query poses.
    angles=np.linspace(-np.pi,np.pi,24,endpoint=False)
    poses=np.broadcast_to(pose,(len(angles),14)).copy();poses[:,ROLL]=angles
    harmonics=np.linalg.lstsq(harmonic_design(angles),mass_rows(nominal,poses,PITCH),rcond=None)[0]
    qrows=mass_rows(nominal,probes['q'],PITCH)
    representation_error=float(np.max(abs(qrows-harmonic_design(probes['q'][:,ROLL])@harmonics)))
    delta_q=qrows@delta[:140]
    native_delta=original[SUBSETS[1]]['mass'][:,PITCH,PITCH]-original[SUBSETS[0]]['mass'][:,PITCH,PITCH]
    if representation_error>1e-10 or np.max(abs(delta_q-native_delta))>1e-12:
        raise ValueError('Roll-only functional failed native operator checks')
    centers=_read(root/'configuration_coverage.json')['centers']
    query_angles=[-.8,-.68609459,-.3,0.,.3,.8]
    qp=np.broadcast_to(pose,(len(query_angles),14)).copy();qp[:,ROLL]=query_angles
    query=mass_rows(nominal,qp,PITCH)
    goal=mass_rows(nominal,[pose],PITCH)@project[:140]
    left_out=sum(g for k,g in grams.items() if k[0]!=OMIT)
    weak=solve(left_out,goal[0],assume_a='pos')
    noise=lambda g:float(goal_noise_gain(g,goal)[0]/goal_noise_gain(full,goal)[0])
    energy={k:float(weak@g@weak) for k,g in grams.items()}
    omitted_energy=sum(v for k,v in energy.items() if k[0]==OMIT)
    c02=lambda predicate:sum(g for k,g in grams.items() if k[0]==OMIT and predicate(k))
    groups={
        'all_c02_periodic':lambda k:k[1]=='inertia',
        'right_pitch_periodic_only':lambda k:k[1]=='inertia' and k[2]==PITCH,
        'right_roll_periodic_only':lambda k:k[1]=='inertia' and k[2]==ROLL,
        'right_yaw_periodic_only':lambda k:k[1]=='inertia' and k[2]==11,
        'right_yaw_probes_only':lambda k:k[1]=='inertia_probe' and k[2]==11,
        'c02_nonnegative_roll_only':lambda k:k[3]=='nonnegative',
        'c02_negative_roll_only':lambda k:k[3]=='negative',
        'all_but_right_pitch':lambda k:not(k[1]=='inertia' and k[2]==PITCH),
    }
    restorations={name:dict(noise_gain_ratio=noise(left_out+c02(rule)),
        fraction_of_c02_weak_direction_energy=sum(v for k,v in energy.items() if k[0]==OMIT and rule(k))/omitted_energy)
        for name,rule in groups.items()}
    coefficients=harmonics@delta[:140]
    at_pose=harmonic_design(np.array([pose[ROLL]]))[0]*coefficients
    harmonic_goals=harmonics@project[:140]
    # Diagnose when the full common basis does not span the target after gravity
    # constraints; do not silently count unobserved directions as measured.
    equality=np.pad(coord['equality'],((0,0),(0,28)))
    free=null_space(equality*coord['scales'],rcond=1e-7)
    raw_goal=np.r_[mass_rows(nominal,[pose],PITCH)[0],np.zeros(28)]*coord['scales']
    free_goal=free@(free.T@raw_goal)
    common_goal=basis['basis']@(basis['basis'].T@free_goal)
    report=dict(scope='original weighted train only, R2 operator and observed c02 ablation; no new fit',
        right_pitch_joint=nominal.model.joints[PITCH].name, downstream_roll_joint=nominal.model.joints[ROLL].name,
        worst_probe=dict(index=index,q_rad=pose.tolist(),roll_rad=float(pose[ROLL]),mass_full_kg_m2=worst_mass[0],
            mass_omit_c02_kg_m2=worst_mass[1],relative_change=(worst_mass[1]-worst_mass[0])/worst_mass[0]),
        roll_only_functional_error=float(representation_error),harmonic_labels=['constant','cos_roll','sin_roll','cos_2roll','sin_2roll'],
        deletion_delta_harmonic_coefficients_kg_m2=coefficients.tolist(),worst_pose_harmonic_contribution_kg_m2=at_pose.tolist(),
        roll_scan_rad=query_angles,roll_scan_deletion_delta_kg_m2=(query@delta[:140]).tolist(),
        roll_scan_gain_ratio=(goal_noise_gain(left_out,query@project[:140])/goal_noise_gain(full,query@project[:140])).tolist(),
        harmonic_gain_ratio=(goal_noise_gain(left_out,harmonic_goals)/goal_noise_gain(full,harmonic_goals)).tolist(),
        right_pitch_gain_without_c02=noise(left_out),
        fraction_weak_direction_energy_in_c02=omitted_energy/sum(energy.values()),
        fraction_actual_deletion_torque_energy_in_c02=sum(v for k,v in actual.items() if k[0]==OMIT)/sum(actual.values()),
        free_goal_unspanned_fraction=float(np.linalg.norm(free_goal-common_goal)/np.linalg.norm(free_goal)),
        c02_restorations=restorations,
        blocks=[dict(center=k[0],stage=k[1],axis=k[2],roll_sign=k[3],weak_direction_energy=v,
                     actual_deletion_torque_energy=actual[k]) for k,v in sorted(energy.items())],
        measured_train_coverage=coverage,formal_test_consumed=False,selection_observations_used=False,truth_read=False)
    formal_test_untouched(root)
    output.mkdir(parents=True,exist_ok=False)
    write_json(output/'report.json',report)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('root','analysis','local','output'):
        p.add_argument(name,type=Path)
    r=inspect(**vars(p.parse_args()))
    print(r['worst_probe'],r['right_pitch_gain_without_c02'],r['c02_restorations'])
