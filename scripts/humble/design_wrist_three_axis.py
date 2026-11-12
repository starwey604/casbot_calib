#!/usr/bin/env python3
"""Compare left-wrist excitation designs on sealed train data; no fit or collection."""
import argparse
from collections import defaultdict
from itertools import combinations, product
from math import comb
from pathlib import Path

import numpy as np
from scipy.interpolate import BPoly
from scipy.linalg import solve

from design_wrist_supplement import goal_noise_gain, mass_rows, protect_observations
from casbot_calib.data import write_json
from casbot_calib.model_runtime import collision_for, model_for
from casbot_calib.staged_attribution import arrays, check_manifest, formal_test_untouched
from casbot_calib.staged_inertia import _project
from casbot_calib.staged_motion import Motion, derivative_bounds, gate_motion
from casbot_calib.staged_multiconfiguration import _write_motion
from casbot_calib.staged_multifit import assemble_training, load_problems
from casbot_calib.staged_protocol import _read, _unseal, file_hash, load_protocol
from casbot_calib.staged_wrist_supplement import VERSION

AXES = (4, 5, 6)
BINS = (7, 11, 17)
PHASE = (0., np.pi/3, 2*np.pi/3)
ACCEL = 2.5
ANGLES = (-.3, 0., .3)


def motion_reference(name, center, active, config):
    period, periods, ramp = (config[k] for k in ('dynamic_period_s','dynamic_periods','dynamic_ramp_s'))
    home = np.asarray(config['home_rad']); held = np.asarray(center)
    j = np.asarray(active); index = j - AXES[0]
    omega = 2*np.pi*np.asarray(BINS)[index]/period
    phase = np.asarray(PHASE)[index]
    motion = Motion(name, 'inertia', home, dict(active_joints=list(active), period_s=period,
        harmonics=np.asarray(BINS)[index].tolist(), amplitude_rad=(ACCEL/omega**2).tolist(),
        phases_rad=phase.tolist(), source='offline wrist-only design'))
    motion.add(config['settle_s'], home, label='initial_hold')
    motion.add(config['transition_s'], held, label='approach')
    envelope = BPoly.from_derivatives([0.,ramp], [[0.,0.,0.,0.],[1.,0.,0.,0.]])
    for mode, length, offset in [('entry',ramp,-ramp),('periodic',period*periods,0.),
                                 ('exit',ramp,period*periods)]:
        count = round(length*64); step = length/count
        times = np.arange(1,count+1)*step
        angle = times[:,None]*omega+offset*omega+phase
        jets = [ACCEL*omega**(k-2)*np.sin(angle+k*np.pi/2) for k in range(4)]
        if mode == 'periodic':
            weights = [np.ones(count), *[np.zeros(count) for _ in range(3)]]
        else:
            weights = [envelope(times,nu=k) for k in range(4)]
            if mode == 'exit':
                weights = [1-weights[0], *[-x for x in weights[1:]]]
        states = np.zeros((count,4,14)); states[:,0] = held
        for k in range(4):
            states[:,k,j] += sum(comb(k,m)*weights[m][:,None]*jets[k-m] for m in range(k+1))
        for state in states:
            motion.add(step,state[0],state[1],label=mode);motion.states[-1]=state
    return motion.add(config['transition_s'],home,label='return_home').add(config['settle_s'],home,label='final_hold')


def reference_information(motion, model, config, project, normalizers, probe_count):
    prep=config['p0']['preprocessing']; n=prep['phase_points']; h=prep['harmonics']
    begin=motion.windows('periodic')[0][0]
    q,v,a,_=motion.at(begin+np.arange(n)*config['dynamic_period_s']/n)
    y=_project(model.regressor_batch(q,v,a)['regressor'],h)
    dry=_project(np.sign(v)*(1-np.exp(-(v/.002)**2)),h)
    velocity=_project(v,h)
    info=np.zeros((project.shape[1],project.shape[1]))
    for j in motion.conditions['active_joints']:
        mask=abs(v[:,j])>.015
        x=np.zeros((mask.sum(),28));x[:,j]=velocity[mask,j];x[:,14+j]=dry[mask,j]
        rows=np.column_stack([y[mask,j],x])@project
        rows*=np.sqrt(1024/(len(rows)*probe_count))/normalizers[j]
        info+=rows.T@rows
    return info


def assess(root, analysis, output):
    root,analysis,output=(Path(p).resolve() for p in (root,analysis,output))
    if output.exists() or output.is_relative_to(root):
        raise ValueError('Use a new output directory outside the sealed train session')
    parent,config,manifest=load_protocol(root)
    if parent['experiment_version']!=VERSION:
        raise ValueError('Expected the 004 yaw-supplement train session')
    formal_test_untouched(root)
    protect_observations(root,{(r['trial_id'],) for r in manifest['trials'] if r['split']=='train'})
    protocol=_unseal(analysis,'analysis_protocol.json')
    setup=_unseal(analysis,'training_setup.json')
    if (protocol['parent_protocol_sha256']!=file_hash(root/'protocol.json')
            or protocol['train_manifest_sha256']!=file_hash(root/'role_inputs/train.json')
            or protocol['inputs']['coordinates.npz']!=file_hash(analysis/'coordinates.npz')
            or setup['files']['comparison_basis.npz']!=file_hash(analysis/'comparison_basis.npz')):
        raise ValueError('Train or frozen comparison coordinates do not match R2')
    check_manifest(analysis,'delivery_manifest.json')
    coordinate=arrays(analysis/'coordinates.npz');basis=arrays(analysis/'comparison_basis.npz')
    rank=setup['common_rank']
    project=(coordinate['scales'][:,None]*basis['basis'])/basis['singular_values'][:rank]
    train=load_problems(root,'train',coordinate['static'])
    y,x,_,weights=assemble_training(train,config)
    if weights!=setup['weights']:
        raise ValueError('Measured train row weights differ from frozen R2')
    design=np.column_stack([y,x])@project
    del y,x
    by_center=defaultdict(lambda:np.zeros((rank,rank)))
    by_batch=defaultdict(lambda:np.zeros((rank,rank)))
    begin=0
    for w in weights:
        end=begin+w['rows']; block=design[begin:end]
        gram=block.T@block
        by_center[w['center_id']]+=gram;by_batch[w['acquisition_batch_id']]+=gram;begin=end
    full=sum(by_center.values())
    if np.max(abs(full-np.eye(rank)))>1e-8:
        raise ValueError('Full-train coordinates do not reproduce the R2 Gram')
    del design,train
    centers=sorted(by_center); batches=sorted(by_batch)
    omit={c:full-by_center[c] for c in centers}
    nominal,_=model_for(config)
    # The 128 sealed R2 q values are planned operator queries, not test observations.
    q=arrays(analysis/'operator_probe_set.npz')['q']
    same=arrays(analysis/'variants/ridge_0.0001_full/solution.npz')
    d=np.diagonal(same['mass'],axis1=1,axis2=2)[:,AXES]
    floor=config['p0']['stability']['mass_diagonal_floor_kg_m2']
    goals=np.stack([mass_rows(nominal,q,j)@project[:140] for j in AXES],axis=1)/np.maximum(d,floor)[:,:,None]
    goals=goals.reshape(-1,rank)
    gain=goal_noise_gain(full,goals)
    sensitivity={c:goal_noise_gain(g,goals)/gain for c,g in omit.items()}
    cutoff=1.02  # retain near ties only for descriptive output, not qualification
    normalizers={j:float(np.median([w['torque_normalizer_nm'] for w in weights if w['joint']==j and
        w['stage']==('inertia_probe' if j==4 else 'inertia')])) for j in AXES}
    count=sum(w['stage']=='inertia_probe' for w in weights)
    pose_map=_read(root/'configuration_coverage.json')['centers']
    options={}
    for c in centers:
        for pitch,roll in product(ANGLES,repeat=2):
            pose=np.array(pose_map[c]);pose[5:7]=(pitch,roll)
            key=f'{c}_p{pitch:+.2f}_r{roll:+.2f}'
            options[key]={}
            for active in [(j,) for j in AXES]+[AXES]:
                m=motion_reference(f'{key}_axes'+''.join(map(str,active)),pose,active,config)
                options[key][active]=(m,reference_information(m,nominal,config,project,normalizers,count))

    def info(key, mode):
        if mode=='separate_three':
            return sum(options[key][(j,)][1] for j in AXES)
        active=AXES if mode.startswith('mux') else (int(mode[-1]),)
        scale=3 if mode=='mux_equal_record' else 1
        return options[key][active][1]/scale

    def members(key,mode):
        if mode=='separate_three':
            return [options[key][(j,)][0] for j in AXES]
        return [options[key][AXES if mode.startswith('mux') else (int(mode[-1]),)][0]]

    def summarize(ratios):
        per_axis=ratios.reshape(len(q),len(AXES)).max(axis=0)
        return dict(worst=float(per_axis.max()), per_axis=per_axis.tolist())

    def evaluate(entries, *, detailed=False):
        ratios={}
        for c,g in omit.items():
            added=sum(info(k,m) for k,m in entries if not k.startswith(c+'_'))
            ratios[c]=goal_noise_gain(g+len(batches)*added,goals)/gain
        result=dict(worst=max(float(v.max()) for v in ratios.values()))
        if detailed:
            result['by_omitted_center']={k:summarize(v) for k,v in ratios.items()}
            result['by_omitted_batch']={b:summarize(goal_noise_gain(full-by_batch[b]+
                (len(batches)-1)*sum(info(k,m) for k,m in entries),goals)/gain) for b in batches}
        return result

    profile=collision_for(config);gates={}
    def gate(entry):
        passed=True
        for motion in members(*entry):
            if motion.name not in gates:
                curve=motion.polynomial()
                bounds=[float(np.max(derivative_bounds(curve,k))) for k in (1,2,3)]
                report=gate_motion(profile,motion,profile.metadata['scene']['clearance_m'])
                report['derivative_bounds']=bounds
                report['passed']=report['passed'] and all(x<=lim for x,lim in zip(bounds,(2,8,60)))
                gates[motion.name]=report
            passed=passed and gates[motion.name]['passed']
        return passed

    modes=('axis4','axis5','axis6')
    keys={c:[k for k in options if k.startswith(c+'_')] for c in centers}
    def search(label, search_space):
        rows=[]
        for entries in search_space:
            score=evaluate(entries)['worst']
            rows.append((score,entries))
        rows.sort()
        for score,entries in rows:
            if all(gate(e) for e in entries):
                return dict(label=label, screened=len(rows), entries=entries,
                    new_records=len(batches)*sum(len(members(*e)) for e in entries),
                    **evaluate(entries,detailed=True))
        raise ValueError('No continuously admissible design: '+label)

    def combinations_for(n, modes):
        for groups in combinations(centers,n):
            yield from product(*[list(product(keys[c],modes)) for c in groups])

    comparisons=[
        search('single_center_any',combinations_for(1,(*modes,'mux_equal_record'))),
        search('two_centers_axis_only_equal_record',combinations_for(2,modes)),
        search('two_centers_mux_equal_record',combinations_for(2,('mux_equal_record',))),
        search('two_centers_mux_equal_axis',combinations_for(2,('mux_equal_axis',))),
        search('two_centers_separate_three_equal_axis',combinations_for(2,('separate_three',))),
        search('three_centers_mux_equal_record',combinations_for(3,('mux_equal_record',))),
    ]
    # Minimum candidate with a meaningful improvement in the worst case under a
    # fixed record budget. Always report the larger alternatives, not a new gate.
    baseline=max(float(v.max()) for v in sensitivity.values())
    best_equal=min((r for r in comparisons if r['label'] in {
        'two_centers_axis_only_equal_record','two_centers_mux_equal_record','three_centers_mux_equal_record'}),
        key=lambda r:r['worst'])
    eligible=[r for r in comparisons if r['label'] in {'single_center_any','two_centers_axis_only_equal_record',
        'two_centers_mux_equal_record','three_centers_mux_equal_record'} and
        r['worst']<=best_equal['worst']*cutoff and r['worst']<baseline]
    chosen=min(eligible,key=lambda r:(r['new_records'],r['worst'])) if eligible else None
    output.mkdir(parents=True,exist_ok=False)
    plans=[]
    if chosen:
        for entry in chosen['entries']:
            for m in members(*entry):
                _write_motion(output,m,gates[m.name])
                plans.append(dict(name=m.name, group=entry[0].split('_p')[0], center_rad=m.states[2][0].tolist(),
                                  duration_s=m.duration,conditions=m.conditions,
                                  min_sampled_distance_m=gates[m.name]['minimum_distance_m'],
                                  derivative_bounds=gates[m.name]['derivative_bounds']))
    report=dict(axes=list(AXES), axis_names=[nominal.model.joints[j].name for j in AXES],
        scope='train-only measured design and offline Pinocchio reference rows; no fit or collection',
        target='three relative mass-diagonal functionals at 128 sealed nominal R2 operator q; no formal observations',
        baseline=dict(worst=baseline,by_omitted_center={k:summarize(v) for k,v in sensitivity.items()},
                      by_omitted_batch={b:summarize(goal_noise_gain(full-g,goals)/gain) for b,g in by_batch.items()}),
        comparisons=comparisons, chosen=chosen,plans=plans,
        normalizers_nm={str(k):v for k,v in normalizers.items()}, phase_points=config['p0']['preprocessing']['phase_points'],
        frozen_current_train_probe_count=count, equal_record_multiplex_axis_weight_squared_multiplier=1/3,
        tie_fraction=cutoff-1, no_formal_observation=True, selection_observations_used=False)
    write_json(output/'report.json',report)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('root',type=Path);p.add_argument('--analysis',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();result=assess(a.root,a.analysis,a.output)
    print(result['baseline'], result['chosen'],flush=True)
