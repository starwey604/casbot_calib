#!/usr/bin/env python3
"""Locate c02 phase windows that push the frozen right-pitch inertia upwards."""
import argparse
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.linalg import solve
from scipy.ndimage import minimum_filter1d
from scipy.signal import butter, sosfiltfilt

from design_wrist_supplement import mass_rows, protect_observations
from diagnose_right_pitch_balance import diagnostic_fit
from diagnose_right_wrist_pitch import PITCH, ROLL
from diagnose_wrist_periodic_counterfactual import RIDGE, model, sealed_r2
from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for
from casbot_calib.staged_attribution import arrays, formal_test_untouched
from casbot_calib.staged_collection import load_trial
from casbot_calib.staged_identification import read_record
from casbot_calib.staged_inertia import _project
from casbot_calib.staged_multifit import assemble_training, load_problems
from casbot_calib.staged_protocol import _read, _unseal, load_protocol
from casbot_calib.staged_residuals import matched_components
from casbot_calib.staged_right_pitch import right_companion
from casbot_calib.staged_wrist_periodic import companion

CENTER='train_c02'
WINDOW=.5
SCALE=.02
CROSS=(7,8,9,10,11,13)


def rms(x):
    return float(np.sqrt(np.mean(np.asarray(x)**2)))


def psi(residual):
    return residual/np.hypot(1,residual/SCALE)


def loss_improvement(before,after):
    # Cancellation-free equivalent of the difference of the two soft-l1 losses.
    return (before-after)*(before+after)/(np.hypot(1,before/SCALE)+np.hypot(1,after/SCALE))


def motion_terms(engine,p,vector,h):
    y=engine.evaluate_batch(p['q'],p['dq'],p['ddq'])
    terms,prediction,checks=matched_components(engine,vector[:140],vector[140:],p,h)
    split={f'cross_j{j:02}':_project(y['mass_matrix'][:,PITCH,j]*p['ddq'][:,j],h) for j in CROSS}
    self_t=_project(y['mass_matrix'][:,PITCH,PITCH]*p['ddq'][:,PITCH],h)
    other=terms['rigid_inertia'][:,PITCH]+terms['rotor'][:,PITCH]-self_t-sum(split.values())
    components=dict(self_inertia=self_t,**split,other_inertia_and_rotor=other,
                    **{k:terms[k][:,PITCH] for k in ('velocity','gravity','friction')})
    np.testing.assert_allclose(sum(components.values()),prediction[:,PITCH],rtol=0,atol=1e-10)
    return components,checks['maximum_matched_closure_nm']


def raw_cycles(root,name,config,names,engines,phase_windows,static,friction):
    meta,_=load_trial(root,name);_,obs=read_record(root,name,meta,names)
    time=obs['sample_ns'][2:-2]; step=config['inertia_sample_period_ns'];dt=step*1e-9
    period_ns=round(meta['conditions']['period_s']*1e9);n=period_ns//step
    begin=round(meta['knots_s'][meta['labels'].index('periodic')]*1e9)+2*period_ns
    start=np.searchsorted(time,begin)
    np.testing.assert_array_equal(time[start:start+2*n],np.arange(begin,begin+2*period_ns,step))
    q=obs['q'][2:-2];v=obs['dq'];w=v[2:-2]
    a=(v[:-4]-8*v[1:-3]+8*v[3:-1]-v[4:])/(12*dt)
    span=round(.2/dt);sel=slice(start-span,start+2*n+span);trim=slice(span,span+2*n)
    y=obs['current'][2:-2,PITCH]*config['drive']['gain_nm_a']
    vj=w[:,PITCH]
    sign=np.sign(vj);transition=sign*np.exp(-(vj/.002)**2)
    target={}
    components={}
    for label,engine in engines.items():
        parts=engine.evaluate_batch(q[sel],w[sel],a[sel])
        self_t=parts['mass_matrix'][:,PITCH,PITCH]*a[sel,PITCH]
        f=friction[label][PITCH]*vj[sel]+friction[label][14+PITCH]*(sign-transition)[sel]+static[PITCH]*transition[sel]
        total=(parts['inertia']+parts['gravity']+parts['velocity'])[:,PITCH]+f
        target[label]=total
        components[label]=self_t
    robust_speed=(minimum_filter1d(vj,size=41,mode='nearest')>.02)|(minimum_filter1d(-vj,size=41,mode='nearest')>.02)
    filtered=sosfiltfilt(butter(6,30.,fs=1/dt,output='sos'),np.column_stack([
        y[sel]-target['base'],target['full']-target['base'],components['full']-components['base'],a[sel,PITCH],vj[sel]]),axis=0)[trim]
    out=[]
    for cycle in range(2):
        for window in phase_windows:
            nbin=round(WINDOW/dt);ix=slice(cycle*n+window*nbin,cycle*n+(window+1)*nbin)
            sustained=robust_speed[start:start+2*n][ix]
            residual,delta,delta_self,acc,vel=filtered[ix].T
            # The optional scalar slope projects the base residual onto the
            # local own acceleration after constant and velocity nuisance.
            def slope(mask):
                if mask.sum()<32:return None
                local=np.column_stack((np.ones(mask.sum()),vel[mask]))
                r=np.column_stack((acc[mask],residual[mask]))
                r-=local@np.linalg.lstsq(local,r,rcond=None)[0]
                ap,rp=r.T
                return dict(effective_mass_correction_kg_m2=float(ap@rp/(ap@ap)),acceleration_rms_rad_s2=rms(ap))
            pressure=residual*delta
            out.append(dict(trial=name,cycle=cycle+2,window=window,raw_sustained_fraction=float(sustained.mean()),
                raw_near_stall_fraction=float(np.mean(abs(vj[start:start+2*n][ix])<=.015)),
                raw_directional_pressure_nm2_s=float(pressure.sum()*dt),
                sustained_directional_pressure_nm2_s=float(pressure[sustained].sum()*dt),
                raw_own_directional_pressure_nm2_s=float((residual*delta_self).sum()*dt),
                sustained_local_projection=slope(sustained)))
    return out


def transition_targets(root, name, config, names, engine, vector, static, p, *, joint=PITCH):
    """Split raw residual before the exact original phase projection."""
    meta,_=load_trial(root,name);_,obs=read_record(root,name,meta,names)
    prep=config['p0']['preprocessing']
    period_ns=round(meta['conditions']['period_s']*1e9)
    dt=config['inertia_sample_period_ns']*1e-9
    n=round(period_ns*1e-9/dt)
    begin=round(meta['knots_s'][meta['labels'].index('periodic')]*1e9)+prep['discard_periods']*period_ns
    t=obs['sample_ns']; start=np.searchsorted(t,begin); k=np.arange(start,start+2*n)
    np.testing.assert_array_equal(t[k],begin+np.arange(2*n)*config['inertia_sample_period_ns'])
    v=obs['dq']; q=obs['q']; vj=v[:,joint]
    a=(v[k-2]-8*v[k-1]+8*v[k+1]-v[k+2])/(12*dt)
    parts=engine.evaluate_batch(q[k],v[k],a)
    rigid=(parts['inertia']+parts['gravity']+parts['velocity'])[:,joint]
    observed=obs['current'][k,joint]*config['drive']['gain_nm_a']
    transition=np.sign(vj[k])*np.exp(-(vj[k]/.002)**2)
    friction=vector[140+joint]*vj[k]+vector[154+joint]*(np.sign(vj[k])-transition)+static[joint]*transition
    residual=observed-rigid-friction
    sustained=((minimum_filter1d(vj,size=41,mode='nearest')>.02)
              | (minimum_filter1d(-vj,size=41,mode='nearest')>.02))[k]
    stopped=abs(vj[k])<=1e-5
    groups={'stopped':stopped,'near_transition':~(stopped|sustained),'sustained':sustained}
    h=prep['harmonics']; m=prep['phase_points']; use=p['moving'][:,joint]
    def project(value):
        phases=value.reshape(2,n).mean(axis=0)
        return np.fft.irfft(np.fft.rfft(phases)[:h+1],n=m)*(m/n)
    np.testing.assert_allclose(project(observed),p['current_torque'][:,joint],rtol=0,atol=1e-11)
    blocks={label:project(residual*mask)[use] for label,mask in groups.items()}
    np.testing.assert_allclose(sum(blocks.values()),project(residual)[use],rtol=0,atol=1e-12)
    reaction=abs((observed-rigid)[stopped])
    runs=np.flatnonzero(np.diff(np.r_[False,stopped,False]))
    report=dict(raw_samples=len(k),raw_group_counts={c:int(np.count_nonzero(mask)) for c,mask in groups.items()},
        frozen_static_friction_nm=float(static[joint]),
        stopped_reaction_p95_nm=float(np.quantile(reaction,.95)),
        stopped_reaction_within_static_fraction=float(np.mean(reaction<=static[joint]+.01)),
        stopped_reaction_within_exact_static_fraction=float(np.mean(reaction<=static[joint])),
        longest_stop_s=float(np.max(runs[1::2]-runs[::2])*dt),
        maximum_stop_other_axis_speed_rad_s=float(np.max(abs(v[k[stopped]][:,np.arange(14)!=joint]))),
        raw_residual_rms_nm={label:rms(residual[mask]) for label,mask in groups.items()},
        projected_residual_rms_nm={label:rms(block) for label,block in blocks.items()})
    return blocks,report


def transitions_only(root,output,config,coord,qset,nominal,engines,vectors,X,target,weights,problems,base,full,index):
    goal_mass=lambda s:float(s['mass'][index,PITCH,PITCH])
    names=_read(root/'session.json')['joint_names']
    selected=np.zeros(len(target),dtype=bool)
    pieces={c:np.zeros(len(target)) for c in ('stopped','near_transition','sustained')}
    reports={}
    offset=0
    for w in weights:
        end=offset+w['rows'];s=slice(offset,end);offset=end
        if (w['center_id'],w['stage'],w['joint'])!=(CENTER,'inertia',PITCH):continue
        name=w['trial_id'];selected[s]=True
        blocks,report=transition_targets(root,name,config,names,engines['base'],vectors['base'],coord['static'],
                                         problems[name]['problem'])
        for label,v in blocks.items():pieces[label][s]=v*w['sqrt_weight']
        reports[name]=report
    output.mkdir(parents=True,exist_ok=False)
    tests={
        'neutralize_stopped_residual':pieces['stopped'],
        'neutralize_non_sustained_residual':pieces['stopped']+pieces['near_transition'],
        'neutralize_sustained_residual':pieces['sustained'],
    }
    result=dict(ridge=RIDGE,query_index=index,base_mass_kg_m2=goal_mass(base),full_mass_kg_m2=goal_mass(full),
        source_rows=int(selected.sum()),records=reports,
        target_intervention='subtract only projected residual vs no-c02-pitch model, computed in the declared raw speed state; no sample/weight/regressor changes',
        ab_observations_used=False,formal_test_consumed=False,truth_read=False,cases={})
    for label,correction in tests.items():
        state,fit=diagnostic_fit(X,target-correction,coord,nominal,qset)
        np.savez_compressed(output/f'{label}.npz',**state)
        write_json(output/f'{label}_fit.json',fit)
        m=goal_mass(state)
        result['cases'][label]=dict(mass_at_query_kg_m2=m,fraction_of_original_upward_change=(m-goal_mass(base))
                                   /(goal_mass(full)-goal_mass(base)))
        print(label,m,flush=True)
    formal_test_untouched(root)
    write_json(output/'report.json',result)


def run(root,r2,restoration,output,*,screen_only=False,transitions=False):
    root,r2,restoration,output=(p.resolve() for p in (root,r2,restoration,output))
    if output.exists() or any(output.is_relative_to(p) for p in (root,r2,restoration)):
        raise ValueError('Use a new diagnostic output directory outside the input sessions')
    _,config,manifest=load_protocol(root);formal_test_untouched(root)
    protect_observations(root,{(r['trial_id'],) for r in manifest['trials'] if r['split']=='train'})
    for get_companion in (companion,right_companion):
        cap,names=get_companion(root);protect_observations(cap,{(n,) for n in names})
    protect_observations(root.parent/'motion_pair_capture_v1',set())
    sealed_r2(root,r2)
    coord=arrays(r2/'coordinates.npz');qset=arrays(r2/'operator_probe_set.npz')
    base=arrays(restoration/'c02_except_pitch_periodic.npz')
    full=arrays(r2/'variants'/f'ridge_{RIDGE:g}_full'/'solution.npz')
    old=_read(restoration/'restorations.json');index=old['worst_measured_probe']
    goal_mass=lambda s:float(s['mass'][index,PITCH,PITCH])
    np.testing.assert_allclose(goal_mass(base),old['restorations']['c02_except_pitch_periodic']['mass_at_original_worst_kg_m2'],rtol=0,atol=1e-14)
    np.testing.assert_allclose(goal_mass(full),old['full_mass_at_worst_kg_m2'],rtol=0,atol=1e-14)
    vectors=dict(base=base['vector'],full=full['vector'])
    nominal,_=model_for(config)
    engines={k:model(nominal,v,coord['static']) for k,v in vectors.items()}
    problems=load_problems(root,'train',coord['static'])
    y,x,target,weights=assemble_training(problems,config)
    setup=_unseal(r2,'training_setup.json')
    if weights!=setup['weights']:raise ValueError('Original weights must reproduce')
    X=np.column_stack((y,x));del y,x
    if transitions:
        return transitions_only(root,output,config,coord,qset,nominal,engines,vectors,X,target,weights,problems,base,full,index)
    h=config['p0']['preprocessing']['harmonics']
    period=config['dynamic_period_s']
    rank=setup['common_rank']
    basis=arrays(r2/'comparison_basis.npz')['basis']
    transform=coord['scales'][:,None]*basis[:,:rank]
    del basis
    # Local objective linearization in the frozen gravity-constrained subspace.
    # Used to rank; the exact bounded solver below decides whether it generalizes.
    Z=X@transform
    r0=target-X@vectors['base'];r1=target-X@vectors['full']
    w=np.hypot(1,r0/SCALE)**-3
    H=Z.T@(w[:,None]*Z)+.2**2*RIDGE*(transform/coord['scales'][:,None]).T@coord['metric']@(transform/coord['scales'][:,None])
    goal=mass_rows(nominal,qset['q'][[index]],PITCH)[0]@transform[:140]
    gain=Z@solve(H,goal,assume_a='pos')
    del Z
    score=psi(r0)*gain
    section={}
    selected_rows=np.zeros(len(target),dtype=bool)
    rows=[]; raw=[]; records={}
    start=0
    for entry in weights:
        stop=start+entry['rows'];indices=np.arange(start,stop);start=stop
        if (entry['center_id'],entry['stage'],entry['joint'])!=(CENTER,'inertia',PITCH):continue
        name=entry['trial_id'];p=problems[name]['problem'];meta=problems[name]['meta']
        use=p['moving'][:,PITCH];phase=p['phase_s'][use];roll=p['q'][use,ROLL]
        ref=meta['conditions']['reference_id'];batch=entry['acquisition_batch_id']
        parts0,cl0=motion_terms(engines['base'],p,vectors['base'],h)
        parts1,cl1=motion_terms(engines['full'],p,vectors['full'],h)
        weighted_parts={k:(parts1[k]-parts0[k])[use]*entry['sqrt_weight'] for k in parts0}
        np.testing.assert_allclose(sum(weighted_parts.values()),r0[indices]-r1[indices],rtol=0,atol=1e-11)
        section[name]=(indices,phase,ref,roll)
        selected_rows[indices]=True
        records[name]=dict(reference_id=ref,batch=batch,closure_nm=max(cl0,cl1),
            original_phase_count=len(p['phase_s']),used_rows=len(phase),
            original_rmse_nm=rms(r0[indices])/entry['sqrt_weight'],full_rmse_nm=rms(r1[indices])/entry['sqrt_weight'])
        for i in range(round(period/WINDOW)):
            mask=(phase>=i*WINDOW)&(phase<(i+1)*WINDOW)
            ix=indices[mask];a=r0[ix];b=r1[ix]
            residual_weight=psi(a)
            component={k:float(residual_weight@v[mask]) for k,v in weighted_parts.items()}
            row=dict(trial=name,reference_id=ref,batch=batch,window=i,start_phase_s=i*WINDOW,end_phase_s=(i+1)*WINDOW,
                rows=len(ix),roll_range_rad=[float(roll[mask].min()),float(roll[mask].max())],
                positive_roll_above_055_fraction=float(np.mean(roll[mask]>=.55)),
                pitch_range_rad=[float(np.min(p['q'][use,PITCH][mask])),float(np.max(p['q'][use,PITCH][mask]))],
                base_residual_rms_nm=rms(a)/entry['sqrt_weight'],full_residual_rms_nm=rms(b)/entry['sqrt_weight'],
                goal_score_kg_m2=float(score[ix].sum()),
                directional_pressure=float(residual_weight@(a-b)),
                robust_loss_improvement=float(loss_improvement(a,b).sum()),components=component)
            np.testing.assert_allclose(sum(component.values()),row['directional_pressure'],rtol=0,atol=1e-13)
            rows.append(row)
    bins=defaultdict(float)
    for row in rows:bins[row['reference_id'],row['window']]+=row['goal_score_kg_m2']
    ranked=sorted(bins,key=bins.get,reverse=True)
    top=set(ranked[:len(ranked)//4])
    names=_read(root/'session.json')['joint_names']
    for n,(_,_,ref,_) in section.items():
        raw.extend(raw_cycles(root,n,config,names,engines,range(round(period/WINDOW)),coord['static'],
                              {k:v[140:] for k,v in vectors.items()}))
    output.mkdir(parents=True,exist_ok=False)
    result=dict(scope='original frozen train only; diagnostic attribution, no qualification',
        fixed_queries=len(qset['q']),query_index=index,query_roll_rad=float(qset['q'][index,ROLL]),
        base_mass_kg_m2=goal_mass(base),full_mass_kg_m2=goal_mass(full),ridge=RIDGE,period_s=period,
        phase_window_s=WINDOW,records=records,phase_windows=rows,raw_cycle_windows=raw,
        ranking=[dict(reference_id=k[0],window=k[1],goal_score_kg_m2=bins[k],selected_top_quarter=k in top) for k in ranked],
        diagnostic_only=True,formal_test_consumed=False,ab_observations_used=False,truth_read=False)
    write_json(output/'screen.json',result)
    print('Phase screen complete',ranked[:len(top)],flush=True)
    if screen_only:return
    top_rows=np.zeros(len(target),dtype=bool)
    high=np.zeros(len(target),dtype=bool)
    ref01=np.zeros(len(target),dtype=bool)
    for ix,phase,ref,roll in section.values():
        k=np.floor(phase/WINDOW).astype(int)
        top_rows[ix]=np.isin(k,[i for r,i in top if r==ref])
        high[ix]=roll>=.55
        ref01[ix]=ref.endswith('_01')
    keep_base=~selected_rows
    cases={
        'restore_top_quarter_only':keep_base|top_rows,
        'restore_other_windows_only':~top_rows,
        'restore_high_roll_only':keep_base|high,
        'restore_below_high_roll_only':~high,
        'restore_ref01_only':keep_base|ref01,
        'restore_ref00_only':keep_base|(selected_rows&~ref01),
    }
    for label,mask in cases.items():
        state,fit=diagnostic_fit(X[mask],target[mask],coord,nominal,qset)
        np.savez_compressed(output/f'{label}.npz',**state)
        write_json(output/f'{label}_fit.json',fit)
        m=goal_mass(state)
        result.setdefault('counterfactuals',{})[label]=dict(restored_rows=int(np.count_nonzero(selected_rows&mask)),
            mass_at_query_kg_m2=m,fraction_of_full_upward_change=(m-goal_mass(base))/(goal_mass(full)-goal_mass(base)),
            max_pitch_mass_difference_from_full_fraction=float(np.max(abs(state['mass'][:,PITCH,PITCH]-full['mass'][:,PITCH,PITCH])
                /np.maximum(full['mass'][:,PITCH,PITCH],config['p0']['stability']['mass_diagonal_floor_kg_m2']))))
        print(label,m,flush=True)
    formal_test_untouched(root)
    write_json(output/'report.json',result)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('root','r2','restoration','output'):p.add_argument(name,type=Path)
    mode=p.add_mutually_exclusive_group()
    mode.add_argument('--screen-only',action='store_true')
    mode.add_argument('--transitions',action='store_true')
    run(**vars(p.parse_args()))
