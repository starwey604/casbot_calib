#!/usr/bin/env python3
"""Design a same-pose single-/two-axis contrast for the c02 right-pitch constraint."""
import argparse
from pathlib import Path

import numpy as np
from scipy.interpolate import BPoly
from scipy.ndimage import gaussian_filter1d
from scipy.signal import butter, sosfiltfilt

from collect_wrist_three_axis import read_certified_reference
from design_wrist_supplement import protect_observations
from diagnose_right_wrist_pitch import PITCH, ROLL
from diagnose_wrist_periodic_counterfactual import RIDGE, SUBSETS, model, sealed_r2
from casbot_calib.data import write_json
from casbot_calib.model_runtime import collision_for, model_for
from casbot_calib.staged_attribution import arrays, formal_test_untouched
from casbot_calib.staged_collection import load_trial
from casbot_calib.staged_identification import read_record
from casbot_calib.staged_motion import derivative_bounds, gate_motion, wrist_probe_motion
from casbot_calib.staged_multiconfiguration import _write_motion
from casbot_calib.staged_protocol import _read, load_protocol

C02 = 'train_c02'
ROLL_RADIUS = .025
PITCH_RECIPE = dict(frequency_hz=4., amplitude_rad=.0035, bias_speed_rad_s=.12, duration_s=.5)
COUPLING_RECIPE = dict(frequency_hz=6., amplitude_rad=.00105, bias_speed_rad_s=.065, duration_s=.5)
# Keep the stricter velocity limit from the dedicated probe and jerk limit
# from the multi-axis workflow; the acceleration limit is common to both.
LIMITS = (1.,8.,60.)
TRANSITION_S = 2.


def rms(value, axis=0):
    return np.sqrt(np.mean(value**2,axis=axis))


def bin_rms(t, values, hz):
    # Exact integer-cycle window; use both quadratures. Inputs include no
    # extrapolated or acquired signal, only the frozen reference calculation.
    phase=2*np.pi*hz*t
    return float(np.hypot(values@np.cos(phase), values@np.sin(phase))*np.sqrt(2.)/len(t))


def original_support(root, manifest, worst, config, models):
    names=_read(root/'session.json')['joint_names']
    chosen=[]; step=config['inertia_sample_period_ns']*1e-9
    for r in manifest['trials']:
        if (r['split'],r['center_id'],r['stage'])!=('train',C02,'inertia'):
            continue
        name=r['trial_id'];m,_=load_trial(root,name)
        _,obs=read_record(root,name,m,names)
        period=m['conditions']['period_s']
        start=m['knots_s'][m['labels'].index('periodic')]+2*period
        t=obs['sample_ns'][2:-2]*1e-9
        q=obs['q'][2:-2]; v=obs['dq']
        where=np.flatnonzero((t>=start)&(t<start+2*period)&(abs(q[:,ROLL]-worst[ROLL])<=ROLL_RADIUS))
        if not len(where):continue
        closest=where[np.argmin(np.linalg.norm(q[where]-worst,axis=1))]
        # Smooth ddq for a descriptive ranking of the old train trajectories.
        # This is not used to estimate parameters or to select observed torque.
        acc=(v[:-4]-8*v[1:-3]+8*v[3:-1]-v[4:])/(12*step)
        acc=gaussian_filter1d(acc,1/(2*np.pi*30*step),axis=0,truncate=3)
        query=where[::8]
        cross={}
        for label,engine in models.items():
            parts=engine.evaluate_batch(q[query],v[query+2],acc[query])
            cross[label]=rms(parts['mass_matrix'][:,PITCH,:]*acc[query]).tolist()
        chosen.append(dict(trial=name,reference_id=m['conditions']['reference_id'],
            acquisition_batch_id=m['conditions']['acquisition_batch_id'],near_roll_samples=len(where),
            pitch_rad=[float(q[where,PITCH].min()),float(q[where,PITCH].max())],
            roll_rad=[float(q[where,ROLL].min()),float(q[where,ROLL].max())],
            nearest_time_s=float(t[closest]),nearest_q_rad=q[closest].tolist(),
            nearest_pose_l2_rad=float(np.linalg.norm(q[closest]-worst)),
            pitch_acceleration_rms_rad_s2=float(rms(acc[query,PITCH])),
            cross_inertia_rms_by_axis_nm=cross))
    return chosen


def probe(center, config, direction, coupled=None):
    prefix=f'{C02}_right_pitch_pair_{"neg" if direction<0 else "pos"}'
    m=wrist_probe_motion(prefix+('_single' if coupled is None else f'_plus_j{coupled:02d}'),
        config['home_rad'],center,{**config,'wrist_probe':PITCH_RECIPE,'transition_s':TRANSITION_S},
        PITCH,direction,dict(center_id=C02,center_rad=center.tolist(),role='offline_diagnostic',
        active_joints=[PITCH] if coupled is None else [PITCH,coupled],
        paired_reference=prefix,split=None,hardware_executable=False))
    if coupled is not None:
        extra=wrist_probe_motion('component',config['home_rad'],center,
            {**config,'wrist_probe':COUPLING_RECIPE,'transition_s':TRANSITION_S},coupled,direction)
        np.testing.assert_array_equal(m.knots,extra.knots)
        np.testing.assert_array_equal(m.labels,extra.labels)
        for a,b in zip(m.states,extra.states):
            a[:,coupled]=b[:,coupled]
        m.conditions.update(protocol='paired sustained signed pitch with a frequency-separated coupled axis',
                            coupling_axis=coupled,coupling_waveform=COUPLING_RECIPE)
    return m


def compare(single,multi,engines):
    f=single.polynomial();g=multi.polynomial()
    j=multi.conditions['coupling_axis'];unchanged=[k for k in range(14) if k!=j]
    np.testing.assert_array_equal(f.x,g.x)
    np.testing.assert_array_equal(f.c[:,:,unchanged],g.c[:,:,unchanged])
    a,b=single.windows('probe_measure')[0][0],single.windows('probe_measure')[-1][1]
    step=single.conditions['sample_period_ns']*1e-9
    t=np.arange(round((b-a)/step))*step
    q,v,ddq=(f(a+t,nu=k) for k in range(3));Q,V,A=(g(a+t,nu=k) for k in range(3))
    reports={}
    for label,engine in engines.items():
        p=engine.evaluate_batch(q,v,ddq);m=engine.evaluate_batch(Q,V,A)
        own=p['mass_matrix'][:,PITCH,PITCH]*ddq[:,PITCH]
        delta_own=(m['mass_matrix'][:,PITCH,PITCH]*A[:,PITCH])-own
        cross=m['mass_matrix'][:,PITCH,j]*A[:,j]
        vel=m['velocity'][:,PITCH]-p['velocity'][:,PITCH]
        grav=m['gravity'][:,PITCH]-p['gravity'][:,PITCH]
        delta=m['total'][:,PITCH]-p['total'][:,PITCH]
        closure=float(np.max(abs(delta-(delta_own+cross+vel+grav))))
        if closure>1e-10 or np.max(abs(delta_own))>1e-10:
            raise ValueError('Pair did not isolate the intended additional torque terms')
        reports[label]=dict(single_pitch_mass_kg_m2=float(np.mean(p['mass_matrix'][:,PITCH,PITCH])),
            pitch_self_4hz_rms_nm=bin_rms(t,own,4.),
            coupled_inertia_6hz_rms_nm=bin_rms(t,cross,6.),
            gravity_difference_rms_nm=float(rms(grav)),velocity_difference_rms_nm=float(rms(vel)),
            dynamic_contrast_6hz_rms_nm=bin_rms(t,delta-grav,6.),
            dynamic_contrast_4hz_rms_nm=bin_rms(t,delta-grav,4.),closure_maximum_nm=closure)
    return dict(models=reports,duration_s=float(b-a),samples=len(t),
        pitch_travel_rad=sorted(f([a,b])[:,PITCH].tolist()),
        pitch_acceleration_rms_rad_s2=float(rms(ddq[:,PITCH])),
        added_axis_travel_rad=sorted(g([a,b])[:,j].tolist()),
        roll_reference_rad=float(q[0,ROLL]))


def plan(root, r2, restoration, output):
    root,r2,restoration,output=(Path(p).resolve() for p in (root,r2,restoration,output))
    if output.exists() or any(output.is_relative_to(p) for p in (root,r2,restoration.parent)):
        raise ValueError('Use fresh output outside input artifacts')
    _,config,manifest=load_protocol(root)
    formal_test_untouched(root)
    protect_observations(root,{(r['trial_id'],) for r in manifest['trials'] if r['split']=='train'})
    r2_spec=sealed_r2(root,r2)
    query=arrays(r2/'operator_probe_set.npz')
    states=[arrays(r2/'variants'/f'ridge_{RIDGE:g}_{s}'/'solution.npz') for s in SUBSETS]
    full,omit=[s['mass'][:,PITCH,PITCH] for s in states]
    difference=(omit-full)/np.maximum(full,r2_spec['settings']['mass_diagonal_floor_kg_m2'])
    idx=int(np.argmax(abs(difference)))
    if idx!=_read(restoration)['worst_measured_probe']:
        raise ValueError('Diagnosis and frozen R2 worst point differ')
    nominal,_=model_for(config)
    static=arrays(r2/'coordinates.npz')['static']
    engines={'nominal':nominal,**{label:model(nominal,s['vector'],static) for label,s in zip(SUBSETS,states)}}
    worst=query['q'][idx]
    support=original_support(root,manifest,worst,config,engines)
    nearest=min(support,key=lambda row:row['nearest_pose_l2_rad'])
    repeats=[row for row in support if row['reference_id']==nearest['reference_id']]
    lower=max(row['pitch_rad'][0] for row in repeats); upper=min(row['pitch_rad'][1] for row in repeats)
    center=worst.copy();center[PITCH]=(lower+upper)/2
    singles={d:probe(center,config,d) for d in (1,-1)}
    profile=collision_for(config,scene=_read(root/'session.json')['collision']['scene'])
    clearance=profile.metadata['scene']['clearance_m']
    floor=config['p0']['frequency_qualification']['minimum_bin_rms_nm']
    trials={};gates={}
    def certify(m):
        if m.name not in gates:
            curve=m.polynomial()
            envelope=[float(np.max(derivative_bounds(curve,k))) for k in (1,2,3)]
            window=[i for i,l in enumerate(m.labels) if l=='probe_measure']
            speeds={str(j):float(np.min(m.conditions['direction']*curve.derivative().c[:,window,j]))
                    for j in m.conditions['active_joints']}
            gate=gate_motion(profile,m,clearance)
            gates[m.name]=dict(gate,collision_and_urdf_passed=gate['passed'],
                passed=gate['passed'] and all(x<=y for x,y in zip(envelope,LIMITS)) and min(speeds.values())>.02,
                diagnostic_derivative_bounds=envelope,diagnostic_derivative_limits=LIMITS,minimum_signed_speed_rad_s=speeds)
        return gates[m.name]
    if not all(certify(m)['passed'] for m in singles.values()):
        raise ValueError('Original pose single-axis reference not certified')
    for j in range(7,12):
        motions={d:probe(center,config,d,j) for d in singles}
        metrics={str(d):compare(singles[d],m,engines) for d,m in motions.items()}
        amplitude=min(p['dynamic_contrast_6hz_rms_nm'] for a in metrics.values() for p in a['models'].values())
        historical=min(np.mean([row['cross_inertia_rms_by_axis_nm'][label][j] for row in repeats]) for label in engines)
        usable_pitch=all(a['pitch_travel_rad'][0]>=lower and a['pitch_travel_rad'][1]<=upper for a in metrics.values())
        trials[j]=dict(motions=motions,metrics=metrics,dynamic_contrast_minimum_nm=amplitude,
            observed_support_cross_inertia_rms_minimum_nm=float(historical),
            within_original_pitch_interval=usable_pitch)
    # Among signal-bearing references, prioritize the coupling term actually
    # present in the closest original c02 waveform, not an arbitrary extra axis.
    ranked=sorted(trials,key=lambda j:trials[j]['observed_support_cross_inertia_rms_minimum_nm'],reverse=True)
    chosen=None
    for j in ranked:
        option=trials[j]
        if not option['within_original_pitch_interval'] or option['dynamic_contrast_minimum_nm']<floor:continue
        results=[certify(m)['passed'] for m in option['motions'].values()]
        if all(results):
            chosen=j
            break
    if chosen is None:raise ValueError('No same-pose paired waveform has signal and a continuous reference proof')
    output.mkdir(parents=True,exist_ok=False)
    plans=[]
    for d in (1,-1):
        for m in (singles[d],trials[chosen]['motions'][d]):
            gate=gates[m.name]
            _write_motion(output,m,gate)
            plans.append(dict(name=m.name,direction=d,active_joints=m.conditions['active_joints'],
                duration_s=m.duration,minimum_sampled_clearance_m=gate['minimum_distance_m'],
                derivative_bounds=gate['diagnostic_derivative_bounds'],
                minimum_signed_speed_rad_s=gate['minimum_signed_speed_rad_s']))
    sequence=[[singles[1].name,trials[chosen]['motions'][1].name,trials[chosen]['motions'][-1].name,singles[-1].name],
              [trials[chosen]['motions'][1].name,singles[1].name,singles[-1].name,trials[chosen]['motions'][-1].name]]
    formal_test_untouched(root)
    write_json(output/'report.json',dict(scope='offline references for a paired diagnostic, not a training augmentation',
        model_context=config['model_context'],original_c02_train=support,source_r2=str(r2),fixed_queries=len(full),
        worst_query_index=idx,worst_query_roll_rad=float(worst[ROLL]),worst_query_pitch_rad=float(worst[PITCH]),
        pitch_interval_at_matched_roll_rad=[lower,upper],center_rad=center.tolist(),clearance_gate_m=clearance,
        pitch_recipe=PITCH_RECIPE,coupling_recipe=COUPLING_RECIPE,transition_s=TRANSITION_S,signal_floor_nm=floor,
        selected_axis=chosen,selected_axis_name=_read(root/'session.json')['joint_names'][chosen],
        coupled_candidates={str(j):dict({k:v for k,v in a.items() if k!='motions'},
            reference_checks={str(d):gates.get(m.name,{}).get('passed') for d,m in a['motions'].items()})
            for j,a in trials.items()},
        suggested_order_by_batch=sequence,proposed_record_count=sum(map(len,sequence)),plans=plans,
        formal_test_consumed=False,observations_collected=False,truth_read=False,hardware_executable=False))
    print('Selected axis',chosen, 'center',center, 'plans',plans,flush=True)


def verify(root, r2, output):
    """Read back the exported curves and check the proposed 30 Hz readout."""
    report=_read(output/'report.json')
    _,config,_=load_protocol(root)
    nominal,_=model_for(config)
    coord=arrays(r2/'coordinates.npz')
    engines={s:model(nominal,arrays(r2/'variants'/f'ridge_{RIDGE:g}_{s}'/'solution.npz')['vector'],coord['static'])
             for s in SUBSETS}
    joint=report['selected_axis']
    dt=config['inertia_sample_period_ns']*1e-9
    sos=butter(6,30.,fs=1/dt,output='sos')
    readout={}
    for direction in (1,-1):
        single,multi=[p for p in report['plans'] if p['direction']==direction]
        curves=[]
        for p in (single,multi):
            bundle=output/'plans'/p['name']
            meta,gate=read_certified_reference(bundle)
            stored=arrays(bundle/'reference.npz')
            curve=BPoly(stored['coefficients'],stored['breakpoints'])
            rebuilt=BPoly.from_derivatives(meta['knots_s'],meta['states'])
            np.testing.assert_array_equal(curve.x,rebuilt.x)
            np.testing.assert_array_equal(curve.c,rebuilt.c)
            if gate['diagnostic_derivative_bounds']!=p['derivative_bounds'] or not meta['initial_final_home']:
                raise ValueError('Saved plan disagrees with the reference envelope')
            curves.append(curve)
        changed=[j for j in range(14) if not np.array_equal(curves[0].c[:,:,j],curves[1].c[:,:,j])]
        if changed!=[joint]:raise ValueError('Only the selected coupling axis may change')
        measure=np.flatnonzero(np.asarray(meta['labels'])=='probe_measure')
        interval=curves[0]([curves[0].x[measure[0]],curves[0].x[measure[-1]+1]])
        pitch_interval=sorted(interval[:,PITCH].tolist())
        lo,hi=report['pitch_interval_at_matched_roll_rad']
        if not lo<=pitch_interval[0]<=pitch_interval[1]<=hi:
            raise ValueError('Saved pitch scan leaves the original c02 interval')
        t=np.arange(round(meta['duration_s']/dt))*dt
        start=int(round(curves[0].x[measure[0]]/dt));n=round(PITCH_RECIPE['duration_s']/dt)
        support=slice(start,start+n)
        tau=np.arange(n)*dt
        trend=2*tau/(n*dt)-1
        # Jointly fit the two frequencies with a constant/quadratic drift
        # nuisance; unlike a shortened FFT this retains both quadratures.
        X=np.column_stack([np.ones(n),trend,trend**2,*[
            v(2*np.pi*f*tau) for f in (4.,6.) for v in (np.sin,np.cos)]])
        phase_rows={}
        for label,engine in engines.items():
            parts=[engine.evaluate_batch(*[curve(t,nu=k) for k in range(3)]) for curve in curves]
            S,M=[(x['inertia']+x['velocity'])[:,PITCH] for x in parts]
            filtered=sosfiltfilt(sos,np.column_stack((S,M-S)),axis=0)[support]
            c=np.linalg.lstsq(X,filtered,rcond=None)[0]
            phase_rows[label]=dict(single_4hz_dynamic_rms_nm=float(np.linalg.norm(c[3:5,0])/np.sqrt(2)),
                paired_6hz_dynamic_rms_nm=float(np.linalg.norm(c[5:7,1])/np.sqrt(2)),
                paired_4hz_leakage_rms_nm=float(np.linalg.norm(c[3:5,1])/np.sqrt(2)))
        readout[str(direction)]=dict(samples=n,certified_monotone_pitch_interval_rad=pitch_interval,
            design_condition_number=float(np.linalg.cond(X)),models=phase_rows)
    write_json(output/'verification.json',dict(reference_files_reconstructed=True,
        pair_controlled_axis=joint,matched_30hz_reference_readout=readout,
        scope='model-predicted force balance on persisted references; no acquisition',observations_collected=False))
    print('Verified four stored references and matched 30 Hz force readout',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for arg in ('root','r2','restoration','output'):
        parser.add_argument(arg,type=Path)
    parser.add_argument('--verify-only',action='store_true')
    args=parser.parse_args()
    if args.verify_only:
        verify(args.root,args.r2,args.output)
    else:
        plan(args.root,args.r2,args.restoration,args.output)
