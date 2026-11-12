#!/usr/bin/env python3
"""Read captured A/B observations and score the frozen 4/6 Hz diagnostic."""
import argparse
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfiltfilt

from collect_right_pitch_motion_pair import PITCH, ROLL, SHOULDER, STAGE
from design_wrist_supplement import protect_observations
from diagnose_wrist_periodic_counterfactual import RIDGE, SUBSETS, model, sealed_r2
from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for
from casbot_calib.staged_attribution import arrays, check_manifest, formal_test_untouched
from casbot_calib.staged_collection import load_trial
from casbot_calib.staged_identification import read_record
from casbot_calib.staged_protocol import _read, _unseal, file_hash, load_protocol
from casbot_calib.torque_observations import output_torque


def rms(x):
    return float(np.sqrt(np.mean(np.asarray(x)**2)))


def quadratures(t, data):
    u=2*t/(t[-1]+t[1])-1
    X=np.column_stack([np.ones(len(t)),u,u*u,*[f(2*np.pi*hz*t) for hz in (4,6) for f in (np.sin,np.cos)]])
    # Two vectors of sine/cosine RMS coefficients, preserving the direction of
    # signed phase projections (rather than only comparing amplitudes).
    coef=np.linalg.lstsq(X,data,rcond=None)[0][3:]/np.sqrt(2)
    return coef.reshape(2,2,-1)


def phase_summary(obs, pred):
    norm=float(np.linalg.norm(pred))
    return dict(observed_sin_cos_rms_nm=obs.tolist(), predicted_sin_cos_rms_nm=pred.tolist(),
        observed_rms_nm=float(np.linalg.norm(obs)), predicted_rms_nm=norm,
        observed_in_predicted_phase_rms_nm=float(obs@pred/norm), residual_rms_nm=float(np.linalg.norm(obs-pred)))


def record_rows(capture, name, config, joint_names, engines, coord, policy):
    meta,curve=load_trial(capture,name)
    record,obs=read_record(capture,name,meta,joint_names)
    c=meta['conditions'];active=c['active_joints'];held=[k for k in range(14) if k not in active]
    if (c['split'] is not None or c['role']!='diagnostic' or meta['stage']!=STAGE
            or record['control_period_ns']!=config['control_period_ns']
            or record['sample_period_ns']!=config['inertia_sample_period_ns']):
        raise ValueError('Wrong diagnostic role or clock')
    step=record['sample_period_ns'];dt=step*1e-9
    measure=[i for i,l in enumerate(meta['labels']) if l=='probe_measure']
    lo,hi=[round(meta['knots_s'][i]*1e9) for i in (measure[0],measure[-1]+1)]
    if hi-lo!=500_000_000:raise ValueError('Expected the complete 0.50 s measurement window')
    time=obs['sample_ns'][2:-2];i,j=np.searchsorted(time,[lo,hi]);use=slice(i,j)
    np.testing.assert_array_equal(time[use],np.arange(lo,hi,step))
    q=obs['q'][2:-2];v=obs['dq'];w=v[2:-2]
    a=(v[:-4]-8*v[1:-3]+8*v[3:-1]-v[4:])/(12*dt)
    tau=output_torque(obs,config['drive']['gain_nm_a'])[2:-2,PITCH]
    sos=butter(6,30.,fs=1/dt,output='sos')
    ref=curve(time[use]*1e-9)
    # Apply the same finite record filter to observation and all evaluated
    # terms, then crop. Never move/crop the raw data to maximize agreement.
    band=sosfiltfilt(sos,np.column_stack([w[:,PITCH],a[:,PITCH],w[:,SHOULDER],a[:,SHOULDER],tau]),axis=0)[use]
    local_t=(time[use]-lo)*1e-9
    kin=quadratures(local_t,band)
    sign=c['direction']; signed=np.min(sign*w[use][:,active],axis=0)
    hold_error=float(np.max(abs(q[use][:,held]-ref[:,held])))
    held_speed=float(np.max(abs(w[use][:,held])))
    qc=dict(actual_continuous_path=record['actual_interpolant_gate_passed'],
        sustained_active_speed=bool(np.all(signed>policy['minimum_signed_speed_rad_s'])),
        held_speed=held_speed<=policy['maximum_held_speed_rad_s'],
        held_position=hold_error<=policy['maximum_hold_position_error_rad'],
        own_4hz_kinematics=bool(np.linalg.norm(kin[0,:,0])>=policy['minimum_speed_bin_rms_rad_s']
            and np.linalg.norm(kin[0,:,1])>=policy['minimum_acceleration_bin_rms_rad_s2']),
        added_6hz_kinematics=(SHOULDER not in active or bool(np.linalg.norm(kin[1,:,2])>=policy['minimum_speed_bin_rms_rad_s']
            and np.linalg.norm(kin[1,:,3])>=policy['minimum_acceleration_bin_rms_rad_s2'])),
        own_4hz_torque=bool(np.linalg.norm(kin[0,:,4])>=policy['minimum_bin_rms_nm']))
    # Freeze the same moving-friction nuisance for both candidates; also
    # retain a friction-free projection to show the inferred mass sensitivity.
    vj=w[:,PITCH];transition=np.exp(-(vj/.002)**2)
    b,coulomb=coord['friction'][[PITCH,14+PITCH]]
    friction=b*vj+np.sign(vj)*(coulomb*(1-transition)+coord['static'][PITCH]*transition)
    models={}
    acceleration=kin[0,:,1];acc2=acceleration@acceleration
    for key,engine in engines.items():
        channels=[]
        for offset in range(0,len(q),4096):
            s=slice(offset,offset+4096)
            p=engine.evaluate_batch(q[s],w[s],a[s])
            own=p['mass_matrix'][:,PITCH,PITCH]*a[s,PITCH]
            g=p['gravity'][:,PITCH]
            cross=p['inertia'][:,PITCH]-own+p['velocity'][:,PITCH]
            # Measured channel and all contributions share filtering and sample support.
            channels.append(np.column_stack((tau[s]-g-friction[s],own,cross,friction[s])))
        series=sosfiltfilt(sos,np.vstack(channels),axis=0)[use]
        z=quadratures(local_t,series)
        observed_own=z[0,:,0]-z[0,:,2]
        predicted_own=z[0,:,1]
        own_report=phase_summary(observed_own,predicted_own)
        own_report.update(effective_mass_kg_m2=float(observed_own@acceleration/acc2),
            predicted_effective_mass_kg_m2=float(predicted_own@acceleration/acc2),
            mass_without_friction_subtraction_kg_m2=float((observed_own+z[0,:,3])@acceleration/acc2))
        models[key]=dict(own_4hz=own_report,
            observed_dynamic_minus_self_6hz=z[1,:,0]-z[1,:,1],
            predicted_cross_and_velocity_6hz=z[1,:,2],
            observed_dynamic_6hz=z[1,:,0])
    row=dict(trial=name,batch=c['diagnostic_batch'],direction=sign,condition='B' if SHOULDER in active else 'A',
        checks=qc,samples=j-i,minimum_signed_speed_rad_s=dict(zip(map(str,active),signed.tolist())),
        maximum_held_speed_rad_s=held_speed,maximum_hold_position_error_rad=hold_error,
        minimum_actual_clearance_m=_read(capture/'runs'/name/'collision_actual.json')['minimum_sampled_distance_m'],
        roll_mean_rad=float(np.mean(q[use,ROLL])),roll_span_rad=float(np.ptp(q[use,ROLL])),
        pitch_bounds_rad=[float(q[use,PITCH].min()),float(q[use,PITCH].max())],
        pitch_speed_4hz_rms_rad_s=float(np.linalg.norm(kin[0,:,0])),
        pitch_acceleration_4hz_rms_rad_s2=float(np.linalg.norm(kin[0,:,1])),
        wrist_torque_4hz_rms_nm=float(np.linalg.norm(kin[0,:,4])),
        wrist_torque_6hz_rms_nm=float(np.linalg.norm(kin[1,:,4])),models=models)
    return row,np.column_stack([q[use,PITCH],band[:,:2]])


def run(capture, r2, output):
    capture,r2,output=(p.resolve() for p in (capture,r2,output))
    if output.exists() or output.is_relative_to(capture):
        raise ValueError('Use a new analysis directory outside capture')
    frozen=_unseal(capture,'capture_protocol.json');source=Path(frozen['source'])
    formal_test_untouched(source)
    protect_observations(source,set())
    protect_observations(capture,{(n,) for n in frozen['order']})
    capture_set=check_manifest(capture,'diagnostic_capture_manifest.json')
    check_manifest(capture,'capture_protocol.json')
    if (capture_set['trials']!=frozen['order'] or capture_set['role']!='diagnostic' or len(frozen['order'])!=8
            or capture_set['capture_protocol_sha256']!=file_hash(capture/'capture_protocol.json')
            or frozen['source_protocol_sha256']!=file_hash(source/'protocol.json')
            or frozen['source_predecessor_freeze_sha256']!=file_hash(source/'predecessor_freeze.json')):
        raise ValueError('Capture has no matching diagnostic seal')
    sealed_r2(source,r2)
    _,config,_=load_protocol(source);session=_read(capture/'session.json')
    nominal,_=model_for(config);coord=arrays(r2/'coordinates.npz')
    engines={s:model(nominal,arrays(r2/'variants'/f'ridge_{RIDGE:g}_{s}'/'solution.npz')['vector'],coord['static'])
             for s in SUBSETS}
    policy={**frozen['comparison_checks'],**config['p0']['frequency_qualification']}
    records=[record_rows(capture,n,config,session['joint_names'],engines,coord,policy) for n in frozen['order']]
    pairs=[]
    for batch in (0,1):
        for d in (1,-1):
            matching={row['condition']:(row,sig) for row,sig in records if (row['batch'],row['direction'])==(batch,d)}
            A,a=matching['A'];B,b=matching['B']
            dqmax=float(np.max(abs(a[:,0]-b[:,0])))
            v_rel=rms(a[:,1]-b[:,1])/A['pitch_speed_4hz_rms_rad_s']
            a_rel=rms(a[:,2]-b[:,2])/A['pitch_acceleration_4hz_rms_rad_s2']
            comparisons={}
            for key in engines:
                lhs,rhs=A['models'][key],B['models'][key]
                observed=rhs['observed_dynamic_minus_self_6hz']-lhs['observed_dynamic_minus_self_6hz']
                pred=rhs['predicted_cross_and_velocity_6hz']-lhs['predicted_cross_and_velocity_6hz']
                comparisons[key]=dict(coupling_6hz=phase_summary(observed,pred),
                    own_mass_A_kg_m2=lhs['own_4hz']['effective_mass_kg_m2'],
                    own_mass_B_kg_m2=rhs['own_4hz']['effective_mass_kg_m2'],
                    own_mass_B_minus_A_kg_m2=rhs['own_4hz']['effective_mass_kg_m2']-lhs['own_4hz']['effective_mass_kg_m2'])
            checks=dict(trial_checks=all(A['checks'].values()) and all(B['checks'].values()),
                wrist_position_matched=dqmax<=policy['maximum_paired_wrist_position_difference_rad'],
                wrist_velocity_matched=v_rel<=policy['maximum_paired_filtered_speed_and_acceleration_relative_rms'],
                wrist_acceleration_matched=a_rel<=policy['maximum_paired_filtered_speed_and_acceleration_relative_rms'],
                paired_6hz_signal=all(x['coupling_6hz']['observed_rms_nm']>=policy['minimum_bin_rms_nm'] for x in comparisons.values()))
            pairs.append(dict(batch=batch,direction=d,trial_A=A['trial'],trial_B=B['trial'],checks=checks,
                wrist_position_difference_max_rad=dqmax,wrist_speed_difference_relative_rms=v_rel,
                wrist_acceleration_difference_relative_rms=a_rel,models=comparisons))
    formal_test_untouched(source)
    output.mkdir(parents=True,exist_ok=False)
    report=dict(scope='captured native Mock observations; paired diagnosis only; no fit or training/test eligibility',
        capture=str(capture),formal_test_consumed=False,truth_read=False,all_pairs_qualified=all(
            all(p['checks'].values()) for p in pairs),records=[row for row,_ in records],pairs=pairs,policy=policy)
    write_json(output/'report.json',report)
    print(f"Analyzed {len(records)} records / {len(pairs)} pairs; qualified={report['all_pairs_qualified']}",flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ('capture','r2','output'):p.add_argument(arg,type=Path)
    run(**vars(p.parse_args()))
