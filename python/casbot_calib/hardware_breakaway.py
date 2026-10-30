"""SDK operational onset ramps; callbacks are never labelled acquisition time."""
import csv
from pathlib import Path
import numpy as np
from . import hardware_field as field
from .data import write_json
from .staged_motion import Motion
from .static_field import _move as bounded_move
from .vendor_identity import sha256


def validate_settings(c):
    if type(c['enabled']) is not bool:raise ValueError('Boolean breakaway enable required')
    if c['directions']!=[-1,1] or type(c['repeats']) is not int or c['repeats']!=2:raise ValueError('Two directions and independent two-repeat ramp design required')
    for key in ('dwell_s','slopes_nm_s'):
        if not c[key] or len(set(c[key]))!=len(c[key]) or not np.isfinite(c[key]).all() or min(c[key])<=0:raise ValueError('Distinct positive ramp conditions required')
    for key in ('ramp_s','zero_pd_dwell_s','recovery_hold_s','maximum_increment_nm','displacement_rad','speed_rad_s',
                'maximum_departure_rad','maximum_speed_rad_s','onset_speed_resolution_rad_s','still_speed_rad_s',
                'validation_tolerance_nm','torque_noise_allowance_nm','other_hold_allowance_rad','hold_natural_frequency_rad_s','hold_damping_ratio','hold_minimum_kp','hold_minimum_kd'):
        if not np.isfinite(c[key]) or c[key]<=0:raise ValueError('Positive ramp condition required: '+key)
    if type(c['sustain_ns']) is not int or not 1000000<=c['sustain_ns']<=100000000:raise ValueError('Bounded callback confirmation duration required')
    if c['zero_pd_dwell_s']<.25 or c['maximum_departure_rad']<=c['displacement_rad'] or c['maximum_speed_rad_s']<=c['speed_rad_s']:raise ValueError('Invalid ramp guard envelope')
    if c['gravity_source'] not in ('host_internal','external') or c['effort_semantics'] not in ('additive_output_nm_assumed','additive_output_nm_user_confirmed') or c['custom_pd_semantics'] not in ('explicit_zero_pd_applied_assumed','explicit_zero_pd_user_confirmed') or c['internal_friction_compensation'] not in ('assumed_disabled','user_confirmed_none'):raise ValueError('Explicit commissioning assumptions required for effort/PD/internal compensation')


def jobs(c,hz):
    b=c['breakaway'];rows=[]
    if not b['enabled']:return rows
    for joint in c['joints']:
        for pose,offset in enumerate(c['pose_offsets_rad']['train']):
            for direction in b['directions']:
                for dwell in b['dwell_s']:
                    for slope in b['slopes_nm_s']:
                        for repeat in range(2):
                            rows.append(dict(id=f'breakaway_{len(rows):04}',kind='breakaway',joint=joint,hz=hz,
                                pose=pose,pose_offset_rad=offset,direction=direction,dwell_s=dwell,slope_nm_s=slope,
                                repeat=repeat,split='train' if repeat==0 else 'validation'))
    return rows


def prepare_trial(root,protocol,job,profile,start,pose,snapshot,output):
    from .hardware_friction import copy_gravity
    from .dual_model import nominal_dual_arm
    c=protocol['h3'];b=c['breakaway'];j=protocol['joint_names'].index(job['joint'])
    allowance=np.full(14,b['other_hold_allowance_rad']);allowance[j]=b['maximum_departure_rad']
    conditions=dict(**job,joint_index=j,position_allowance_rad=allowance.tolist(),
        allowance_labels=['static_dwell','torque_ramp','recovery_hold','return_measured_start'],joint_box_refinement=True)
    m=Motion(job['id'],'breakaway',start,conditions)
    envelope=dict(speed_rad_s=.3,acceleration_rad_s2=1.,jerk_rad_s3=10.)
    m.add(profile['initial_hold_s'],start,label='observed_start_hold')
    bounded_move(m,pose,c['transition_s'],envelope,'to_ramp_pose');m.add(c['settle_s'],pose,label='pose_settle')
    m.add(job['dwell_s'],pose,label='position_dwell')
    free=m.duration;m.add(b['zero_pd_dwell_s'],pose,label='static_dwell')
    begin=m.duration;m.add(b['ramp_s'],pose,label='torque_ramp');end=m.duration
    m.add(b['recovery_hold_s'],pose,label='recovery_hold');return_begin=m.duration
    bounded_move(m,start,c['transition_s'],envelope,'return_measured_start');return_end=m.duration
    m.add(profile['final_hold_s'],start,label='final_hold')
    out=field.output_outside(output,Path(root)/'protocol',snapshot);out.mkdir(parents=True,exist_ok=False)
    copy_gravity(Path(root)/'protocol/gravity',out/'gravity');write_json(out/'h3.json',c)
    from .hardware_friction import holding_gains
    kp,kd=holding_gains(pose,b)
    if np.max(kp)>1000 or np.max(kd)>100:raise ValueError('Nominal holding gains outside declared commissioning range')
    p=dict(breakaway_enabled=True,breakaway_joint=j,breakaway_direction=job['direction'],
        breakaway_free_begin_ns=round(free*1e9),breakaway_ramp_begin_ns=round(begin*1e9),breakaway_ramp_end_ns=round(end*1e9),
        breakaway_return_begin_ns=round(return_begin*1e9),breakaway_return_end_ns=round(return_end*1e9),
        breakaway_sustain_ns=b['sustain_ns'],breakaway_slope_nm_s=float(job['slope_nm_s']),
        breakaway_maximum_increment_nm=b['maximum_increment_nm'],breakaway_displacement_rad=b['displacement_rad'],
        breakaway_speed_rad_s=b['speed_rad_s'],breakaway_maximum_departure_rad=b['maximum_departure_rad'],
        breakaway_maximum_speed_rad_s=b['maximum_speed_rad_s'],breakaway_still_speed_rad_s=b['still_speed_rad_s'],
        breakaway_other_allowance_rad=b['other_hold_allowance_rad'],command_kp=kp.tolist(),command_kd=kd.tolist(),
        gravity_feedforward_enabled=b['gravity_source']=='external')
    if b['gravity_source']=='external':p.update(gravity_model_path=str(out/'gravity/candidate_exploratory.urdf'),gravity_world=profile['gravity_world'])
    profile=dict(profile,breakaway_parameters=p)
    return field.finish_plan(out,profile,m,[],dict(**field.snapshot_binding(snapshot,profile),h3=True,job=job,
        ramp_windows=dict(free_begin_s=free,ramp_begin_s=begin,ramp_end_s=end,return_begin_s=return_begin,return_end_s=return_end),
        command_assumptions={k:b[k] for k in ('gravity_source','effort_semantics','custom_pd_semantics','internal_friction_compensation')}))


def inspect_trial(plan,capture,output,metadata,commands,origin,obs,elapsed,collision,c):
    from .hardware_friction import read,gravity_engine
    b=c['breakaway'];job=metadata['job'];j=metadata['joint_names'].index(job['joint']);sign=job['direction']
    guard=read(Path(capture)/'execution/guard.json')
    if not guard['confirmed'] or guard['fault'] or guard['sample_time_known']:raise ValueError('Exactly one callback-based ramp confirmation required')
    times=obs['receive_ns'];w=metadata['ramp_windows'];confirmation=guard['confirmed_callback_elapsed_ns']*1e-9
    pre=(elapsed>=w['ramp_begin_s']-.2)&(elapsed<w['ramp_begin_s']-.02)
    if sum(pre)<5 or np.max(abs(obs['dq'][pre,j]))>b['still_speed_rad_s']:raise ValueError('Target was not stationary in zero-PD dwell')
    mask=(elapsed>=w['ramp_begin_s'])&(elapsed<=confirmation);indices=np.flatnonzero(mask)
    moving=np.flatnonzero(sign*obs['dq'][indices,j]>b['onset_speed_resolution_rad_s'])
    if not len(moving) or moving[0]==0:raise ValueError('No callback still-to-moving onset bracket')
    first=indices[moving[0]];last=first-1;confirmed=np.searchsorted(elapsed,confirmation)
    if confirmed>=len(elapsed):raise ValueError('Confirmation callback missing')
    if np.max(np.diff(times[pre|mask]))*1e-9>c['maximum_gap_s']:raise ValueError('Onset bracket spans a callback gap')
    profile=metadata['config'];effort=(obs['effort_raw']-profile['effort_zero_nm'])*profile['effort_gain']
    g=gravity_engine(Path(plan)/'gravity').gravity_batch(obs['q'][[last,first,confirmed]])[:,j]
    values=sign*(effort[[last,first,confirmed],j]-g)
    lo=float(min(values[:2])-b['torque_noise_allowance_nm']);hi=float(max(values[:2])+b['torque_noise_allowance_nm'])
    # Audit the actual SDK command stream, not just the requested ramp metadata.
    in_free=[r for r in commands if w['free_begin_s']<=int(r['planned_index'])*metadata['command_period_ns']*1e-9<confirmation]
    if not in_free or any(float(r['kp'+str(j)])!=0 or float(r['kd'+str(j)])!=0 for r in in_free):raise ValueError('Executed target PD was not zero during free dwell/ramp')
    after=[r for r in commands if int(r['publish_ns'])-origin>guard['decision_elapsed_ns']+metadata['command_period_ns']]
    if not after or any(float(r['ramp_increment_nm'])!=0 for r in after):raise ValueError('Ramp continued after motion confirmation')
    if sign*(obs['q'][confirmed,j]-guard['baseline_q_rad'])<b['displacement_rad']*.8:raise ValueError('Confirmation displacement inconsistent with feedback')
    report=dict(passed=bool(lo>0 and hi-lo<=b['validation_tolerance_nm']),stage='breakaway',job=job,
        estimate_nm=float(np.mean(values[:2])),callback_onset_interval_nm=[lo,hi],operational_confirmation_nm=float(values[2]),
        last_still_callback_ns=int(times[last]),first_moving_callback_ns=int(times[first]),
        confirmation_callback_ns=int(times[confirmed]),guard=guard,callback_bracket_width_s=float((times[first]-times[last])*1e-9),
        physical_onset_interval_known=False,sample_time_known=False,hardware_calibration_accepted=False,
        limitations=['Cache age, asynchronous fields and effort filtering unknown; interval is operational callback evidence only',
                    'Gravity validation/approach uncertainty reported separately; not eliminated by a fast ramp'],
        source_plan_manifest_sha256=sha256(Path(plan)/'manifest.json'),source_capture_manifest_sha256=sha256(Path(capture)/'manifest.json'))
    out=Path(output);out.mkdir(parents=True,exist_ok=False);write_json(out/'report.json',report)
    write_json(out/'collision_received_interpolant.json',collision);field.seal(out);return report


def result(root,protocol,history):
    from .hardware_batch import accepted
    from .hardware_friction import read
    selected=[j for j in protocol['jobs'] if j['kind']=='breakaway']
    if not selected:return dict(available=False,passed=not protocol['h3']['breakaway']['enabled'])
    if any(not accepted(history[j['id']]) for j in selected):return dict(available=False,passed=False)
    rows=[]
    for job in selected:
        source=accepted(history[job['id']]);r=read(Path(root)/source['attempt']/'inspection/report.json')
        rows.append(dict(**r,source_attempt=source['attempt'],manifest_sha256=source['manifest_sha256']))
    axes=[]
    for joint in protocol['h3']['joints']:
        directions={}
        for sign in (-1,1):
            train=[r['estimate_nm'] for r in rows if r['job']['joint']==joint and r['job']['direction']==sign and r['job']['split']=='train']
            validation=[r['estimate_nm'] for r in rows if r['job']['joint']==joint and r['job']['direction']==sign and r['job']['split']=='validation']
            estimate=float(np.median(train));error=np.asarray(validation)-estimate
            directions[str(sign)]=dict(estimate_nm=estimate,validation_rmse_nm=float(np.sqrt(np.mean(error**2))),
                passed=bool(np.max(abs(error))<=protocol['h3']['breakaway']['validation_tolerance_nm']))
        axes.append(dict(joint=joint,directions=directions,passed=all(d['passed'] for d in directions.values())))
    gravity=read(Path(root)/'protocol/gravity/report.json')
    return dict(available=True,passed=all(a['passed'] for a in axes) and all(r['passed'] for r in rows),axes=axes,ramps=rows,
        gravity_validation_rmse_nm=gravity['validation_rmse_nm'],gravity_approach_half_spread_nm=gravity['paired_approach_half_spread_nm'],
        sample_time_known=False,physical_onset_interval_known=False,formal_hardware_calibration_accepted=False)
