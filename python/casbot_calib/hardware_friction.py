"""H3 SDK plateau protocol, shared friction fit and frozen-stage orchestration.

Only callback-time operational candidates are produced. No acquisition timestamps
or current are invented, and historical S3/S4 sampling policy is unchanged.
"""
import csv
import json
from pathlib import Path
import shutil
import numpy as np
import yaml
from . import hardware_field as field
from .data import write_json
from .model_runtime import model_for, check_same_context
from .rigid_identification import model_from_parameters
from .staged_motion import Motion,add_friction_sweep
from .static_field import _move as bounded_move
from .staged_identification import pair_friction_parts,fit_friction_features
from .vendor_identity import sha256


def read(path):return json.loads(Path(path).read_text())


def settings(path,names):
    c=yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if c.get('schema_version')!='casbot_hardware_friction_v1':raise ValueError('Explicit H3 protocol required')
    if c['joints']=='all':c['joints']=names
    if not c['joints'] or len(set(c['joints']))!=len(c['joints']) or not set(c['joints'])<=set(names):raise ValueError('Unique named H3 joints required')
    if np.asarray(c['home_rad']).shape!=(14,) or not np.isfinite(c['home_rad']).all():raise ValueError('Finite nominal14-axis center required')
    for key in ('stroke_half_rad','transition_s','settle_s','speed_ramp_s','plateau_trim_s'):
        if not np.isfinite(c[key]) or c[key]<=0:raise ValueError('Positive H3 condition required: '+key)
    if c['inter_job_pause_s']<0 or not np.isfinite(c['inter_job_pause_s']):raise ValueError('Invalid H3 pause')
    if type(c['pair_points']) is not int or c['pair_points']<2:raise ValueError('Spatial grid needs at least two points')
    for role in ('train','validation'):
        poses=c['pose_offsets_rad'][role];speeds=c['speeds_rad_s'][role]
        if not poses or len(set(poses))!=len(poses) or not np.isfinite(poses).all():raise ValueError('Distinct finite H3 poses required')
        if not speeds or len(set(speeds))!=len(speeds) or not np.isfinite(speeds).all() or min(speeds)<=0:raise ValueError('Distinct positive speeds required')
    if set(c['pose_offsets_rad']['train'])&set(c['pose_offsets_rad']['validation']):raise ValueError('H3 training/validation postures must be disjoint')
    from .hardware_breakaway import validate_settings
    validate_settings(c['breakaway'])
    return c


def gravity_source(path,profile):
    root=Path(path).resolve()
    if (root/'latest.json').exists():
        from .hardware_batch import load_batch
        load_batch(root);pointer=read(root/'latest.json');report_root=root/pointer['report'];field.check_seal(report_root)
        if sha256(report_root/'manifest.json')!=pointer['manifest_sha256']:raise ValueError('Accepted H2 report required')
        root=report_root/'gravity'
    field.check_seal(root);report=read(root/'report.json');calibration=read(root/'calibration.json')
    if not read(root/'urdf_reload.json')['passed']:
        raise field.FieldContractError('Gravity candidate URDF does not reproduce its numerical model')
    check_same_context(calibration,profile)
    engine,source=model_for(profile)
    if calibration['source_model_sha256']!=source['sha256'] or calibration['joint_names']!=[j.name for j in engine.model.joints]:raise ValueError('Gravity source/model mismatch')
    for key in ('effort_gain','effort_zero_nm','gravity_world'):
        if not np.allclose(np.asarray(calibration[key]),np.asarray(profile[key]),rtol=0,atol=1e-12):raise ValueError('H3 gravity/effort scale changed: '+key)
    return root,calibration,report


def gravity_engine(folder):
    c=read(Path(folder)/'calibration.json');nominal,_=model_for(c)
    model=nominal.model;model.gravity_world=np.asarray(c['gravity_world']);nominal=type(nominal)(model)
    return model_from_parameters(nominal,c['parameters'])


def center(config,offset):
    direction=np.array([.5,0,.2,-.5,.2,0,0,.5,0,.2,-.5,.2,0,0])
    return np.asarray(config['home_rad'])+offset*direction


def holding_gains(pose,b,profile=None):
    from .control_gains import nominal_pd_gains
    nominal,_=model_for({} if profile is None else profile)
    return nominal_pd_gains(nominal,pose,b['hold_natural_frequency_rad_s'],b['hold_damping_ratio'],b['hold_minimum_kp'],b['hold_minimum_kd'])


def plan_friction(config_path,profile_path,gravity,output,*,domain_id):
    from . import hardware_batch as batch
    from .sdk_connection import connection_remaps
    profile=field.load_profile(profile_path);connection_remaps(profile,domain_id)
    if type(domain_id) is not int or not 0<=domain_id<=232:raise ValueError('Explicit H3 ROS domain required')
    engine,source=model_for(profile);names=[j.name for j in engine.model.joints];c=settings(config_path,names)
    origin,calibration,report=gravity_source(gravity,profile)
    jobs=[]
    for role in ('train','validation'):
        for p,offset in enumerate(c['pose_offsets_rad'][role]):
            for joint in c['joints']:
                for speed in c['speeds_rad_s'][role]:
                    jobs.append(dict(id=f'friction_{len(jobs):04}',kind='moving_friction',joint=joint,hz=profile['command_hz'],
                        split=role,pose=p,pose_offset_rad=offset,speed=speed))
    from .hardware_breakaway import jobs as ramp_jobs
    jobs.extend(ramp_jobs(c,profile['command_hz']))
    root=field.output_outside(output,origin);root.mkdir(parents=True,exist_ok=False);protocol=root/'protocol';protocol.mkdir()
    source_gravity=protocol/'gravity';source_gravity.mkdir()
    copy_gravity(origin,source_gravity,created=True)
    write_json(source_gravity/'source_binding.json',dict(source_manifest_sha256=sha256(origin/'manifest.json'),
        files={name:sha256(origin/name) for name in ('calibration.json','report.json','candidate_exploratory.urdf','urdf_reload.json')}))
    field.seal(source_gravity)
    write_json(protocol/'profile.json',profile);write_json(protocol/'h3_settings.json',c)
    (protocol/'scene.yaml').write_text(yaml.safe_dump(profile['scene']),encoding='utf-8')
    generic=dict(direction=dict(enabled=False,joints=[]),frequency=dict(enabled=False,joints=[]),
        static=dict(enabled=False),inter_job_pause_s=c['inter_job_pause_s'])
    record=dict(schema_version='casbot_hardware_batch_protocol_v1',workflow='h3_friction',jobs=jobs,domain_id=domain_id,
        connection_scope=profile.get('connection',{}).get('scope','hardware'),joint_names=names,source_model_sha256=source['sha256'],
        implementation=batch.implementation(),config=generic,h3=c,profile=profile,
        recovery_policy='log_noop_and_terminate_session_no_fault_resume',hardware_calibration_accepted=False,device_writes_during_planning=0)
    write_json(protocol/'protocol.json',record);field.seal(protocol)
    batch.event(root,'planned',protocol_sha256=sha256(protocol/'manifest.json'))
    return batch.summarize(root,'planned')


def prepare_trial(root,protocol,job,profile_path,snapshot,output):
    c=protocol['h3'];profile=field.load_profile(profile_path);start=field._read_snapshot(snapshot);binding=field.snapshot_binding(snapshot,profile)
    names=protocol['joint_names'];j=names.index(job['joint']);pose=center(c,job['pose_offset_rad'])
    if job['kind']=='breakaway':
        from .hardware_breakaway import prepare_trial as ramp_prepare
        return ramp_prepare(root,protocol,job,profile,start,pose,snapshot,output)
    target=pose.copy();target[j]-=c['stroke_half_rad']
    m=Motion(job['id'],'moving_friction',start,dict(**job,joint_index=j))
    m.add(profile['initial_hold_s'],start,label='observed_start_hold')
    envelope=dict(speed_rad_s=.3,acceleration_rad_s2=1.,jerk_rad_s3=10.)
    bounded_move(m,target,c['transition_s'],envelope,'to_friction_start');m.add(c['settle_s'],target,label='pre_scan_settle')
    add_friction_sweep(m,j,c['stroke_half_rad'],job['speed'],c['speed_ramp_s'],c['settle_s'])
    bounded_move(m,start,c['transition_s'],envelope,'return_measured_start');m.add(profile['final_hold_s'],start,label='final_hold')
    out=field.output_outside(output,Path(root)/'protocol',snapshot);out.mkdir(parents=True,exist_ok=False)
    copy_gravity(Path(root)/'protocol/gravity',out/'gravity')
    write_json(out/'h3.json',c)
    kp,kd=holding_gains(pose,c['breakaway'],profile)
    profile=dict(profile,breakaway_parameters=dict(gravity_feedforward_enabled=True,
        gravity_model_path=str(out/'gravity/candidate_exploratory.urdf'),gravity_world=profile['gravity_world'],
        command_kp=kp.tolist(),command_kd=kd.tolist()))
    return field.finish_plan(out,profile,m,[],dict(**binding,h3=True,job=job))


def copy_gravity(source,target,*,created=False):
    from .urdf_resources import rebase_resources
    import xml.etree.ElementTree as ET
    source,target=Path(source),Path(target)
    if created:
        for name in ('calibration.json','report.json','candidate_exploratory.urdf','urdf_reload.json'):
            shutil.copyfile(source/name,target/name)
    else:shutil.copytree(source,target)
    path=target/'candidate_exploratory.urdf'
    tree=ET.parse(path);rebase_resources(tree.getroot(),source/path.name,path);tree.write(path,encoding='unicode')
    field.seal(target)


def plateau_pairs(obs,elapsed,motion,gravity,c,profile):
    j=motion['conditions']['joint_index']
    efforts=(obs['effort_raw']-np.asarray(profile['effort_zero_nm']))*np.asarray(profile['effort_gain'])
    residual=efforts[:,j]-gravity.gravity_batch(obs['q'])[:,j]
    parts=[];audit=[]
    for segment,label in enumerate(motion['labels']):
        if label!='constant_velocity':continue
        a=motion['knots_s'][segment]+c['plateau_trim_s'];b=motion['knots_s'][segment+1]-c['plateau_trim_s']
        bins=[]
        count=max(4,int((b-a)/.1));edges=np.linspace(a,b,count+1)
        for left,right in zip(edges[:-1],edges[1:]):
            use=(elapsed>=left)&(elapsed<right);t=elapsed[use]
            if not len(t):continue
            weights=np.diff(np.r_[t,t[-1]+np.median(np.diff(t))]) if len(t)>1 else np.ones(1);weights/=weights.sum()
            tc=t-np.average(t,weights=weights);den=np.sum(weights*tc**2)
            q=obs['q'][use];v=obs['dq'][use]
            qbar=np.average(q,axis=0,weights=weights);vbar=np.average(v,axis=0,weights=weights)
            trend_q=np.sum(weights[:,None]*tc[:,None]*q,axis=0)/den if den>0 else np.zeros(14)
            acceleration=np.sum(weights[:,None]*tc[:,None]*v,axis=0)/den if den>0 else np.zeros(14)
            inertia=float(abs((gravity.mass_matrix(qbar)@acceleration)[j]))
            velocity_bias=float(gravity.velocity_bias(qbar,vbar)[j])
            bins.append((qbar[j],vbar[j],np.average(residual[use],weights=weights)-velocity_bias))
            audit.append(dict(begin_s=left,end_s=right,callback_publications=int(use.sum()),position_trend_rad_s=trend_q[j],
                reported_speed_rad_s=vbar[j],maximum_reported_velocity_trend_rad_s2=float(np.max(abs(acceleration))),
                nominal_inertia_contamination_nm=inertia,nominal_velocity_bias_nm=velocity_bias,
                independent_acquisition_count=None,time_basis='callback-window trends, not acquisition derivatives'))
        if not bins:raise ValueError('No observations in declared scan segment')
        values=np.array(bins);order=np.argsort(values[:,0]);parts.append(tuple(values[order,k] for k in range(3)))
    if len(parts)!=2:raise ValueError('Both observed directions required')
    x,y,pair=pair_friction_parts(parts,0.,c['pair_points'])
    return x,y,dict(**pair,windows=audit,qualification='exploratory SDK callback-window plateau protocol; acquisition/refresh/filter unknown')


def inspect_trial(plan,capture,output):
    plan=Path(plan);capture=Path(capture);field.output_outside(output,plan,capture)
    metadata,commands,origin,obs,elapsed,collision=field.field_observations(plan,capture)
    motion=read(plan/'motion.json');c=read(plan/'h3.json')
    if metadata['stage']=='breakaway':
        from .hardware_breakaway import inspect_trial as ramp_inspect
        return ramp_inspect(plan,capture,output,metadata,commands,origin,obs,elapsed,collision,c)
    x,y,pair=plateau_pairs(obs,elapsed,motion,gravity_engine(plan/'gravity'),c,metadata['config'])
    out=Path(output);out.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(out/'friction_features.npz',x=x,y=y)
    report=dict(passed=True,stage='moving_friction',job=metadata['job'],pairing=pair,sample_time_known=False,
        hardware_calibration_accepted=False,source_plan_manifest_sha256=sha256(plan/'manifest.json'),source_capture_manifest_sha256=sha256(capture/'manifest.json'))
    write_json(out/'collision_received_interpolant.json',collision);write_json(out/'report.json',report);field.seal(out);return report


def friction_result(root,protocol,history):
    from .hardware_batch import completed_capture
    selected=[j for j in protocol['jobs'] if j['kind']=='moving_friction']
    if any(not completed_capture(history[j['id']]) for j in selected):return dict(available=False,passed=False)
    axes=[];sources=[]
    for joint in protocol['h3']['joints']:
        data={role:[] for role in ('train','validation')}
        for job in selected:
            if job['joint']!=joint:continue
            source=completed_capture(history[job['id']]);path=Path(root)/source['attempt']/'inspection/friction_features.npz'
            if not path.exists():return dict(available=False,passed=False,reason='friction_features_unavailable')
            with np.load(path,allow_pickle=False) as d:data[job['split']].append((d['x'],d['y']))
            sources.append(dict(job=job,attempt=source['attempt'],manifest_sha256=source['manifest_sha256']))
        x,y=[np.vstack([d[0] for d in data['train']]),np.concatenate([d[1] for d in data['train']])]
        vx,vy=[np.vstack([d[0] for d in data['validation']]),np.concatenate([d[1] for d in data['validation']])]
        try:
            parameters,fit=fit_friction_features(x,y,vx,vy,None)
            axes.append(dict(joint=joint,viscous_nm_s_rad=parameters[0],coulomb_nm=parameters[1],**fit))
        except ValueError as exc:
            axes.append(dict(joint=joint,viscous_nm_s_rad=None,coulomb_nm=None,passed=False,reason=str(exc)))
    return dict(available=all(a['viscous_nm_s_rad'] is not None and a['coulomb_nm'] is not None for a in axes),passed=all(a['passed'] for a in axes),axes=axes,sources=sources,
        sample_time_known=False,formal_hardware_calibration_accepted=False,
        model='b*dq + fc*tanh(dq/0.001)',sampling_qualification_passed=False,
        limitations=['Callback-window protocol is not S3/S4 acquisition qualification','Nominal inertial terms are descriptive estimates, not calibrated inertia','No thermal/posture-specific friction law identified'])


def summarise_h3(root,protocol,history,destination,report):
    result=friction_result(root,protocol,history);write_json(Path(destination)/'moving_friction.json',result)
    from .hardware_breakaway import result as ramp_result
    onset=ramp_result(root,protocol,history);write_json(Path(destination)/'breakaway.json',onset)
    report.update(workflow='h3_friction',moving_friction=result,breakaway=onset,
        formal_hardware_calibration_accepted=False,observation_protocol='SDK callback-window operational candidates')
    report['gravity']=dict(available=True,frozen_input=True,manifest_sha256=sha256(Path(root)/'protocol/gravity/manifest.json'))
    if result['available']:
        with (Path(destination)/'friction_parameters.csv').open('w',newline='',encoding='utf-8-sig') as stream:
            writer=csv.writer(stream);writer.writerow(['joint','viscous_Nm_s_rad','coulomb_Nm','validation_RMSE_Nm','passed'])
            writer.writerows([a['joint'],a['viscous_nm_s_rad'],a['coulomb_nm'],a.get('validation_rmse_nm'),a['passed']] for a in result['axes'])
        write_json(Path(destination)/'runtime_friction.json',dict(schema_version='casbot_h3_runtime_friction_v1',
            source_model_sha256=protocol['source_model_sha256'],gravity_manifest_sha256=report['gravity']['manifest_sha256'],
            identified_axes=result['axes'],unidentified_joints=[j for j in protocol['joint_names'] if j not in protocol['h3']['joints']],
            onset_axes=onset.get('axes',[]),command_hz=protocol['profile']['command_hz'],conditions=protocol['h3'],
            sample_time_known=False,temperature_observed=False,formal_hardware_calibration_accepted=False,auto_deploy=False))
    if onset['available']:
        with (Path(destination)/'onset_thresholds.csv').open('w',newline='',encoding='utf-8-sig') as stream:
            writer=csv.writer(stream);writer.writerow(['joint','direction','split','pose','dwell_s','slope_Nm_s','estimate_Nm','callback_lower_Nm','callback_upper_Nm','confirmation_Nm'])
            writer.writerows([r['job']['joint'],r['job']['direction'],r['job']['split'],r['job']['pose'],r['job']['dwell_s'],r['job']['slope_nm_s'],
                r['estimate_nm'],*r['callback_onset_interval_nm'],r['operational_confirmation_nm']] for r in onset['ramps'])
    if result['available'] and all(a['coulomb_nm'] is not None for a in result['axes']):
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        fig=Figure(figsize=(9,4));FigureCanvasAgg(fig);ax=fig.subplots();x=np.arange(len(result['axes']))
        ax.bar(x-.2,[a['coulomb_nm'] for a in result['axes']],.4,label='Coulomb Nm')
        ax.bar(x+.2,[a['viscous_nm_s_rad'] for a in result['axes']],.4,label='Viscous Nm s/rad');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(Path(destination)/'friction.png')
