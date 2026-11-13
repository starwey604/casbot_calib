"""Bind two existing SDK probes, collect six, and assess fixed-policy deletions."""
from copy import deepcopy
from pathlib import Path
from shutil import copyfile
import numpy as np
import yaml
from .data import write_json
from . import hardware_field as field
from .hardware_wrist_exploration import FLAGS,read,require_local
from .hardware_probe_capture import collect_jobs,inspect_observations
from .hardware_inertia import gravity_constraints
from .model_runtime import model_for
from .rigid_identification import fit_rigid,model_from_parameters,parameter_scales
from .sdk_mock import SDKMockSession
from .staged_stability_math import operator_difference
from .run_marker import run_marker
from .artifact_io import sha256


def source_binding(source):
    source=Path(source).resolve();field.check_seal(source/'protocol')
    p=read(source/'protocol/experiment.json')
    if p['schema_version']!='casbot_sdk_wrist_probe_pair_v1' or not read(source/'collection.json')['completed']:
        raise ValueError('Completed two-probe source required')
    field.check_seal(source/'analysis')
    if not read(source/'analysis/report.json')['all_motion_and_preprocessing_checks_passed']:
        raise ValueError('Source probe quality did not pass')
    records=[]
    for job in p['jobs']:
        trial=source/job['id']
        for part in ('plan','capture'):field.check_seal(trial/part)
        if not read(trial/'capture/result.json')['local_contract_test']:raise ValueError('Local SDK source required')
        records.append(dict(source_id=job['id'],direction=job['direction'],
            plan_manifest=sha256(trial/'plan/manifest.json'),capture_manifest=sha256(trial/'capture/manifest.json')))
    return dict(root=str(source),protocol_manifest=sha256(source/'protocol/manifest.json'),records=records)


def design(source,output,operator_probes):
    binding=source_binding(source);source=Path(binding['root']);old=read(source/'protocol/experiment.json')
    root=field.output_outside(output,source);root.mkdir(parents=True,exist_ok=False)
    protocol=root/'protocol';protocol.mkdir();p=deepcopy(old)
    p.pop('training_dataset_refitted',None)
    p.update(original_4khz_training_dataset_refitted=False,sdk_training_fit_planned=True)
    poses=dict(c00=old['pose_rad'].copy(),c01=old['pose_rad'].copy())
    poses['c01'][5]=poses['c01'][12]=-.2;poses['c01'][6]=poses['c01'][13]=.15
    reused=[];jobs=[]
    for batch in ('b00','b01'):
        for center in ('c00','c01'):
            for direction,tag in [(-1,'neg'),(1,'pos')]:
                row=dict(id=f'{center}_right_pitch_{tag}_{batch}',center=center,batch=batch,direction=direction,pose_rad=poses[center])
                if batch=='b00' and center=='c00':
                    row.update(session='source_pair',source_trial=f'c00_right_pitch_{tag}');reused.append(row)
                else:
                    row['session']='b00_extension' if batch=='b00' else 'b01';jobs.append(row)
    first=read(source/old['jobs'][0]['id']/'plan/plan.json')['config'];profile=deepcopy(first)
    profile.pop('connection');profile.pop('breakaway_parameters',None);scene=profile.pop('scene')
    profile['collision_scene']='scene.yaml'
    (protocol/'scene.yaml').write_text(yaml.safe_dump(scene));(protocol/'base_profile.yaml').write_text(yaml.safe_dump(profile))
    copyfile(source/'protocol/mock.yaml',protocol/'mock.yaml')
    with np.load(operator_probes,allow_pickle=False) as d:probes={k:d[k] for k in ('q','dq','ddq')}
    if any(v.shape!=(128,14) or not np.isfinite(v).all() for v in probes.values()):raise ValueError('Original128 finite public operator points required')
    np.savez_compressed(protocol/'operator_probes.npz',**probes)
    engine,_=model_for(first);theta=engine.rigid_parameters()
    np.savez_compressed(protocol/'coordinates.npz',prior=theta,scales=parameter_scales(theta),metric=np.eye(len(theta)))
    stability=yaml.safe_load(Path('configs/dvt2/staged_dual_arm_p0_v4.yaml').read_text())['p0']['stability']
    policy={k:stability[k] for k in ('maximum_mass_relative_frobenius','maximum_mass_diagonal_relative','mass_diagonal_floor_kg_m2','maximum_inverse_axis_normalized_rms')}
    p.update(schema_version='casbot_sdk_wrist_probe_campaign_v1',jobs=jobs,reused_jobs=reused,poses=poses,
        source_binding=binding,model_profile=first,operator_probe_source=str(Path(operator_probes).resolve()),
        sessions=['b00_extension','b01'],maximum_trials=6,total_records=8,
        fitting=dict(ridge=1e-4,row_squared_weight_budget=256.,torque_scale_nm=.2,robust_scale_nm=.02,
            rank_rtol=1e-7,physical_length_scale_m=.1,frozen_coordinates='nominal, not full-fit; complete identity metric',
            friction='frozen synthetic fixture',group_rows_not_renormalized=True),operator_policy=policy,
        batch_meaning='two logical preregistered acquisition groups; b00 spans source_pair and b00_extension sessions',
        actual_session_deletions_also_required=True,full_p0_r1_r2_completed=False,**run_marker())
    write_json(protocol/'experiment.json',p);field.seal(protocol);return p


def check_source(p):
    if source_binding(p['source_binding']['root'])!=p['source_binding']:
        raise ValueError('Bound original SDK pair changed')


def collect(root,domain):
    root=Path(root).resolve();field.check_seal(root/'protocol');p=read(root/'protocol/experiment.json');check_source(p)
    sessions=root/'sessions';sessions.mkdir()
    for name in p['sessions']:
        with SDKMockSession(root/'protocol/mock.yaml',sessions/name,domain,profile=root/'protocol/base_profile.yaml') as session:
            collect_jobs(root,p,session,domain,[j for j in p['jobs'] if j['session']==name])
    write_json(root/'collection.json',dict(completed=True,new_records=6,reused_records=2,total_records=8,**FLAGS))


def record_root(root,p,job):
    return Path(p['source_binding']['root'])/job['source_trial'] if 'source_trial' in job else Path(root)/job['id']


def operators(engine,probes):
    return (np.array([engine.mass_matrix(q) for q in probes['q']]),
            engine.evaluate_batch(probes['q'],probes['dq'],probes['ddq'])['total'])


def variant_jobs(jobs):
    variants=[('full',jobs)]
    for key,groups in [('center',['c00','c01']),('batch',['b00','b01']),('session',['source_pair','b00_extension','b01'])]:
        for value in groups:variants.append((f'delete_{key}_{value}',[j for j in jobs if j[key]!=value]))
    return variants


def fit_variant(problems,jobs,engine,prior,scales,metric,equalities,policy):
    y=[];target=[];weights=[]
    for job in jobs:
        p=problems[job['id']];count=len(p['rigid_target']);weight=np.sqrt(policy['row_squared_weight_budget']/count)
        y.append(p['regression']*weight);target.append(p['rigid_target']*weight)
        weights.append(dict(id=job['id'],rows=count,weight=weight,squared_budget=policy['row_squared_weight_budget']))
    fit=fit_rigid(np.vstack(y),np.concatenate(target),prior,equalities=equalities,
        ridge=policy['ridge'],torque_scale=policy['torque_scale_nm'],robust_scale=policy['robust_scale_nm'],
        rank_rtol=policy['rank_rtol'],physical_length_scale=policy['physical_length_scale_m'],
        fixed_scales=scales,regularization_metric=metric,lift_residuals=True)
    return fit,weights


def export_candidate(root,p,out,parameters,operator_checks_passed):
    """A self-contained development comparison input, never automatic deployment."""
    folder=Path(out)/'development_candidate';folder.mkdir()
    write_json(folder/'candidate.json',dict(schema_version='casbot_sdk_wrist_probe_candidate_v1',
        parameters=parameters,**p['fixture_friction_prior'],
        gravity_world=p['model_profile']['gravity_world'],model_context=p['model_profile']['model_context'],
        identified_scope=['right_wrist_pitch_joint'],individual_body_parameters_unique=False,
        prior_dependent_unexcited_axes=True,source_protocol_manifest_sha256=sha256(Path(root)/'protocol/manifest.json'),
        source_analysis=str((Path(out)/'report.json').resolve()),
        source_operator_checks_passed=operator_checks_passed,auto_deploy=False,**FLAGS))
    field.seal(folder)


def analyse(root,output):
    root=Path(root).resolve();out=field.output_outside(output,root/'protocol');out.mkdir(parents=True,exist_ok=False)
    field.check_seal(root/'protocol');p=read(root/'protocol/experiment.json');check_source(p)
    jobs=sorted(p['reused_jobs']+p['jobs'],key=lambda j:(j['batch'],j['center'],j['direction']))
    engine,_=model_for(p['model_profile'])
    model=engine.model;model.gravity_world=np.asarray(p['model_profile']['gravity_world']);engine=type(engine)(model)
    reports=[];problems={};prior_friction=p['fixture_friction_prior']
    for job in jobs:
        source=record_root(root,p,job)
        plan,captured=field.resolve_field_capture(source/'plan',source/'capture')
        meta,_,_,obs,elapsed,collision=field.field_observations(plan,captured)
        require_local(meta['config'],read(captured/'request.json')['domain_id'])
        report,problem,trace=inspect_observations(obs,elapsed,read(plan/'motion.json'),engine,p['probe_config'],meta['config'],prior_friction)
        report.update(job=job,received_collision_passed=collision['passed']);reports.append(report)
        if problem is not None:
            problem['rigid_target']=problem['target']-problem['extra']@np.r_[prior_friction['viscous'],prior_friction['coulomb']]-prior_friction['static_friction'][p['joint']]*problem['static_feature']
            problems[job['id']]=problem
            np.savez_compressed(out/(job['id']+'_problem.npz'),**{k:v for k,v in problem.items() if isinstance(v,np.ndarray)})
    write_json(out/'observations.json',reports)
    if len(problems)!=8:
        result=dict(completed=False,reason='probe_quality_rejected',rejected_jobs=[r['job']['id'] for r in reports if not r['probe_preprocessing_passed']],**FLAGS)
        write_json(out/'report.json',result);field.seal(out);return result
    with np.load(root/'protocol/coordinates.npz',allow_pickle=False) as d:coordinates={k:d[k] for k in d.files}
    with np.load(root/'protocol/operator_probes.npz',allow_pickle=False) as d:probes={k:d[k] for k in d.files}
    equality=gravity_constraints(engine,p['pose_rad']);fitcache={};rows=[];full=None
    for label,selected in variant_jobs(jobs):
        key=tuple(j['id'] for j in selected)
        if key not in fitcache:
            fit,weights=fit_variant(problems,selected,engine,coordinates['prior'],coordinates['scales'],coordinates['metric'],equality,p['fitting'])
            candidate=model_from_parameters(engine,fit['parameters'],prior_friction['viscous'],prior_friction['coulomb'],prior_friction['static_friction'],stribeck_velocity=.005)
            mass,tau=operators(candidate,probes);fitcache[key]=(fit,weights,candidate,mass,tau)
        fit,weights,candidate,mass,tau=fitcache[key]
        if full is None:full=(mass,tau)
        op=operator_difference(full[0],mass,full[1],tau,p['operator_policy'])
        diag=abs(np.diagonal(mass-full[0],axis1=1,axis2=2))/np.maximum(np.diagonal(full[0],axis1=1,axis2=2),p['operator_policy']['mass_diagonal_floor_kg_m2'])
        point,joint=np.unravel_index(np.argmax(diag),diag.shape)
        prediction={j['id']:float(np.sqrt(np.mean((problems[j['id']]['regression']@fit['parameters']-problems[j['id']]['rigid_target'])**2))) for j in jobs}
        row=dict(label=label,selected_records=list(key),fit=fit,weights=weights,operators=op,
            worst_mass_diagonal_point=int(point),worst_mass_diagonal_joint=int(joint),prediction_rmse_nm=prediction,
            gravity_preservation_maximum_nm=float(np.max(abs(equality[0]@np.asarray(fit['parameters'])-equality[1]))))
        rows.append(row);print('FITTED',label,flush=True)
    group=lambda tag:max(r['operators']['maximum_mass_diagonal_relative'] for r in rows if r['label'].startswith(tag))
    result=dict(completed=True,records=8,new_records=6,reused_records=2,unique_fits=len(fitcache),variants=rows,
        maximum_delete_center_mass_diagonal_relative=group('delete_center'),
        maximum_delete_batch_mass_diagonal_relative=group('delete_batch'),
        maximum_delete_actual_session_mass_diagonal_relative=group('delete_session'),
        all_operator_deletions_passed=all(r['operators']['passed'] for r in rows),
        operator_policy=p['operator_policy'],operator_probe_count=128,
        batch_meaning=p['batch_meaning'],full_p0_r1_r2_completed=False,formal_test_consumed=False,**FLAGS)
    write_json(out/'report.json',result)
    export_candidate(root,p,out,rows[0]['fit']['parameters'],result['all_operator_deletions_passed'])
    plot(result,reports,out);field.seal(out);return result


def plot(report,observations,out):
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from .presentation import _font
    path=_font(12).path;font_manager.fontManager.addfont(path)
    plt.rcParams['font.family']=font_manager.FontProperties(fname=path).get_name();plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(1,2,figsize=(13,5),layout='constrained')
    rows=report['variants'][1:];labels=[r['label'].replace('delete_center_','删构型 ').replace('delete_batch_','删逻辑批次 ').replace('delete_session_','删会话 ') for r in rows]
    axes[0].barh(labels,[r['operators']['maximum_mass_diagonal_relative']*100 for r in rows]);axes[0].axvline(10,color='r',ls='--');axes[0].set_xlabel('原128点：最差对角惯量差（%）')
    names=[r['job']['id'].replace('right_pitch_','') for r in observations]
    axes[1].barh(names,[r['observed_minus_frozen_gravity_friction_5hz_rms_nm']*1000 for r in observations]);axes[1].set_xlabel('公开力矩 − 声明重力/摩擦：5Hz RMS（mNm）')
    for a in axes:a.grid(axis='x',alpha=.2)
    fig.suptitle('右腕 pitch 两构型×两方向×两批次｜5ms SDK Mock')
    fig.savefig(out/'campaign_stability.png',dpi=160);plt.close(fig)
