"""Frozen-model, third-pose SDK development validation; no training fit."""
from copy import deepcopy
from pathlib import Path
from shutil import copyfile,copytree
import numpy as np
from .data import write_json
from . import hardware_field as field
from .hardware_wrist_exploration import FLAGS,read,require_local
from .hardware_probe_capture import collect_jobs,inspect_observations
from .hardware_probe import probe_problem,inertia_balance
from .model_runtime import model_for
from .rigid_identification import model_from_parameters
from .sdk_mock import SDKMockSession
from .run_marker import run_marker
from .vendor_identity import sha256


def design(source,output):
    source=Path(source).resolve();field.check_seal(source/'protocol');field.check_seal(source/'analysis')
    old=read(source/'protocol/experiment.json');training=read(source/'analysis/report.json')
    if old['schema_version']!='casbot_sdk_wrist_probe_campaign_v1' or not training['all_operator_deletions_passed']:
        raise ValueError('Stable SDK right-pitch training candidate required')
    checkpoint=source/'analysis/development_candidate';field.check_seal(checkpoint)
    candidate=read(checkpoint/'candidate.json')
    if not np.array_equal(candidate['parameters'],training['variants'][0]['fit']['parameters']):
        raise ValueError('Candidate differs from completed full-training fit')
    root=field.output_outside(output,source);root.mkdir(parents=True,exist_ok=False)
    protocol=root/'protocol';protocol.mkdir();copytree(checkpoint,protocol/'candidate')
    for name in ('mock.yaml','base_profile.yaml','scene.yaml'):copyfile(source/'protocol'/name,protocol/name)
    pose=((np.asarray(old['poses']['c00'])+np.asarray(old['poses']['c01']))/2).tolist()
    jobs=[dict(id='v00_right_pitch_'+tag,direction=d,pose_rad=pose,split='development_validation') for d,tag in [(-1,'neg'),(1,'pos')]]
    p={k:deepcopy(old[k]) for k in ('probe_config','joint','holding','transition_s','settle_s','fixture_friction_prior','command_hz','model_profile','prior_status')}
    p.update(schema_version='casbot_sdk_wrist_probe_validation_v1',source_campaign=str(source),
        source_candidate_manifest_sha256=sha256(checkpoint/'manifest.json'),candidate_frozen_before_collection=True,
        pose_rad=pose,training_poses=old['poses'],jobs=jobs,maximum_trials=2,
        primary=dict(id='primary_30hz_zero_offset',fit_band_hz=30.,effort_offset_s=0.),
        diagnostic_variants=[dict(id='band10',fit_band_hz=10.,effort_offset_s=0.),dict(id='band20',fit_band_hz=20.,effort_offset_s=0.),
            dict(id='effort_minus5ms',fit_band_hz=30.,effort_offset_s=-.005),dict(id='effort_plus5ms',fit_band_hz=30.,effort_offset_s=.005)],
        acceptance=dict(maximum_relative_inertia_disagreement=.10,maximum_torque_rmse_nm=.002),
        diagnostics_used_for_selection=False,candidate_parameter_updates_allowed=False,original_formal_test_consumed=False,
        role='independent_development_validation_not_original_formal_test',**FLAGS,**run_marker())
    write_json(protocol/'experiment.json',p);field.seal(protocol);return p


def collect(root,domain):
    root=Path(root).resolve();field.check_seal(root/'protocol');p=read(root/'protocol/experiment.json')
    with SDKMockSession(root/'protocol/mock.yaml',root/'mock',domain,profile=root/'protocol/base_profile.yaml') as session:
        collect_jobs(root,p,session,domain,p['jobs'])
    write_json(root/'collection.json',dict(completed=True,records=2,role=p['role'],**FLAGS))


def engines(p,candidate):
    prior,_=model_for(p['model_profile']);model=prior.model;model.gravity_world=np.asarray(candidate['gravity_world']);prior=type(prior)(model)
    fixed=model_from_parameters(prior,candidate['parameters'],candidate['viscous'],candidate['coulomb'],candidate['static_friction'],stribeck_velocity=.005)
    return prior,fixed


def score(problem,candidate,model,acceptance):
    j=problem['joint'];theta=np.asarray(candidate['parameters'])
    prediction=problem['regression']@theta+problem['extra']@np.r_[candidate['viscous'],candidate['coulomb']]+candidate['static_friction'][j]*problem['static_feature']+problem['rotor']
    residual=prediction-problem['observed_torque_nm'];local=inertia_balance(problem)
    q=problem['q'];zeros=np.zeros_like(q)
    mass=model.evaluate_batch(q,zeros,zeros)['mass_matrix'][:,j,j]
    rigid_mass=float(np.mean(mass))-model.model.joints[j].rotor_inertia
    observed=local['inertia_kg_m2'];relative=abs(rigid_mass-observed)/max(abs(observed),1e-8)
    rmse=float(np.sqrt(np.mean(residual**2)))
    result=dict(torque_prediction_rmse_nm=rmse,mean_residual_nm=float(np.mean(residual)),
        frozen_candidate_mean_rigid_inertia_kg_m2=rigid_mass,public_local_inertia_diagnostic=local,
        relative_inertia_disagreement=relative,
        checks=dict(positive_public_inertia=observed>0,inertia_agreement=relative<=acceptance['maximum_relative_inertia_disagreement'],
                    torque_prediction=rmse<=acceptance['maximum_torque_rmse_nm']))
    result['passed']=all(result['checks'].values())
    return result,prediction,residual


def analyse(root,output):
    root=Path(root).resolve();out=field.output_outside(output,root/'protocol');out.mkdir(parents=True,exist_ok=False)
    field.check_seal(root/'protocol');p=read(root/'protocol/experiment.json')
    field.check_seal(root/'protocol/candidate');candidate=read(root/'protocol/candidate/candidate.json')
    prior,model=engines(p,candidate);frozen=np.asarray(candidate['parameters']).copy();rows=[];traces=[]
    for job in p['jobs']:
        folder=root/job['id'];plan,captured=field.resolve_field_capture(folder/'plan',folder/'capture')
        meta,_,_,obs,elapsed,collision=field.field_observations(plan,captured)
        require_local(meta['config'],read(captured/'request.json')['domain_id']);motion=read(plan/'motion.json')
        quality,problem,trace=inspect_observations(obs,elapsed,motion,prior,p['probe_config'],meta['config'],p['fixture_friction_prior'])
        row=dict(job=job,quality=quality,received_collision_passed=collision['passed'],diagnostics=[])
        if problem is not None:
            primary,prediction,residual=score(problem,candidate,model,p['acceptance']);row['primary']=primary
            np.savez_compressed(out/(job['id']+'_problem.npz'),**{k:v for k,v in problem.items() if isinstance(v,np.ndarray)},prediction_nm=prediction,residual_nm=residual)
            trace.update(prediction_time_s=problem['time_s'],projected_observation_nm=problem['observed_torque_nm'],prediction_nm=prediction,residual_nm=residual)
            for variant in p['diagnostic_variants']:
                cfg=deepcopy(p['probe_config']);cfg['processing']['fit_band_hz']=variant['fit_band_hz']
                try:
                    alt=probe_problem(obs,elapsed,motion,prior,cfg,meta['config'],effort_relative_offset_s=variant['effort_offset_s'])
                    evaluation,_,_=score(alt,candidate,model,p['acceptance'])
                    row['diagnostics'].append(dict(variant=variant,available=True,**evaluation))
                except ValueError as error:row['diagnostics'].append(dict(variant=variant,available=False,reason=str(error)))
        else:row['primary']=dict(passed=False,reason=quality['preprocessing_rejection'])
        rows.append(row);traces.append(trace)
    result=dict(schema_version='casbot_sdk_wrist_probe_validation_report_v1',records=rows,
        development_validation_passed=all(r['primary']['passed'] and r['quality']['probe_preprocessing_passed'] and all(r['quality']['raw_checks'].values()) and r['received_collision_passed'] for r in rows),
        acceptance=p['acceptance'],primary_variant=p['primary'],candidate_refit_performed=False,
        training_data_access_required=False,diagnostics_used_for_selection=False,original_formal_test_consumed=False,
        candidate_parameters_unchanged=bool(np.array_equal(frozen,candidate['parameters'])),role=p['role'],**FLAGS)
    write_json(out/'report.json',result);plot(rows,traces,out);field.seal(out);return result


def plot(rows,traces,out):
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from .presentation import _font
    path=_font(12).path;font_manager.fontManager.addfont(path)
    plt.rcParams['font.family']=font_manager.FontProperties(fname=path).get_name();plt.rcParams['axes.unicode_minus']=False
    fig,ax=plt.subplots(2,2,figsize=(13,7),layout='constrained')
    for col,(row,t) in enumerate(zip(rows,traces)):
        if 'prediction_nm' not in t:continue
        x=t['prediction_time_s']-t['prediction_time_s'][0]
        # Remove the same observation mean from BOTH curves; prediction bias remains.
        zero=np.mean(t['projected_observation_nm'])
        ax[0,col].plot(x,(t['projected_observation_nm']-zero)*1000,label='公开力矩（去共同均值）',lw=1)
        ax[0,col].plot(x,(t['prediction_nm']-zero)*1000,label='冻结模型预测',lw=1)
        ax[0,col].set_title('负向' if row['job']['direction']<0 else '正向');ax[0,col].set_ylabel('力矩（mNm）');ax[0,col].legend()
        ax[1,col].plot(x,t['residual_nm']*1000,lw=1);ax[1,col].set_ylabel('预测残差（mNm）');ax[1,col].set_xlabel('保留窗口内时间（s）')
    for panel in ax.flat:panel.grid(alpha=.2)
    fig.suptitle('第三姿态独立开发验证｜5ms SDK Mock｜候选参数冻结')
    fig.savefig(out/'heldout_prediction.png',dpi=160);plt.close(fig)
