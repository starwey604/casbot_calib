"""Frozen local SDK dual-tone roll protocol and public observation analysis."""
from copy import deepcopy
from functools import partial
from pathlib import Path
from shutil import copyfile,copytree
import numpy as np
import yaml
from . import hardware_field as field
from . import hardware_probe_capture as capture
from .hardware_probe import exposure,field_probe_config,legacy_quality
from .hardware_roll_frequency import multitone_motion,phasors,fit_components
from .hardware_wrist_exploration import FLAGS,read,require_local
from .model_runtime import model_for
from .data import write_json
from .run_marker import run_marker
from .vendor_identity import sha256

SCHEMA='casbot_sdk_wrist_roll_dual_v2'
LEGACY_SCHEMA='casbot_sdk_wrist_roll_dual_v1'


def freeze(design,output):
    design=Path(design).resolve();field.check_seal(design);d=read(design/'report.json')
    selected=next(row for row in d['designs'] if row['id']==d['selected_id'])
    source=Path(d['protocol']['source']);field.check_seal(source/'protocol');old=read(source/'protocol/experiment.json')
    if old['schema_version']!='casbot_sdk_wrist_roll_screen_v1' or selected['tones']!=[[2.5,.0075],[10.,.00068]]:
        raise ValueError('Settled c00 roll dual-tone design required')
    if not all(r['full_reference_passed'] and r['command_preview_passed'] for r in d['selected_reference_checks']):
        raise ValueError('Selected offline reference checks did not pass')
    root=field.output_outside(output,design,source);root.mkdir(parents=True,exist_ok=False);folder=root/'protocol';folder.mkdir()
    for name in ('mock.yaml','base_profile.yaml','scene.yaml'):copyfile(source/'protocol'/name,folder/name)
    copytree(design/'references',folder/'reference_previews')
    p={k:deepcopy(old[k]) for k in ('probe_config','holding','transition_s','settle_s','fixture_friction_prior','prior_status','command_hz','pose_rad')}
    p['probe_config']=field_probe_config(p['probe_config'])
    profile=yaml.safe_load((folder/'base_profile.yaml').read_text());defaults=yaml.safe_load(Path('configs/hardware_field.yaml').read_text())
    for key in ('tracking_limit_rad','position_allowance_rad','static_velocity_rad_s','static_position_range_rad','static_max_gap_s','gravity_validation_rmse_nm'):
        profile.pop(key,None)
    for key in ('max_feedback_age_ns','max_mode_age_ns','maximum_velocity_rad_s'):
        profile[key]=defaults[key]
    profile['assumptions']['mode_exit']=defaults['assumptions']['mode_exit']
    (folder/'base_profile.yaml').write_text(yaml.safe_dump(profile))
    # No stale5Hz/.004rad single-tone recipe in the new contract.
    p['probe_config']['probe'].pop('frequency_hz');p['probe_config']['probe'].pop('amplitude_rad')
    p.update(schema_version=SCHEMA,role='local_dual_tone_observation_exploration',joint=13,joint_label='右腕 roll',center='c00',
        tones=selected['tones'],base_profile='base_profile.yaml',
        jobs=[dict(id='c00_roll_dual_'+name,direction=sign) for name,sign in [('neg',-1),('pos',1)]],
        source_design=str(design),source_design_manifest_sha256=sha256(design/'manifest.json'),
        observation_policy=dict(primary_model='free_b_complex',comparison_models=['zero_phase_local','frozen_b_complex','free_b_complex'],
            inertia_scale_kg_m2=d['planning_inertia_kg_m2'],viscous_scale_nm_s_rad=d['planning_viscous_nm_s_rad'],
            effective_advance_branch_s=[-.025,.025],leave_block_duration_s=.4,leave_blocks=4,
            row_weights='original exposure weights retained when blocks are omitted',
            torque_model='unit gain, native frozen gravity and rotor, frozen fc/fs; fitted b is diagnostic only',
            frequency_model='all declared tones plus quadratic drift fitted simultaneously',
            effective_phase_is_acquisition_time=False,phase_correction_written_to_observations=False),
        scope='localhost_contract_mock_only',maximum_trials=2,automatic_expansion=False,
        original_inertia_relative_threshold=.1,full_p0_r1_r2_acceptance=False,
        friction_candidate_updated=False,rigid_candidate_fitted=False,formal_test_consumed=False,
        recovery_policy='log_noop_and_terminate_whole_session_no_fault_resume',**FLAGS,**run_marker())
    write_json(folder/'experiment.json',p);field.seal(folder)
    write_json(root/'status.json',dict(stage='protocol_frozen_no_collection',records_collected=0,**FLAGS))
    return p


def load_protocol(root):
    root=Path(root);field.check_seal(root/'protocol');p=read(root/'protocol/experiment.json')
    if p['schema_version'] not in (SCHEMA,LEGACY_SCHEMA):raise ValueError('Frozen SDK dual-tone roll protocol required')
    return p


def protocol_motion(name,home,pose,joint,direction,*,config,tones,transition_s,settle_s):
    motion=multitone_motion(name,home,pose,joint,direction,config,tones,transition_s=transition_s,settle_s=settle_s)
    motion.conditions.pop('execution_authorized')
    motion.conditions.update(observation_protocol='sdk_callback_dual_tone_v1',execution_scope='localhost_contract_mock_only')
    return motion


def collect(root,domain):
    root=Path(root).resolve();p=load_protocol(root)
    if (root/'attempt.json').exists() or (root/'mock').exists() or any((root/j['id']).exists() for j in p['jobs']):
        raise RuntimeError('Existing dual-tone attempt cannot resume; preserve it and freeze a new session')
    write_json(root/'attempt.json',dict(started=True,resume_permitted=False,**run_marker()))
    write_json(root/'status.json',dict(stage='collection_in_progress',records_collected=None,**FLAGS))
    try:
        with capture.SDKMockSession(root/'protocol/mock.yaml',root/'mock',domain,profile=root/'protocol/base_profile.yaml') as session:
            capture.collect_jobs(root,p,session,domain,p['jobs'],motion_builder=partial(protocol_motion,tones=p['tones']))
    except BaseException:
        write_json(root/'status.json',dict(stage='attempt_terminated_no_resume',records_collected=None,**FLAGS))
        raise
    write_json(root/'collection.json',dict(completed=True,records=len(p['jobs']),**FLAGS))
    write_json(root/'status.json',dict(stage='collected_pending_analysis',records_collected=len(p['jobs']),**FLAGS))


def frequency_analysis(problem,prior,policy,frequencies):
    d=problem;j=d['joint'];t=d['time_s'];origin=float(t.mean());weights=exposure(t);f=np.asarray(frequencies)
    fc=prior['coulomb'][j]*d['extra'][:,14+j]+prior['static_friction'][j]*d['static_feature']
    channels=np.column_stack([d['observed_torque_nm'],d['ddq'][:,j],d['dq'][:,j],d['projected_gravity_nm']+d['rotor']+fc])
    js=policy['inertia_scale_kg_m2'];bs=policy['viscous_scale_nm_s_rad']
    def estimate(mask):
        z=phasors(t[mask],channels[mask],f,origin_s=origin,weights=weights[mask]);Y,A,V,C=z.T
        model=js*A+bs*V+C
        deriv=np.column_stack([js*A,bs*V,1j*2*np.pi*f*model*.005]);real=np.vstack([deriv.real,deriv.imag]);s=np.linalg.svd(real,compute_uv=False)
        rank=int(np.count_nonzero(s>s[0]*1e-10))
        result=dict(available=rank==3,observed_rank=rank,parameters=3,scaled_singular_values_nm=s.tolist(),
            components_real_imag=np.stack([z.real,z.imag],axis=-1).tolist())
        if rank<3:
            result['reason']='Delivered tone components do not separate J/b/effective phase'
            return result
        free=fit_components(f,A,V,C,Y,js,bs)
        fixed=fit_components(f,A,V,C,Y,js,bs,fixed_viscous=prior['viscous'][j])
        result.update(free_b_complex=free,frozen_b_complex=fixed)
        result['available']=free['solver_converged'] and fixed['solver_converged']
        response=free['inertia_kg_m2']*A+free['viscous_nm_s_rad']*V+C
        result['per_frequency_effective_advance_s']=(np.angle(Y/response)/(2*np.pi*f)).tolist()
        result['per_frequency_residual_rms_nm']=(abs(np.exp(2j*np.pi*f*free['effective_advance_s'])*response-Y)/np.sqrt(2)).tolist()
        return result
    full=estimate(np.ones(len(t),bool));blocks=[]
    if full['available']:
        start=t[0]
        for k in range(policy['leave_blocks']):
            span=policy['leave_block_duration_s'];mask=~((t>=start+k*span-1e-8)&(t<start+(k+1)*span-1e-8))
            fit=estimate(mask)
            if fit['available']:
                fit['inertia_change_from_full']=abs(fit['free_b_complex']['inertia_kg_m2']-full['free_b_complex']['inertia_kg_m2'])/max(abs(full['free_b_complex']['inertia_kg_m2']),1e-6)
            blocks.append(dict(block=k,**fit))
    return dict(frequencies_hz=f.tolist(),primary_model=policy['primary_model'],full=full,leave_block_out=blocks,
        effective_phase_is_acquisition_time=False,phase_correction_written_to_observations=False,
        friction_candidate_updated=False,full_p0_r1_r2_acceptance=False)


def inspect(obs,elapsed,motion,engine,config,profile,prior,policy,tones):
    frequencies=[f for f,a in tones]
    report,problem,traces=capture.inspect_observations(obs,elapsed,motion,engine,config,profile,prior,signal_frequencies=frequencies)
    if problem is not None:
        try:report['frequency_analysis']=frequency_analysis(problem,prior,policy,frequencies)
        except field.FieldContractError:raise
        except (ValueError,RuntimeError,np.linalg.LinAlgError) as error:
            report['frequency_analysis']=dict(full=dict(available=False,reason=str(error)),leave_block_out=[])
    return report,problem,traces


def pair_summary(rows,p):
    models={};threshold=p['original_inertia_relative_threshold']
    available=all(r.get('frequency_analysis',{}).get('full',{}).get('available',False) for r in rows)
    for model in p['observation_policy']['comparison_models']:
        usable=all('inertia_kg_m2' in r.get('inertia_diagnostic',{}) for r in rows) if model=='zero_phase_local' else available
        if usable:
            a,b=[r['inertia_diagnostic']['inertia_kg_m2'] if model=='zero_phase_local' else r['frequency_analysis']['full'][model]['inertia_kg_m2'] for r in rows]
            relative=abs(a-b)/max(abs(a),1e-6)
            models[model]=dict(direction_inertia_difference_relative=relative,within_original_10_percent=relative<=threshold)
    summary=dict(schema_version='casbot_sdk_wrist_roll_dual_report_v1',joint=13,records=rows,primary_model=p['observation_policy']['primary_model'],model_comparisons=models,
        all_motion_and_preprocessing_checks_passed=all(all(r['raw_checks'].values()) and r['probe_preprocessing_passed'] for r in rows),
        all_frequency_models_available=available,local_direction_comparison_only=True,full_p0_r1_r2_acceptance=False,
        original_inertia_relative_threshold=threshold,automatic_expansion=False,formal_test_consumed=False,**FLAGS)
    if p['schema_version']==SCHEMA:
        summary['schema_version']='casbot_sdk_wrist_roll_dual_report_v2'
        summary.pop('all_motion_and_preprocessing_checks_passed')
        summary.update(all_records_numerically_available=available,empirical_quality_gate_applied=False,recorded=len(rows))
    return summary



def check_record(p,job,motion,actual_profile,frozen_profile):
    conditions=motion['conditions']
    if (conditions['joint']!=p['joint'] or conditions['direction']!=job['direction'] or
        conditions['tones']!=[dict(frequency_hz=f,amplitude_rad=a) for f,a in p['tones']]):
        raise ValueError('Delivered plan differs from the frozen dual-tone job')
    for key in ('model_context','gravity_world','effort_gain','effort_zero_nm','gravity_feedforward'):
        if actual_profile[key]!=frozen_profile[key]:raise ValueError('Delivered profile differs from frozen '+key)
    if actual_profile['command_hz']!=p['command_hz']:raise ValueError('Delivered command frequency differs from frozen protocol')

def analyse(root,output):
    root=Path(root).resolve();p=load_protocol(root)
    if not (root/'collection.json').exists() or not read(root/'collection.json')['completed']:
        raise ValueError('No completed dual-tone collection; freeze does not collect observations')
    out=field.output_outside(output,root/'protocol');out.mkdir(parents=True,exist_ok=False);rows=[];traces=[]
    frozen_profile=field.load_profile(root/'protocol/base_profile.yaml')
    for job in p['jobs']:
        trial=root/job['id'];plan,captured=field.resolve_field_capture(trial/'plan',trial/'capture')
        meta,_,_,obs,elapsed,collision=field.field_observations(plan,captured)
        require_local(meta['config'],read(captured/'request.json')['domain_id'])
        motion=read(plan/'motion.json');check_record(p,job,motion,meta['config'],frozen_profile)
        engine,_=model_for(meta['config'])
        report,problem,trace=inspect(obs,elapsed,motion,engine,p['probe_config'],meta['config'],p['fixture_friction_prior'],p['observation_policy'],p['tones'])
        report.update(job=job,received_collision_passed=collision['passed']);rows.append(report);traces.append(trace)
        if problem is not None:np.savez_compressed(out/(job['id']+'_problem.npz'),**{k:v for k,v in problem.items() if isinstance(v,np.ndarray)})
        np.savez_compressed(out/(job['id']+'_public_traces.npz'),**trace)
    summary=pair_summary(rows,p);summary['source_protocol_manifest_sha256']=sha256(root/'protocol/manifest.json');write_json(out/'report.json',summary);plot(rows,traces,out);field.seal(out)
    return summary


def plot(rows,traces,out,*,fixture_only=False):
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from .presentation import _font
    font_manager.fontManager.addfont(_font(12).path);plt.rcParams['font.family']=font_manager.FontProperties(fname=_font(12).path).get_name();plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(2,2,figsize=(11,6.5),layout='constrained')
    for col,(row,t) in enumerate(zip(rows,traces)):
        axes[0,col].plot(t['time_s']-t['time_s'][0],t['reported_speed']);axes[0,col].set_title('负向' if row['job']['direction']<0 else '正向');axes[0,col].set_ylabel('右腕 roll 速度（rad/s）')
        signals=row['frequency_signals'];x=np.arange(len(signals))
        axes[1,col].bar(x,[s['observed_minus_frozen_gravity_friction_rms_nm']*1000 for s in signals],color='#218c80')
        axes[1,col].set_xticks(x,[str(s['frequency_hz'])+'Hz' for s in signals]);axes[1,col].set_ylabel('扣声明重力／摩擦的频点 RMS（mNm）')
        for ax in axes[:,col]:ax.grid(alpha=.2)
    fig.suptitle('双频解析读出夹具｜非采集数据' if fixture_only else 'c00 右腕 roll 双频｜本机 SDK Mock，5ms 缓存')
    fig.savefig(Path(out)/'dual_frequency_signals.png',dpi=160);plt.close(fig)
