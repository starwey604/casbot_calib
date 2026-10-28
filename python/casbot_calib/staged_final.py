"""R4 continuation: preregister, acquire fresh tests and score one frozen model."""
from pathlib import Path
from time import perf_counter
import csv
import numpy as np
from . import _core
from .data import write_json
from .staged_protocol import load_protocol,load_frozen_candidate,file_hash,_read,_seal,_unseal,_local
from .staged_attribution import arrays,check_manifest,candidate_gate,component_arrays
from .staged_development import frozen_predecessors,collect_final_test
from .staged_identification import read_record,_windows,_residual,friction_pairs
from .staged_breakaway import onset_record
from .staged_collection import load_trial
from .staged_multifit import load_problems,score_problems,measured_center_coverage
from .staged_residuals import MODELS,frequency_budget,aggregate_benefit,raw_period_spectra,raw_residual_budget,residual_statistics
from .dual_model import nominal_dual_arm
from .staged_control import load_candidate

EXECUTION_FILES=('staged_final.py','staged_residuals.py','staged_inertia.py','staged_multifit.py',
    'staged_identification.py','staged_breakaway.py','staged_collection.py','staged_control.py',
    'staged_development.py','dual_identification.py','rigid_identification.py','staged_protocol.py')


def prepare_control(root,config,profile):
    """An isolated nine-run controller experiment using the existing schema-1 API."""
    from copy import deepcopy
    from .staged_motion import build_recipes,gate_motion
    from .staged_multiconfiguration import _write_motion
    root.mkdir();legacy=deepcopy(config);legacy.pop('p0');legacy['schema_version']=1
    motions=[m for m in build_recipes(legacy) if m.stage in ('control_tuning','control_validation')]
    if len(motions)!=9:raise ValueError('Five tuning and four validation references required')
    write_json(root/'session.json',dict(config=legacy,collision=profile.metadata,joint_names=[j.name for j in profile.engine.model.joints],
        synthetic_only=True,hardware_executable=False,scope='R4 controller-only companion; not legacy dynamics acceptance'))
    files={}
    for motion in motions:
        gate=gate_motion(profile,motion,profile.metadata['scene']['clearance_m'])
        if not gate['passed']:raise ValueError('R4 controller reference rejected: '+motion.name)
        files.update(_write_motion(root,motion,gate))
    write_json(root/'plan_report.json',dict(passed=True,selected_trials=[m.name for m in motions],scope='controller references only'))
    for name in ('session.json','plan_report.json'):files[name]=file_hash(root/name)
    return files


def prepare_final(root,attribution,output):
    from .collision import nominal_profile
    from .staged_attribution import formal_test_untouched
    root=Path(root).resolve();attribution=Path(attribution).resolve();output=Path(output).resolve()
    parent,config,manifest=load_protocol(root);frozen=load_frozen_candidate(root);formal_test_untouched(root)
    if (root/'final_test_protocol.json').exists():raise ValueError('R4 already registered; use its original output and --resume')
    snapshot=(root/frozen['candidate']).parent
    if file_hash(attribution/'analysis_protocol.json')!=file_hash(snapshot/'analysis_protocol.json') or file_hash(attribution/'residual_attribution.json')!=file_hash(snapshot/'residual_attribution.json'):
        raise ValueError('R3 analysis is not the frozen selection proof')
    check_manifest(attribution,'delivery_manifest.json');r3=_unseal(attribution,'analysis_protocol.json')
    r2=Path(r3['stability_analysis']);check_manifest(r2,'delivery_manifest.json')
    if file_hash(r2/'coordinates.npz')!=r3['coordinates_sha256']:raise ValueError('R2 coordinates changed')
    spec=next(s for s in r3['candidates'] if s['ridge']==frozen['ridge']);b=Path(spec['directory'])/'candidate_initial.json'
    if file_hash(b)!=spec['B_sha256']:raise ValueError('Fixed-friction B changed')
    output.mkdir(parents=True,exist_ok=False);model=output/'models';model.mkdir()
    for name,path in {'coordinates.npz':r2/'coordinates.npz','B.json':b,'D.json':root/frozen['candidate'],
        'training_bias.json':attribution/f'ridge_{frozen["ridge"]:g}'/'training_bias.json',
        'training_bias.json.sha256':attribution/f'ridge_{frozen["ridge"]:g}'/'training_bias.json.sha256',
        'selection_attribution.json':snapshot/'residual_attribution.json'}.items():(model/name).write_bytes(path.read_bytes())
    prior=frozen_predecessors(root);(model/'prior_friction.json').write_bytes((prior/'friction/report.json').read_bytes())
    source=Path(__file__).parent;archive=output/'source';archive.mkdir()
    for p in source.glob('*.py'):(archive/p.name).write_bytes(p.read_bytes())
    profile=nominal_profile(scene=_read(root/'session.json')['collision']['scene'])
    control_files=prepare_control(output/'control',config,profile)
    motion=_read(root/'motion_plan.json');names=manifest['planned_execution_order']['test']+motion['final_prior_execution_order']
    all_names=manifest['planned_execution_order']['train']+manifest['planned_execution_order']['selection']+names
    # Fixed representative replay set, never chosen by observed test residual.
    tests=[s for s in manifest['trials'] if s['split']=='test' and s['repeat_id']==0]
    replay=[next(s['trial_id'] for s in tests if s['stage']=='inertia' and s['active_arm_group']==group) for group in ('both','left','right')]
    replay += [next(s['trial_id'] for s in tests if s['stage']=='inertia_probe' and s['joint']==j) for j in (4,11)]
    prior_slots=motion['final_prior_trials']
    replay += [next(s['trial_id'] for s in prior_slots if s['stage']=='gravity')]
    replay += [next(s['trial_id'] for s in prior_slots if s['stage']==stage and s['joint']==4) for stage in ('friction','breakaway')]
    settings={**r3['settings']}
    protocol=dict(schema_version=1,round='R4',parent=str(root),parent_protocol_sha256=file_hash(root/'protocol.json'),
        candidate_freeze_sha256=file_hash(root/'candidate_freeze.json'),candidate_sha256=file_hash(root/frozen['candidate']),ridge=frozen['ridge'],
        attribution_protocol_sha256=file_hash(attribution/'analysis_protocol.json'),native_binary_sha256=file_hash(_core.__file__),
        predecessor_freeze_sha256=file_hash(root/'predecessor_freeze.json'),settings=settings,acceptance=config['acceptance'],
        model_files={p.name:file_hash(p) for p in model.iterdir()},control_files=control_files,
        source_sha256={p.name:file_hash(p) for p in archive.glob('*.py')},execution_files=list(EXECUTION_FILES),
        acquisition_order=names,all_calibration_trials=all_names,replay_trials=replay,
        expected_counts=dict(periodic=12,wrist=16,gravity=16,friction=84,breakaway=448,control=9),
        frequency_policy='same R3 thresholds/A proxy; required axes fixed by manifest active_arm_group; held axes use static interval, never silently drop an active axis',
        static_onset_tolerance_nm=.01,minimum_control_feedforward_reduction=.2,
        final_selection_forbidden=True,test_used_for_fit=False,synthetic_only=True,hardware_executable=False,
        scope='fresh final tests under R3 freeze; no model/gain/noise/threshold refitting; PD choice uses separate tuning reference only')
    _seal(output,'final_protocol.json',protocol)
    _seal(root,'final_test_protocol.json',dict(schema_version=1,analysis=str(output),candidate_freeze_sha256=protocol['candidate_freeze_sha256'],
        final_protocol_sha256=file_hash(output/'final_protocol.json')))
    return protocol,config


def load_final_protocol(root,analysis):
    root=Path(root).resolve();analysis=Path(analysis).resolve();parent,config,_=load_protocol(root);frozen=load_frozen_candidate(root)
    protocol=_unseal(analysis,'final_protocol.json');binding=_unseal(root,'final_test_protocol.json')
    if protocol['parent']!=str(root) or protocol['parent_protocol_sha256']!=file_hash(root/'protocol.json') or protocol['candidate_freeze_sha256']!=file_hash(root/'candidate_freeze.json') or protocol['candidate_sha256']!=file_hash(root/frozen['candidate']):
        raise ValueError('R4 parent/candidate changed')
    if binding['analysis']!=str(analysis) or binding['final_protocol_sha256']!=file_hash(analysis/'final_protocol.json') or binding['candidate_freeze_sha256']!=protocol['candidate_freeze_sha256']:
        raise ValueError('R4 acquisition protocol binding changed')
    if file_hash(_core.__file__)!=protocol['native_binary_sha256']:raise ValueError('R4 native mechanics changed')
    for name,expected in protocol['model_files'].items():
        if file_hash(analysis/'models'/name)!=expected:raise ValueError('R4 model/bias input changed')
    for name,expected in protocol['control_files'].items():
        if file_hash(analysis/'control'/name)!=expected:raise ValueError('R4 control reference changed')
    for name in protocol['execution_files']:
        if file_hash(Path(__file__).parent/name)!=protocol['source_sha256'][name]:raise ValueError('R4 computation changed after registration: '+name)
    for name,expected in protocol['source_sha256'].items():
        if file_hash(analysis/'source'/name)!=expected:raise ValueError('R4 source archive changed')
    frozen_predecessors(root)
    return protocol,config


def final_prior_records(root,stage):
    root=Path(root);load_frozen_candidate(root);seal=_unseal(root,'final_prior_inputs.json')
    slots=_read(root/'motion_plan.json')['final_prior_trials'];names=_read(root/'session.json')['joint_names']
    if seal['protocol_sha256']!=file_hash(root/'protocol.json') or seal['candidate_freeze_sha256']!=file_hash(root/'candidate_freeze.json') or [r['trial_id'] for r in seal['records']]!=[s['trial_id'] for s in slots]:
        raise ValueError('Incomplete or mismatched final predecessor seal')
    for row in seal['records']:
        for path,expected in row['files'].items():
            if file_hash(_local(root,path))!=expected:raise ValueError('Final predecessor changed: '+path)
    for slot in slots:
        if slot['stage']!=stage:continue
        name=slot['trial_id'];meta,_=load_trial(root,name);record,obs=read_record(root,name,meta,names)
        if record['candidate_freeze_sha256']!=seal['candidate_freeze_sha256'] or record['final_protocol_sha256']!=_unseal(root,'final_test_protocol.json')['final_protocol_sha256']:
            raise ValueError('Final predecessor acquired under another R4 freeze')
        yield name,meta,obs


def score_predecessors(root,analysis,model,config,protocol):
    gravity=[];friction=[];ramps=[];gain=config['drive']['gain_nm_a'];accept=protocol['acceptance']
    for name,meta,obs in final_prior_records(root,'gravity'):
        _,end=_windows(meta,'static_measure')[0];t=obs['sample_ns']*1e-9
        mask=(t>end-.5)&(t<end-.05)&(np.max(abs(obs['dq']),axis=1)<.002)
        if mask.sum()<100:raise ValueError('Final gravity hold did not settle')
        gravity.append(dict(trial=name,pose=meta['conditions']['pose'],approach=meta['conditions']['approach'],
            predicted=model.gravity(obs['q'][mask].mean(axis=0)).tolist(),observed=(obs['current'][mask].mean(axis=0)*gain).tolist()))
    predicted=np.asarray([r['predicted'] for r in gravity]);observed=np.asarray([r['observed'] for r in gravity])
    pairs=[]
    for pose in sorted({r['pose'] for r in gravity}):
        indices=[i for i,r in enumerate(gravity) if r['pose']==pose]
        if len(indices)!=2 or {gravity[i]['approach'] for i in indices}!={-1,1}:raise ValueError('Missing final opposite-approach pair')
        pairs.append((predicted[indices].mean(axis=0),observed[indices].mean(axis=0)))
    raw=_residual(predicted,observed,accept['torque_floor_nm']);paired=_residual(np.array([p for p,t in pairs]),np.array([t for p,t in pairs]),accept['torque_floor_nm'])
    g=dict(passed=raw['maximum_absolute_rmse_nm']<=accept['gravity_absolute_rmse_nm'] and paired['maximum_absolute_rmse_nm']<=accept['gravity_pair_rmse_nm'],
        holds=gravity,raw_hold=raw,paired_approaches=paired,fit_performed=False)
    joints=model.model.joints;frozen=np.array([[j.friction.viscous,j.friction.coulomb] for j in joints])
    for name,meta,obs in final_prior_records(root,'friction'):
        x,y,info=friction_pairs(obs,meta,model,gain,config['control_period_ns']);j=meta['conditions']['joint'];error=x@frozen[j]-y
        friction.append(dict(trial=name,pose=meta['conditions']['pose'],**info,rmse_nm=float(np.sqrt(np.mean(error**2))),bias_nm=float(error.mean())))
    axis=[float(np.sqrt(np.mean([r['rmse_nm']**2 for r in friction if r['joint']==j]))) for j in range(14)]
    sampling=_read(analysis/'models/prior_friction.json')['sampling_qualification']
    f=dict(passed=max(axis)<=accept['friction_absolute_rmse_nm'] and sampling['passed'],axis_rmse_nm=axis,scans=friction,fit_performed=False,
        sampling_qualification=dict(passed=sampling['passed'],source='frozen predecessor development 4/8 kHz convergence; reused, not new final 8 kHz data',evidence=sampling))
    static=np.array([j.friction.static_friction for j in joints])
    for name,meta,obs in final_prior_records(root,'breakaway'):
        policy=_read(Path(root)/'runs'/name/'guard_events.json');row=onset_record(name,meta,obs,policy,model,config)
        row['frozen_prediction_nm']=float(static[row['joint']]);row['frozen_prediction_error_nm']=abs(row['estimate_nm']-row['frozen_prediction_nm'])
        row['passed']=row['passed'] and row['frozen_prediction_error_nm']<=protocol['static_onset_tolerance_nm'];ramps.append(row)
    s=dict(passed=bool(ramps and all(r['passed'] for r in ramps)),ramps=ramps,fit_performed=False,
        maximum_frozen_prediction_error_nm=max(r['frozen_prediction_error_nm'] for r in ramps),
        convention='same observation-only backtracked still/moving bracket; compare frozen symmetric threshold without fitting final medians')
    expected=protocol['expected_counts']
    if [len(gravity),len(friction),len(ramps)]!=[expected['gravity'],expected['friction'],expected['breakaway']]:raise ValueError('Final predecessor count mismatch')
    return dict(passed=bool(g['passed'] and f['passed'] and s['passed']),gravity=g,friction=f,breakaway=s,
        scope='new independent final references/acquisition seeds; frozen physical parameters only; no final fitting')


def score_final(root,analysis,output):
    started=perf_counter();root=Path(root).resolve();analysis=Path(analysis).resolve();output=Path(output).resolve()
    protocol,config=load_final_protocol(root,analysis);frozen=load_frozen_candidate(root);nominal,_=nominal_dual_arm()
    c=arrays(analysis/'models/coordinates.npz');d=_read(analysis/'models/D.json');b=_read(analysis/'models/B.json')
    model=load_candidate(analysis/'models/D.json');static=c['static'];fa=c['friction'];fd=np.r_[d['viscous'],d['coulomb']]
    theta=dict(A=c['gravity_theta'],B=np.asarray(b['parameters']),C=np.asarray(d['parameters']),D=np.asarray(d['parameters']))
    fric=dict(A=fa,B=fa,C=fa,D=fd);h=config['p0']['preprocessing']['harmonics']
    problems=load_problems(root,'test',static);metrics,probes=score_problems(problems,model,theta['D'],fd,static,config)
    prediction=candidate_gate(metrics,probes,config);budget={};trajectories={};csv_rows=[]
    biases={k:np.asarray(v) for k,v in _unseal(analysis/'models','training_bias.json')['bias_nm'].items()}
    threshold=np.asarray(_read(root/'qualification/report.json')['threshold_bin_rms_nm'])
    from .staged_identification import recordings
    raw={name:raw_period_spectra(obs,meta,config) for name,meta,obs in recordings(root,'inertia',split='test',purpose='final_test')}
    output.mkdir(parents=True,exist_ok=False)
    for name,item in problems.items():
        if item['meta']['stage']!='inertia':continue
        p=item['problem'];conditions=item['meta']['conditions'];parts={};predictions={};audit={}
        for k in MODELS:parts[k],predictions[k],audit[k]=component_arrays(nominal,theta[k],fric[k],static,p,h)
        coeff=np.zeros((len(p['q'])//2+1,14),complex);coeff[:h+1]=raw[name]['measured_velocity_coefficients'][:h+1]
        velocity=np.fft.irfft(coeff*len(p['q']),n=len(p['q']),axis=0)
        budget[name]=frequency_budget(p['current_torque'],predictions,velocity,p['ddq'],
            p['current_torque']-parts['A']['gravity']-parts['A']['friction'],parts['A']['rigid_inertia']+parts['A']['rotor'],
            conditions['excitation_harmonics'],h,threshold,protocol['settings'],period_s=conditions['period_s'],active_joints=metrics[name]['active_joints'])
        stats=residual_statistics(p['current_torque'],predictions,biases)
        identity={k:conditions[k] for k in ('center_id','acquisition_batch_id','repeat_id','active_arm_group')}
        trajectories[name]=dict(**identity,statistics=stats,closure=audit,raw_diagnostic=raw_residual_budget(raw[name],predictions,h))
        for j in range(14):
            for k in MODELS:csv_rows.append(dict(trial=name,**identity,joint=j,model=k,active=j in metrics[name]['active_joints'],
                qualified_bins=budget[name]['qualified_bins_per_axis'][j],rmse_nm=stats[k]['rmse_nm'][j],bias_nm=stats[k]['bias_nm'][j],
                demeaned_rmse_nm=stats[k]['demeaned_rmse_nm'][j],qualified_band_rmse_nm=budget[name]['bands'][k]['qualified_excitation_rms_nm'][j]))
    benefit=aggregate_benefit(budget,protocol['settings']);prior=score_predecessors(root,analysis,model,config,protocol)
    coverage=measured_center_coverage(metrics,config)
    # Fresh test centers must also remain separated from all observed development centers.
    from .staged_multiconfiguration import center_separation
    planned=_read(root/'configuration_coverage.json')['centers'];cross=[]
    for name,q in coverage['centers'].items():
        for other,value in planned.items():
            if not other.startswith('test_'):cross.append(dict(first=name,second=other,**center_separation(q,value,np.asarray(config['pose_amplitude_rad']),config['p0']['center_design'])))
    coverage['development_separation']=cross;coverage['passed']=coverage['passed'] and all(r['passed'] for r in cross)
    r3=_read(analysis/'models/selection_attribution.json');selection=next(r for r in r3['candidates'] if r['ridge']==frozen['ridge'])
    selection_benefit=selection['splits']['selection']['attribution'];stability=selection['stability_passed']
    accepted=bool(prediction['passed'] and prior['passed'] and coverage['passed'] and stability and benefit['qualified_non_degradation_passed'] and benefit['full_domain_evidence'])
    report=dict(schema_version=1,round='R4',passed=accepted,model_acceptance_passed=accepted,completed=True,
        final_protocol_sha256=file_hash(analysis/'final_protocol.json'),parent_protocol_sha256=file_hash(root/'protocol.json'),candidate_sha256=file_hash(root/frozen['candidate']),
        candidate_freeze_sha256=file_hash(root/'candidate_freeze.json'),ridge=frozen['ridge'],test_consumed=True,test_used_for_fit=False,truth_read=False,rigid_fit_calls=0,
        publishable=False,hardware_executable=False,p0_acceptance_passed=False,pending='comprehensive collision/numerics/truth isolation, control and presentation',
        prediction=prediction,frequency_attribution=benefit,selection_attribution=selection_benefit,
        final_inertia_benefit_claim_supported=bool(benefit['inertia_benefit_claim_supported'] and selection_benefit['inertia_benefit_claim_supported']),
        prior_holdout=prior,configuration_coverage=coverage,trajectories=trajectories,
        input_sha256={name:file_hash(root/name) for name in ('role_inputs/test.json','final_prior_inputs.json','candidate_freeze.json')},
        seconds=perf_counter()-started)
    write_json(output/'report.json',report);write_json(output/'frequency_budget.json',budget)
    with (output/'axis_center_batch.csv').open('w',newline='',encoding='utf-8') as s:
        writer=csv.DictWriter(s,fieldnames=list(csv_rows[0]));writer.writeheader();writer.writerows(csv_rows)
    _seal(output,'manifest.json',dict(files={p.name:file_hash(p) for p in output.iterdir() if p.is_file()}))
    return report


def run_final(root,attribution,output,config,*,resume=False,fps=12):
    root=Path(root).resolve();output=Path(output).resolve();started=perf_counter()
    if output.exists():
        if not resume:raise FileExistsError(output)
        protocol,public=load_final_protocol(root,output)
    else:protocol,public=prepare_final(root,attribution,output)
    if public!={k:v for k,v in config.items() if k!='mock'}:raise ValueError('R4 configuration changed')
    collect_final_test(root,config,resume=resume)
    if (output/'acceptance').exists():
        check_manifest(output/'acceptance','manifest.json');accepted=_read(output/'acceptance/report.json')
    else:accepted=score_final(root,output,output/'acceptance')
    if not accepted['model_acceptance_passed']:
        write_json(output/'checkpoint.json',dict(passed=False,completed_through='final_model_rejected',model_acceptance_passed=False,test_consumed=True,
            p0_acceptance_passed=False,scope='Frozen test failed; no model/threshold tuning on this test'))
        return accepted
    from .staged_final_validation import verify_final
    if not (output/'verification').exists():verify_final(root,output,output/'verification')
    checked=_read(output/'verification/verification.json')
    if not checked['passed']:raise RuntimeError('R4 comprehensive verification rejected; controls blocked')
    from .staged_control import run_frozen_control
    from .staged_collection import synthetic_plant
    from .collision import nominal_profile
    if not (output/'control/control_comparison.json').exists():
        legacy=_read(output/'control/session.json')['config'];plant=synthetic_plant(config)
        profile=nominal_profile(scene=_read(root/'session.json')['collision']['scene'])
        run_frozen_control(output/'control',plant,legacy,profile,root,output/'acceptance/report.json',resume=resume)
        _seal(output/'control','results_manifest.json',dict(files={str(p.relative_to(output/'control')):file_hash(p) for p in (output/'control').rglob('*') if p.is_file() and 'evaluation' not in p.parts}))
    check_manifest(output/'control','results_manifest.json')
    from .staged_final_presentation import render_final
    if not (output/'presentation').exists():render_final(output,output/'presentation',fps=fps)
    return finalize_final(root,output,started)


def finalize_final(root,output,started=None):
    protocol,config=load_final_protocol(root,output);check_manifest(output/'acceptance','manifest.json');check_manifest(output/'control','results_manifest.json')
    check_manifest(output/'verification','manifest.json')
    accepted=_read(output/'acceptance/report.json');checked=_read(output/'verification/verification.json');control=_read(output/'control/control_comparison.json');view=_read(output/'presentation/presentation.json')
    for path,expected in checked['input_sha256'].items():
        if file_hash(root/path)!=expected:raise ValueError('R4 comprehensive inputs changed')
    for path,expected in view['source_sha256'].items():
        if file_hash(Path(path))!=expected:raise ValueError('R4 playback inputs changed')
    video=output/'presentation/staged_calibration_zh.mp4'
    passed=bool(accepted['model_acceptance_passed'] and checked['passed'] and control['passed'] and view['passed'] and video.is_file() and video.stat().st_size>0)
    result=dict(schema_version=1,round='R4',passed=passed,p0_acceptance_passed=passed,full_plan_complete=passed,
        synthetic_only=True,hardware_executable=False,publishable=False,test_consumed=True,test_used_for_fit=False,rigid_fit_calls=0,
        candidate_sha256=protocol['candidate_sha256'],candidate_freeze_sha256=protocol['candidate_freeze_sha256'],ridge=protocol['ridge'],
        p0=dict(multiconfiguration=accepted['prediction']['passed'] and accepted['configuration_coverage']['passed'],operator_stability=True,
            frequency_attribution=accepted['frequency_attribution']['qualified_non_degradation_passed'],final_inertia_benefit_claim_supported=accepted['final_inertia_benefit_claim_supported']),
        collected_counts=protocol['expected_counts'],new_calibration_trials=len(protocol['acquisition_order']),new_control_trials=9,
        new_virtual_duration_s=sum(_read(root/'plans'/n/'motion.json')['duration_s'] for n in protocol['acquisition_order']),
        fixed_pd_feedforward_rmse_reduction=control['fixed_pd_feedforward_rmse_reduction'],video_sha256=file_hash(video),
        checks=dict(model_acceptance=accepted['model_acceptance_passed'],comprehensive=checked['passed'],control=control['passed'],presentation=view['passed']),
        seconds=None if started is None else perf_counter()-started,
        input_sha256={str(p):file_hash(p) for p in (output/'final_protocol.json',output/'acceptance/report.json',output/'verification/verification.json',output/'control/results_manifest.json',output/'presentation/presentation.json')},
        limitations=['Nominal fixed-waist synthetic model; fixture and real limits remain unmeasured','Current-derived torque and declared scale; no independent torque sensor',
            '40-harmonic validation with explicit outside-band residual; not full-band performance','No new physical network, real device, cross-day thermal or Windows acceptance'])
    write_json(output/'delivery.json',result)
    return result
