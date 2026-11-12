"""Independent R1 checks; deliberately cannot claim three-P0/R4 acceptance."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import numpy as np
from .data import write_json
from .model_runtime import collision_for,context_for
from .staged_protocol import file_hash, load_protocol, _read


def verify_development(root, output,*,analysis=None,diagnose_rejected_right_pitch=False):
    from .collision import certify_polyline
    from .staged_collection import load_trial
    from .staged_motion import Motion, gate_motion
    from .staged_validation import numerical_replays
    from .staged_control import load_candidate
    from .staged_multiconfiguration import load_development_reference
    from .staged_development import frozen_predecessors
    root=Path(root).resolve();output=Path(output).resolve()
    protocol,config,manifest=load_protocol(root);workflow=_read(root/'workflow.json')
    reuse=False
    if analysis is not None:
        from .staged_attribution import check_manifest
        analysis=Path(analysis).resolve();check_manifest(analysis,'manifest.json');workflow=_read(analysis/'analysis.json')
        if workflow.get('protocol_sha256')!=file_hash(root/'protocol.json'):raise ValueError('Development reanalysis belongs to another protocol')
        fit_folder=analysis/workflow['inertia_folder'];report=_read(fit_folder/'report.json');reuse=workflow.get('reuse_development_fit',False)
        expected={k:file_hash(root/f'role_inputs/{k}.json') for k in ('train','selection')}
        if report['protocol_sha256']!=workflow['protocol_sha256'] or report['input_manifests']!=expected or not report['development_passed']:
            raise ValueError('Rejected or stale development reanalysis')
    else:fit_folder=root/workflow.get('inertia_folder','identification_development');report=_read(fit_folder/'report.json')
    if diagnose_rejected_right_pitch:
        from .staged_right_pitch import rejected_r1_evidence
        if analysis is not None:
            raise ValueError('Use the original right-pitch checkpoint for rejection diagnostics')
        rejected_r1_evidence(root)
    elif workflow.get('round')!='R1' or not workflow['passed']:
        raise ValueError('An accepted R1 development checkpoint or explicit accepted reanalysis is required')
    if output.exists():raise FileExistsError(output)
    output.mkdir(parents=True,exist_ok=False);load_development_reference(root);frozen_predecessors(root)
    profile=collision_for(config,scene=_read(root/'session.json')['collision']['scene'])
    names=manifest['planned_execution_order']['train']+manifest['planned_execution_order']['selection'];rows=[]
    for name in names:
        meta,_=load_trial(root,name)
        motion=Motion(name,meta['stage'],np.asarray(config['home_rad']),meta['conditions'])
        motion.knots=meta['knots_s'];motion.states=[np.asarray(s) for s in meta['states']];motion.labels=meta['labels']
        reference=gate_motion(profile,motion,profile.metadata['scene']['clearance_m'])
        record=_read(root/'runs'/name/'metadata.json')
        if file_hash(root/'runs'/name/'observations.npz')!=record['raw_sha256']:raise ValueError('Raw changed before verification')
        with np.load(root/'runs'/name/'observations.npz',allow_pickle=False) as arrays:
            actual=certify_polyline(profile,arrays['q'],arrays['sample_ns']*1e-9,
                clearance_m=profile.metadata['scene']['clearance_m'],max_gap_s=record['sample_period_ns']*1.5e-9)
        rows.append(dict(trial=name,reference_passed=reference['passed'],observed_interpolant_passed=actual['passed'],minimum_distance_m=actual['minimum_sampled_distance_m']))
    companions=[]
    if 'wrist_periodic_train' in config['p0']:
        from .staged_wrist_periodic import companion,wrist_records
        companions.append((*companion(root),wrist_records,'inertia'))
    if 'right_pitch_train' in config['p0']:
        from .staged_right_pitch import right_companion,right_records
        companions.append((*right_companion(root),right_records,'inertia_probe'))
    for _,_,reader,stage in companions:
        for name,meta,obs in reader(root):
            motion=Motion(name,stage,np.asarray(config['home_rad']),meta['conditions'])
            motion.knots=meta['knots_s'];motion.states=[np.asarray(s) for s in meta['states']];motion.labels=meta['labels']
            reference=gate_motion(profile,motion,profile.metadata['scene']['clearance_m'])
            actual=certify_polyline(profile,obs['q'],obs['sample_ns']*1e-9,
                clearance_m=profile.metadata['scene']['clearance_m'],max_gap_s=config['inertia_sample_period_ns']*1.5e-9)
            rows.append(dict(trial=name,reference_passed=reference['passed'],observed_interpolant_passed=actual['passed'],
                minimum_distance_m=actual['minimum_sampled_distance_m']))
    model=load_candidate(fit_folder/'candidate.json',context_for(config))
    selected=[next(r['trial_id'] for r in manifest['trials'] if r['split']==split and r['stage']=='inertia' and r['repeat_id']==0) for split in ('train','selection')]
    selected += [next(r['trial_id'] for r in manifest['trials'] if r['split']=='selection' and r['stage']=='inertia_probe' and r['joint']==j and r['repeat_id']==0) for j in (4,11)]
    selected += [r['trial_id'] for r in manifest['trials'] if r.get('supplemental_wrist') and r['repeat_id']==0]
    numerical=numerical_replays(root,model,config,selected,model_scope='R1 development fitted model; no plant truth')
    for capture,names,_,_ in companions:
        numerical+=numerical_replays(capture,model,config,[n for n in names if n.endswith('_b00')],
                                     model_scope='R1 supplemental wrist fitted model; no plant truth')
    args=[sys.executable,'-m','casbot_calib.cli','staged-identify',str(root),'--output',str(output/'without_evaluation')]
    if reuse:args.append('--reuse-development-fit')
    if 'wrist_supplement' in config['p0']:
        # Deny access in a fresh interpreter; leave the reused source session untouched.
        guard = """import os, sys
from pathlib import Path
def guard(event, args):
    if event == 'open' and isinstance(args[0], (str, bytes, os.PathLike)):
        if 'evaluation' in Path(os.fsdecode(args[0])).resolve().parts:
            raise PermissionError('R1 reanalysis may not open evaluation truth')
sys.addaudithook(guard)
from casbot_calib.cli import main
raise SystemExit(main(sys.argv[1:]))
"""
        process=subprocess.run([sys.executable,'-c',guard,*args[3:]],text=True,capture_output=True)
    else:
        # Both the current session and reused predecessor truth are inaccessible.
        source=load_development_reference(root);moved=[]
        with tempfile.TemporaryDirectory(prefix='p0_r1_private_',dir=root.parent) as temp:
            try:
                for i,path in enumerate((root/'evaluation',source/'evaluation')):
                    if path.exists():
                        archive=Path(temp)/str(i);path.rename(archive);moved.append((path,archive))
                process=subprocess.run(args,text=True,capture_output=True)
            finally:
                for path,archive in reversed(moved):archive.rename(path)
    (output/'truth_isolation_process.txt').write_text(process.stdout+'\n'+process.stderr,encoding='utf-8')
    if process.returncode!=int(diagnose_rejected_right_pitch):
        raise RuntimeError('R1 truth-isolated reanalysis did not reproduce its expected outcome; see process log')
    if _read(output/'without_evaluation/analysis.json')['passed']!= (not diagnose_rejected_right_pitch):
        raise ValueError('Independent R1 outcome differs from the checkpoint')
    other=load_candidate(output/'without_evaluation/inertia/candidate.json',context_for(config));difference=0.
    for name in selected[:2]:
        _,curve=load_trial(root,name)
        for t in np.linspace(3.,25.,16):
            q,v,a=(curve(t,nu=k) for k in range(3))
            difference=max(difference,float(np.max(abs(model.mass_matrix(q)-other.mass_matrix(q)))),float(np.max(abs(model.inverse(q,v,a)-other.inverse(q,v,a)))))
    from .staged_attribution_validation import equivalent
    reproduced=_read(output/'without_evaluation/inertia/report.json')
    equivalent(report['validation'],reproduced['validation']);equivalent(report['checks'],reproduced['checks'])
    if diagnose_rejected_right_pitch:
        equivalent(report['probe_validation'],reproduced['probe_validation'])
    # Explicit evaluator: plant parameters enter only this numerical replay, not
    # inference, weighting, diagnostics or model/candidate selection.
    from .staged_collection import synthetic_plant
    plant=synthetic_plant({**config,'mock':_read(root/'evaluation/plant_config.json')})
    plant_numerical=numerical_replays(root,plant,config,selected,model_scope='declared synthetic plant, evaluation only')
    for capture,names,_,_ in companions:
        plant_numerical+=numerical_replays(capture,plant,config,[n for n in names if n.endswith('_b00')],
                                           model_scope='declared synthetic plant, evaluation only')
    test_names=[r['trial_id'] for r in manifest['trials'] if r['split']=='test']
    test_names += [r['trial_id'] for r in _read(root/'motion_plan.json')['final_prior_trials']]
    checks=dict(all_reference_gates=all(r['reference_passed'] for r in rows),all_observed_interpolants=all(r['observed_interpolant_passed'] for r in rows),
        integration_step_halving=all(r['passed'] for r in numerical),truth_isolation=difference<1e-6,
        evaluation_plant_integration_step_halving=all(r['passed'] for r in plant_numerical),
        formal_test_untouched=not any((root/'runs'/name).exists() for name in test_names))
    result=dict(round='R1',passed=all(checks.values()) and not diagnose_rejected_right_pitch,
        development_verified=all(checks.values()) and not diagnose_rejected_right_pitch,p0_acceptance_passed=False,
        formal_test_consumed=False,checks=checks,collision=rows,numerical_replays=numerical,evaluation_plant_numerical_replays=plant_numerical,
        truth_isolation_maximum_operator_difference=difference,development_analysis=None if analysis is None else str(analysis),
        development_report_sha256=file_hash(fit_folder/'report.json'),stationary_metric_reproduced=True,protocol_sha256=file_hash(root/'protocol.json'),
        scope='R1 development references/observed interpolation, representative integration convergence and fresh-process reanalysis; no formal test, R2/R3 stability/attribution or hardware')
    if diagnose_rejected_right_pitch:
        result.update(rejection_reproduced=True, diagnostic_checks_passed=all(checks.values()),
                      rejected_development_checks=report['checks'])
    write_json(output/'verification.json',result)
    return result
