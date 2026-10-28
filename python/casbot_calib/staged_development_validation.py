"""Independent R1 checks; deliberately cannot claim three-P0/R4 acceptance."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import numpy as np
from .data import write_json
from .staged_protocol import file_hash, load_protocol, _read


def verify_development(root, output):
    from .collision import nominal_profile, certify_polyline
    from .staged_collection import load_trial
    from .staged_motion import Motion, gate_motion
    from .staged_validation import numerical_replays
    from .staged_control import load_candidate
    from .staged_multiconfiguration import load_development_reference
    from .staged_development import frozen_predecessors
    root=Path(root).resolve();output=Path(output).resolve()
    protocol,config,manifest=load_protocol(root);workflow=_read(root/'workflow.json')
    if workflow.get('round')!='R1' or not workflow['passed']:
        raise ValueError('An accepted R1 development checkpoint is required')
    if output.exists():raise FileExistsError(output)
    output.mkdir(parents=True,exist_ok=False);load_development_reference(root);frozen_predecessors(root)
    profile=nominal_profile(scene=_read(root/'session.json')['collision']['scene'])
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
    model=load_candidate(root/'identification_development/candidate.json')
    selected=[next(r['trial_id'] for r in manifest['trials'] if r['split']==split and r['stage']=='inertia' and r['repeat_id']==0) for split in ('train','selection')]
    selected += [next(r['trial_id'] for r in manifest['trials'] if r['split']=='selection' and r['stage']=='inertia_probe' and r['joint']==j and r['repeat_id']==0) for j in (4,11)]
    numerical=numerical_replays(root,model,config,selected,model_scope='R1 development fitted model; no plant truth')
    # Both the current session and reused predecessor truth are inaccessible.
    source=load_development_reference(root);moved=[]
    with tempfile.TemporaryDirectory(prefix='p0_r1_private_',dir=root.parent) as temp:
        try:
            for i,path in enumerate((root/'evaluation',source/'evaluation')):
                if path.exists():
                    archive=Path(temp)/str(i);path.rename(archive);moved.append((path,archive))
            process=subprocess.run([sys.executable,'-m','casbot_calib.cli','staged-identify',str(root),'--output',str(output/'without_evaluation')],text=True,capture_output=True)
        finally:
            for path,archive in reversed(moved):archive.rename(path)
    (output/'truth_isolation_process.txt').write_text(process.stdout+'\n'+process.stderr,encoding='utf-8')
    if process.returncode!=0:raise RuntimeError('R1 truth-isolated reanalysis failed; see process log')
    other=load_candidate(output/'without_evaluation/inertia/candidate.json');difference=0.
    for name in selected[:2]:
        _,curve=load_trial(root,name)
        for t in np.linspace(3.,25.,16):
            q,v,a=(curve(t,nu=k) for k in range(3))
            difference=max(difference,float(np.max(abs(model.mass_matrix(q)-other.mass_matrix(q)))),float(np.max(abs(model.inverse(q,v,a)-other.inverse(q,v,a)))))
    # Explicit evaluator: plant parameters enter only this numerical replay, not
    # inference, weighting, diagnostics or model/candidate selection.
    from .staged_collection import synthetic_plant
    plant=synthetic_plant({**config,'mock':_read(root/'evaluation/plant_config.json')})
    plant_numerical=numerical_replays(root,plant,config,selected,model_scope='declared synthetic plant, evaluation only')
    test_names=[r['trial_id'] for r in manifest['trials'] if r['split']=='test']
    test_names += [r['trial_id'] for r in _read(root/'motion_plan.json')['final_prior_trials']]
    checks=dict(all_reference_gates=all(r['reference_passed'] for r in rows),all_observed_interpolants=all(r['observed_interpolant_passed'] for r in rows),
        integration_step_halving=all(r['passed'] for r in numerical),truth_isolation=difference<1e-6,
        evaluation_plant_integration_step_halving=all(r['passed'] for r in plant_numerical),
        formal_test_untouched=not any((root/'runs'/name).exists() for name in test_names))
    result=dict(round='R1',passed=all(checks.values()),development_verified=all(checks.values()),p0_acceptance_passed=False,
        formal_test_consumed=False,checks=checks,collision=rows,numerical_replays=numerical,evaluation_plant_numerical_replays=plant_numerical,
        truth_isolation_maximum_operator_difference=difference,protocol_sha256=file_hash(root/'protocol.json'),
        scope='R1 development references/observed interpolation, representative integration convergence and fresh-process reanalysis; no formal test, R2/R3 stability/attribution or hardware')
    write_json(output/'verification.json',result)
    return result
