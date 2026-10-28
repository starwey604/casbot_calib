"""R4 geometry/numerics and fresh-process observation-only final acceptance audit."""
from pathlib import Path
import subprocess
import sys
import tempfile
import numpy as np
from .data import write_json
from .staged_protocol import file_hash,_read,_seal
from .staged_final import load_final_protocol
from .staged_attribution import check_manifest
from .staged_attribution_validation import equivalent


def verify_final(root,analysis,output):
    from .collision import nominal_profile,certify_polyline
    from .staged_collection import load_trial,synthetic_plant
    from .staged_motion import Motion,gate_motion
    from .staged_validation import numerical_replays,energy_oracle
    from .staged_control import load_candidate
    from .staged_multiconfiguration import load_development_reference
    root=Path(root).resolve();analysis=Path(analysis).resolve();output=Path(output).resolve()
    protocol,config=load_final_protocol(root,analysis);check_manifest(analysis/'acceptance','manifest.json')
    # The collector enforces its critical core immediately. The comprehensive
    # audit additionally checks all package dependencies before release. Only
    # this independent checker and the read-only presentation have separately
    # archived runtime revisions; neither changes fit/score/threshold inputs.
    runtime_archived={'staged_final_validation.py','staged_final_presentation.py'}
    for name,expected in protocol['source_sha256'].items():
        if name not in runtime_archived and file_hash(Path(__file__).parent/name)!=expected:
            raise ValueError('R4 dependency changed since preregistration: '+name)
    accepted=_read(analysis/'acceptance/report.json')
    if not accepted['model_acceptance_passed']:raise ValueError('Final candidate rejected; no comprehensive pass')
    for name,expected in accepted['input_sha256'].items():
        if file_hash(root/name)!=expected:raise ValueError('Final acceptance input changed')
    output.mkdir(parents=True,exist_ok=False)
    (output/'verification_source.py').write_bytes(Path(__file__).read_bytes())
    # Preserve the pre-existing synthetic evaluator's 5% limits. This is a
    # separate truth-only gate, never an input to score_final or model choice.
    _seal(output,'synthetic_evaluation_policy.json',dict(schema_version=1,maximum_matrix_diagonal_relative_error=.05,
        maximum_friction_relative_error=.05,query_references=protocol['replay_trials'][:3],points_per_reference=128,
        source='unchanged 5% limits from staged_delivery.evaluate_synthetic; fixed planned reference probes, not residual-selected poses',
        used_by_identification=False,used_for_selection=False))
    profile=nominal_profile(scene=_read(root/'session.json')['collision']['scene']);collision=[]
    for i,name in enumerate(protocol['all_calibration_trials']):
        meta,_=load_trial(root,name);motion=Motion(name,meta['stage'],np.asarray(config['home_rad']),meta['conditions'])
        motion.knots=meta['knots_s'];motion.states=[np.asarray(s) for s in meta['states']];motion.labels=meta['labels']
        reference=gate_motion(profile,motion,profile.metadata['scene']['clearance_m'])
        record=_read(root/'runs'/name/'metadata.json')
        if file_hash(root/'runs'/name/'observations.npz')!=record['raw_sha256']:raise ValueError('Raw changed before collision audit')
        if name in protocol['acquisition_order'] and record.get('final_protocol_sha256')!=file_hash(analysis/'final_protocol.json'):raise ValueError('Acquisition predates R4 registration')
        with np.load(root/'runs'/name/'observations.npz',allow_pickle=False) as data:
            actual=certify_polyline(profile,data['q'],data['sample_ns']*1e-9,clearance_m=profile.metadata['scene']['clearance_m'],max_gap_s=record['sample_period_ns']*1.5e-9)
        collision.append(dict(trial=name,reference_passed=reference['passed'],observed_interpolant_passed=actual['passed'],minimum_distance_m=actual['minimum_sampled_distance_m']))
        if i%64==0:print(f'R4 independent collision {i+1}/{len(protocol["all_calibration_trials"])}',flush=True)
    events=[_read_json(line) for line in (root/'execution_order.jsonl').read_text().splitlines()]
    completed=[r['trial_id'] for r in events if r['event']=='completed']
    from .staged_protocol import load_protocol
    manifest=load_protocol(root)[2]
    expected=manifest['planned_execution_order']['train']+manifest['planned_execution_order']['selection']+protocol['acquisition_order']
    if completed!=expected:raise ValueError('Acquisition order/duplicates differ from frozen block order')
    model=load_candidate(analysis/'models/D.json')
    numerical=numerical_replays(root,model,config,protocol['replay_trials'],model_scope='R4 frozen candidate')
    energy=energy_oracle()
    before={name:file_hash(root/'runs'/name/'observations.npz') for name in protocol['all_calibration_trials']}
    # Every current and externally reused truth directory is physically moved.
    source=load_development_reference(root);moved=[]
    with tempfile.TemporaryDirectory(prefix='p0_r4_private_',dir=root.parent) as tmp:
        try:
            for i,path in enumerate((root/'evaluation',source/'evaluation')):
                if not path.is_dir():raise ValueError('Missing declared evaluation directory before isolation')
                archive=Path(tmp)/str(i);path.rename(archive);moved.append((path,archive))
            guard='''import sys
from casbot_calib import cli
import casbot_calib.staged_final
def forbidden(*args, **kwargs):
    raise RuntimeError("Read-only final scoring attempted model fitting or collection")
for module in tuple(sys.modules.values()):
    if str(getattr(module, "__name__", "")).startswith("casbot_calib."):
        for name in ("fit_rigid", "synthetic_plant", "collect_trial", "collect_final_test"):
            if hasattr(module, name):
                setattr(module, name, forbidden)
raise SystemExit(cli.main(sys.argv[1:]))
'''
            process=subprocess.run([sys.executable,'-c',guard,'staged-identify',str(root),'--through','final','--final-protocol',str(analysis),'--output',str(output/'without_evaluation')],capture_output=True,text=True)
        finally:
            for path,archive in reversed(moved):archive.rename(path)
    (output/'truth_isolation_process.txt').write_text(process.stdout+'\n'+process.stderr,encoding='utf-8')
    if process.returncode!=0:raise RuntimeError('R4 truth-isolated score failed; see process log')
    if before!={name:file_hash(root/'runs'/name/'observations.npz') for name in protocol['all_calibration_trials']}:raise ValueError('R4 inference modified observations')
    other=_read(output/'without_evaluation/report.json')
    equivalent({k:v for k,v in accepted.items() if k!='seconds'},{k:v for k,v in other.items() if k!='seconds'})
    # Plant is constructed only for numerical evaluation, after inference finishes.
    plant=synthetic_plant({**config,'mock':_read(root/'evaluation/plant_config.json')})
    plant_numerical=numerical_replays(root,plant,config,protocol['replay_trials'],model_scope='declared synthetic plant (evaluation only)')
    evaluation=evaluate_frozen_operators(root,model,plant,_read(output/'synthetic_evaluation_policy.json'))
    write_json(output/'synthetic_evaluation.json',evaluation)
    files=[root/'candidate_freeze.json',root/'role_inputs/test.json',root/'final_prior_inputs.json',root/'execution_order.jsonl']
    files += [p for name in protocol['all_calibration_trials'] for directory in ('plans','runs') for p in (root/directory/name).iterdir() if p.is_file()]
    checks=dict(all_reference_gates=all(r['reference_passed'] for r in collision),all_observed_interpolants=all(r['observed_interpolant_passed'] for r in collision),
        frozen_model_numerical=all(r['passed'] for r in numerical),evaluation_plant_numerical=all(r['passed'] for r in plant_numerical),
        synthetic_operator_evaluation=evaluation['passed'],energy=energy['passed'],truth_isolation=True,acquisition_order=True)
    report=dict(schema_version=1,round='R4',passed=all(checks.values()),checks=checks,collision=collision,numerical_replays=numerical,
        evaluation_plant_numerical_replays=plant_numerical,synthetic_evaluation=evaluation,energy=energy,truth_isolation=dict(passed=True,report_equivalent=True,raw_unchanged=True,
            model_fit_and_collection_factories_forbidden=True,
            subprocess_returncode=process.returncode,truth_directories_restored=all(p.is_dir() for p,a in moved),scope='two entire evaluation directories moved; fresh process; no fitting'),
        final_protocol_sha256=file_hash(analysis/'final_protocol.json'),candidate_sha256=protocol['candidate_sha256'],p0_acceptance_passed=False,
        input_sha256={str(p.relative_to(root)):file_hash(p) for p in files},source_sha256=file_hash(__file__),
        pending='independent controller experiment and Chinese viewer delivery',synthetic_only=True,hardware_executable=False)
    write_json(output/'verification.json',report)
    _seal(output,'manifest.json',dict(files={str(p.relative_to(output)):file_hash(p) for p in output.rglob('*') if p.is_file()}))
    return report


def evaluate_frozen_operators(root,model,plant,policy):
    """Explicit evaluation-only adapter of the legacy M(q)/friction comparison."""
    from .staged_collection import load_trial
    queries=[]
    for name in policy['query_references']:
        meta,curve=load_trial(root,name);begin=meta['knots_s'][meta['labels'].index('periodic')]+12.
        queries.append(curve(begin+np.arange(policy['points_per_reference'])*meta['conditions']['period_s']/policy['points_per_reference']))
    q=np.vstack(queries);zero=np.zeros_like(q)
    actual=plant.evaluate_batch(q,zero,zero)['mass_matrix'];estimated=model.evaluate_batch(q,zero,zero)['mass_matrix']
    diagonal=np.max(abs(np.diagonal(estimated,axis1=1,axis2=2)/np.diagonal(actual,axis1=1,axis2=2)-1),axis=0)
    friction={}
    for field in ('viscous','coulomb','static_friction'):
        truth=np.array([getattr(j.friction,field) for j in plant.model.joints]);fit=np.array([getattr(j.friction,field) for j in model.model.joints])
        friction[field]=dict(truth=truth.tolist(),estimate=fit.tolist(),maximum_relative_error=float(np.max(abs(fit/truth-1))))
    return dict(passed=bool(np.max(diagonal)<=policy['maximum_matrix_diagonal_relative_error'] and all(r['maximum_relative_error']<=policy['maximum_friction_relative_error'] for r in friction.values())),
        query_count=len(q),matrix_diagonal_maximum_relative_errors=diagonal.tolist(),
        maximum_matrix_relative_frobenius_error=float(np.max(np.linalg.norm(estimated-actual,axis=(1,2))/np.linalg.norm(actual,axis=(1,2)))),
        friction=friction,used_by_identification=False,used_for_selection=False,
        scope='synthetic declared plant only; not unique link inertias or hardware truth')


def _read_json(line):
    import json
    return json.loads(line)


def audit_final_control(analysis,output):
    """Post-capture control audit; independent of both gain choice and rendering."""
    from .collision import nominal_profile,certify_polyline
    from .dual_model import nominal_dual_arm
    from .staged_collection import load_trial
    from .staged_control import load_candidate,_metrics,_verify_reused_controller
    from .staged_motion import Motion,gate_motion
    analysis=Path(analysis);control=analysis/'control';session=_read(control/'session.json');config=session['config']
    check_manifest(control,'results_manifest.json');selection=_read(control/'controller_selection.json');result=_read(control/'control_comparison.json')
    profile=nominal_profile(scene=session['collision']['scene']);nominal,_=nominal_dual_arm();candidate=load_candidate(analysis/'models/D.json');rows=[]
    for name in _read(control/'plan_report.json')['selected_trials']:
        meta,_=load_trial(control,name);conditions=meta['conditions'];is_tuning=meta['stage']=='control_tuning'
        factor=conditions['kp_factor'] if is_tuning else selection['selected_kp_factor'] if conditions['gain_strategy']=='tuned' else 1.
        model=nominal if conditions['feedforward']=='nominal' else candidate
        _verify_reused_controller(control,name,model,config,factor)
        metrics=_metrics(control,name,config)
        saved=next(r for r in selection['grid'] if r['trial']==name) if is_tuning else result['groups'][name]
        equivalent(metrics,{k:saved[k] for k in metrics})
        motion=Motion(name,meta['stage'],np.asarray(config['home_rad']),conditions)
        motion.knots=meta['knots_s'];motion.states=[np.asarray(s) for s in meta['states']];motion.labels=meta['labels']
        reference=gate_motion(profile,motion,profile.metadata['scene']['clearance_m'])
        record=_read(control/'runs'/name/'metadata.json')
        with np.load(control/'runs'/name/'observations.npz',allow_pickle=False) as data:
            actual=certify_polyline(profile,data['q'],data['sample_ns']*1e-9,clearance_m=profile.metadata['scene']['clearance_m'],max_gap_s=record['sample_period_ns']*1.5e-9)
        rows.append(dict(trial=name,reference_passed=reference['passed'],actual_interpolant_passed=actual['passed'],model_and_gains_recomputed=True,metrics_recomputed=True))
    audit=dict(passed=bool(len(rows)==9 and result['passed'] and all(r['reference_passed'] and r['actual_interpolant_passed'] for r in rows)),
        records=rows,source_sha256=file_hash(__file__),results_manifest_sha256=file_hash(control/'results_manifest.json'),
        scope='nine control references/actual interpolants rechecked, native feedforward and gains recomputed, scores reproduced; no reselection')
    write_json(output,audit);return audit
