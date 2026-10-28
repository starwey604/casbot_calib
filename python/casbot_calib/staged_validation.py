"""Comprehensive checks beyond fitting: geometry, numerics and truth isolation."""
import hashlib,json,subprocess,sys,tempfile
from pathlib import Path
from time import perf_counter
import numpy as np
from . import _core
from .data import write_json
from .collision import nominal_profile,certify_polyline
from .staged_collection import load_trial
from .staged_control import load_candidate
from .staged_motion import Motion,gate_motion
from .dual_model import nominal_dual_arm


def numerical_replays(root,model,config,names,model_scope='frozen fitted model'):
    root=Path(root);rows=[]
    for name in names:
        meta,curve=load_trial(root,name)
        with np.load(root/'runs'/name/'commands.npz',allow_pickle=False) as data:times=data['send_ns'];commands=data['commands']
        record=json.loads((root/'runs'/name/'metadata.json').read_text());outputs=[];start=perf_counter()
        for step in (record['integration_ns'],record['integration_ns']//2):
            c=_core.ArmSimulationConfig();c.gain=np.full(14,config['drive']['gain_nm_a']);c.stiction_enabled=True
            c.integration_ns=step;c.sample_period_ns=record['sample_period_ns'];c.position_noise=0.;c.current_noise=0.
            c.current_tau=config['drive']['current_tau'];c.command_delay_ns=config['drive']['command_delay_ns'];c.feedback_delay_ns=config['drive']['feedback_delay_ns'];c.torque_limit=config['drive']['torque_limit_nm']
            obs,_=_core.ArmSimulationRunner(model,c).run(times,commands,round(meta['duration_s']*1e9),curve(0),np.zeros(14));outputs.append(obs)
        a,b=outputs;q_scale=np.maximum(np.ptp(b['q'],axis=0),.01)
        trajectory=np.max(abs(a['q']-b['q']),axis=0)/q_scale
        speed=abs(np.max(abs(a['dq']),axis=0)-np.max(abs(b['dq']),axis=0))/np.maximum(np.max(abs(b['dq']),axis=0),.05)
        current=abs(np.max(abs(a['current']),axis=0)-np.max(abs(b['current']),axis=0))/np.maximum(np.max(abs(b['current']),axis=0),.05)
        error=max(float(max(trajectory)),float(max(speed)),float(max(current)))
        rows.append(dict(trial=name,maximum_integration_step_ns=record['integration_ns'],half_step_ns=record['integration_ns']//2,
            maximum_normalized_trajectory_change=float(max(trajectory)),maximum_peak_speed_change=float(max(speed)),
            maximum_peak_current_change=float(max(current)),passed=error<.01,wall_s=perf_counter()-start,
            scope='same '+model_scope+' and actually executed commands; no refitting; comparison of maximum step and half step'))
        print(f"积分步收敛 {name}: {'PASS' if error<.01 else 'FAIL'} ({error:.3%})",flush=True)
    return rows


def energy_oracle():
    nominal,_=nominal_dual_arm();model=nominal.model;model.gravity_world=np.zeros(3);joints=model.joints
    for joint in joints:
        f=joint.friction;f.viscous=.035;f.coulomb=.04;f.static_friction=.07;f.stribeck_velocity=.002;joint.friction=f
    model.joints=joints;plant=_core.ArmDynamics(model);workspace=plant.workspace()
    c=_core.ArmSimulationConfig();c.gain=np.ones(14);c.current_tau=0.;c.command_delay_ns=0;c.position_noise=0.;c.current_noise=0.;c.stiction_enabled=True;c.sample_period_ns=c.integration_ns
    times=np.array([0],dtype=np.int64);commands=np.zeros((1,14,5));q=np.array([0,.45,0,-.9,0,0,0,0,-.45,0,-.9,0,0,0])
    obs,_=_core.ArmSimulationRunner(plant,c).run(times,commands,200000000,q,np.linspace(-.2,.2,14))
    energy=np.array([workspace.energy(q,v).kinetic for q,v in zip(obs['q'],obs['dq'])]);increase=float(np.max(np.diff(energy)))
    return dict(passed=bool(increase<1e-10 and energy[-1]<energy[0]),initial_joule=float(energy[0]),final_joule=float(energy[-1]),
        largest_step_increase_joule=increase,scope='public nominal inertias, zero gravity/input, declared dissipative friction; every integration step observed')


def component_audit(root,model):
    root=Path(root);_,curve=load_trial(root,'inertia_validation');t=np.linspace(3.,25.,128);q,v,a=(curve(t,nu=d) for d in range(3))
    parts=model.evaluate_batch(q,v,a);sum_error=np.max(abs(parts['total']-(parts['inertia']+parts['velocity']+parts['gravity']+parts['friction'])))
    eigen=np.linalg.eigvalsh(parts['mass_matrix']);minimum=float(eigen.min())
    return dict(passed=bool(sum_error<1e-10 and minimum>0),maximum_component_sum_error_nm=float(sum_error),minimum_mass_eigenvalue=minimum,
        maximum_velocity_coupling_nm=float(np.max(abs(parts['velocity']))),
        maximum_mass_variation=float(np.max(abs(parts['mass_matrix']-parts['mass_matrix'][0]))),
        scope='Pinocchio M(q), velocity coupling, gravity and fitted friction; matrix is configuration dependent')


def truth_isolation(root,output,reference_candidate):
    root=Path(root).resolve();output=Path(output).resolve();evaluation=root/'evaluation';archive=None
    before={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in root.glob('runs/*/observations.npz')}
    with tempfile.TemporaryDirectory(prefix='casbot_private_',dir=root.parent) as temp:
        if evaluation.exists():archive=Path(temp)/'private';evaluation.rename(archive)
        try:
            process=subprocess.run([sys.executable,'-m','casbot_calib.cli','staged-identify',str(root),'--output',str(output)],capture_output=True,text=True)
        finally:
            if archive is not None:archive.rename(evaluation)
    log=output.parent/'truth_isolation_process.txt';log.write_text(process.stdout+'\n'+process.stderr,encoding='utf-8')
    if process.returncode!=0:raise RuntimeError('Fresh-process inference without evaluation failed; see '+str(log))
    other=load_candidate(output/'inertia'/'candidate.json');reference=load_candidate(reference_candidate);error=0.
    _,curve=load_trial(root,'inertia_validation')
    for t in np.linspace(3.,25.,24):
        q,v,a=(curve(np.array([t]),nu=d)[0] for d in range(3))
        error=max(error,float(np.max(abs(reference.inverse(q,v,a)-other.inverse(q,v,a)))),float(np.max(abs(reference.mass_matrix(q)-other.mass_matrix(q)))))
    unchanged=before=={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in root.glob('runs/*/observations.npz')}
    return dict(passed=error<1e-6 and unchanged,maximum_operator_difference=error,raw_unchanged=unchanged,
        protocol='entire evaluation directory physically moved outside session; new Python process; restored in finally; no latent plant passed to inference')


def verify_staged(folder,output,stability_analysis=None,attribution_analysis=None,freeze=False,final_analysis=None):
    from .staged_protocol import require_legacy_execution,p0_status
    root=Path(folder);config=json.loads((root/'session.json').read_text())['config']
    if final_analysis is not None:
        if stability_analysis is not None or attribution_analysis is not None or freeze or config.get('schema_version')!=2:raise ValueError('R4 verification requires exclusive final analysis and P0 parent')
        from .staged_final_validation import verify_final
        return verify_final(root,final_analysis,output)
    if attribution_analysis is not None:
        if stability_analysis is not None or config.get('schema_version')!=2:raise ValueError('R3 verification requires P0 parent and exclusive analysis input')
        from .staged_attribution_validation import verify_attribution
        return verify_attribution(root,attribution_analysis,output,freeze=freeze)
    if freeze:raise ValueError('Candidate freeze requires R3 attribution verification')
    if stability_analysis is not None:
        if config.get('schema_version')!=2:raise ValueError('R2 verification requires P0 parent records')
        from .staged_stability_validation import verify_stability
        return verify_stability(root,stability_analysis,output)
    if config.get('schema_version')==2:
        from .staged_protocol import load_protocol
        protocol,_,_=load_protocol(root)
        if protocol['schema_version']==2:
            from .staged_development_validation import verify_development
            return verify_development(root,output)
    require_legacy_execution(config)
    output=Path(output);output.mkdir(parents=True,exist_ok=False);workflow=json.loads((root/'workflow.json').read_text())
    if not workflow['passed']:raise ValueError('Calibration/control workflow has not passed')
    session=json.loads((root/'session.json').read_text());config=session['config'];profile=nominal_profile(scene=session['collision']['scene'])
    plan=json.loads((root/'plan_report.json').read_text());rows=[]
    for index,name in enumerate(plan['selected_trials']):
        meta,_=load_trial(root,name);record=json.loads((root/'runs'/name/'metadata.json').read_text())
        with np.load(root/'runs'/name/'observations.npz',allow_pickle=False) as data:obs={k:data[k] for k in data.files}
        if hashlib.sha256((root/'runs'/name/'observations.npz').read_bytes()).hexdigest()!=record['raw_sha256']:raise ValueError('Raw changed: '+name)
        motion=Motion(meta['name'],meta['stage'],np.asarray(config['home_rad']),meta['conditions']);motion.knots=meta['knots_s'];motion.states=[np.asarray(s) for s in meta['states']];motion.labels=meta['labels']
        reference=gate_motion(profile,motion,session['collision']['scene']['clearance_m'])
        actual=certify_polyline(profile,obs['q'],obs['sample_ns']*1e-9,clearance_m=session['collision']['scene']['clearance_m'],max_gap_s=record['sample_period_ns']*1.5e-9)
        rows.append(dict(trial=name,reference_passed=reference['passed'],observed_interpolant_passed=actual['passed'],
            minimum_sampled_distance_m=actual['minimum_sampled_distance_m'],scope=actual['path_model']))
        if (index+1)%25==0:print(f'复核碰撞 {index+1}/{len(plan["selected_trials"])}',flush=True)
    model=load_candidate(root/workflow['inertia_folder']/'candidate.json')
    replay=numerical_replays(root,model,config,['direction_00','timing_train','friction_validation_00_03_0.14','breakaway_0_03_+1_0.5_0.24_0','wrist_probe_1_04_+1','inertia_validation'])
    energy=energy_oracle();components=component_audit(root,model)
    isolated=truth_isolation(root,output/'without_evaluation',root/workflow['inertia_folder']/'candidate.json')
    checks=dict(all_trials_collected=len(rows)==len(plan['selected_trials']),all_reference_gates=all(r['reference_passed'] for r in rows),
        all_recorded_interpolants=all(r['observed_interpolant_passed'] for r in rows),step_halving=all(r['passed'] for r in replay),
        dissipative_energy=energy['passed'],physical_components=components['passed'],truth_isolation=isolated['passed'])
    inputs=[root/name for name in ('session.json','plan_report.json','workflow.json','control_comparison.json','controller_selection.json')]
    inputs += [root/workflow['inertia_folder']/'candidate.json']+[root/value['report'] for value in workflow['stages'].values()]
    inputs += [p for directory in ('runs','plans') for p in (root/directory).rglob('*') if p.is_file()]
    report=dict(passed=all(checks.values()),p0=p0_status(config),checks=checks,collision=rows,numerical_replays=replay,energy=energy,components=components,truth_isolation=isolated,
        input_sha256={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
        synthetic_only=True,hardware_executable=False,limitations=['Unmeasured fixture proxy and nominal geometry','Recorded interpolation is conditional, not unobserved hardware motion certification','Physical Ethernet and real SDK not tested'])
    write_json(output/'verification.json',report);return report
