"""Ordered acquisition checkpoints for the new staged Mock workflow.

M5/M6 are added to this same pipeline, with a gate before each new stage. A
checkpoint is a partial result and never reported as completion of the plan.
"""
import json
from pathlib import Path
from .data import write_json
from .collision import nominal_profile
from .dual_model import nominal_dual_arm
from .staged_plan import plan_session
from .staged_collection import synthetic_plant,collect_trial,load_trial,preserve_incomplete_trial
from .staged_preflight import collect_preflight
from .staged_identification import identify_gravity,identify_friction,session_config,recordings
from .staged_breakaway import collect_breakaway,identify_breakaway
from .rigid_identification import model_from_parameters
from .staged_inertia import identify_inertia
from .staged_control import run_control


def run_staged(folder,config,through='all',resume=False,development_reference=None):
    if config.get('schema_version')==2 and config.get('p0',{}).get('planning_round')=='R1':
        if through not in ('inertia','all'):raise ValueError('P0 R1 runs the complete development checkpoint through inertia')
        from .staged_development import run_development
        return run_development(folder,config,resume=resume,development_reference=development_reference)
    from .staged_protocol import require_legacy_execution
    require_legacy_execution(config)
    if through not in ('gravity','friction','breakaway','inertia','all'):raise ValueError('Unknown staged checkpoint')
    root=Path(folder)
    if not root.exists():
        plan=plan_session(root,config)
        if not plan['passed']:raise RuntimeError('Reference planning rejected; collection blocked')
    elif not resume:raise FileExistsError('Use a new output folder, or explicitly resume a checked plan')
    declared=session_config(root)
    if declared!={k:v for k,v in config.items() if k!='mock'}:raise ValueError('Configuration changed after planning')
    mock=json.loads((root/'evaluation'/'plant_config.json').read_text())
    if mock!=config.get('mock'):raise ValueError('Synthetic plant changed after planning')
    session=json.loads((root/'session.json').read_text());profile=nominal_profile(scene=session['collision']['scene']);nominal,_=nominal_dual_arm()
    plant=synthetic_plant({**declared,'mock':mock})
    plan=json.loads((root/'plan_report.json').read_text());results={};gravity=nominal
    if resume:
        for name in plan['selected_trials']:preserve_incomplete_trial(root,name)
    collect_preflight(root,plant,nominal,declared,profile,resume=resume)
    for stage in ('gravity','friction','breakaway','inertia'):
        if stage=='inertia':
            for probe_name in plan['selected_trials']:
                probe_meta,_=load_trial(root,probe_name)
                if probe_meta['stage']!='inertia_probe':continue
                if resume and (root/'runs'/probe_name/'metadata.json').exists():continue
                collect_trial(root,probe_name,plant,gravity_friction_static,declared,profile)
        for name in plan['selected_trials']:
            meta,_=load_trial(root,name)
            if meta['stage']!=stage:continue
            if resume and (root/'runs'/name/'metadata.json').exists():continue
            if stage=='breakaway':collect_breakaway(root,name,plant,gravity_friction,declared,profile)
            else:collect_trial(root,name,plant,nominal if stage=='gravity' else (gravity_friction_static if stage=='inertia' else gravity),declared,profile)
        # records() rechecks all raw hashes, sequence continuity and validity even
        # when acquisition was resumed. Analysis never opens evaluation files.
        for _ in recordings(root,stage):pass
        output=root/('identification_'+stage)
        if output.exists():
            suffix=1
            while (root/(output.name+f'_reanalysis_{suffix}')).exists():suffix+=1
            output=root/(output.name+f'_reanalysis_{suffix}')
        if stage=='gravity':
            report,gravity=identify_gravity(root,output);gravity_report=report;gravity_constraints=output/'gravity_constraints.npz'
        elif stage=='friction':
            report=identify_friction(root,output,gravity)
            friction_report=report
            gravity_friction=model_from_parameters(nominal,gravity.rigid_parameters(),report['viscous'],report['coulomb'])
        elif stage=='breakaway':
            report=identify_breakaway(root,output,gravity);breakaway_report=report
            gravity_friction_static=model_from_parameters(nominal,gravity.rigid_parameters(),friction_report['viscous'],friction_report['coulomb'],report['static_friction'])
            write_json(root/'pre_inertia_acceptance.json',dict(passed=all(r['passed'] for r in (gravity_report,friction_report,breakaway_report)),
                gravity_parameters=gravity.rigid_parameters(),viscous=friction_report['viscous'],coulomb=friction_report['coulomb'],static_friction=report['static_friction'],
                convention='previous stages independently accepted and frozen before acceleration excitation; known declared output torque scale'))
        else:report,candidate=identify_inertia(root,output,gravity_report,friction_report,breakaway_report,gravity_constraints)
        results[stage]=dict(passed=report['passed'],report=str(output.relative_to(root)/'report.json'))
        checkpoint=dict(checkpoint_passed=all(r['passed'] for r in results.values()),stages=results,completed_through=stage,
            full_plan_complete=False,synthetic_only=True,hardware_executable=False,
            pending=(['guarded breakaway acquisition','independent static-friction acceptance'] if stage in ('gravity','friction') else [])+
                (['inertia identification','joint refinement'] if stage!='inertia' else [])+['control validation','comprehensive validation','Chinese viewer video'])
        write_json(root/'checkpoint.json',checkpoint)
        if not report['passed']:raise RuntimeError(stage+' independent acceptance failed; following stage blocked')
        if stage==through:return checkpoint
    control=run_control(root,plant,declared,profile,output,resume=resume)
    final=dict(checkpoint_passed=all(r['passed'] for r in results.values()) and control['passed'],completed_through='control',stages=results,
        passed=all(r['passed'] for r in results.values()) and control['passed'],inertia_folder=str(output.relative_to(root)),
        control_report='control_comparison.json',full_plan_complete=False,synthetic_only=True,hardware_executable=False,
        pending=['comprehensive validation','Chinese viewer video'])
    write_json(root/'workflow.json',final);write_json(root/'checkpoint.json',final)
    if not final['passed']:raise RuntimeError('Independent four-way control acceptance failed')
    return final
