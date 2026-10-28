"""Read-only raw inputs -> new staged reports. No plant factory or truth reads."""
from pathlib import Path
from .data import write_json
from .staged_identification import session_config,identify_gravity,identify_friction
from .staged_preflight import check_mapping,check_driver
from .staged_breakaway import identify_breakaway
from .staged_inertia import identify_inertia


def identify_staged(folder,output,through='inertia',resume=False,stability_verification=None,final_protocol=None):
    from .staged_protocol import require_legacy_execution,p0_status
    root=Path(folder);config=session_config(root)
    if stability_verification is not None and through!='attribution':raise ValueError('R2 verification input is for R3 attribution only')
    if final_protocol is not None and through!='final':raise ValueError('Final protocol input is for R4 final scoring only')
    if config.get('schema_version')==2:
        from .staged_protocol import load_protocol
        protocol,_,_=load_protocol(root)
        if protocol['schema_version']==2 and through=='final':
            if resume or final_protocol is None:raise ValueError('Final read-only scoring requires registered R4 protocol and a new output')
            from .staged_final import score_final
            return score_final(root,final_protocol,output)
        if protocol['schema_version']==2 and through=='attribution':
            if resume or stability_verification is None:raise ValueError('R3 requires verified R2 and a new output directory')
            from .staged_attribution import generate_attribution
            return generate_attribution(root,output,stability_verification)
        if protocol['schema_version']==2 and through=='stability':
            from .staged_stability import run_stability
            return run_stability(root,output,resume=resume)
        if protocol['schema_version']==2 and through=='inertia':
            from .staged_development import analyse_development
            return analyse_development(root,output)
    if through in ('stability','attribution','final') or resume:raise ValueError('Stability/attribution/final/resume analysis requires P0')
    require_legacy_execution(config)
    output=Path(output);output.mkdir(parents=True,exist_ok=False);stages={}
    direction=check_mapping(root,config,output/'preflight');driver=check_driver(root,config,output/'preflight')
    if not direction['passed'] or not driver['passed']:raise ValueError('Preflight reanalysis failed')
    gravity,model=identify_gravity(root,output/'gravity');stages['gravity']=gravity
    if not gravity['passed']:raise ValueError('Gravity stage rejected')
    if through!='gravity':
        friction=identify_friction(root,output/'friction',model,driver);stages['friction']=friction
        if not friction['passed']:raise ValueError('Moving-friction stage rejected')
    if through in ('breakaway','inertia'):
        onset=identify_breakaway(root,output/'breakaway',model);stages['breakaway']=onset
        if not onset['passed']:raise ValueError('Static-onset stage rejected')
    if through=='inertia':
        inertia,_=identify_inertia(root,output/'inertia',gravity,friction,onset,output/'gravity'/'gravity_constraints.npz');stages['inertia']=inertia
        if not inertia['passed']:raise ValueError('Inertia stage rejected')
    report=dict(passed=all(r['passed'] for r in stages.values()),completed_through=through,truth_read=False,p0=p0_status(config),
        stages={key:dict(passed=value['passed'],report=key+'/report.json') for key,value in stages.items()},
        scope='identification and independent prediction only; control/video/comprehensive validation are separate')
    write_json(output/'analysis.json',report);return report
