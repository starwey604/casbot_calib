"""Read-only raw inputs -> new staged reports. No plant factory or truth reads."""
from pathlib import Path
from .data import write_json
from .staged_identification import session_config,identify_gravity,identify_friction
from .staged_preflight import check_mapping,check_driver
from .staged_breakaway import identify_breakaway
from .staged_inertia import identify_inertia


def identify_staged(folder,output,through='inertia'):
    root=Path(folder);output=Path(output);output.mkdir(parents=True,exist_ok=False);config=session_config(root);stages={}
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
    report=dict(passed=all(r['passed'] for r in stages.values()),completed_through=through,truth_read=False,
        stages={key:dict(passed=value['passed'],report=key+'/report.json') for key,value in stages.items()},
        scope='identification and independent prediction only; control/video/comprehensive validation are separate')
    write_json(output/'analysis.json',report);return report
