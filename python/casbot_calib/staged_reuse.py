"""Audited reuse of unchanged experiments when extending the staged plan."""
import hashlib,json,shutil
from pathlib import Path
import numpy as np
from .data import write_json
from .staged_collection import load_trial
from .staged_identification import recordings


def clone_plan(source,target):
    """Copy only accepted immutable planning inputs, never measurements/truth."""
    source=Path(source).resolve();target=Path(target).resolve()
    if (source/'plan_rejected.json').exists() or not json.loads((source/'plan_report.json').read_text())['passed']:
        raise ValueError('Cannot clone a rejected plan')
    target.mkdir(parents=True,exist_ok=False)
    shutil.copytree(source/'plans',target/'plans')
    for name in ('session.json','plan_report.json','excitation_design.json','README.md'):shutil.copyfile(source/name,target/name)
    (target/'evaluation').mkdir();shutil.copyfile(source/'evaluation'/'plant_config.json',target/'evaluation'/'plant_config.json')
    write_json(target/'plan_lineage.json',dict(source=str(source),copied='planning inputs only; fresh acquisition required',
        session_sha256=hashlib.sha256((source/'session.json').read_bytes()).hexdigest(),
        plan_report_sha256=hashlib.sha256((source/'plan_report.json').read_bytes()).hexdigest()))
    return target


def reuse_trials(source,target,stages=('direction','timing','gravity','friction')):
    source=Path(source).resolve();target=Path(target).resolve()
    if source==target:raise ValueError('Separate source and target sessions required')
    sessions=[json.loads((r/'session.json').read_text()) for r in (source,target)]
    # Generation knobs for an uncollected later stage may change. Acquired
    # trials still require exactly equal conditions/polynomial coefficients.
    acquisition=('schema_version','mode','model','control_period_ns','sample_period_ns','integration_ns','drive','acceptance','acquisition_seed')
    base=lambda config:{k:config.get(k) for k in acquisition}
    if base(sessions[0]['config'])!=base(sessions[1]['config']) or sessions[0]['collision']!=sessions[1]['collision']:
        raise ValueError('Acquisition/model/geometry configuration changed; unchanged-trial reuse rejected')
    # Reuse is acquisition bookkeeping, not an inference entry point. The plant
    # declaration must also be identical when evaluating a combined Mock run.
    if (source/'evaluation'/'plant_config.json').read_bytes()!=(target/'evaluation'/'plant_config.json').read_bytes():raise ValueError('Synthetic plant changed')
    target_plan=json.loads((target/'plan_report.json').read_text())
    if not target_plan['passed']:raise ValueError('Target plan is rejected')
    manifest=[]
    for stage in stages:
        for name,meta,_ in recordings(source,stage):
            if name not in target_plan['selected_trials']:continue
            other,_=load_trial(target,name)
            if meta!=other:raise ValueError('Trial conditions changed: '+name)
            with np.load(source/'plans'/name/'reference.npz',allow_pickle=False) as a,np.load(target/'plans'/name/'reference.npz',allow_pickle=False) as b:
                if a.files!=b.files or any(not np.array_equal(a[k],b[k]) for k in a.files):raise ValueError('Reference coefficients changed: '+name)
            src=source/'runs'/name;dest=target/'runs'/name
            if dest.exists():raise FileExistsError('Cannot overwrite a target recording: '+name)
            manifest.append(dict(trial=name,stage=stage,source_metadata_sha256=hashlib.sha256((src/'metadata.json').read_bytes()).hexdigest(),
                raw_sha256=hashlib.sha256((src/'observations.npz').read_bytes()).hexdigest(),commands_sha256=hashlib.sha256((src/'commands.npz').read_bytes()).hexdigest()))
    # Validate the entire source before copying any recordings. A partially
    # completed source therefore cannot leave an apparently reusable target.
    for row in manifest:
        name=row['trial'];src=source/'runs'/name;dest=target/'runs'/name
        dest.parent.mkdir(parents=True,exist_ok=True);shutil.copytree(src,dest)
        truth=source/'evaluation'/name
        if truth.exists():shutil.copytree(truth,target/'evaluation'/name)
    provenance=source/'source_native_provenance_correction.json'
    if not provenance.exists():provenance=source/'native_provenance_correction.json'
    if provenance.exists():shutil.copyfile(provenance,target/'source_native_provenance_correction.json')
    entry=dict(source=str(source),target=str(target),trials=manifest,
        source_session_sha256=hashlib.sha256((source/'session.json').read_bytes()).hexdigest(),
        target_session_sha256=hashlib.sha256((target/'session.json').read_bytes()).hexdigest(),
        contract='same acquisition/model/geometry and exactly equal per-trial conditions/reference; raw data copied without modification; every stage reanalysed')
    path=target/'reused_trials.json'
    if path.exists():
        previous=json.loads(path.read_text());imports=previous.get('imports',[previous]);imports.append(entry)
        write_json(path,dict(imports=imports,trials=[row for item in imports for row in item['trials']]))
    else:write_json(path,entry)
    return manifest
