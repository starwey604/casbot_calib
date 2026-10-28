"""R1 development capture/analysis; formal tests require an R2/R3 freeze."""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import shutil
from time import perf_counter
import numpy as np
from .data import write_json
from .staged_protocol import (_read, _seal, _unseal, file_hash, load_protocol,
                              load_frozen_candidate, seal_role_records)
from .staged_multiconfiguration import load_development_reference


@contextmanager
def collection_lock(root):
    lock = Path(root) / '.collection_lock'
    try:
        lock.mkdir()
    except FileExistsError:
        raise RuntimeError('Another collector or interrupted lock exists; inspect .collection_lock before resuming') from None
    try:
        write_json(lock / 'owner.json', dict(pid=os.getpid()))
        yield
    finally:
        shutil.rmtree(lock)


def prepare_predecessors(root, output):
    from .staged_analysis import identify_staged
    source = load_development_reference(root)
    report = identify_staged(source, output, through='breakaway')
    if not report['passed']:
        raise RuntimeError('Development predecessor reanalysis failed')
    return report


def frozen_predecessors(root):
    seal = _unseal(root, 'predecessor_freeze.json')
    if seal['protocol_sha256'] != file_hash(Path(root) / 'protocol.json'):
        raise ValueError('Predecessors belong to another P0 protocol')
    for path, expected in seal['files'].items():
        if file_hash(Path(root) / path) != expected:
            raise ValueError('Frozen predecessor results changed')
    return Path(root) / seal['analysis_folder']


def predecessor_model(folder):
    from .dual_model import nominal_dual_arm
    from .rigid_identification import model_from_parameters
    folder = Path(folder); nominal, _ = nominal_dual_arm()
    g,f,s = (_read(folder / stage / 'report.json') for stage in ('gravity', 'friction', 'breakaway'))
    return model_from_parameters(nominal, g['fit']['parameters'], f['viscous'], f['coulomb'], s['static_friction'])


def _collect_order(root, order, plant, feedforward, config, profile, resume):
    from .staged_collection import collect_trial, preserve_incomplete_trial, load_trial
    from .staged_identification import read_record
    from .staged_breakaway import collect_breakaway
    root = Path(root); names = _read(root / 'session.json')['joint_names']
    journal = root / 'execution_order.jsonl'
    for name in order:
        target = root / 'runs' / name
        if resume and (target / 'metadata.json').exists():
            meta,_ = load_trial(root,name);read_record(root,name,meta,names)
            events=[json.loads(line) for line in journal.read_text().splitlines()] if journal.exists() else []
            completed=[r for r in events if r['event']=='completed' and r['trial_id']==name]
            if completed:
                if len(completed)!=1 or completed[0]['metadata_sha256']!=file_hash(target/'metadata.json'):
                    raise ValueError('Acquisition journal or completed metadata changed')
            elif any(r['event']=='start' and r['trial_id']==name for r in events):
                with journal.open('a',encoding='utf-8') as stream:
                    stream.write(json.dumps(dict(event='completed',trial_id=name,metadata_sha256=file_hash(target/'metadata.json'),reconciled_after_interruption=True))+'\n')
            else:raise ValueError('Existing recording has no acquisition start journal')
            continue
        if resume:
            preserve_incomplete_trial(root,name)
        with journal.open('a',encoding='utf-8') as stream:
            stream.write(json.dumps(dict(event='start',trial_id=name))+'\n')
        meta,_ = load_trial(root,name)
        if meta['stage'] == 'breakaway':
            collect_breakaway(root,name,plant,feedforward,config,profile)
        else:
            collect_trial(root,name,plant,feedforward,config,profile)
        with journal.open('a',encoding='utf-8') as stream:
            stream.write(json.dumps(dict(event='completed',trial_id=name,metadata_sha256=file_hash(target/'metadata.json')))+'\n')


def run_development(folder, config, *, resume=False, development_reference=None):
    from .staged_plan import plan_session
    from .staged_collection import synthetic_plant
    from .collision import nominal_profile
    from .staged_multifit import identify_development
    root = Path(folder); started = perf_counter()
    if not root.exists():
        if development_reference is None:
            raise ValueError('P0 contract-only without a certified R1 development reference; supply --development-reference')
        plan_session(root,config,development_reference)
    elif not resume:
        raise FileExistsError('New session required, or explicitly resume the sealed R1 plan')
    protocol, public, manifest = load_protocol(root)
    if protocol['schema_version'] != 2 or 'development_reference.json' not in protocol['input_sha256']:
        raise ValueError('P0 contract-only: certified R1 references and development reference required')
    if public != {k:v for k,v in config.items() if k != 'mock'}:
        raise ValueError('Configuration changed since R1 planning')
    hidden = _read(root / 'evaluation/plant_config.json')
    if hidden != config.get('mock'):
        raise ValueError('Mock plant declaration changed since planning')
    with collection_lock(root):
        source = load_development_reference(root)
        if _read(source / 'evaluation/plant_config.json') != hidden:
            raise ValueError('Reused development data does not describe the same synthetic plant')
        if (root / 'predecessor_freeze.json').exists():
            prior = frozen_predecessors(root)
        else:
            prior = root / 'development_analysis'
            prepare_predecessors(root,prior)
            files = {str(p.relative_to(root)):file_hash(p) for p in prior.rglob('*') if p.is_file()}
            _seal(root,'predecessor_freeze.json',dict(schema_version=1,protocol_sha256=file_hash(root/'protocol.json'),
                analysis_folder=str(prior.relative_to(root)),files=files,scope='development only; final independent predecessor holdout remains uncollected'))
        model = predecessor_model(prior); plant = synthetic_plant(config)
        profile = nominal_profile(scene=_read(root/'session.json')['collision']['scene'])
        for split in ('train','selection'):
            _collect_order(root,manifest['planned_execution_order'][split],plant,model,public,profile,resume)
            index = root / f'role_inputs/{split}.json'
            if not index.exists():
                seal_role_records(root,split)
        # Never loops over test or final-prior references during development.
        events = [json.loads(line) for line in (root/'execution_order.jsonl').read_text().splitlines()]
        actual = [r['trial_id'] for r in events if r['event']=='completed']
        expected = manifest['planned_execution_order']['train']+manifest['planned_execution_order']['selection']
        if actual != expected:
            raise ValueError('Actual execution order differs from the preregistered block order; inspect interrupted journal')
        output = root/'identification_development'
        if output.exists():
            if not resume:raise FileExistsError(output)
            from .staged_protocol import role_recordings
            for split,purpose in (('train','fit'),('selection','select')):
                for _ in role_recordings(root,'inertia',split=split,purpose=purpose):pass
            saved=_unseal(root,'development_fit_freeze.json')
            for path,expected_hash in saved['files'].items():
                if file_hash(root/path)!=expected_hash:raise ValueError('Previously accepted development fit changed')
            report = _read(output/'report.json')
            if report['protocol_sha256'] != file_hash(root/'protocol.json'):
                raise ValueError('Existing development fit belongs to another protocol')
        else:
            report = identify_development(root,output,prior)
            _seal(root,'development_fit_freeze.json',dict(schema_version=1,files={str(p.relative_to(root)):file_hash(p) for p in output.rglob('*') if p.is_file()}))
        result = dict(round='R1',passed=report['development_passed'],checkpoint_passed=report['development_passed'],
            completed_through='R1_development',full_plan_complete=False,p0_acceptance_passed=False,
            formal_test_consumed=False,synthetic_only=True,hardware_executable=False,
            collected_trials=len(expected),actual_execution_order=actual,
            plan_report='plan_report.json',inertia_folder='identification_development',
            new_virtual_duration_s=sum(_read(root/'plans'/name/'motion.json')['duration_s'] for name in expected),
            wall_s=perf_counter()-started,pending=['R2 stability','R3 attribution and candidate freeze','R4 independent final test, control, full verification and video'])
        write_json(root/'checkpoint.json',result);write_json(root/'workflow.json',result)
        if not result['passed']:
            raise RuntimeError('R1 development prediction gate failed; no candidate release or formal test permitted')
        return result


def analyse_development(root, output):
    from .staged_multifit import identify_development
    root=Path(root);output=Path(output);load_protocol(root)
    if output.exists():raise FileExistsError(output)
    output.mkdir(parents=True,exist_ok=False)
    prepare_predecessors(root,output/'predecessors')
    report=identify_development(root,output/'inertia',output/'predecessors')
    result=dict(round='R1',passed=report['development_passed'],truth_read=False,formal_test_consumed=False,
        p0_acceptance_passed=False,scope='fresh read-only development reanalysis; no R2/R3 candidate selection or final-test acceptance')
    write_json(output/'analysis.json',result)
    return result


def collect_final_test(root, config, *, resume=False):
    """R4 collection hook. Authorization is checked before even constructing Mock."""
    from .staged_collection import synthetic_plant
    from .staged_control import load_candidate
    from .collision import nominal_profile
    root=Path(root);frozen=load_frozen_candidate(root);_,public,manifest=load_protocol(root)
    if public != {k:v for k,v in config.items() if k != 'mock'}:
        raise ValueError('Final test configuration changed after candidate freeze')
    if _read(root/'evaluation/plant_config.json')!=config.get('mock'):
        raise ValueError('Final test Mock plant differs from the declared development plant')
    motion_plan=_read(root/'motion_plan.json')
    with collection_lock(root):
        plant=synthetic_plant(config);candidate=load_candidate(root/frozen['candidate'])
        profile=nominal_profile(scene=_read(root/'session.json')['collision']['scene'])
        _collect_order(root,manifest['planned_execution_order']['test'],plant,candidate,public,profile,resume)
        if not (root/'role_inputs/test.json').exists():seal_role_records(root,'test')
        _collect_order(root,motion_plan['final_prior_execution_order'],plant,candidate,public,profile,resume)
        seal_final_predecessors(root)
        # Final scoring/publication is deliberately not implemented by R1.
        return dict(collected=True,p0_acceptance_passed=False,pending='R4 final independent scoring and comprehensive acceptance')


def seal_final_predecessors(root):
    from .staged_collection import load_trial
    from .staged_identification import read_record
    root=Path(root);frozen=load_frozen_candidate(root);protocol,config,_=load_protocol(root)
    rows=[];names=_read(root/'session.json')['joint_names']
    for slot in _read(root/'motion_plan.json')['final_prior_trials']:
        name=slot['trial_id'];meta,_=load_trial(root,name);record,_=read_record(root,name,meta,names)
        if record.get('candidate_freeze_sha256')!=file_hash(root/'candidate_freeze.json') or record.get('protocol_sha256')!=file_hash(root/'protocol.json'):
            raise ValueError('Final predecessor recording belongs to another frozen candidate/protocol')
        expected_period=meta['conditions'].get('sample_period_ns',config['sample_period_ns'])
        if record['sample_period_ns']!=expected_period or record['native_binary_sha256']!=protocol['native_binary_sha256']:
            raise ValueError('Final predecessor native/sample conditions changed')
        files=[p for directory in ('plans','runs') for p in (root/directory/name).iterdir() if p.is_file()]
        if meta['stage']=='breakaway' and not (root/'runs'/name/'guard_events.json').exists():
            raise ValueError('Missing final breakaway feedback-guard evidence')
        rows.append(dict(trial_id=name,files={str(p.relative_to(root)):file_hash(p) for p in files}))
    if not (root/'final_prior_inputs.json').exists():
        _seal(root,'final_prior_inputs.json',dict(schema_version=1,protocol_sha256=file_hash(root/'protocol.json'),
            candidate_freeze_sha256=file_hash(root/'candidate_freeze.json'),records=rows))
    else:
        previous=_unseal(root,'final_prior_inputs.json')
        if previous['records']!=rows:raise ValueError('Final predecessor inputs changed after sealing')
    return dict(collected_trials=len(rows),p0_acceptance_passed=False)
