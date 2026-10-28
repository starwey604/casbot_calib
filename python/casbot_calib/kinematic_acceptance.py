"""K4 preregistered repeated acquisitions and observation-only acceptance.

Only capture_repeat reads hidden Mock parameters. Analysis and playback consume
sealed public observations; hardware metrology remains a separate input.
"""
from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
import numpy as np
from . import _core
from .data import write_json
from .kinematics import nominal_kinematics, digest
from .kinematic_parameters import KinematicParameters
from .kinematic_identification import identify, load_batch


def file_hash(path):
    return sha256(Path(path).read_bytes()).hexdigest()


def prepare(reference, output):
    reference=Path(reference).resolve();root=Path(output)
    if root.exists():raise FileExistsError(root)
    report=json.loads((reference/'report.json').read_text())
    if not report['passed'] or not report['synthetic_only']:raise ValueError('An accepted K3 Mock reference is required')
    source=json.loads((reference/'train/profile.json').read_text())
    engine,geometry=nominal_kinematics(source);parameters=KinematicParameters(engine)
    saved=json.loads((reference/'identification/parameterization.json').read_text())
    if saved!=parameters.description():raise ValueError('Reference parameter convention changed')
    selected=report['identification']['selected_parameters']
    # Thresholds are fixed before acquiring repetitions; never selected on outcomes.
    settings=dict(repetitions=3,seed_offsets=[0,20000,40000],max_parameter_std_scaled=.15,
        max_prediction_std_m=20e-6,position_constraint_limit_m=20e-6,
        parameter_recalibration_rule='Not evaluated: no separately timed/remounted recalibration; do not reuse repeat batches as an interval test')
    paths=['report.json','identification/report.json','identification/parameterization.json','identification/solution.npz',
        'train/manifest.json','train/profile.json','validation/manifest.json','validation/profile.json']
    protocol=dict(schema_version=1,stage='K4',reference=str(reference),reference_files={p:file_hash(reference/p) for p in paths},
        selected_parameters=selected,parameterization_sha256=digest(parameters.description()),geometry_sha256=geometry['geometry_sha256'],
        settings=settings,synthetic_only=True,hardware_executable=False,
        scope='Conditional same-fixture/same-plant repeatability, common independent validation; no external metrology or interval drift acceptance')
    root.mkdir(parents=True,exist_ok=False);(root/'batches').mkdir();(root/'gates').mkdir()
    write_json(root/'protocol.json',protocol);write_json(root/'protocol_digest.json',dict(sha256=file_hash(root/'protocol.json')))
    return protocol


def load_protocol(folder):
    root=Path(folder);expected=json.loads((root/'protocol_digest.json').read_text())['sha256']
    if file_hash(root/'protocol.json')!=expected:raise ValueError('K4 preregistered protocol changed')
    protocol=json.loads((root/'protocol.json').read_text());reference=Path(protocol['reference'])
    for name,expected in protocol['reference_files'].items():
        p=Path(name)
        if p.is_absolute() or '..' in p.parts or 'evaluation' in p.parts:raise ValueError('Invalid K4 reference path')
        if file_hash(reference/p)!=expected:raise ValueError('K4 reference source changed')
    return root,protocol,reference


def capture_repeat(folder, index):
    from .trical_workflow import run_mock
    from .trical_analysis import analyse
    root,protocol,reference=load_protocol(folder)
    if index not in (1,2):raise ValueError('K4 captures fresh repeats 1 and 2; repeat 0 is the sealed K3 reference')
    config=json.loads((reference/'train/profile.json').read_text())
    hidden=json.loads((reference/'train/evaluation/mock.json').read_text())
    hidden['seed']+=protocol['settings']['seed_offsets'][index]
    config['k3']['batch_id']='k4_repeat_'+str(index)+'_'+file_hash(root/'protocol.json')[:12]
    for visit in config['workflow']['visits']:visit['batch_id']=config['k3']['batch_id']
    # Correct informational K1 flags without changing any measured geometry.
    config['parameter_policy'].update(fit_enabled=True,mdh_hayati='K3_exact_chart_Pinocchio_geometry_regressor')
    config['mock']=hidden
    result=run_mock(root/'batches'/f'repeat_{index:02}',config)
    if not result['passed']:return result
    gate=analyse(root/'batches'/f'repeat_{index:02}',root/'gates'/f'repeat_{index:02}')
    result=dict(passed=gate['passed'],acquisition=result,gate=gate)
    write_json(root/f'capture_{index:02}.json',result);return result


def check_reference_gates(folder):
    from .trical_analysis import analyse
    root,_,reference=load_protocol(folder)
    return {name:analyse(reference/part,root/'gates'/name) for name,part in [('repeat_00','train'),('validation','validation')]}


def repeatability(parameters, vectors, q, frames, *, max_parameter_std_scaled=.15, max_prediction_std_m=20e-6, selected=None):
    vectors=np.asarray(vectors,dtype=float)
    if vectors.ndim!=2 or vectors.shape[1]!=parameters.size or len(vectors)<3 or not np.isfinite(vectors).all():raise ValueError('At least three finite independent calibration vectors required')
    selected=list(range(parameters.size)) if selected is None else selected
    std=np.std(vectors,axis=0,ddof=1)
    predictions=np.array([parameters.evaluate(x,q,frames,np.zeros((len(q),3)))[0] for x in vectors])
    prediction_std=np.sqrt(np.sum(np.var(predictions,axis=0,ddof=1),axis=1))
    checks=dict(parameter_repeatability=bool(np.max(std[selected])<=max_parameter_std_scaled),
        prediction_repeatability=bool(np.max(prediction_std)<=max_prediction_std_m))
    return dict(passed=all(checks.values()),checks=checks,batches=len(vectors),degrees_of_freedom=len(vectors)-1,
        parameter_std_scaled=std.tolist(),parameter_std_physical=(std*parameters.scales).tolist(),
        maximum_selected_parameter_std_scaled=float(np.max(std[selected])),
        prediction_std_m=prediction_std.tolist(),maximum_prediction_std_m=float(prediction_std.max()),
        mean_prediction_std_m=float(prediction_std.mean()),
        definition='Sample standard deviation (ddof=1), frozen gauge/subset; TCP predictions at common independent observed validation q; not full population uncertainty')


def analyse_acceptance(folder,output,external_meter=None):
    root,protocol,reference=load_protocol(folder);output=Path(output)
    if output.exists():raise FileExistsError(output)
    batches=[reference/'train',root/'batches/repeat_01',root/'batches/repeat_02'];validation=reference/'validation'
    data=[load_batch(p,'train') for p in batches];heldout=load_batch(validation,'validation')
    acquisition_native=[json.loads((p/'report.json').read_text())['native_sha256'] for p in batches+[validation]]
    if len(set(acquisition_native))!=1:raise ValueError('Native Mock kernel changed across repetition acquisitions')
    if len({d['batch_id'] for d in data} | {heldout['batch_id']})!=4:raise ValueError('Repeated or shared acquisition batch IDs')
    if len({d['source_manifest_sha256'] for d in data})!=3:raise ValueError('Repeated acquisition source')
    for i,d in enumerate(data):
        if d['ids']!=data[0]['ids']:raise ValueError('Repeat posture schedule changed')
        if d['geometry']['geometry_sha256']!=protocol['geometry_sha256']:raise ValueError('Repeated geometry changed')
        for key in ('fixture','tools','indicators'):
            if digest(d['config'][key])!=digest(heldout['config'][key]):raise ValueError('Metrology changed across repetition batches')
    for name,d in list(zip(['repeat_00','repeat_01','repeat_02'],data))+[('validation',heldout)]:
        gate=json.loads((root/'gates'/name/'report.json').read_text())
        if not gate['passed'] or gate.get('source_manifest_sha256')!=d['source_manifest_sha256']:raise ValueError('Missing or stale raw motion/stability gate: '+name)
    engine,_=nominal_kinematics(data[0]['config']);parameters=KinematicParameters(engine)
    if digest(parameters.description())!=protocol['parameterization_sha256']:raise ValueError('K4 parameter convention changed')
    output.mkdir(parents=True,exist_ok=False);fits=[];vectors=[]
    for i,batch in enumerate(batches):
        result=identify(batch,validation,output/f'fit_{i:02}',selected_parameters=protocol['selected_parameters']);fits.append(result)
        if not result['passed']:
            report=dict(passed=False,stage='K4',reason='repeat_identification_rejected',fits=fits,truth_read=False)
            write_json(output/'report.json',report);return report
        with np.load(output/f'fit_{i:02}/solution.npz',allow_pickle=False) as arrays:vectors.append(arrays['scaled_parameters'])
    settings=protocol['settings'];stats=repeatability(parameters,vectors,heldout['q'],heldout['frames'],selected=protocol['selected_parameters'],
        max_parameter_std_scaled=settings['max_parameter_std_scaled'],max_prediction_std_m=settings['max_prediction_std_m'])
    external=dict(status='not_measured',passed=None,reason='No independent external position metrology supplied; artifact position constraints alone are not absolute/pose accuracy')
    if external_meter is not None:
        from .kinematic_metrology import evaluate_meter
        external=evaluate_meter(external_meter,parameters,vectors[0],excluded_batch_ids=[d['batch_id'] for d in data]+[heldout['batch_id']])
    checks=dict(all_batches=all(r['passed'] for r in fits),repeatability=stats['passed'],
        independent_constraints=all(r['validation']['geometry_layer']['position_rmse_m']<=settings['position_constraint_limit_m'] for r in fits))
    if external_meter is not None:checks['external_meter']=external['passed']
    result=dict(schema_version=1,stage='K4',passed=all(checks.values()),checks=checks,truth_read=False,synthetic_only=True,
        hardware_executable=False,product_acceptance_passed=False,parameter_subset_frozen_before_repeats=True,
        acquisition_native_sha256=acquisition_native[0],analysis_native_sha256=file_hash(_core.__file__),
        source_protocol_sha256=file_hash(root/'protocol.json'),source_manifests=[d['source_manifest_sha256'] for d in data],
        validation_manifest=heldout['source_manifest_sha256'],batch_ids=[d['batch_id'] for d in data],validation_batch_id=heldout['batch_id'],
        fits=[dict(train_samples=r['train_samples'],validation=r['validation'],fit_condition=r['fit_observability']['condition_number']) for r in fits],
        repeatability=stats,external_metrology=external,interval_recalibration=dict(status='not_measured',reason=settings['parameter_recalibration_rule']),
        scope=protocol['scope'],pending=['Physical metrology/fixture uncertainty','Interval/remount/thermal drift and product AP/RP protocol','K5 real instrument/robot adapters'])
    np.savez_compressed(output/'repeatability.npz',scaled_parameters=np.array(vectors),common_validation_q=heldout['q'],frames=heldout['frames'])
    write_json(output/'report.json',result)
    import csv
    with (output/'parameter_repeatability.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['parameter','unit','status','mean','sample_std']);writer.writeheader()
        for i,definition in enumerate(parameters.definitions):
            writer.writerow(dict(parameter=definition['name'],unit=definition['unit'],status='estimated' if i in protocol['selected_parameters'] else 'fixed_prior_not_measured',
                mean=float(np.mean(vectors,axis=0)[i]*definition['scale']),sample_std=stats['parameter_std_physical'][i] if i in protocol['selected_parameters'] else ''))
    with (output/'validation_residuals.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['batch','visit','tool_frame','dx_m','dy_m','dz_m','norm_m']);writer.writeheader()
        for batch,x in enumerate(vectors):
            points,_=parameters.evaluate(x,heldout['q'],heldout['frames'],heldout['offsets'])
            for i,residual in enumerate(points-heldout['targets']):
                writer.writerow(dict(batch=batch,visit=heldout['ids'][i],tool_frame=heldout['frames'][i],
                    dx_m=float(residual[0]),dy_m=float(residual[1]),dz_m=float(residual[2]),norm_m=float(np.linalg.norm(residual))))
    return result


def _child(root,args,log,allow_rejection=False):
    with (root/log).open('w') as stream:
        result=subprocess.run([sys.executable,'-m','casbot_calib.cli',*args],stdout=stream,stderr=subprocess.STDOUT)
    if result.returncode and not (allow_rejection and result.returncode==1):raise RuntimeError('K4 child failed; see '+str(root/log))
    return result.returncode


def finish(folder,output=None,*,video=True,fps=12,external_meter=None):
    root,protocol,reference=load_protocol(folder);output=Path(output) if output else root/'analysis'
    sources=[reference/'train/evaluation',reference/'validation/evaluation',root/'batches/repeat_01/evaluation',root/'batches/repeat_02/evaluation',reference/'synthetic_evaluation']
    quarantine=root/'withheld_evaluation';quarantine.mkdir(exist_ok=False);moved=[]
    try:
        for i,source in enumerate(sources):
            if source.exists():
                target=quarantine/str(i);source.rename(target);moved.append((source,target))
        args=['kinematic-verify',str(root),'--output',str(output)]
        if external_meter is not None:args+=['--external-meter',str(external_meter)]
        _child(root,args,'verification.log',allow_rejection=True)
    finally:
        for source,target in reversed(moved):target.rename(source)
        quarantine.rmdir()
    result=json.loads((output/'report.json').read_text())
    result['fresh_process_truth_isolation']=True
    load_protocol(root)
    if result['passed']:
        from .kinematic_presentation import render_kinematic,plot_acceptance
        plot_acceptance(output,root/'repeatability_zh.png')
        result['presentation']=render_kinematic(root,output,root/'presentation',fps=fps,video=video)
        result['raw_sources_unchanged']=result['presentation']['raw_sources_unchanged']
    write_json(root/'delivery.json',result);return result


def run_acceptance(reference,output,*,video=True,fps=12,external_meter=None):
    prepare(reference,output);root=Path(output)
    jobs=[(['kinematic-repeat',str(root),'--index',str(i)],f'repeat_{i:02}.log') for i in (1,2)]
    jobs.append((['kinematic-reference-gates',str(root)],'reference_gates.log'))
    with ThreadPoolExecutor(max_workers=3) as pool:
        futures=[pool.submit(_child,root,args,log) for args,log in jobs]
        for future in as_completed(futures):future.result()
    return finish(root,video=video,fps=fps,external_meter=external_meter)
