"""Read-only R2 verification and cross-ridge comparison; no new fits or truth."""
from pathlib import Path
import numpy as np
from .data import write_json
from .staged_protocol import load_protocol,_read,_unseal,_seal,file_hash
from .staged_stability_math import operator_difference,parameter_difference


def regularization_path(analysis,settings,reference_ridge=1e-4):
    analysis=Path(analysis);raw=_read(analysis/'report.json') if (analysis/'report.json').exists() else None
    results=raw['variants'] if raw is not None else [_unseal(p.parent,'result.json') for p in sorted((analysis/'variants').glob('*/result.json'))]
    full=[r for r in results if r['kind']=='full']
    reference=next((r for r in full if r['ridge']==reference_ridge and r['completed']),None)
    if reference is None:return dict(completed=False,passed=False,reference_ridge=reference_ridge,comparisons=[],error='Missing fixed 1e-4 full-train reference')
    def arrays(path):
        with np.load(path,allow_pickle=False) as data:return {k:data[k] for k in data.files}
    anchor=arrays(analysis/'variants'/reference['label']/'solution.npz')
    coordinates=arrays(analysis/'coordinates.npz');basis=arrays(analysis/'comparison_basis.npz')['basis'];rows=[]
    for row in full:
        if not row['completed']:
            rows.append(dict(ridge=row['ridge'],passed=False,error='Full-train solve failed'));continue
        variant=arrays(analysis/'variants'/row['label']/'solution.npz')
        difference=operator_difference(anchor['mass'],variant['mass'],anchor['torque'],variant['torque'],settings)
        rows.append(dict(ridge=row['ridge'],**difference,
            parameters=parameter_difference(anchor['vector'],variant['vector'],coordinates['scales'],basis)))
    return dict(completed=True,passed=bool(rows and all(r['passed'] for r in rows)),reference_ridge=reference_ridge,comparisons=rows,
        convention='same full-train nominal center; fixed 1e-4 reference declared from R1, not self-comparison or selection-score choice')


def verify_stability(root,analysis,output):
    from .staged_identification import recordings
    from .rigid_identification import model_from_parameters
    from .dual_model import nominal_dual_arm
    root=Path(root).resolve();analysis=Path(analysis).resolve();output=Path(output)
    parent,config,manifest=load_protocol(root);protocol=_unseal(analysis,'analysis_protocol.json')
    if protocol['parent_protocol_sha256']!=file_hash(root/'protocol.json') or protocol['train_manifest_sha256']!=file_hash(root/'role_inputs/train.json'):
        raise ValueError('R2 verification parent data changed')
    for path,expected in protocol['inputs'].items():
        if file_hash(analysis/path)!=expected:raise ValueError('R2 fixed inputs changed')
    delivery=_unseal(analysis,'delivery_manifest.json')
    for path,expected in delivery['files'].items():
        if file_hash(analysis/path)!=expected:raise ValueError('R2 analysis output changed: '+path)
    # Validate the complete train role without opening selection/test observations.
    for stage in ('inertia','inertia_probe'):
        for _ in recordings(root,stage,split='train',purpose='stability'):pass
    raw=_read(analysis/'report.json');results=raw['variants']
    expected={'full':1,'center':3,'batch':2,'prior':4}
    for ridge in protocol['ridge_candidates']:
        rows=[r for r in results if r['ridge']==ridge]
        if len(rows)!=10 or {kind:sum(r['kind']==kind for r in rows) for kind in expected}!=expected:
            raise ValueError('Missing/repeated R2 sensitivity variant')
    if len(results)!=30 or len({r['label'] for r in results})!=30:raise ValueError('Incomplete R2 analysis')
    with np.load(analysis/'coordinates.npz',allow_pickle=False) as data:coordinates={k:data[k] for k in data.files}
    with np.load(analysis/'operator_probe_set.npz',allow_pickle=False) as data:probes={k:data[k] for k in data.files}
    nominal,source=nominal_dual_arm()
    if source['sha256']!=parent['model']['sha256']:raise ValueError('Nominal geometry changed')
    identity=[]
    for row in results:
        if not row['completed']:continue
        directory=analysis/'variants'/row['label'];bundle=_read(directory/'candidate.json');fit=_read(directory/'fit.json')
        if not np.array_equal(fit['parameter_scales'],coordinates['scales']):raise ValueError('Variant changed comparison scales')
        if fit['regularization_metric_source']!='explicit_complete_quadratic':raise ValueError('Variant changed the regularization metric')
        with np.load(directory/'solution.npz',allow_pickle=False) as data:solution={k:data[k] for k in data.files}
        if not np.array_equal(solution['vector'],np.r_[bundle['parameters'],bundle['viscous'],bundle['coulomb']]):raise ValueError('Candidate and operator solution differ')
        model=model_from_parameters(nominal,bundle['parameters'],bundle['viscous'],bundle['coulomb'],bundle['static_friction'])
        parts=model.evaluate_batch(probes['q'],probes['dq'],probes['ddq'])
        difference=max(float(np.max(abs(parts['mass_matrix']-solution['mass']))),float(np.max(abs(parts['total']-solution['torque']))))
        if difference>1e-10:raise ValueError('Saved operator arrays do not match Pinocchio candidate')
        identity.append(dict(variant=row['label'],maximum_native_operator_difference=difference))
    path=regularization_path(analysis,protocol['settings'])
    path_pass={r['ridge']:r['passed'] for r in path['comparisons']}
    eligible=[ridge for ridge in protocol['ridge_candidates'] if path_pass.get(ridge,False) and all(r['passed'] for r in results if r['ridge']==ridge)]
    formal=[r['trial_id'] for r in manifest['trials'] if r['split']=='test']+ [r['trial_id'] for r in _read(root/'motion_plan.json')['final_prior_trials']]
    untouched=not any((root/'runs'/name).exists() for name in formal)
    if not untouched:raise ValueError('R2 has consumed formal test observations')
    if output.exists():raise FileExistsError(output)
    output.mkdir(parents=True,exist_ok=False)
    write_json(output/'regularization_path.json',path)
    report=dict(schema_version=1,round='R2',passed=bool(eligible),completed=True,stability_qualified=bool(eligible),
        eligible_ridges=eligible,p0_acceptance_passed=False,publishable=False,truth_read=False,test_consumed=False,
        original_analysis=str(analysis),analysis_protocol_sha256=file_hash(analysis/'analysis_protocol.json'),
        parent_protocol_sha256=file_hash(root/'protocol.json'),delivery_manifest_sha256=file_hash(analysis/'delivery_manifest.json'),
        regularization_path=path,native_operator_identity=identity,formal_test_untouched=True,
        maximum_native_operator_difference=max((r['maximum_native_operator_difference'] for r in identity),default=None),
        sensitivity_variants=len(results),rigid_fit_calls=raw['fit_calls'],selection_status='not_selected; attribution and selection prediction pending R3',
        source_sha256=file_hash(__file__),scope='independent artifact/native check plus fixed-reference regularization path; no new fit or measurement')
    for row in (r for r in results if r['kind']=='full' and r['completed']):
        write_json(output/f'eligibility_ridge_{row["ridge"]:g}.json',dict(schema_version=1,kind='stability',
            protocol_sha256=file_hash(root/'protocol.json'),input_manifests={'train':file_hash(root/'role_inputs/train.json')},
            candidate_sha256=file_hash(analysis/'variants'/row['label']/'candidate.json'),completed=True,passed=row['ridge'] in eligible,
            verification_source_sha256=file_hash(__file__),source_analysis_protocol_sha256=file_hash(analysis/'analysis_protocol.json')))
    write_json(output/'verification.json',report)
    _seal(output,'manifest.json',dict(files={p.name:file_hash(p) for p in output.iterdir() if p.is_file()}))
    return report
