"""Read-only R3 numerical/input audit and optional one-way R4 candidate freeze."""
from pathlib import Path
import numpy as np
from . import _core
from .data import write_json
from .dual_model import nominal_dual_arm
from .staged_protocol import load_protocol, file_hash, _read, _seal, _unseal, freeze_candidate
from .staged_attribution import arrays, check_manifest, formal_test_untouched, raw_diagnostics, candidate_gate
from .staged_multifit import load_problems, score_problems
from .staged_stability_prior import check_prior_cache
from .staged_residuals import MODELS, energy_spectrum, frequency_budget, aggregate_benefit, residual_statistics, raw_residual_budget
from .rigid_identification import model_from_parameters
from .staged_inertia import _project


def equivalent(left,right):
    if isinstance(left,dict):
        if set(left)!=set(right):raise ValueError('R3 report fields differ')
        for key in left:equivalent(left[key],right[key])
    elif isinstance(left,list):
        if len(left)!=len(right):raise ValueError('R3 report length differs')
        for a,b in zip(left,right):equivalent(a,b)
    elif isinstance(left,(float,int)) and not isinstance(left,bool):
        if not np.isclose(left,right,rtol=1e-9,atol=1e-11):raise ValueError('R3 numerical evidence differs')
    elif left!=right:raise ValueError('R3 report evidence differs')


def verify_attribution(root,analysis,output,*,freeze=False):
    root=Path(root).resolve();analysis=Path(analysis).resolve();output=Path(output).resolve()
    parent,config,_=load_protocol(root);formal=formal_test_untouched(root)
    protocol=_unseal(analysis,'analysis_protocol.json');check_manifest(analysis,'delivery_manifest.json')
    if protocol['parent_protocol_sha256']!=file_hash(root/'protocol.json') or protocol['parent']!=str(root):raise ValueError('R3 parent changed')
    for split in ('train','selection'):
        if protocol['input_manifests'][split]!=file_hash(root/f'role_inputs/{split}.json'):raise ValueError('R3 raw role changed')
    for name,expected in protocol['source_sha256'].items():
        if file_hash(analysis/'source'/name)!=expected:raise ValueError('R3 source archive changed')
    if file_hash(_core.__file__)!=protocol['native_binary_sha256']:raise ValueError('Native model changed')
    verified=Path(protocol['stability_verification']);r2=Path(protocol['stability_analysis'])
    if file_hash(verified/'manifest.json')!=protocol['stability_manifest_sha256'] or file_hash(r2/'delivery_manifest.json')!=protocol['stability_delivery_sha256']:raise ValueError('R2 provenance changed')
    check_manifest(verified,'manifest.json');check_manifest(r2,'delivery_manifest.json')
    report=_read(analysis/'residual_attribution.json');budgets=_read(analysis/'frequency_budget.json')
    if report['protocol_sha256']!=file_hash(analysis/'analysis_protocol.json'):raise ValueError('R3 protocol/report mismatch')
    settings={**config['p0']['frequency_qualification'],**config['p0']['acceptance'],'minimum_qualified_bins':1}
    equivalent(settings,protocol['settings']);equivalent(config['p0']['selection'],protocol['selection_policy'])
    nominal,source=nominal_dual_arm()
    if source['sha256']!=parent['model']['sha256']:raise ValueError('Nominal changed')
    c=arrays(r2/'coordinates.npz');cache=arrays(r2/'predecessor_cache.npz');static=c['static'];fa=c['friction'];h=config['p0']['preprocessing']['harmonics']
    threshold=np.asarray(_read(root/'qualification/report.json')['threshold_bin_rms_nm'])
    problems={s:load_problems(root,s,static) for s in ('train','selection')}
    raw={s:raw_diagnostics(root,s,config) for s in ('train','selection')}
    if [r['ridge'] for r in report['candidates']]!=config['p0']['selection']['ridge_candidates']:raise ValueError('Missing R3 candidate')
    audits=[];eligible=[]
    for row in report['candidates']:
        ridge=row['ridge'];label=f'ridge_{ridge:g}';directory=analysis/label
        spec=next(r for r in protocol['candidates'] if r['ridge']==ridge)
        if file_hash(directory/'candidate.json')!=spec['D_sha256'] or file_hash(directory/'candidate_initial.json')!=spec['B_sha256']:raise ValueError('R3 is not the unchanged R2 full-train model')
        b=_read(directory/'candidate_initial.json');d=_read(directory/'candidate.json');fd=np.r_[d['viscous'],d['coulomb']]
        theta=dict(A=c['gravity_theta'],B=np.asarray(b['parameters']),C=np.asarray(d['parameters']),D=np.asarray(d['parameters']))
        friction=dict(A=fa,B=fa,C=fa,D=fd);models={k:model_from_parameters(nominal,theta[k],friction[k][:14],friction[k][14:],static) for k in MODELS}
        prior=check_prior_cache(cache,theta['D'],fd,static,config['acceptance']);equivalent(prior,row['prior_development'])
        bias={k:[] for k in MODELS};biases=None;split_summary={}
        for split in ('train','selection'):
            metrics,probes=score_problems(problems[split],models['D'],theta['D'],fd,static,config)
            prediction=candidate_gate(metrics,probes,config);equivalent(prediction,row['splits'][split]['prediction'])
            trial_budgets={}
            expected_names={n for n,r in problems[split].items() if r['meta']['stage']=='inertia'}
            if set(row['splits'][split]['trajectories'])!=expected_names or set(budgets[label][split])!=expected_names:raise ValueError('Missing R3 trajectory')
            for name in expected_names:
                saved=arrays(directory/(name+'.npz'));p=problems[split][name]['problem'];conditions=problems[split][name]['meta']['conditions']
                for k in ('q','dq','ddq','rotor','extra','fixed_friction'):
                    if not np.allclose(saved[k],p[k],rtol=0,atol=1e-12):raise ValueError('R3 saved arrays differ from raw input')
                if not np.allclose(saved['observed'],p['current_torque'],rtol=0,atol=1e-12):raise ValueError('R3 observed current changed')
                if not np.array_equal(saved['raw_current_coefficients'],raw[split][name]['current_coefficients']):raise ValueError('R3 raw spectra changed')
                coeff=np.zeros((len(p['q'])//2+1,14),complex);coeff[:h+1]=raw[split][name]['measured_velocity_coefficients'][:h+1]
                velocity=np.fft.irfft(coeff*len(p['q']),n=len(p['q']),axis=0)
                if not np.allclose(velocity,saved['measured_velocity'],rtol=0,atol=1e-12) or not np.array_equal(p['ddq'],saved['acceleration']):raise ValueError('R3 qualification motion differs')
                predictions={};max_closure=0.;parseval=0.
                for k in MODELS:
                    native=models[k].evaluate_batch(p['q'],p['dq'],p['ddq'])
                    components=dict(rigid_inertia=_project(native['inertia'],h)-p['rotor'],rotor=p['rotor'],
                        velocity=_project(native['velocity'],h),gravity=_project(native['gravity'],h),friction=p['extra']@friction[k]+p['fixed_friction'])
                    predicted=sum(components.values());predictions[k]=predicted
                    for component,value in components.items():
                        max_closure=max(max_closure,float(np.max(abs(value-saved[k+'_'+component]))))
                    max_closure=max(max_closure,float(np.max(abs(predicted-saved['prediction_'+k]))))
                    power=energy_spectrum(predicted-saved['observed'])
                    parseval=max(parseval,float(np.max(abs(power.sum(axis=0)-np.mean((predicted-saved['observed'])**2,axis=0)))))
                    if np.max(energy_spectrum(predicted)[h+1:])>1e-20:raise ValueError('Predictions exceed declared bandwidth')
                if max_closure>1e-10 or parseval>1e-10:raise ValueError('R3 native closure / independent Parseval check failed')
                dynamic=p['current_torque']-saved['A_gravity']-saved['A_friction'];proxy=saved['A_rigid_inertia']+saved['A_rotor']
                if not np.allclose(dynamic,saved['dynamic_torque'],rtol=0,atol=1e-12) or not np.allclose(proxy,saved['inertial_proxy'],rtol=0,atol=1e-12):raise ValueError('Candidate-independent A qualification changed')
                budget=frequency_budget(p['current_torque'],predictions,velocity,p['ddq'],dynamic,proxy,
                    conditions['excitation_harmonics'],h,threshold,settings,period_s=conditions['period_s'])
                equivalent(budget,budgets[label][split][name]);trial_budgets[name]=budget
                stats=residual_statistics(p['current_torque'],predictions,biases)
                recorded=row['splits'][split]['trajectories'][name]
                equivalent(stats,recorded['statistics']);equivalent(raw_residual_budget(raw[split][name],predictions,h),recorded['raw_diagnostic'])
                if split=='train':
                    for k in MODELS:bias[k].append(stats[k]['bias_nm'])
                audits.append(dict(ridge=ridge,trial=name,native_component_closure_nm=max_closure,parseval_energy_difference_nm2=parseval))
            if split=='train':
                biases={k:np.mean(v,axis=0) for k,v in bias.items()};sealed_bias=_unseal(directory,'training_bias.json')
                equivalent({k:v.tolist() for k,v in biases.items()},sealed_bias['bias_nm'])
                if sealed_bias['input_manifests']!={'train':protocol['input_manifests']['train']}:raise ValueError('Training bias role leakage')
            benefit=aggregate_benefit(trial_budgets,settings);equivalent(benefit,row['splits'][split]['attribution'])
            split_summary[split]=dict(prediction=prediction['passed'],band=benefit['qualified_non_degradation_passed'])
        evidence=_read(verified/f'eligibility_ridge_{ridge:g}.json')
        passed=bool(evidence['passed'] and prior['passed'] and all(r['prediction'] and r['band'] for r in split_summary.values()))
        if passed!=row['eligible']:raise ValueError('R3 eligibility differs')
        score=float(np.mean([r['normalized_rmse'] for r in row['splits']['selection']['prediction']['periodic'].values()]))
        equivalent(score,row['selection_score'])
        if passed:eligible.append(row)
    winner=None
    if eligible:
        best=min(r['selection_score'] for r in eligible);winner=max((r for r in eligible if r['selection_score']<=best+protocol['selection_policy']['tie_absolute_nrmse']),key=lambda r:r['ridge'])
    if (None if winner is None else winner['ridge'])!=report['proposed_ridge']:raise ValueError('Selection winner changed')
    formal_test_untouched(root)
    result=dict(schema_version=1,round='R3',passed=bool(eligible),completed=True,p0_acceptance_passed=False,publishable=False,
        original_analysis=str(analysis),analysis_protocol_sha256=file_hash(analysis/'analysis_protocol.json'),
        delivery_manifest_sha256=file_hash(analysis/'delivery_manifest.json'),parent_protocol_sha256=file_hash(root/'protocol.json'),
        eligible_ridges=[r['ridge'] for r in eligible],proposed_ridge=None if winner is None else winner['ridge'],
        native_audits=audits,formal_records_uncollected=formal,test_consumed=False,truth_read=False,candidate_frozen=False,
        scope='fresh raw preprocessing, Pinocchio components, spectral Parseval/qualification, bias and eligibility recomputation; no fits or test measurements',
        verification_source_sha256=file_hash(__file__))
    output.mkdir(parents=True,exist_ok=False)
    write_json(output/'verification.json',result)
    _seal(output,'manifest.json',dict(files={'verification.json':file_hash(output/'verification.json')}))
    if freeze:
        if winner is None:raise ValueError('No qualified candidate; verification preserved without a freeze')
        frozen=freeze_verified(root,analysis,output,report,protocol,winner)
        write_json(output/'freeze_result.json',frozen);result['candidate_frozen']=True
    return result


def freeze_verified(root,analysis,verification,report,protocol,winner):
    """Copy only immutable proof/model JSONs into the parent's local-path contract."""
    snapshot=root/('selection_r3_'+file_hash(analysis/'analysis_protocol.json')[:12]);snapshot.mkdir(exist_ok=False)
    for name,source in {'analysis_protocol.json':analysis/'analysis_protocol.json',
        'residual_attribution.json':analysis/'residual_attribution.json','frequency_budget.json':analysis/'frequency_budget.json',
        'verification.json':verification/'verification.json','verification_manifest.json':verification/'manifest.json',
        'analysis_delivery_manifest.json':analysis/'delivery_manifest.json'}.items():(snapshot/name).write_bytes(source.read_bytes())
    files={str(p.relative_to(root)):file_hash(p) for p in snapshot.iterdir()}
    selected=dict(schema_version=1,protocol_sha256=file_hash(root/'protocol.json'),input_manifests=protocol['input_manifests'],candidates=[])
    for row in report['candidates']:
        label=f'ridge_{row["ridge"]:g}';candidate=snapshot/(label+'.json');candidate.write_bytes((analysis/row['candidate']).read_bytes())
        evidence={}
        for kind in ('stability','prior_development','attribution'):
            target=snapshot/(label+'_'+kind+'.json');evidence[kind]=str(target.relative_to(root))
            if kind=='stability':
                payload=_read(Path(protocol['stability_verification'])/f'eligibility_ridge_{row["ridge"]:g}.json')
                payload['source_files']=files
            else:
                payload=dict(schema_version=1,kind=kind,protocol_sha256=selected['protocol_sha256'],candidate_sha256=file_hash(candidate),completed=True,
                    input_manifests=selected['input_manifests'] if kind=='attribution' else {'train':selected['input_manifests']['train']},source_files=files,
                    passed=(row['prediction_passed'] and row['qualified_band_non_degradation_passed']) if kind=='attribution' else row['prior_development']['passed'])
                if kind=='attribution':payload.update(prediction_passed=row['prediction_passed'],qualified_band_non_degradation_passed=row['qualified_band_non_degradation_passed'],selection_attribution=row['splits']['selection']['attribution'])
            write_json(target,payload)
        selected['candidates'].append(dict(ridge=row['ridge'],candidate=str(candidate.relative_to(root)),candidate_sha256=file_hash(candidate),
            selection_axis_nrmse=[r['normalized_rmse'] for r in row['splits']['selection']['prediction']['periodic'].values()],evidence=evidence))
    selection_path=snapshot/'selection_report.json';write_json(selection_path,selected)
    proposed=next(r['candidate'] for r in selected['candidates'] if r['ridge']==winner['ridge'])
    return freeze_candidate(root,proposed,str(selection_path.relative_to(root)))
