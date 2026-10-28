"""R3: sealed train/selection component attribution, without fitting or test reads."""
from pathlib import Path
from time import perf_counter
import csv
import numpy as np
from . import _core
from .data import write_json
from .dual_model import nominal_dual_arm
from .rigid_identification import model_from_parameters
from .staged_protocol import load_protocol, file_hash, _read, _seal, _unseal
from .staged_identification import recordings
from .staged_multifit import load_problems, score_problems, batch_repeatability, wrist_repeatability, grouped_prediction_summary
from .staged_stability_prior import check_prior_cache
from .staged_residuals import MODELS, matched_components, residual_statistics, frequency_budget, aggregate_benefit, raw_period_spectra, raw_residual_budget


def arrays(path):
    with np.load(path,allow_pickle=False) as data:return {k:data[k] for k in data.files}


def check_manifest(root,name):
    manifest=_unseal(root,name)
    for path,expected in manifest['files'].items():
        target=Path(root)/path
        if target.resolve().is_relative_to(Path(root).resolve()) is False or file_hash(target)!=expected:
            raise ValueError('Changed/escaping sealed analysis file: '+path)
    return manifest


def formal_test_untouched(root):
    trials=_read(Path(root)/'split_manifest.json')['trials']
    names=[r['trial_id'] for r in trials if r['split']=='test']
    names += [r['trial_id'] for r in _read(Path(root)/'motion_plan.json')['final_prior_trials']]
    if (Path(root)/'role_inputs/test.json').exists() or any((Path(root)/'runs'/n).exists() for n in names):
        raise ValueError('Formal test has already been consumed; R3 needs a fresh protocol')
    return len(names)


def prepare_attribution(root,verified,output):
    parent,config,_=load_protocol(root);formal_test_untouched(root)
    check_manifest(verified,'manifest.json');verification=_read(verified/'verification.json')
    analysis=Path(verification['original_analysis'])
    check_manifest(analysis,'delivery_manifest.json')
    if verification['parent_protocol_sha256']!=file_hash(root/'protocol.json') or verification['delivery_manifest_sha256']!=file_hash(analysis/'delivery_manifest.json'):
        raise ValueError('R2 verification is for another analysis/parent')
    r2=_unseal(analysis,'analysis_protocol.json')
    if r2['train_manifest_sha256']!=file_hash(root/'role_inputs/train.json') or not verification['passed']:
        raise ValueError('R2 train/stability qualification rejected')
    if r2['native_binary_sha256']!=file_hash(_core.__file__):raise ValueError('Native mechanics changed since R2')
    # Freeze the additional explicit qualification policy before any selection values.
    settings={**config['p0']['frequency_qualification'],**config['p0']['acceptance'], 'minimum_qualified_bins':1}
    candidates=[]
    for ridge in config['p0']['selection']['ridge_candidates']:
        directory=analysis/'variants'/f'ridge_{ridge:g}_full'
        evidence=_read(verified/f'eligibility_ridge_{ridge:g}.json')
        if evidence['candidate_sha256']!=file_hash(directory/'candidate.json') or evidence['input_manifests']!={'train':file_hash(root/'role_inputs/train.json')}:
            raise ValueError('Mismatched R2 candidate qualification')
        candidates.append(dict(ridge=ridge,directory=str(directory),stability_passed=evidence['passed'],
            B_sha256=file_hash(directory/'candidate_initial.json'),D_sha256=file_hash(directory/'candidate.json'),
            stability_evidence_sha256=file_hash(verified/f'eligibility_ridge_{ridge:g}.json')))
    source=Path(__file__).parent;archive=output/'source';archive.mkdir()
    for p in sorted(source.glob('*.py')):(archive/p.name).write_bytes(p.read_bytes())
    protocol=dict(schema_version=1,round='R3',parent=str(root),parent_protocol_sha256=file_hash(root/'protocol.json'),
        input_manifests={s:file_hash(root/f'role_inputs/{s}.json') for s in ('train','selection')},
        stability_verification=str(verified),stability_analysis=str(analysis),
        stability_manifest_sha256=file_hash(verified/'manifest.json'),stability_delivery_sha256=file_hash(analysis/'delivery_manifest.json'),
        coordinates_sha256=file_hash(analysis/'coordinates.npz'),predecessor_cache_sha256=file_hash(analysis/'predecessor_cache.npz'),
        qualification_sha256=file_hash(root/'qualification/report.json'),candidates=candidates,settings=settings,
        selection_policy=config['p0']['selection'],source_sha256={p.name:file_hash(p) for p in archive.glob('*.py')},
        native_binary_sha256=file_hash(_core.__file__),truth_read=False,test_consumed=False,
        qualification='candidate-invariant A: observed dynamic torque + frozen A inertial proxy above independent noise, raw measured speed and position-derived acceleration thresholds; at least one declared bin/axis',
        eligibility='stability + prior development + raw prediction + B/D qualified-band non-degradation; missing axes prohibit full-domain benefit, not provisional predictive equivalence',
        scope='selection only; no final test, physical parameter recovery, control selection or publication')
    _seal(output,'analysis_protocol.json',protocol)
    return protocol,config


def raw_diagnostics(root,split,config):
    return {name:raw_period_spectra(obs,meta,config) for name,meta,obs in
        recordings(root,'inertia',split=split,purpose='fit' if split=='train' else 'select')}


def component_arrays(nominal,theta,friction,static,p,h):
    model=model_from_parameters(nominal,theta,friction[:14],friction[14:],static)
    return matched_components(model,theta,friction,p,h)


def candidate_gate(periodic,probes,config):
    repeat=batch_repeatability(periodic,config['p0']['acceptance']['repeat_residual_mean_difference_nm'])
    wrist=wrist_repeatability(probes,config['p0']['acceptance']['repeat_residual_mean_difference_nm'])
    return dict(passed=bool(periodic and probes and all(r['passed'] for r in [*periodic.values(),*probes.values()]) and repeat['passed'] and wrist['passed']),
        periodic=periodic,wrist_probes=probes,periodic_repeatability=repeat,wrist_repeatability=wrist,
        grouped=grouped_prediction_summary(periodic,probes))


def generate_attribution(root,output,verified):
    started=perf_counter();root=Path(root).resolve();output=Path(output).resolve();verified=Path(verified).resolve()
    output.mkdir(parents=True,exist_ok=False)
    protocol,config=prepare_attribution(root,verified,output);analysis=Path(protocol['stability_analysis'])
    coordinates=arrays(analysis/'coordinates.npz');cache=arrays(analysis/'predecessor_cache.npz')
    nominal,source=nominal_dual_arm();theta_a=coordinates['gravity_theta'];friction_a=coordinates['friction'];static=coordinates['static']
    if source['sha256']!=load_protocol(root)[0]['model']['sha256']:raise ValueError('Nominal source changed')
    h=config['p0']['preprocessing']['harmonics'];threshold=np.asarray(_read(root/'qualification/report.json')['threshold_bin_rms_nm'])
    problems={};raw={};baseline={}
    for split in ('train','selection'):
        print('R3 load '+split,flush=True)
        problems[split]=load_problems(root,split,static);raw[split]=raw_diagnostics(root,split,config)
        baseline[split]={name:component_arrays(nominal,theta_a,friction_a,static,row['problem'],h)
            for name,row in problems[split].items() if row['meta']['stage']=='inertia'}
    report=dict(schema_version=1,round='R3',completed=True,publishable=False,p0_acceptance_passed=False,
        protocol_sha256=file_hash(output/'analysis_protocol.json'),parent_protocol_sha256=protocol['parent_protocol_sha256'],
        test_consumed=False,truth_read=False,rigid_fit_calls=0,candidates=[],selection_rule=protocol['selection_policy'],
        formal_records_uncollected=formal_test_untouched(root))
    budgets={};csv_rows=[]
    for spec in protocol['candidates']:
        label=f'ridge_{spec["ridge"]:g}';destination=output/label;destination.mkdir()
        b=_read(Path(spec['directory'])/'candidate_initial.json');d=_read(Path(spec['directory'])/'candidate.json')
        (destination/'candidate_initial.json').write_bytes((Path(spec['directory'])/'candidate_initial.json').read_bytes())
        (destination/'candidate.json').write_bytes((Path(spec['directory'])/'candidate.json').read_bytes())
        theta_b=np.asarray(b['parameters']);theta_d=np.asarray(d['parameters']);friction_d=np.r_[d['viscous'],d['coulomb']]
        if not np.array_equal(static,d['static_friction']) or not np.array_equal(friction_a,np.r_[b['viscous'],b['coulomb']]):
            raise ValueError('A/B/C/D frozen static or B friction differs')
        model=model_from_parameters(nominal,theta_d,friction_d[:14],friction_d[14:],static)
        prior=check_prior_cache(cache,theta_d,friction_d,static,config['acceptance'])
        row=dict(ridge=spec['ridge'],candidate=str((destination/'candidate.json').relative_to(output)),
            candidate_sha256=file_hash(destination/'candidate.json'),stability_passed=spec['stability_passed'],prior_development=prior,splits={})
        biases=None;budgets[label]={}
        for split in ('train','selection'):
            metrics,probes=score_problems(problems[split],model,theta_d,friction_d,static,config)
            gate=candidate_gate(metrics,probes,config);trajectories={};split_budgets={};residual_means={name:[] for name in MODELS}
            for name,item in problems[split].items():
                if item['meta']['stage']!='inertia':continue
                p=item['problem'];conditions=item['meta']['conditions'];parts_a,pred_a,audit_a=baseline[split][name]
                parts_b,pred_b,audit_b=component_arrays(nominal,theta_b,friction_a,static,p,h)
                parts_d,pred_d,audit_d=component_arrays(nominal,theta_d,friction_d,static,p,h)
                parts_c={**parts_d,'friction':parts_a['friction']};pred_c=sum(parts_c.values())
                predictions=dict(A=pred_a,B=pred_b,C=pred_c,D=pred_d)
                obs=p['current_torque'];dynamic=obs-parts_a['gravity']-parts_a['friction']
                # Transfer measured velocity coefficients onto the shared uniform phase;
                # retain nonlinear friction features already constructed before filtering.
                coefficients=np.zeros((len(obs)//2+1,14),dtype=complex)
                coefficients[:h+1]=raw[split][name]['measured_velocity_coefficients'][:h+1]
                velocity=np.fft.irfft(coefficients*len(obs),n=len(obs),axis=0)
                raw_current=raw[split][name]['current_coefficients'].mean(axis=0)[:h+1]
                projection_error=float(np.max(abs(raw_current-np.fft.rfft(obs,axis=0)[:h+1]/len(obs))))
                if projection_error>1e-8:raise ValueError('Raw/phase current time projection differs')
                budget=frequency_budget(obs,predictions,velocity,p['ddq'],dynamic,parts_a['rigid_inertia']+parts_a['rotor'],
                    conditions['excitation_harmonics'],h,threshold,protocol['settings'],period_s=conditions['period_s'])
                stats=residual_statistics(obs,predictions,biases)
                changes={f'{a}_to_{b}':dict(rmse_reduction_nm=(np.asarray(stats[a]['rmse_nm'])-stats[b]['rmse_nm']).tolist(),
                    qualified_band_reduction_nm=(np.asarray(budget['bands'][a]['qualified_excitation_rms_nm'])-budget['bands'][b]['qualified_excitation_rms_nm']).tolist())
                    for a,b in zip(MODELS,MODELS[1:])}
                identity={k:conditions[k] for k in ('center_id','acquisition_batch_id','reference_id','repeat_id','split')}
                trajectories[name]=dict(**identity,statistics=stats,ordered_changes=changes,closure=dict(A=audit_a,B=audit_b,D=audit_d),
                    raw_current_projection_difference=projection_error,raw_diagnostic=raw_residual_budget(raw[split][name],predictions,h))
                split_budgets[name]=budget
                saved=dict(observed=obs,measured_velocity=velocity,acceleration=p['ddq'],dynamic_torque=dynamic,inertial_proxy=parts_a['rigid_inertia']+parts_a['rotor'],
                    q=p['q'],dq=p['dq'],ddq=p['ddq'],rotor=p['rotor'],extra=p['extra'],fixed_friction=p['fixed_friction'],
                    raw_current_coefficients=raw[split][name]['current_coefficients'],raw_samples_per_period=np.array(raw[split][name]['samples_per_period']),
                    raw_periods=np.array(raw[split][name]['periods']),raw_nyquist_hz=np.array(raw[split][name]['nyquist_hz']))
                saved.update({'prediction_'+k:v for k,v in predictions.items()})
                saved.update({k+'_'+component:value for k,parts in zip(MODELS,(parts_a,parts_b,parts_c,parts_d)) for component,value in parts.items()})
                np.savez_compressed(destination/(name+'.npz'),**saved)
                for k in MODELS:residual_means[k].append(np.asarray(stats[k]['bias_nm']))
                for j in range(14):
                    for k in MODELS:
                        csv_rows.append(dict(ridge=spec['ridge'],trial=name,**identity,joint=j,model=k,qualified_bins=budget['qualified_bins_per_axis'][j],
                            rmse_nm=stats[k]['rmse_nm'][j],bias_nm=stats[k]['bias_nm'][j],demeaned_rmse_nm=stats[k]['demeaned_rmse_nm'][j],
                            dc_rms_nm=budget['bands'][k]['dc_rms_nm'][j],excitation_rms_nm=budget['bands'][k]['excitation_rms_nm'][j],
                            other_retained_rms_nm=budget['bands'][k]['other_retained_rms_nm'][j],qualified_excitation_rms_nm=budget['bands'][k]['qualified_excitation_rms_nm'][j]))
            if split=='train':
                biases={k:np.mean(v,axis=0) for k,v in residual_means.items()}
                _seal(destination,'training_bias.json',dict(schema_version=1,input_manifests={'train':protocol['input_manifests']['train']},
                    bias_nm={k:v.tolist() for k,v in biases.items()},diagnostic_only=True,exported_to_physical_model=False))
            budgets[label][split]=split_budgets
            row['splits'][split]=dict(prediction=gate,attribution=aggregate_benefit(split_budgets,protocol['settings']),trajectories=trajectories)
        row['prediction_passed']=all(r['prediction']['passed'] for r in row['splits'].values())
        row['qualified_band_non_degradation_passed']=all(r['attribution']['qualified_non_degradation_passed'] for r in row['splits'].values())
        row['eligible']=bool(row['stability_passed'] and prior['passed'] and row['prediction_passed'] and row['qualified_band_non_degradation_passed'])
        row['selection_score']=float(np.mean([m['normalized_rmse'] for m in row['splits']['selection']['prediction']['periodic'].values()]))
        report['candidates'].append(row)
        print(f'R3 {label}: eligible={row["eligible"]}, attribution={row["splits"]["selection"]["attribution"]["status"]}',flush=True)
    eligible=[r for r in report['candidates'] if r['eligible']]
    winner=None
    if eligible:
        best=min(r['selection_score'] for r in eligible);winner=max((r for r in eligible if r['selection_score']<=best+protocol['selection_policy']['tie_absolute_nrmse']),key=lambda r:r['ridge'])
    report.update(passed=bool(eligible),eligible_ridges=[r['ridge'] for r in eligible],proposed_ridge=None if winner is None else winner['ridge'],
        candidate_frozen=False,seconds=perf_counter()-started,selection_status='provisional until independent R3 verification; formal test pending')
    write_json(output/'residual_attribution.json',report);write_json(output/'frequency_budget.json',budgets)
    with (output/'axis_center_batch.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(csv_rows[0]));writer.writeheader();writer.writerows(csv_rows)
    plot_attribution(report,output/'residual_attribution.png')
    _seal(output,'delivery_manifest.json',dict(files={str(p.relative_to(output)):file_hash(p) for p in output.rglob('*') if p.is_file()}))
    return report


def plot_attribution(report,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from .presentation import _font
    from matplotlib import font_manager
    font=font_manager.FontProperties(fname=_font(12).path)
    plt.rcParams['font.family']=font.get_name()
    font_manager.fontManager.addfont(_font(12).path)
    fig,axes=plt.subplots(1,3,figsize=(16,5));x=np.arange(4)
    for row in report['candidates']:
        selection=row['splits']['selection'];stats=selection['trajectories'].values()
        raw=[np.mean([r['statistics'][k]['rmse_nm'] for r in stats]) for k in MODELS]
        demeaned=[np.mean([r['statistics'][k]['demeaned_rmse_nm'] for r in stats]) for k in MODELS]
        axes[0].plot(x,raw,'o-',label=f'ridge={row["ridge"]:g}')
        axes[1].plot(x,demeaned,'o-')
        axes[2].plot(x,[selection['attribution']['mean_qualified_band_rmse_nm'][k] for k in MODELS],'o-')
    for ax,title in zip(axes,('完整周期误差（含偏置）','去均值形状误差（仅诊断）','合格激励频段误差')):
        ax.set_xticks(x,MODELS);ax.set_title(title);ax.set_ylabel('等权平均 RMSE / Nm');ax.grid(alpha=.3)
    axes[0].legend();fig.suptitle('R3 selection 分量归因：A→B→C→D；弱信号轴保留，最终 test 尚未执行')
    fig.tight_layout();fig.savefig(path,dpi=140);plt.close(fig)
