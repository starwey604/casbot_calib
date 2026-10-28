"""R2: frozen-coordinate, train-only physical-model sensitivity experiments."""
from pathlib import Path
from time import perf_counter
import csv
import numpy as np
from . import _core
from .data import write_json
from .dual_model import nominal_dual_arm
from .rigid_identification import fit_rigid, parameter_scales, model_from_parameters
from .staged_protocol import load_protocol, file_hash, _read, _seal, _unseal
from .staged_development import frozen_predecessors, collection_lock
from .staged_multiconfiguration import load_development_reference
from .staged_multifit import load_problems, assemble_training, score_problems, candidate_bundle
from .staged_stability_math import comparison_basis, subspace_change, operator_difference, parameter_difference, physical_margins
from .staged_stability_prior import prepare_prior_cache, check_prior_cache


def physical_prior_variants(nominal,settings):
    rng=np.random.default_rng(settings['prior_seed']);result=[]
    for _ in range(settings['prior_variants']):
        model=nominal.model;joints=model.joints
        for joint in joints:
            body=joint.body
            body.mass*=rng.uniform(1-settings['prior_mass_relative'],1+settings['prior_mass_relative'])
            body.com=body.com+rng.uniform(-settings['prior_com_component_m'],settings['prior_com_component_m'],3)
            body.inertia_com=body.inertia_com*rng.uniform(1-settings['prior_inertia_scale_relative'],1+settings['prior_inertia_scale_relative'])
            joint.body=body
        model.joints=joints
        result.append(_core.ArmDynamics(model).rigid_parameters())
    return np.asarray(result)


def operator_probes(root,config):
    from .staged_collection import load_trial
    settings=config['p0']['stability'];plan=_read(Path(root)/'motion_plan.json')
    centers=list(dict.fromkeys(r['center_id'] for r in plan['trials']))
    references={center:[r for r in plan['trials'] if r['center_id']==center and r['repeat_id']==0 and r['stage']=='inertia' and r['active_arm_group']=='both'] for center in centers}
    rng=np.random.default_rng(settings['operator_probe_seed']);q=[];v=[];a=[];sources=[];curves={}
    for i,center in enumerate(centers):
        count=settings['operator_probe_count']//len(centers)+(i<settings['operator_probe_count']%len(centers))
        accepted=0
        for _ in range(20000):
            if accepted==count:break
            slot=references[center][int(rng.integers(len(references[center])))];name=slot['trial_id']
            if name not in curves:
                meta,curve=load_trial(root,name)
                start=meta['knots_s'][meta['labels'].index('periodic')];curves[name]=(curve,start)
            curve,start=curves[name];time=start+float(rng.uniform(0,config['dynamic_period_s']))
            position,velocity,acceleration=(curve(time,nu=d) for d in range(3))
            if np.min(abs(velocity))<=.02:continue
            q.append(position);v.append(velocity);a.append(acceleration);sources.append(dict(center_id=center,reference=name,time_s=time));accepted+=1
        if accepted!=count:raise ValueError('Insufficient sustained-motion operator probes at '+center)
    return dict(q=np.array(q),dq=np.array(v),ddq=np.array(a)),dict(schema_version=1,sources=sources,
        source='preregistered nominal collision-certified reference domain, all seven planned centers; no observed test data',
        minimum_absolute_velocity_rad_s=.02,motion_execution_certified=False,
        interpretation='static operator queries only; no measurement truth or independent predictive validation')


def frozen_group_mask(weights,key,value):
    return np.concatenate([np.full(row['rows'],row[key]!=value,dtype=bool) for row in weights])


def _snapshot_sources():
    root=Path(__file__).resolve().parent
    return {p.name:file_hash(p) for p in sorted(root.glob('*.py'))}


def prepare_analysis(root,output,config,parent):
    nominal,source=nominal_dual_arm()
    if source['sha256']!=parent['model']['sha256']:raise ValueError('Nominal geometry differs from the recorded model')
    prior=frozen_predecessors(root)
    g,f,s=(_read(prior/stage/'report.json') for stage in ('gravity','friction','breakaway'))
    if not all(r['passed'] for r in (g,f,s)):raise ValueError('Frozen predecessors were rejected')
    frozen=np.r_[f['viscous'],f['coulomb']];static=np.asarray(s['static_friction'])
    radius=np.maximum(abs(frozen)*.05,np.r_[np.full(14,.0005),np.full(14,.00025)])
    lower=np.maximum(0,frozen-radius);upper=frozen+radius;upper[14:]=np.minimum(upper[14:],static-.001)
    nominal_theta=nominal.rigid_parameters();scales=np.r_[parameter_scales(nominal_theta),np.maximum(abs(frozen),.001)]
    with np.load(prior/'gravity/gravity_constraints.npz',allow_pickle=False) as arrays:
        e=arrays['regressor'];value=arrays['prediction']
    np.savez_compressed(output/'coordinates.npz',scales=scales,metric=np.eye(168),nominal_theta=nominal_theta,
        gravity_theta=np.asarray(g['fit']['parameters']),friction=frozen,static=static,lower=lower,upper=upper,equality=e,equality_target=value)
    np.savez_compressed(output/'prior_variants.npz',parameters=physical_prior_variants(nominal,config['p0']['stability']))
    probes,description=operator_probes(root,config)
    np.savez_compressed(output/'operator_probe_set.npz',**probes);write_json(output/'operator_probe_set.json',description)
    policy=dict(rank_rtol=1e-7,rank_atol=1e-12,torque_scale=.2,robust_scale=.02,physical_length_scale_m=.1,
        regularization_metric='identity in fixed nominal physical coordinates; complete quadratic, no subset-dependent nullspace penalty',
        row_weights='frozen full-train weights; omitted groups removed without renormalization',
        prior_center='nominal physical model, not the initial fit; gravity equalities and auxiliary bounds unchanged',
        sensitivity_reference='same-ridge full train / nominal prior',selection_status='deferred until R3; selection observations are not opened')
    files=['coordinates.npz','prior_variants.npz','operator_probe_set.npz','operator_probe_set.json']
    protocol=dict(schema_version=1,round='R2',parent=str(Path(root).resolve()),parent_protocol_sha256=file_hash(Path(root)/'protocol.json'),
        train_manifest_sha256=file_hash(Path(root)/'role_inputs/train.json'),predecessor_freeze_sha256=file_hash(Path(root)/'predecessor_freeze.json'),
        native_binary_sha256=file_hash(_core.__file__),model_source=source,source_sha256=_snapshot_sources(),policy=policy,
        settings=config['p0']['stability'],ridge_candidates=config['p0']['selection']['ridge_candidates'],
        inputs={name:file_hash(output/name) for name in files},truth_read=False,test_consumed=False)
    _seal(output,'analysis_protocol.json',protocol)
    return protocol


def load_analysis(root,output):
    protocol=_unseal(output,'analysis_protocol.json');parent,config,_=load_protocol(root)
    if protocol['parent']!=str(Path(root).resolve()) or protocol['parent_protocol_sha256']!=file_hash(Path(root)/'protocol.json') or protocol['train_manifest_sha256']!=file_hash(Path(root)/'role_inputs/train.json') or protocol['predecessor_freeze_sha256']!=file_hash(Path(root)/'predecessor_freeze.json'):
        raise ValueError('R2 frozen parent inputs changed')
    if protocol['source_sha256']!=_snapshot_sources() or protocol['native_binary_sha256']!=file_hash(_core.__file__):
        raise ValueError('R2 implementation changed; use a new analysis directory')
    for path,expected in protocol['inputs'].items():
        if file_hash(output/path)!=expected:raise ValueError('R2 fixed coordinates/probes/priors changed')
    return protocol,config


def _arrays(path):
    with np.load(path,allow_pickle=False) as arrays:return {k:arrays[k] for k in arrays.files}


def _operators(model,probes):
    terms=model.evaluate_batch(probes['q'],probes['dq'],probes['ddq'])
    return terms['mass_matrix'],terms['total']


def _train_summary(problems,model,theta,friction,static,config):
    metrics,probes=score_problems(problems,model,theta,friction,static,config)
    return dict(passed=all(r['passed'] for r in [*metrics.values(),*probes.values()]),
        maximum_axis_nrmse=max(r['maximum_normalized_rmse'] for r in metrics.values()),
        maximum_wrist_inertia_disagreement=max(r['relative_inertia_disagreement'] for r in probes.values()),
        maximum_wrist_rmse_nm=max(r['rmse_nm'] for r in probes.values()),
        failed_trials=[name for name,row in {**metrics,**probes}.items() if not row['passed']],
        scope='all train only, including omitted groups as held-group diagnostics; not selection or final test')


def run_stability(root,output,resume=False):
    root=Path(root);output=Path(output);started=perf_counter();parent,config,_=load_protocol(root)
    if parent['schema_version']!=2:raise ValueError('R2 needs a certified R1 session')
    if output.exists() and not resume:raise FileExistsError(output)
    if not output.exists():
        output.mkdir(parents=True,exist_ok=False);prepare_analysis(root,output,config,parent)
    protocol,config=load_analysis(root,output)
    with collection_lock(output):
        coordinates=_arrays(output/'coordinates.npz');probes=_arrays(output/'operator_probe_set.npz')
        nominal,source=nominal_dual_arm();training=load_problems(root,'train',coordinates['static'])
        y,x,target,weights=assemble_training(training,config)
        if not (output/'training_setup.json').exists():
            basis=comparison_basis(np.column_stack([y,x]),coordinates['scales'],coordinates['equality'])
            np.savez_compressed(output/'comparison_basis.npz',basis=basis['basis'],singular_values=basis['singular_values'])
            cache=prepare_prior_cache(load_development_reference(root),nominal,coordinates['gravity_theta'],config)
            np.savez_compressed(output/'predecessor_cache.npz',**cache)
            _seal(output,'training_setup.json',dict(schema_version=1,analysis_protocol_sha256=file_hash(output/'analysis_protocol.json'),
                common_rank=basis['rank'],rank_cutoff=basis['threshold'],equality_rank=basis['equality_rank'],free_dimension=basis['free_dimension'],
                rigid_mode_fraction=basis['rigid_fraction'].tolist(),friction_mode_fraction=basis['auxiliary_fraction'].tolist(),
                weights=weights,files={name:file_hash(output/name) for name in ('comparison_basis.npz','predecessor_cache.npz')}))
        setup=_unseal(output,'training_setup.json')
        for path,expected in setup['files'].items():
            if file_hash(output/path)!=expected:raise ValueError('R2 training setup/cache changed')
        if setup['weights']!=weights:raise ValueError('Full-train row weighting changed')
        common_arrays=_arrays(output/'comparison_basis.npz')
        common=dict(basis=common_arrays['basis'],rank=setup['common_rank'])
        prior_cache=_arrays(output/'predecessor_cache.npz');priors=_arrays(output/'prior_variants.npz')['parameters']
        specifications=[dict(kind='full',name='full',prior=coordinates['nominal_theta'],mask=np.ones(len(target),dtype=bool))]
        for key,kind in (('center_id','center'),('acquisition_batch_id','batch')):
            for value in sorted({r[key] for r in weights}):
                specifications.append(dict(kind=kind,name='omit_'+value,prior=coordinates['nominal_theta'],mask=frozen_group_mask(weights,key,value)))
        specifications.extend(dict(kind='prior',name=f'prior_{i:02}',prior=prior,mask=np.ones(len(target),dtype=bool)) for i,prior in enumerate(priors))
        results=[]
        for ridge in protocol['ridge_candidates']:
            reference=None
            for spec in specifications:
                label=f'ridge_{ridge:g}_{spec["name"]}';destination=output/'variants'/label
                if destination.exists():
                    saved=_unseal(destination,'result.json');results.append(saved)
                    for path,expected in saved['files'].items():
                        if file_hash(destination/path)!=expected:raise ValueError('Completed R2 variant changed: '+label)
                    if spec['kind']=='full' and saved['completed']:
                        reference=(_arrays(destination/'solution.npz'),saved)
                    continue
                destination.mkdir(parents=True,exist_ok=False)
                try:
                    mask=spec['mask'];fit=fit_rigid(y[mask],target[mask],spec['prior'],
                        auxiliary=(x[mask],coordinates['friction'],coordinates['lower'],coordinates['upper']),
                        equalities=(coordinates['equality'],coordinates['equality_target']),ridge=ridge,physical_length_scale=.1,lift_residuals=True,
                        fixed_scales=coordinates['scales'],regularization_metric=coordinates['metric'])
                    theta=np.asarray(fit['parameters']);friction=np.asarray(fit['auxiliary_parameters'])
                    model=model_from_parameters(nominal,theta,friction[:14],friction[14:],coordinates['static'])
                    mass,torque=_operators(model,probes);vector=np.r_[theta,friction]
                    current_basis=comparison_basis(np.column_stack([y[mask],x[mask]]),coordinates['scales'],coordinates['equality'],cutoff=setup['rank_cutoff'])
                    rank=subspace_change(common,current_basis)
                    rank.update(singular_values=current_basis['singular_values'].tolist(),cutoff=current_basis['threshold'],
                        rigid_mode_fraction=current_basis['rigid_fraction'].tolist(),friction_mode_fraction=current_basis['auxiliary_fraction'].tolist())
                    previous=check_prior_cache(prior_cache,theta,friction,coordinates['static'],config['acceptance'])
                    train=_train_summary(training,model,theta,friction,coordinates['static'],config)
                    if spec['kind']=='full':
                        initial=fit_rigid(y,target-x@coordinates['friction'],spec['prior'],
                            equalities=(coordinates['equality'],coordinates['equality_target']),ridge=ridge,physical_length_scale=.1,lift_residuals=True,
                            fixed_scales=coordinates['scales'][:140],regularization_metric=coordinates['metric'][:140,:140])
                        write_json(destination/'initial_fit.json',initial)
                        initial_model=model_from_parameters(nominal,initial['parameters'],coordinates['friction'][:14],coordinates['friction'][14:],coordinates['static'])
                        initial_previous=check_prior_cache(prior_cache,np.asarray(initial['parameters']),coordinates['friction'],coordinates['static'],config['acceptance'])
                        write_json(destination/'candidate_initial.json',candidate_bundle(initial['parameters'],coordinates['friction'],coordinates['static'],nominal,source,config,initial,training))
                        reference_arrays=dict(vector=vector,mass=mass,torque=torque)
                    else:
                        if reference is None:raise ValueError('Full-train reference failed; comparison unavailable')
                        reference_arrays=reference[0]
                    difference=operator_difference(reference_arrays['mass'],mass,reference_arrays['torque'],torque,protocol['settings'])
                    delta=parameter_difference(reference_arrays['vector'],vector,coordinates['scales'],common['basis'])
                    solution=dict(vector=vector,mass=mass,torque=torque)
                    np.savez_compressed(destination/'solution.npz',**solution)
                    write_json(destination/'fit.json',fit)
                    write_json(destination/'candidate.json',candidate_bundle(theta,friction,coordinates['static'],nominal,source,config,fit,training))
                    report=dict(schema_version=1,label=label,kind=spec['kind'],ridge=ridge,completed=True,
                        passed=bool(difference['passed'] and rank['passed'] and previous['passed'] and train['passed']),
                        operators=difference,parameters=delta,subspace=rank,predecessor_development=previous,train_prediction=train,
                        boundaries=physical_margins(theta,friction,coordinates['lower'],coordinates['upper']),
                        equality_maximum_error_nm=float(np.max(abs(coordinates['equality']@theta-coordinates['equality_target']))),
                        files={p.name:file_hash(p) for p in destination.iterdir() if p.is_file()})
                    if spec['kind']=='full':
                        report['initial_predecessor_development']=initial_previous
                        report['passed']=report['passed'] and initial_previous['passed']
                    if spec['kind']=='full':reference=(solution,report)
                except (ValueError,RuntimeError,np.linalg.LinAlgError) as error:
                    report=dict(schema_version=1,label=label,kind=spec['kind'],ridge=ridge,completed=False,passed=False,error=str(error),
                        files={p.name:file_hash(p) for p in destination.iterdir() if p.is_file()})
                _seal(destination,'result.json',report);results.append(report)
                print(f'R2 {label}: {"PASS" if report["passed"] else "REJECT"}',flush=True)
        if (output/'delivery_manifest.json').exists():
            sealed=_unseal(output,'delivery_manifest.json')
            for path,expected in sealed['files'].items():
                if file_hash(output/path)!=expected:raise ValueError('Completed R2 analysis output changed: '+path)
            completed=_read(output/'report.json')
            if completed['variants']!=results:raise ValueError('Completed R2 summary differs from sealed variants')
            return completed
        return _finalize(output,root,protocol,results,started)


def _finalize(output,root,protocol,results,started):
    from .staged_stability_validation import regularization_path
    path=regularization_path(output,protocol['settings'])
    path_pass={r['ridge']:r['passed'] for r in path['comparisons']}
    eligible=[ridge for ridge in protocol['ridge_candidates'] if path_pass.get(ridge,False) and all(r['passed'] for r in results if r['ridge']==ridge)]
    report=dict(schema_version=1,round='R2',completed=True,passed=bool(eligible),stability_qualified=bool(eligible),
        p0_acceptance_passed=False,publishable=False,test_consumed=False,truth_read=False,
        eligible_ridges=eligible,selection_status='not_selected; R3 attribution and selection-only prediction gates pending',
        parent_protocol_sha256=file_hash(root/'protocol.json'),analysis_protocol_sha256=file_hash(output/'analysis_protocol.json'),
        variants=results,regularization_path=path,fit_calls=sum(2 if r['kind']=='full' else 1 for r in results),wall_s=perf_counter()-started,
        scope='train-only sensitivity on fixed declared probes; no confidence intervals, test observation or hardware acceptance')
    write_json(output/'regularization_path.json',path)
    write_json(output/'group_sensitivity.json',dict(variants=[r for r in results if r['kind'] in ('center','batch')]))
    write_json(output/'prior_sensitivity.json',dict(variants=[r for r in results if r['kind']=='prior'],confidence_interval=False))
    write_json(output/'operator_stability.json',report);write_json(output/'report.json',report)
    for full in (r for r in results if r['kind']=='full' and r['completed']):
        directory=output/'variants'/full['label']
        write_json(directory/'eligibility_stability.json',dict(schema_version=1,kind='stability',
            protocol_sha256=file_hash(root/'protocol.json'),input_manifests={'train':file_hash(root/'role_inputs/train.json')},
            candidate_sha256=file_hash(directory/'candidate.json'),completed=True,passed=full['ridge'] in eligible,
            source_analysis_protocol_sha256=file_hash(output/'analysis_protocol.json'),
            scope='train-only complete sensitivity family; selection and attribution gates still required'))
    with (output/'stability.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['variant','ridge','kind','completed','passed','M_frobenius','M_diagonal','inverse_nrmse','rank_lost','error'])
        for row in results:
            op=row.get('operators',{});writer.writerow([row['label'],row['ridge'],row['kind'],row['completed'],row['passed'],op.get('maximum_mass_relative_frobenius'),op.get('maximum_mass_diagonal_relative'),op.get('maximum_inverse_axis_normalized_rms'),row.get('subspace',{}).get('rank_lost'),row.get('error')])
    _plot(output,results)
    (output/'report.md').write_text('# R2 动力学算子稳定性\n\n'+
        ('存在通过稳定性门禁的正则化候选。' if eligible else '没有候选通过全部稳定性门禁；不能冻结或发布该适用域的候选。')+
        '\n\n合格ridge：'+str(eligible)+'。选择与归因仍待R3，正式test未执行。\n\n'+
        '[完整报告](operator_stability.json) · [分组敏感性](group_sensitivity.json) · [先验敏感性](prior_sensitivity.json) · [CSV](stability.csv) · [图](operator_stability.png)\n\n'+
        '固定名义参数尺度、完整正则化度量及train行权重；所有变体保持相同重力等式、摩擦边界和静摩擦。敏感性不是置信区间，单个连杆参数或边界命中不单独判失败。\n',encoding='utf-8')
    _seal(output,'delivery_manifest.json',dict(schema_version=1,files={str(p.relative_to(output)):file_hash(p)
        for p in output.rglob('*') if p.is_file() and '.collection_lock' not in p.parts}))
    return report


def _plot(output,results):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.font_manager import FontProperties
    from .presentation import _font
    font=FontProperties(fname=_font(12).path)
    figure=Figure(figsize=(13,9));FigureCanvasAgg(figure);axes=figure.subplots(3,1)
    for ridge in sorted({r['ridge'] for r in results}):
        rows=[r for r in results if r['ridge']==ridge]
        axes[0].plot([r['label'].split('_',2)[-1] for r in rows],[r.get('operators',{}).get('maximum_mass_relative_frobenius',np.nan)*100 for r in rows],'o-',label=str(ridge))
        axes[1].plot([r['label'].split('_',2)[-1] for r in rows],[r.get('operators',{}).get('maximum_inverse_axis_normalized_rms',np.nan)*100 for r in rows],'o-',label=str(ridge))
        axes[2].plot([r['label'].split('_',2)[-1] for r in rows],[r.get('operators',{}).get('maximum_mass_diagonal_relative',np.nan)*100 for r in rows],'o-',label=str(ridge))
    axes[0].axhline(5.,color='red',ls='--');axes[0].set_ylabel('质量矩阵最大相对F范数差 / %',fontproperties=font)
    axes[1].axhline(2.,color='red',ls='--');axes[1].set_ylabel('逆动力学最大逐轴差异 / %',fontproperties=font)
    axes[2].axhline(10.,color='red',ls='--');axes[2].set_ylabel('质量矩阵对角项最大相对差 / %',fontproperties=font)
    figure.suptitle('R2：固定探针上的敏感性；失败变体保留，非统计置信区间',fontproperties=font)
    for axis in axes:axis.grid(alpha=.2);axis.legend(title='ridge');axis.tick_params(axis='x',labelrotation=25)
    figure.tight_layout();figure.savefig(output/'operator_stability.png',dpi=140)
