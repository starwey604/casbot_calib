"""Training-only rank revelation and bounded robust geometric identification."""
from hashlib import sha256
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from scipy.linalg import qr
from scipy.optimize import least_squares,lsq_linear
from . import _core
from .kinematics import nominal_kinematics,calibration,ball_world,digest,geometry_manifest
from .kinematic_parameters import KinematicParameters
from .trical_data import load_observations
from .trical_workflow import stable_window
from .data import write_json


def load_batch(folder, split):
    root=Path(folder);report=json.loads((root/'report.json').read_text());manifest=json.loads((root/'manifest.json').read_text())
    if not report['passed']:raise ValueError('A rejected measurement session cannot be used for identification')
    for name,expected in manifest['files'].items():
        path=Path(name)
        if path.is_absolute() or '..' in path.parts or 'evaluation' in path.parts:raise ValueError('Invalid measurement manifest path')
        if sha256((root/path).read_bytes()).hexdigest()!=expected:raise ValueError('Measurement source digest mismatch')
    config=json.loads((root/'profile.json').read_text())
    if 'mock' in config:raise ValueError('Hidden Mock configuration is not an identification input')
    arrays,metadata=load_observations(root/'observations')
    with np.load(root/'raw/row_decision_times.npz',allow_pickle=False) as raw:times=raw['time_ns']
    measured=json.loads((root/'measurements.json').read_text());visits={v['id']:v for v in config['workflow']['visits']}
    if len(measured)!=len(visits) or len({m['visit'] for m in measured})!=len(measured):raise ValueError('Incomplete or duplicated measurement batch')
    q=[];frames=[];offsets=[];targets=[];ids=[]
    for item in measured:
        visit=visits[item['visit']]
        if visit.get('split',split)!=split:raise ValueError('Training and validation batch labels disagree')
        if visit['frame']!=item['frame'] or visit['ball_id']!=item['ball_id']:raise ValueError('Measurement target identity changed')
        window=stable_window(arrays,times,item['accepted_ns'],config,item['stable_after_ns'],item['frame'],item['ball_id'])
        if window is None or window['indices']!=item['window']['indices']:raise ValueError('No valid independent stable window')
        if not np.allclose(window['q_rad'],item['window']['q_rad'],atol=1e-14,rtol=0):raise ValueError('Accepted q differs from raw observations')
        offset=_core.trical_offset(np.array(window['readings_m']),calibration(config))
        if np.max(abs(offset))>config['workflow']['centering_tolerance_m']:raise ValueError('Uncentered measurement')
        q.append(window['q_rad']);frames.append(item['frame']);offsets.append(offset);targets.append(ball_world(config,item['ball_id']));ids.append(item['visit'])
    return dict(q=np.array(q),frames=np.array(frames),offsets=np.array(offsets),targets=np.array(targets),ids=ids,
        split=split,config=config,geometry=metadata['geometry'],source_manifest_sha256=sha256((root/'manifest.json').read_bytes()).hexdigest(),
        batch_id=config.get('k3',{}).get('batch_id',root.name))


def observability(matrix, maximum_condition=200., relative_rank_tolerance=1e-7):
    singular=np.linalg.svd(matrix,compute_uv=False)
    threshold=(singular[0]*relative_rank_tolerance) if len(singular) else 0.
    rank=int(np.count_nonzero(singular>threshold))
    practical=int(np.count_nonzero(singular>singular[0]/maximum_condition)) if len(singular) and singular[0]>0 else 0
    return dict(rows=int(matrix.shape[0]),columns=int(matrix.shape[1]),rank=rank,practical_rank=practical,
        singular_values=singular.tolist(),relative_rank_tolerance=relative_rank_tolerance,
        condition_number=float(singular[0]/singular[rank-1]) if rank else None,maximum_fit_condition=maximum_condition)


def select_parameters(jacobian, definitions, maximum_condition=200.):
    """Keep the zero layer first, then admit independent well-conditioned columns."""
    matrix=jacobian.reshape(-1,len(definitions));zero=[i for i,d in enumerate(definitions) if d['zero_stage']]
    zero_info=observability(matrix[:,zero],maximum_condition)
    if zero_info['rank']!=len(zero) or zero_info['condition_number']>maximum_condition:return [],zero_info
    remaining=[i for i in range(len(definitions)) if i not in zero]
    projected=matrix[:,remaining]-matrix[:,zero]@np.linalg.lstsq(matrix[:,zero],matrix[:,remaining],rcond=None)[0]
    order=qr(projected,mode='economic',pivoting=True)[2];selected=list(zero)
    for index in order:
        candidate=selected+[remaining[index]];info=observability(matrix[:,candidate],maximum_condition)
        if info['rank']==len(candidate) and info['condition_number']<=maximum_condition:selected=candidate
    return selected,zero_info


def residual_metrics(values):
    norms=np.linalg.norm(values,axis=1)
    return dict(position_rmse_m=float(np.sqrt(np.mean(norms**2))),mean_m=float(norms.mean()),max_m=float(norms.max()),p95_m=float(np.percentile(norms,95)))


def solve(parameters, train, validation, *, sigma_m=2e-6, maximum_condition=200., validation_limit_m=20e-6):
    started=perf_counter();size=parameters.size;zero=np.zeros(size)
    if not np.isfinite(sigma_m) or sigma_m<=0:raise ValueError('Positive declared measurement uncertainty required')
    before,jacobian=parameters.evaluate(zero,train['q'],train['frames'],train['offsets'])
    matrix=jacobian.reshape(-1,size)/sigma_m;selected,zero_info=select_parameters(jacobian,parameters.definitions,maximum_condition)
    full_info=observability(matrix,maximum_condition)
    result=dict(schema_version=1,identifiable=bool(selected),passed=False,truth_read=False,
        standard_parameter_count=size,full_candidate_observability=full_info,zero_stage_observability=zero_info,
        gauge=parameters.chart['gauge'],anchors=parameters.description()['anchors'])
    anchor_geometry={}
    for frame in np.unique(train['frames']):
        points=np.unique(train['targets'][train['frames']==frame],axis=0)
        rank=int(np.linalg.matrix_rank(points-points.mean(axis=0),tol=1e-9));anchor_geometry[str(frame)]=dict(targets=len(points),rank=rank)
    result['anchor_geometry']=anchor_geometry
    if any(a['targets']<3 or a['rank']<2 for a in anchor_geometry.values()):
        result.update(identifiable=False,reason='insufficient_noncollinear_metrology_targets',selected_parameters=[],solve_wall_s=perf_counter()-started)
        return result,None,None
    if not selected:
        result.update(reason='insufficient_excitation_for_zero_layer',selected_parameters=[],solve_wall_s=perf_counter()-started)
        return result,None,None
    if len(selected)>=len(matrix):
        result.update(identifiable=False,reason='no_residual_degrees_of_freedom');return result,None,None
    zero_columns=[i for i,d in enumerate(parameters.definitions) if d['zero_stage']]
    baseline=lsq_linear(matrix[:,zero_columns],(train['targets']-before).ravel()/sigma_m,bounds=(-3,3),tol=1e-12)
    linear=np.zeros(size);linear[zero_columns]=baseline.x
    def fit(columns, initial):
        fixed=np.zeros(size)
        cached={}
        def evaluate(x):
            if 'x' in cached and np.array_equal(x,cached['x']):return cached['result']
            full=fixed.copy();full[columns]=x
            points,j=parameters.evaluate(full,train['q'],train['frames'],train['offsets'])
            result=(points-train['targets']).ravel()/sigma_m,j.reshape(-1,size)[:,columns]/sigma_m
            cached.update(x=x.copy(),result=result);return result
        optimized=least_squares(lambda x:evaluate(x)[0],initial[columns],jac=lambda x:evaluate(x)[1],bounds=(-3,3),
            loss='soft_l1',f_scale=1.,xtol=1e-11,ftol=1e-11,gtol=1e-10,max_nfev=100)
        value=np.zeros(size);value[columns]=optimized.x
        return value,optimized
    first,zero_fit=fit(zero_columns,linear);fitted,optimized=fit(selected,first)
    def metrics(x,data):
        points,_=parameters.evaluate(x,data['q'],data['frames'],data['offsets']);return residual_metrics(points-data['targets'])
    training={name:metrics(x,train) for name,x in [('nominal',zero),('linear_zero',linear),('zero_layer',first),('geometry_layer',fitted)]}
    independent={name:metrics(x,validation) for name,x in [('nominal',zero),('linear_zero',linear),('zero_layer',first),('geometry_layer',fitted)]}
    predictions,j_final=parameters.evaluate(fitted,train['q'],train['frames'],train['offsets'])
    fit_info=observability(j_final.reshape(-1,size)[:,selected]/sigma_m,maximum_condition)
    hit=bool(np.any(abs(fitted[selected])>2.9999));nominal_rmse=independent['nominal']['position_rmse_m'];final_rmse=independent['geometry_layer']['position_rmse_m']
    improvement=1-final_rmse/max(nominal_rmse,1e-15)
    full_rank=fit_info['rank']==len(selected)
    checks=dict(optimizer=bool(optimized.success),no_bound_hit=not hit,condition=full_rank and fit_info['condition_number']<=maximum_condition,
        independent_constraint_rmse=final_rmse<=validation_limit_m,
        improvement=improvement>=.5 if nominal_rmse>5*sigma_m else True)
    covariance=np.linalg.inv((j_final.reshape(-1,size)[:,selected]/sigma_m).T@(j_final.reshape(-1,size)[:,selected]/sigma_m)) if full_rank else None
    uncertainties=np.sqrt(np.maximum(np.diag(covariance),0.)).tolist() if full_rank else None
    result.update(passed=all(checks.values()),reason='observable_representative_fit',checks=checks,selected_parameters=selected,
        fixed_prior_parameters=[i for i in range(size) if i not in selected],fit_observability=fit_info,
        selected_scaled_standard_error=uncertainties,sigma_m=sigma_m,validation_limit_m=validation_limit_m,
        training=training,validation=independent,validation_improvement_fraction=improvement,
        optimizer=dict(method='bounded TRF / soft_l1',nfev=optimized.nfev,optimality=float(optimized.optimality)),
        solve_wall_s=perf_counter()-started,parameter_interpretation='Training-selected physical representative; frozen prior directions are not measurements',
        metric_scope='Metric artifact position constraints, not global position/orientation accuracy')
    arrays=dict(scaled_parameters=fitted,zero_stage_scaled_parameters=first,linear_zero_parameters=linear,
        selected_indices=np.array(selected,dtype=np.int64),training_jacobian_scaled=matrix,
        nominal_train_points=before,fitted_train_points=predictions)
    return result,fitted,arrays


def identify(train_folder, validation_folder, output):
    output=Path(output)
    if output.exists():raise FileExistsError(output)
    if Path(train_folder).resolve()==Path(validation_folder).resolve():raise ValueError('Independent validation session required')
    train=load_batch(train_folder,'train');validation=load_batch(validation_folder,'validation')
    if train['source_manifest_sha256']==validation['source_manifest_sha256']:raise ValueError('Duplicated train/validation session')
    for key in ('geometry_sha256','robot_model_id'):
        if train['geometry'][key]!=validation['geometry'][key]:raise ValueError('Train/validation robot geometry mismatch')
    for key in ('fixture','tools','indicators'):
        if digest(train['config'][key])!=digest(validation['config'][key]):raise ValueError('Measurement metrology changed between batches')
    if train['batch_id']==validation['batch_id']:raise ValueError('Separate acquisition batch IDs required')
    if set(train['frames'])!=set(validation['frames']):raise ValueError('Training and validation tool coverage differs')
    closest=float('inf')
    for a,frame in zip(validation['q'],validation['frames']):
        q=train['q'][train['frames']==frame]
        if len(q):closest=min(closest,float(np.linalg.norm(q-a,axis=1).min()))
    if closest<.01:raise ValueError('Validation posture duplicates or nearly duplicates a training posture')
    if not np.isfinite(closest):raise ValueError('Training and validation must cover the same tool frames')
    engine,_=nominal_kinematics(train['config']);parameters=KinematicParameters(engine);settings=train['config'].get('k3',{})
    report,fitted,arrays=solve(parameters,train,validation,sigma_m=settings.get('sigma_m',2e-6),maximum_condition=settings.get('maximum_condition',200.),validation_limit_m=settings.get('validation_limit_m',20e-6))
    report.update(train_batch=train['batch_id'],validation_batch=validation['batch_id'],train_samples=len(train['q']),validation_samples=len(validation['q']),
        nearest_validation_posture_distance_rad=closest,train_manifest_sha256=train['source_manifest_sha256'],validation_manifest_sha256=validation['source_manifest_sha256'],
        source_geometry_sha256=train['geometry']['geometry_sha256'],synthetic_only=True,hardware_executable=False)
    output.mkdir(parents=True,exist_ok=False);write_json(output/'parameterization.json',parameters.description())
    if fitted is not None:
        np.savez_compressed(output/'solution.npz',**arrays)
        candidate=parameters.build(fitted);geometry=geometry_manifest(candidate,'trical_mdh_hayati_candidate_v1')
        write_json(output/'candidate_geometry.json',dict(**geometry,accepted=report['passed'],dynamics_revalidation_required=True,collision_revalidation_required=True))
        write_json(output/'parameters.json',dict(chart_sha256=parameters.chart['sha256'],scaled_values=fitted.tolist(),
            parameters=[dict(**d,estimate=float(fitted[i]*d['scale']),status='estimated_in_selected_gauge' if i in report['selected_parameters'] else 'fixed_prior_not_measured') for i,d in enumerate(parameters.definitions)]))
        from .kinematic_export import export_urdf,export_table
        export_table(parameters,fitted,output)
        report['export']=export_urdf(engine,candidate,train['config'],output/'candidate.urdf')
    write_json(output/'report.json',report);return report
