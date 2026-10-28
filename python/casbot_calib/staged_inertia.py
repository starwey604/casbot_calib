"""Full physical inertia fit after frozen gravity/moving/static-friction stages."""
import csv,json
from pathlib import Path
import numpy as np
from scipy.signal import butter,sosfiltfilt
from scipy.interpolate import CubicSpline
from .data import write_json
from .dual_model import nominal_dual_arm
from .dual_identification import periodic_observations
from .rigid_identification import fit_rigid,model_from_parameters
from .staged_identification import recordings,session_config,_windows,_residual,friction_pairs


def _project(values,harmonics):
    frequency=np.fft.rfft(values,axis=0);frequency[harmonics+1:]=0
    return np.fft.irfft(frequency,n=len(values),axis=0)


def probe_problem(nominal,obs,meta,gain):
    """Zero-phase, matched-band observations for a non-reversing acceleration scan."""
    t=obs['sample_ns']*1e-9;step=float(np.median(np.diff(t)));j=meta['conditions']['joint'];sign=meta['conditions']['direction']
    sos=butter(6,30.,fs=1/step,output='sos')
    q=sosfiltfilt(sos,obs['q'],axis=0);v=sosfiltfilt(sos,obs['dq'],axis=0)
    current=sosfiltfilt(sos,obs['current']*gain,axis=0)
    acceleration=CubicSpline(t,v,axis=0).derivative()(t)
    windows=_windows(meta,'probe_measure');begin=windows[0][0]+.2;end=windows[-1][1]-.2
    active=np.flatnonzero((t>=begin)&(t<end))[::4]
    if len(active)<100 or np.min(sign*v[active,j])<=.02:raise ValueError('Probe did not sustain signed motion above the friction transition band')
    held=[k for k in range(14) if k!=j];held_velocity=float(np.max(abs(v[active][:,held])))
    if held_velocity>.001:raise ValueError('Other axes moved during the independent single-axis inertia probe')
    q,v,a=q[active],v[active],acceleration[active];native=nominal.regressor_batch(q,v,a)
    y=native['regressor'][:,j,:];extra=np.zeros((len(q),28));extra[:,j]=v[:,j];extra[:,14+j]=sign
    return dict(regression=y,extra=extra,target=current[active,j]-native['rotor_torque'][:,j],joint=j,
        q=q,dq=v,ddq=a,time_s=t[active],
        preprocessing=dict(filter='sixth-order zero-phase Butterworth, 30 Hz, matched q/velocity/current',
            acceleration='SciPy CubicSpline derivative of filtered measured velocity, no gaps',
            edge_trim_s=.2,minimum_signed_velocity_rad_s=.02,maximum_held_velocity_rad_s=held_velocity,frequency_hz=meta['conditions']['frequency_hz']))


def periodic_problem(nominal,obs,meta,gain,static,harmonics=40,phase_points=1024):
    windows=_windows(meta,'periodic');begin=round(windows[0][0]*1e9);end=round(windows[-1][1]*1e9)
    mask=(obs['sample_ns']>=begin)&(obs['sample_ns']<end)
    selected={k:v[mask].copy() for k,v in obs.items()};selected['sample_ns']-=begin;selected['receive_ns']-=begin
    # Nonlinear friction must be evaluated on acquired velocity BEFORE temporal
    # projection. Filtering velocity and then taking its sign does not commute,
    # especially around drive ripple / stick-slip zero crossings.
    measured_v=selected['dq'];direction=np.sign(measured_v);transition_raw=np.exp(-(measured_v/.002)**2)*direction
    selected['friction_features']=np.column_stack([measured_v,direction-transition_raw,transition_raw])
    data=periodic_observations(selected,meta['conditions']['period_s'],2,harmonics,phase_points,extra_keys=('friction_features',))
    q,v,a=data['q'],data['dq'],data['ddq'];native=nominal.regressor_batch(q,v,a)
    regression=_project(native['regressor'],harmonics);extra=np.zeros((len(q),14,28))
    projected=data['projected_channels']['friction_features'];fixed=projected[:,28:]*np.asarray(static)
    for j in range(14):
        extra[:,j,j]=projected[:,j];extra[:,j,14+j]=projected[:,14+j]
    rotor=_project(native['rotor_torque'],harmonics)
    target=data['current']*gain-fixed-rotor
    moving=abs(v)>.015
    return dict(regression=regression,extra=extra,fixed_friction=fixed,rotor=rotor,target=target,current_torque=data['current']*gain,
        moving=moving,**data,preprocessing=dict(harmonics=harmonics,phase_points=phase_points,discard_periods=2,
            frequency_band_hz=[0,harmonics/meta['conditions']['period_s']],
            source='actual acquired position/current Fourier regression; analytic position derivatives; identical projection of model terms',
            friction_order='acquired velocity -> nonlinear dry/Stribeck features -> same Fourier projection as current',
            moving_velocity_threshold_rad_s=.015))


def _metrics(problem,theta,friction,static,floor):
    predicted=problem['regression']@theta+problem['extra']@friction+problem['fixed_friction']+problem['rotor']
    observed=problem['current_torque'];mask=problem['moving'];rmse=[];normalized=[];counts=[]
    rigid=problem['regression']@theta+problem['rotor'];held_excess=[]
    for j in range(14):
        active=mask[:,j];counts.append(int(active.sum()))
        if active.sum()>=16:
            error=float(np.sqrt(np.mean((predicted[active,j]-observed[active,j])**2)))
            rms=float(np.sqrt(np.mean(observed[active,j]**2)));rmse.append(error);normalized.append(error/max(rms,floor))
        else:rmse.append(None);normalized.append(None)
        quiet=~active
        # Static reaction is set-valued; no unique zero-speed friction torque is
        # inferred from position alone. Test the independently measured interval.
        held_excess.append(float(np.max(np.maximum(abs(rigid[quiet,j]-observed[quiet,j])-static[j],0),initial=0.)))
    return dict(rmse_nm=rmse,normalized_rmse=normalized,moving_samples=counts,
        maximum_normalized_rmse=max((v for v in normalized if v is not None),default=float('inf')),
        stationary_interval_excess_nm=held_excess,maximum_stationary_interval_excess_nm=max(held_excess),
        stationary_convention='rigid predicted effort +/- frozen operational static-friction bound; no false unique hold prediction')


def _preserve_prior_stages(root,candidate,friction,static,original_gravity,acceptance):
    gravity_rows=[]
    for name,meta,obs in recordings(root,'gravity'):
        if meta['conditions']['batch']!='validation':continue
        _,end=_windows(meta,'static_measure')[0];t=obs['sample_ns']*1e-9;mask=(t>end-.5)&(t<end-.05)&(np.max(abs(obs['dq']),axis=1)<.002)
        q=obs['q'][mask].mean(axis=0);torque=obs['current'][mask].mean(axis=0)*acceptance['gain_nm_a']
        gravity_rows.append((candidate.gravity(q),torque))
    predicted=np.array([r[0] for r in gravity_rows]);target=np.array([r[1] for r in gravity_rows])
    gravity=_residual((predicted[::2]+predicted[1::2])/2,(target[::2]+target[1::2])/2)
    errors=[[] for _ in range(14)]
    for name,meta,obs in recordings(root,'friction'):
        if meta['conditions']['batch']!='validation':continue
        x,y,_=friction_pairs(obs,meta,candidate,acceptance['gain_nm_a'],acceptance['control_period_ns']);j=meta['conditions']['joint']
        errors[j].extend(x@np.array([friction[j],friction[14+j]])-y)
    moving_rmse=[float(np.sqrt(np.mean(np.square(e)))) for e in errors]
    onset_change=[]
    for _,meta,obs in recordings(root,'breakaway'):
        begin,end=_windows(meta,'torque_ramp')[0];t=obs['sample_ns']*1e-9;mask=(t>=begin)&(t<=end)
        q=obs['q'][mask][::25]
        difference=candidate.gravity_batch(q)-original_gravity.gravity_batch(q)
        onset_change.append(float(np.max(abs(difference))))
    return dict(gravity=gravity,moving_friction_rmse_nm=moving_rmse,maximum_breakaway_gravity_change_nm=max(onset_change,default=float('inf')),
        static_friction_frozen=static.tolist(),
        passed=bool(gravity['maximum_absolute_rmse_nm']<=acceptance['gravity_pair_rmse_nm'] and
            max(moving_rmse)<=acceptance['friction_absolute_rmse_nm'] and max(onset_change,default=float('inf'))<=1e-5))


def identify_inertia(root,output,gravity_report,friction_report,breakaway_report,gravity_constraints):
    root=Path(root);output=Path(output);output.mkdir(parents=True,exist_ok=False);config=session_config(root);nominal,source=nominal_dual_arm()
    if not all(r['passed'] for r in (gravity_report,friction_report,breakaway_report)):raise ValueError('Earlier independent stage acceptance failed')
    theta_g=np.asarray(gravity_report['fit']['parameters']);b=np.asarray(friction_report['viscous']);fc=np.asarray(friction_report['coulomb']);fs=np.asarray(breakaway_report['static_friction'])
    frozen_friction=np.r_[b,fc];problems={};training=None
    for name,meta,obs in recordings(root,'inertia'):
        problem=periodic_problem(nominal,obs,meta,config['drive']['gain_nm_a'],fs)
        problems[name]=problem
        if meta['conditions']['batch']=='train':training=problem
    if training is None or len(problems)!=4:raise ValueError('Need one complete training and three independent validation trajectories')
    with np.load(gravity_constraints,allow_pickle=False) as data:equality=(data['regressor'],data['prediction'])
    active=training['moving'];y=training['regression'][active];x=training['extra'][active];target=training['target'][active]
    probes={};probe_y=[];probe_x=[];probe_target=[]
    for name,meta,obs in recordings(root,'inertia_probe'):
        problem=probe_problem(nominal,obs,meta,config['drive']['gain_nm_a']);probes[name]=(meta,problem)
        if meta['conditions']['batch']=='train':probe_y.append(problem['regression']);probe_x.append(problem['extra']);probe_target.append(problem['target'])
    if not probe_y:raise ValueError('Dedicated wrist-inertia probes are required, not inferred from small total torque residuals')
    # The low-frequency wrist rows include unresolved zero-speed reaction after
    # temporal filtering. Give the explicitly non-reversing probes more weight;
    # keep all primary rows and report both independent prediction regimes.
    row_weights=np.ones_like(active,dtype=float);row_weights[:,[4,11]]=.1
    row_weights=row_weights[active]
    y=np.vstack([y*row_weights[:,None],*probe_y]);x=np.vstack([x*row_weights[:,None],*probe_x]);target=np.concatenate([target*row_weights,*probe_target])
    initial=fit_rigid(y,target-x@frozen_friction,theta_g,equalities=equality,ridge=1e-4,physical_length_scale=.1,lift_residuals=True)
    # One predeclared joint correction, trained only on inertia_train. Prior
    # gravity combinations remain exact; friction stays inside its staged band.
    radius=np.maximum(abs(frozen_friction)*.05,np.r_[np.full(14,.0005),np.full(14,.00025)])
    lower=np.maximum(0,frozen_friction-radius);upper=frozen_friction+radius;upper[14:]=np.minimum(upper[14:],fs-.001)
    refined=fit_rigid(y,target,np.asarray(initial['parameters']),equalities=equality,ridge=1e-4,physical_length_scale=.1,lift_residuals=True,
        auxiliary=(x,frozen_friction,lower,upper))
    theta=np.asarray(refined['parameters']);friction=np.asarray(refined['auxiliary_parameters'])
    candidate=model_from_parameters(nominal,theta,friction[:14],friction[14:],fs)
    validation={name:_metrics(p,theta,friction,fs,config['acceptance']['torque_floor_nm']) for name,p in problems.items() if name!='inertia_train'}
    probe_validation={}
    for name,(meta,p) in probes.items():
        if meta['conditions']['batch']!='validation':continue
        error=p['regression']@theta+p['extra']@friction-p['target']
        j=p['joint'];time=p['time_s']-np.mean(p['time_s']);gravity_torque=candidate.gravity_batch(p['q'])[:,j]
        design=np.column_stack([p['ddq'][:,j],p['dq'][:,j],np.ones(len(time)),time,time*time])
        scale=np.maximum(np.linalg.norm(design,axis=0),1e-12)
        observed=np.linalg.lstsq(design/scale,(p['target']-gravity_torque),rcond=1e-9)[0]/scale
        predicted_inertia=float(np.mean([candidate.mass_matrix(q)[j,j] for q in p['q'][::32]]))
        relative=abs(predicted_inertia-observed[0])/max(abs(observed[0]),1e-8)
        probe_validation[name]=dict(rmse_nm=float(np.sqrt(np.mean(error**2))),joint=p['joint'],
            independently_regressed_joint_inertia_kg_m2=float(observed[0]),model_joint_inertia_kg_m2=predicted_inertia,
            relative_inertia_disagreement=float(relative),
            method='single-axis actual torque balance: acceleration + velocity + constant/slow drift nuisance; no model inertia used in regression',
            passed=bool(observed[0]>0 and relative<=.1 and np.sqrt(np.mean(error**2))<=.002),preprocessing=p['preprocessing'])
    initial_validation={name:_metrics(p,np.asarray(initial['parameters']),frozen_friction,fs,config['acceptance']['torque_floor_nm']) for name,p in problems.items() if name!='inertia_train'}
    acceptance={**config['acceptance'],**dict(gain_nm_a=config['drive']['gain_nm_a'],control_period_ns=config['control_period_ns'])}
    original_gravity=model_from_parameters(nominal,theta_g)
    preserved=_preserve_prior_stages(root,candidate,friction,fs,original_gravity,acceptance)
    minimum_speed=np.sqrt(np.mean(training['dq']**2,axis=0));minimum_acceleration=np.sqrt(np.mean(training['ddq']**2,axis=0))
    checks=dict(physical_candidate=True,gravity_and_friction_preserved=preserved['passed'],
        all_axes_excited=bool(np.all(minimum_speed>.01) and np.all(minimum_acceleration>.02)),
        dedicated_wrist_prediction=bool(probe_validation and all(p['passed'] for p in probe_validation.values())),
        independent_prediction=all(r['maximum_normalized_rmse']<=config['acceptance']['dynamic_normalized_rmse'] and r['maximum_stationary_interval_excess_nm']<=.01 for r in validation.values()))
    report=dict(stage='inertia',passed=all(checks.values()),checks=checks,initial_fit=initial,joint_refinement=refined,
        refinement_band=dict(lower=lower.tolist(),upper=upper.tolist(),selection='one predeclared bounded fit; validation trajectories never used for selection'),
        validation=validation,initial_validation=initial_validation,prior_stage_acceptance=preserved,
        probe_validation=probe_validation,primary_wrist_row_sqrt_weight=.1,
        preprocessing=training['preprocessing'],velocity_rms_rad_s=minimum_speed.tolist(),acceleration_rms_rad_s2=minimum_acceleration.tolist(),
        model_version='tree_full_physical_inertia_v1',source=source,
        training_trials=['inertia_train']+[name for name,(meta,_) in probes.items() if meta['conditions']['batch']=='train'],
        validation_trials=list(validation)+list(probe_validation),
        limitations=['Only observable parameter combinations identified; physical nullspace representative selected by nominal prior',
            'Fixed waist: no floating-base/waist compliance or fixed torso inertial recovery',
            'Known output-current gain and assumed nominal geometry; no hardware claims'])
    bundle=dict(model_version=report['model_version'],source_sha256=source['sha256'],parameters=theta.tolist(),
        viscous=friction[:14].tolist(),coulomb=friction[14:].tolist(),static_friction=fs.tolist(),
        gain_status=config['drive']['gain_status'],gain_nm_a=config['drive']['gain_nm_a'],
        joint_names=[j.name for j in nominal.model.joints],hardware_executable=False,
        parameter_status=refined['identifiability'],training=report['training_trials'],validation=report['validation_trials'])
    bundle['applicability']=dict(waist='rigidly fixed',geometry='nominal dual-arm URDF; fixture proxy unmeasured',
        control_period_ns=config['control_period_ns'],dynamic_raw_sample_period_ns=config['inertia_sample_period_ns'],
        drive=config['drive'],periodic_fit_band_hz=training['preprocessing']['frequency_band_hz'],
        wrist_probe_frequency_hz=config['wrist_probe']['frequency_hz'],
        observable_combinations_only=True,individual_body_parameters_unique=refined['unique_body_parameters'])
    write_json(output/'candidate.json',bundle);write_json(output/'report.json',report)
    predictions={}
    for name,p in problems.items():
        predicted=p['regression']@theta+p['extra']@friction+p['fixed_friction']+p['rotor']
        predictions[name]=(p['current_torque'],predicted)
        np.savez_compressed(output/(name+'_prediction.npz'),q=p['q'],dq=p['dq'],ddq=p['ddq'],
            observed_current_torque=p['current_torque'],predicted_current_torque=predicted,moving_mask=p['moving'])
    with (output/'parameters.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['parameter','physical_representative','status'])
        writer.writerows((name,value,'not individually identifiable; observable-combination fit') for name,value in zip(nominal.rigid_parameter_names(),theta))
    _plot(output,report)
    _plot_prediction(output,*predictions['inertia_validation'])
    return report,candidate


def _plot_prediction(output,observed,predicted):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    figure=Figure(figsize=(12,9));FigureCanvasAgg(figure);axes=figure.subplots(4,2)
    for row,j in enumerate((3,4,10,11)):
        axes[row,0].plot(observed[:,j],label='Current-derived observed effort',lw=1)
        axes[row,0].plot(predicted[:,j],label='Frozen model prediction',lw=1)
        axes[row,0].set_ylabel(f'Axis {j+1} / Nm');axes[row,1].plot(predicted[:,j]-observed[:,j],lw=1)
        axes[row,1].set_ylabel('Residual / Nm')
        for axis in axes[row]:axis.grid(alpha=.2)
    axes[0,0].legend(fontsize=8);axes[-1,0].set_xlabel('Periodic phase sample');axes[-1,1].set_xlabel('Periodic phase sample')
    figure.tight_layout();figure.savefig(output/'independent_prediction.png',dpi=140)


def _plot(output,report):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    figure=Figure(figsize=(11,6));FigureCanvasAgg(figure);axes=figure.subplots(2,1)
    axes[0].semilogy(np.maximum(report['initial_fit']['singular_values'],1e-16),'o',ms=3)
    axes[0].set(ylabel='Scaled rigid regressor singular values')
    for name,value in report['validation'].items():axes[1].plot([np.nan if x is None else x*100 for x in value['normalized_rmse']],'o-',label=name)
    axes[1].set(xlabel='L1..L7, R1..R7',ylabel='Independent moving torque NRMSE / %');axes[1].legend()
    for axis in axes:axis.grid(alpha=.2)
    figure.tight_layout();figure.savefig(output/'validation.png',dpi=140)
    (output/'report.md').write_text('# 全物理惯性参数阶段\n\n'+f"{'通过' if report['passed'] else '失败'}；完整刚体回归秩 {report['initial_fit']['rank']}/140。\n\n"+
        '前序重力组合冻结；先固定摩擦拟合惯性，再在预设小范围内进行一次联合校正。独立双臂、左臂和右臂轨迹均未用于选择参数，并重新检查前序阶段。M(q)、速度耦合及重力均由Pinocchio生成。\n\n'+
        '[JSON](report.json) · [物理候选](candidate.json) · [参数表](parameters.csv) · [验证图](validation.png)\n\n静止轴按静摩擦区间验收，不声称零速力矩具有唯一值。全部为合成Mock。\n',encoding='utf-8')
