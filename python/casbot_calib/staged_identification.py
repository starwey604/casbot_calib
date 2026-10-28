"""Sequential gravity and motion-friction fits from checked raw observations."""
import csv,hashlib,json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.signal import savgol_filter
from .data import write_json
from .dual_model import nominal_dual_arm
from .rigid_identification import fit_rigid,model_from_parameters
from .staged_collection import load_trial


def session_config(root):
    config=json.loads((Path(root)/'session.json').read_text())['config']
    if 'mock' in config:raise ValueError('Plant truth must not be present in observable session inputs')
    if config['drive']['gain_status'] not in ('synthetic_declared','independently_validated'):raise ValueError('Absolute torque scale is not anchored')
    return config


def recordings(root,stage):
    root=Path(root);plan=json.loads((root/'plan_report.json').read_text())
    if not plan['passed']:raise ValueError('Complete plan has not passed')
    nominal,_=nominal_dual_arm();names=[j.name for j in nominal.model.joints]
    if json.loads((root/'session.json').read_text())['joint_names']!=names:raise ValueError('Session joint order mismatch')
    for name in plan['selected_trials']:
        meta,_=load_trial(root,name)
        if meta['stage']!=stage:continue
        target=root/'runs'/name;record=json.loads((target/'metadata.json').read_text());payload=target/'observations.npz'
        if not record['passed'] or record['schema_version']!=3:raise ValueError('Invalid/rejected acquisition')
        if record['joint_names']!=names or any(record['units'].get(k)!=v for k,v in dict(q='rad',dq='rad/s',current='A',time='int64 ns').items()):
            raise ValueError('Observation joint order/units mismatch')
        if hashlib.sha256(payload.read_bytes()).hexdigest()!=record['raw_sha256']:raise ValueError('Raw observations changed')
        if hashlib.sha256((target/'commands.npz').read_bytes()).hexdigest()!=record['command_sha256']:raise ValueError('Executed command recording changed')
        if 'feedforward_sha256' in record and hashlib.sha256((target/'feedforward.json').read_bytes()).hexdigest()!=record['feedforward_sha256']:
            raise ValueError('Frozen feedforward model changed')
        with np.load(payload,allow_pickle=False) as data:obs={k:data[k] for k in data.files}
        if obs['sample_ns'].dtype!=np.dtype('int64') or any(obs[k].shape!=(len(obs['sample_ns']),14) for k in ('q','dq','current','status')):
            raise ValueError('Observation array schema mismatch')
        if len(obs['sample_ns'])<5 or np.any(np.diff(obs['sample_ns'])<=0) or np.any(np.diff(obs['sequence'])!=1):
            raise ValueError('Missing/repeated samples: segment before differentiation')
        if np.any(np.diff(obs['sample_ns'])!=record['sample_period_ns']):raise ValueError('Nonuniform acquisition times: resample contiguous blocks before uniform-time differentiation')
        if np.any(obs['status']) or any(not np.isfinite(obs[k]).all() for k in ('q','dq','current')):raise ValueError('Invalid/saturated observations')
        yield name,meta,obs


def _windows(meta,label):return [(meta['knots_s'][i],meta['knots_s'][i+1]) for i,x in enumerate(meta['labels']) if x==label]


def _residual(prediction,target,floor=.2):
    error=np.asarray(prediction)-target;rmse=np.sqrt(np.mean(error**2,axis=0));rms=np.sqrt(np.mean(np.asarray(target)**2,axis=0))
    return dict(rmse_nm=rmse.tolist(),bias_nm=np.mean(error,axis=0).tolist(),normalized_rmse=(rmse/np.maximum(rms,floor)).tolist(),
        maximum_absolute_rmse_nm=float(max(rmse)),maximum_normalized_rmse=float(max(rmse/np.maximum(rms,floor))))


def identify_gravity(root,output):
    root=Path(root);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    config=session_config(root);nominal,_=nominal_dual_arm();gain=config['drive']['gain_nm_a'];batches=dict(train=[],validation=[])
    for name,meta,obs in recordings(root,'gravity'):
        a,b=_windows(meta,'static_measure')[0];t=obs['sample_ns']*1e-9
        mask=(t>b-.5)&(t<b-.05)&(np.max(abs(obs['dq']),axis=1)<.002)
        if mask.sum()<100:raise ValueError('Insufficient settled gravity observations: '+name)
        q=obs['q'][mask].mean(axis=0);torque=obs['current'][mask].mean(axis=0)*gain
        batches[meta['conditions']['batch']].append(dict(name=name,q=q,torque=torque,approach=meta['conditions']['approach'],pose=meta['conditions']['pose']))
    def problem(rows):
        q=np.array([x['q'] for x in rows]);z=np.zeros_like(q)
        return nominal.regressor_batch(q,z,z)['regressor'],np.array([x['torque'] for x in rows])
    raw_y,raw_target=problem(batches['train'])
    y=(raw_y[::2]+raw_y[1::2])/2;target=(raw_target[::2]+raw_target[1::2])/2
    fit=fit_rigid(y,target,nominal.rigid_parameters(),robust_scale=.02)
    theta=np.array(fit['parameters']);candidate=model_from_parameters(nominal,theta)
    vy,vt=problem(batches['validation']);raw_validation=_residual(vy@theta,vt,config['acceptance']['torque_floor_nm'])
    validation=_residual(((vy[::2]+vy[1::2])/2)@theta,(vt[::2]+vt[1::2])/2,config['acceptance']['torque_floor_nm'])
    # The two approach directions bracket static reaction/hysteresis. This is
    # retained as uncertainty rather than labelled an independent force reading.
    uncertainty=[]
    for rows in batches.values():
        for i in range(0,len(rows),2):uncertainty.append(abs((rows[i]['torque']-rows[i+1]['torque'])/2))
    spread=np.max(uncertainty,axis=0)
    report=dict(stage='gravity',passed=bool(validation['maximum_absolute_rmse_nm']<=config['acceptance'].get('gravity_pair_rmse_nm',.01) and
            raw_validation['maximum_absolute_rmse_nm']<=config['acceptance']['gravity_absolute_rmse_nm']),
        fit=fit,validation=validation,validation_convention='mean regressor/torque of opposite approaches; individual hold residual reported separately',
        raw_hold_validation=raw_validation,paired_approach_half_spread_nm=spread.tolist(),
        training_trials=[r['name'] for r in batches['train']],validation_trials=[r['name'] for r in batches['validation']],
        selected_representative=theta.tolist(),gain_status=config['drive']['gain_status'],
        limitations=['Static data identifies mass/first-moment combinations, not every individual mass/COM',
            'Stiction reaction is bracketed by approach pairs; absolute torque scale is assumed declared/validated',
            'Inertial-tensor nullspace remains selected by physical nominal prior'])
    write_json(output/'report.json',report)
    np.savez_compressed(output/'gravity_constraints.npz',regressor=y.reshape(-1,140),prediction=(y@theta).ravel(),prior=theta)
    _stage_assets(output,report)
    return report,candidate


def friction_pairs(obs,meta,gravity,gain,control_period_ns=2000000):
    j=meta['conditions']['joint'];speed=meta['conditions']['speed'];t=obs['sample_ns']*1e-9;dt=float(np.median(np.diff(t)))
    stride=round(control_period_ns*1e-9/dt)
    if stride<1 or not np.isclose(stride*dt,control_period_ns*1e-9,rtol=0,atol=1e-9):raise ValueError('Uniform integer-ratio control-cycle averaging required')
    gravity_torque=gravity.gravity_batch(obs['q'])[:,j]
    residual=obs['current'][:,j]*gain-gravity_torque
    count=len(t)//stride
    average=lambda value:value[:count*stride].reshape((count,stride)+value.shape[1:]).mean(axis=1)
    t=average(t);q=average(obs['q']);velocity=average(obs['dq']);residual=average(residual);dt*=stride
    # Remove the drive's control-cycle ripple BEFORE position interpolation.
    # Interpolating alternating current samples on opposing paths can otherwise
    # turn a phase-dependent ripple into a systematic viscous coefficient bias.
    window=2*round(.025/dt)+1
    q=savgol_filter(q,window,3,axis=0);velocity=savgol_filter(velocity,window,3,axis=0)
    residual=savgol_filter(residual,window,3)
    # Differentiate only this verified contiguous trial, then exclude ramp/turn
    # boundaries and samples with appreciable acceleration.
    acceleration=savgol_filter(average(obs['dq']),window,3,deriv=1,delta=dt,axis=0)
    intervals=_windows(meta,'constant_velocity');parts=[]
    for a,b in intervals:
        mask=(t>a+.2)&(t<b-.2)&(abs(abs(velocity[:,j])-speed)<speed*.1)&(abs(acceleration[:,j])<.03)
        if mask.sum()<40:raise ValueError('Insufficient constant-speed plateau')
        position=q[mask,j];order=np.argsort(position);parts.append((position[order],velocity[mask,j][order],residual[mask][order]))
    low=max(p[0].min() for p in parts);high=min(p[0].max() for p in parts)
    if high-low<.02:raise ValueError('Forward/reverse positions do not overlap sufficiently')
    grid=np.linspace(low,high,48);interpolated=[(np.interp(grid,p[0],p[1]),np.interp(grid,p[0],p[2])) for p in parts]
    vp,tp=interpolated[0];vm,tm=interpolated[1]
    features=np.column_stack([(vp-vm)/2,(np.tanh(vp/.001)-np.tanh(vm/.001))/2]);target=(tp-tm)/2
    return features,target,dict(joint=j,requested_speed_rad_s=speed,matched_position_range_rad=[float(low),float(high)],
        maximum_accepted_acceleration_rad_s2=.03,paired_points=len(grid),gravity_error_cancellation='position-matched forward/reverse half difference')


def identify_friction(root,output,gravity,driver_report=None):
    root=Path(root);output=Path(output);output.mkdir(parents=True,exist_ok=False);config=session_config(root)
    rows={b:[[] for _ in range(14)] for b in ('train','validation','sampling')};evidence=[];sample_periods=[];convergence_base=[[] for _ in range(14)]
    for name,meta,obs in recordings(root,'friction'):
        x,y,info=friction_pairs(obs,meta,gravity,config['drive']['gain_nm_a'],config['control_period_ns']);j=meta['conditions']['joint']
        rows[meta['conditions']['batch']][j].append((x,y));evidence.append(dict(trial=name,**info))
        sample_periods.append(int(np.median(np.diff(obs['sample_ns']))))
        if meta['conditions']['batch']=='validation' and meta['conditions']['pose']==0:convergence_base[j].append((x,y))
    viscous=[];coulomb=[];axis=[]
    for j in range(14):
        x=np.vstack([a for a,_ in rows['train'][j]]);y=np.concatenate([b for _,b in rows['train'][j]])
        scale=np.maximum(np.linalg.norm(x,axis=0),1e-12);s=np.linalg.svd(x/scale,compute_uv=False)
        rank=int(np.count_nonzero(s>s[0]*1e-7))
        if rank<2:raise ValueError('Multiple distinct speeds are required to separate viscous and Coulomb friction')
        result=least_squares(lambda p:x@p-y,[.02,.03],bounds=(0,np.inf),loss='soft_l1',f_scale=.01,
            jac=lambda p:x.copy(),xtol=1e-12,ftol=1e-12,gtol=1e-12)
        vx=np.vstack([a for a,_ in rows['validation'][j]]);vy=np.concatenate([b for _,b in rows['validation'][j]])
        rmse=float(np.sqrt(np.mean((vx@result.x-vy)**2)));viscous.append(result.x[0]);coulomb.append(result.x[1])
        axis.append(dict(joint=j,rank=rank,singular_values=s.tolist(),validation_rmse_nm=rmse,success=result.success,
            passed=bool(result.success and rmse<=config['acceptance']['friction_absolute_rmse_nm'])))
    convergence=[]
    for j in range(14):
        fitted=[]
        for source in (convergence_base[j],rows['sampling'][j]):
            if not source:break
            x=np.vstack([a for a,_ in source]);y=np.concatenate([b for _,b in source])
            solution=least_squares(lambda p:x@p-y,[.02,.03],jac=lambda p:x.copy(),bounds=(0,np.inf),loss='soft_l1',f_scale=.01)
            fitted.append(solution.x)
        if len(fitted)==2:
            relative=abs(fitted[0]-fitted[1])/np.maximum(abs(fitted[1]),[.005,.005])
            convergence.append(dict(joint=j,primary_rate_parameters=fitted[0].tolist(),denser_rate_parameters=fitted[1].tolist(),
                relative_difference=relative.tolist(),passed=bool(np.all(relative<=.05))))
        else:convergence.append(dict(joint=j,passed=False,reason='denser independent acquisition missing'))
    driver=driver_report if driver_report is not None else json.loads((root/'driver_timing.json').read_text());tau=min(r['fit']['current_tau_s'] for r in driver['axes'])
    # This criterion is specific to the Mock's unfiltered instantaneous current.
    # At 500-Hz held position commands, 1-kHz point samples alias the intra-cycle
    # acceleration/current ripple into a plausible viscous bias. A low prediction
    # residual at the same sampling phase does not establish parameter validity.
    sampling=bool(driver['passed'] and tau/(max(sample_periods)*1e-9)>=3.8 and all(r['passed'] for r in convergence))
    report=dict(stage='moving_friction',passed=all(r['passed'] for r in axis) and sampling,prediction_passed=all(r['passed'] for r in axis),viscous=viscous,coulomb=coulomb,
        axes=axis,preprocessing=evidence,training_batch='train',validation_batch='validation',
        model='b*dq + fc*tanh(dq/0.001); fit outside the Stribeck band',
        sampling_qualification=dict(passed=sampling,maximum_sample_period_ns=max(sample_periods),minimum_fitted_current_tau_s=tau,
            rate_convergence=convergence,
            samples_per_current_tau=tau/(max(sample_periods)*1e-9),minimum_ratio_with_estimation_tolerance=3.8,
            scope='synthetic instantaneous-current/ZOH protocol only; additionally requires a sampling-rate convergence experiment',
            real_device_requirement='independently validate effort bandwidth, filtering, clock alignment and sampling phase; this rule is not a manufacturer requirement'),
        filtering=dict(control_cycle_average_ns=config['control_period_ns'],savgol_support_s=.05,polynomial_order=3,
            order='synchronous current/gravity residual and kinematics filtering before position pairing'),
        limitations=['Uniform-speed pairing rejects inertial transients; unmodelled acceleration remains a reported contamination bound',
            'No directional/posture-specific friction parameters fitted unless independent residuals require them'])
    write_json(output/'report.json',report);_stage_assets(output,report);return report


def _stage_assets(output,report):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    figure=Figure(figsize=(11,5));FigureCanvasAgg(figure);axes=figure.subplots(2,1)
    with (output/'parameters.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.writer(stream)
        if report['stage']=='gravity':
            writer.writerow(['joint_index','paired_validation_rmse_Nm','individual_hold_rmse_Nm','approach_half_spread_Nm'])
            writer.writerows(zip(range(14),report['validation']['rmse_nm'],report['raw_hold_validation']['rmse_nm'],report['paired_approach_half_spread_nm']))
            axes[0].plot(report['validation']['rmse_nm'],'o-',label='opposite-approach pair average')
            axes[0].plot(report['raw_hold_validation']['rmse_nm'],'o-',label='individual hold (includes static reaction)')
            axes[0].set_ylabel('Validation torque RMSE / Nm');axes[0].legend()
            singular=np.asarray(report['fit']['singular_values']);axes[1].semilogy(np.maximum(singular,1e-16),'o',ms=3)
            axes[1].set(xlabel='Singular value index',ylabel='Scaled gravity regressor')
            text=f"重力阶段：{'通过' if report['passed'] else '失败'}；可观测组合秩 {report['fit']['rank']}/140。\n\n正反接近配对后验收；单次保持的静摩擦反力残差单独保留。质量/质心不具备逐连杆唯一性，完整物理参数只是先验选择的可用代表。"
        else:
            writer.writerow(['joint_index','viscous_Nm_s_rad','coulomb_Nm','validation_rmse_Nm','rank'])
            writer.writerows((j,b,fc,r['validation_rmse_nm'],r['rank']) for j,(b,fc,r) in enumerate(zip(report['viscous'],report['coulomb'],report['axes'])))
            axes[0].plot(report['viscous'],'o-',label='viscous / Nm s rad^-1');axes[0].plot(report['coulomb'],'o-',label='Coulomb / Nm');axes[0].legend()
            axes[1].plot([r['validation_rmse_nm'] for r in report['axes']],'o-');axes[1].set(xlabel='L1..L7, R1..R7',ylabel='Independent paired RMSE / Nm')
            text=f"运动摩擦阶段：{'通过' if report['passed'] else '失败'}。\n\n多构型、三档速度、正反方向的匀速平台按实际位置配对；剔除起停、换向及加速度较大的样本。冻结重力参数，单独验收库仑与黏滞摩擦。"
    for axis in axes:axis.grid(alpha=.2)
    figure.tight_layout();figure.savefig(output/'validation.png',dpi=140)
    (output/'report.md').write_text('# 分阶段动力学辨识\n\n'+text+'\n\n[JSON](report.json) · [CSV](parameters.csv) · [验证图](validation.png)\n\n全部为合成Mock结果，无实机性能承诺。\n',encoding='utf-8')
