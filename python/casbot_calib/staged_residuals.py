"""Matched full-period residual attribution; never fits parameters or reads truth."""
import numpy as np
from .staged_inertia import _project

MODELS=('A','B','C','D')


def energy_spectrum(values):
    values=np.asarray(values,dtype=float)
    if values.ndim!=2 or len(values)<4 or not np.isfinite(values).all():
        raise ValueError('A complete finite uniform period [phase,joint] is required')
    coefficients=np.fft.rfft(values,axis=0)/len(values)
    weights=np.full(len(coefficients),2.);weights[0]=1.
    if len(values)%2==0:weights[-1]=1.
    return abs(coefficients)**2*weights[:,None]


def same_band(reference,candidate):
    if reference!=candidate:raise ValueError('Residual comparison requires identical period, grid and retained bandwidth')


def matched_components(model,theta,friction,problem,harmonics):
    native=model.evaluate_batch(problem['q'],problem['dq'],problem['ddq'])
    inertia=_project(native['inertia'],harmonics)
    parts=dict(rigid_inertia=inertia-problem['rotor'],rotor=problem['rotor'],
        velocity=_project(native['velocity'],harmonics),gravity=_project(native['gravity'],harmonics),
        friction=problem['extra']@friction+problem['fixed_friction'])
    prediction=sum(parts.values())
    expected=problem['regression']@theta+problem['rotor']+parts['friction']
    closure=float(np.max(abs(prediction-expected)))
    native_closure=float(np.max(abs(native['total']-native['inertia']-native['velocity']-native['gravity']-native['friction'])))
    if max(closure,native_closure)>1e-10:raise ValueError('Pinocchio component / matched regressor closure failed')
    return parts,prediction,dict(maximum_matched_closure_nm=closure,maximum_native_closure_nm=native_closure,
        rotor_convention='native inertia includes armature; subtract rotor once for rigid term, then add once',
        friction_convention='acquired velocity nonlinear features before shared time projection; native friction at fitted velocity is not substituted')


def residual_statistics(observed,predictions,train_bias=None):
    result={}
    for name,predicted in predictions.items():
        error=np.asarray(predicted)-observed;mean=error.mean(axis=0)
        row=dict(rmse_nm=np.sqrt(np.mean(error**2,axis=0)).tolist(),bias_nm=mean.tolist(),
            demeaned_rmse_nm=np.sqrt(np.mean((error-mean)**2,axis=0)).tolist())
        if train_bias is not None:
            row['training_bias_corrected_diagnostic_rmse_nm']=np.sqrt(np.mean((error-train_bias[name])**2,axis=0)).tolist()
        result[name]=row
    return result


def frequency_budget(observed,predictions,measured_velocity,acceleration,dynamic_torque,inertial_proxy,
                     excitation_harmonics,harmonics,torque_threshold,settings,*,period_s,signatures=None,active_joints=None):
    observed=np.asarray(observed);n,joints=observed.shape
    signature=dict(period_s=period_s,phase_points=n,harmonics=harmonics)
    if signatures is not None:
        for name in MODELS:same_band(signature,signatures[name])
    if set(predictions)!=set(MODELS) or any(np.asarray(p).shape!=observed.shape for p in predictions.values()):
        raise ValueError('A/B/C/D must use exactly the same observed phase grid')
    excitation=np.asarray(excitation_harmonics,dtype=int)
    if len(excitation)==0 or len(set(excitation))!=len(excitation) or min(excitation)<1 or max(excitation)>harmonics or 2*harmonics+1>=n:
        raise ValueError('Invalid frozen excitation/retained harmonics')
    threshold=np.asarray(torque_threshold)
    if threshold.shape!=(harmonics,joints) or not np.isfinite(threshold).all() or np.any(threshold<=0):
        raise ValueError('Frozen positive torque-bin noise thresholds required')
    speed=np.sqrt(energy_spectrum(measured_velocity));acc=np.sqrt(energy_spectrum(acceleration))
    dynamic=np.sqrt(energy_spectrum(dynamic_torque));inertial=np.sqrt(energy_spectrum(inertial_proxy))
    qualified=(speed[excitation]>=settings['minimum_speed_bin_rms_rad_s']) & (acc[excitation]>=settings['minimum_acceleration_bin_rms_rad_s2'])
    qualified &= (dynamic[excitation]>=threshold[excitation-1]) & (inertial[excitation]>=threshold[excitation-1])
    required=np.ones(joints,dtype=bool)
    if active_joints is not None:
        active=np.asarray(active_joints,dtype=int)
        if not len(active) or len(set(active))!=len(active) or min(active)<0 or max(active)>=joints:
            raise ValueError('Invalid declared active joint group')
        required[:]=False;required[active]=True
        qualified[:,~required]=False
    # Both qualification torque signals are frozen against A, independent of the
    # candidate. The inertial proxy prevents gravity/friction residuals alone
    # qualifying an axis; it remains a prior-based proxy, not a torque sensor.
    powers={name:energy_spectrum(np.asarray(predicted)-observed) for name,predicted in predictions.items()}
    other=np.array([h for h in range(1,harmonics+1) if h not in set(excitation)],dtype=int)
    bands={}
    for name,power in powers.items():
        selected=power[excitation]
        bands[name]=dict(dc_rms_nm=np.sqrt(power[0]).tolist(),
            excitation_rms_nm=np.sqrt(selected.sum(axis=0)).tolist(),
            other_retained_rms_nm=np.sqrt(power[other].sum(axis=0)).tolist(),
            total_retained_rms_nm=np.sqrt(power[:harmonics+1].sum(axis=0)).tolist(),
            qualified_excitation_rms_nm=np.sqrt((selected*qualified).sum(axis=0)).tolist(),
            excitation_bin_energy_nm2=selected.tolist())
    counts=qualified.sum(axis=0);enough=counts>=settings['minimum_qualified_bins']
    baseline=np.array(bands['A']['qualified_excitation_rms_nm'])
    limit=np.maximum(settings['band_non_degradation_factor']*baseline,baseline+settings['band_non_degradation_additive_nm'])
    comparisons={}
    for name in ('B','D'):
        values=np.array(bands[name]['qualified_excitation_rms_nm'])
        comparisons[name]=dict(axis_non_degradation=[bool(values[j]<=limit[j]) if enough[j] else None for j in range(joints)],
            axis_limit_nm=limit.tolist(),qualified_subset_passed=bool(enough.any() and np.all(values[enough]<=limit[enough])))
    result=dict(signature=signature,excitation_harmonics=excitation.tolist(),other_retained_harmonics=other.tolist(),
        frequency_hz=(excitation/period_s).tolist(),qualified_bins=qualified.tolist(),qualified_bins_per_axis=counts.tolist(),
        axis_sufficient=enough.tolist(),full_domain_evidence=bool(enough[required].all()),
        measured_speed_bin_rms_rad_s=speed[excitation].tolist(),measured_acceleration_bin_rms_rad_s2=acc[excitation].tolist(),
        observed_dynamic_bin_rms_nm=dynamic[excitation].tolist(),frozen_A_inertial_proxy_bin_rms_nm=inertial[excitation].tolist(),
        torque_threshold_bin_rms_nm=threshold[excitation-1].tolist(),bands=bands,comparisons=comparisons,
        qualified_non_degradation_passed=all(r['qualified_subset_passed'] for r in comparisons.values()),
        energy_convention='one-sided Parseval sum, DC/Nyquist weight 1, other bins weight 2; no division by bin count; complete phase before moving-mask scoring',
        qualification_convention='measured velocity + position-derived acceleration + observed dynamic residual + frozen A inertia proxy; no candidate-dependent frequency selection')
    if active_joints is not None:result['active_joints']=active.tolist()
    return result


def aggregate_benefit(budgets,settings):
    if not budgets:raise ValueError('Complete trajectory budgets are required')
    means={name:[] for name in MODELS};missing=[]
    for trial,budget in budgets.items():
        good=np.asarray(budget['axis_sufficient'],dtype=bool)
        required=np.zeros(len(good),dtype=bool);required[budget.get('active_joints',list(range(len(good))))]=True
        good=good&required
        missing.extend(dict(trial=trial,joint=j) for j in np.flatnonzero(required&~good).tolist())
        for name in MODELS:
            values=np.asarray(budget['bands'][name]['qualified_excitation_rms_nm'])
            means[name].append(float(values[good].mean()) if good.any() else None)
    evaluable=all(all(v is not None for v in row) for row in means.values())
    avg={name:float(np.mean(row)) if all(v is not None for v in row) else None for name,row in means.items()}
    base=avg['A'];gain={}
    for name in ('B','D'):
        gain[name]=None if base is None or base<=1e-12 else (base-avg[name])/base
    non_degradation=all(b['qualified_non_degradation_passed'] for b in budgets.values())
    full=not missing and evaluable
    b_gain=gain['B'] is not None and gain['B']>=settings['minimum_inertia_band_improvement_fraction']
    d_gain=gain['D'] is not None and gain['D']>=settings['minimum_inertia_band_improvement_fraction']
    status=('insufficient_signal' if not full else 'inertia_update_benefit' if b_gain and non_degradation else
            'joint_model_benefit_only' if d_gain and non_degradation else 'prediction_equivalence' if non_degradation else 'band_degradation')
    return dict(status=status,full_domain_evidence=full,missing_axis_evidence=missing,
        mean_qualified_band_rmse_nm=avg,improvement_fraction=gain,qualified_non_degradation_passed=non_degradation,
        inertia_benefit_claim_supported=bool(full and b_gain and non_degradation),
        qualified_subset_inertia_improvement=bool(evaluable and b_gain and non_degradation),
        weighting='equal trajectories, then equal sufficient axes within each trajectory; missing axes retained and forbid a full-domain claim',
        causality_note='A->B->C->D is an ordered decomposition; cancellation is possible, not unique causal attribution')


def raw_period_spectra(obs,meta,config):
    windows=[(meta['knots_s'][i],meta['knots_s'][i+1]) for i,label in enumerate(meta['labels']) if label=='periodic']
    period_ns=round(meta['conditions']['period_s']*1e9);step=config['inertia_sample_period_ns']
    if period_ns%step:raise ValueError('Integer raw samples per full period required')
    begin=round(windows[0][0]*1e9)+config['p0']['preprocessing']['discard_periods']*period_ns
    end=round(windows[-1][1]*1e9);mask=(obs['sample_ns']>=begin)&(obs['sample_ns']<end)
    times=obs['sample_ns'][mask];n=period_ns//step
    if len(times)%n or len(times)<2*n or not np.array_equal(times,np.arange(begin,end,step,dtype=np.int64)):
        raise ValueError('Raw diagnostic requires complete contiguous steady periods')
    current=obs['current'][mask]*config['drive']['gain_nm_a'];velocity=obs['dq'][mask]
    periods=len(times)//n
    return dict(current_coefficients=np.fft.rfft(current.reshape(periods,n,14),axis=1)/n,
        measured_velocity_coefficients=np.mean(np.fft.rfft(velocity.reshape(periods,n,14),axis=1)/n,axis=0),
        samples_per_period=n,periods=periods,nyquist_hz=1e9/(2*step))


def raw_residual_budget(raw,predictions,harmonics):
    observed=raw['current_coefficients'];weights=np.full(observed.shape[1],2.);weights[0]=1.
    if raw['samples_per_period']%2==0:weights[-1]=1.
    result={}
    for name,predicted in predictions.items():
        coefficients=np.fft.rfft(predicted,axis=0)/len(predicted)
        error=observed.copy();error[:,:harmonics+1]-=coefficients[None,:harmonics+1]
        power=abs(error)**2*weights[None,:,None]
        result[name]=dict(raw_full_period_rmse_nm=np.sqrt(np.mean(power.sum(axis=1),axis=0)).tolist(),
            raw_outside_retained_rmse_nm=np.sqrt(np.mean(power[:,harmonics+1:].sum(axis=1),axis=0)).tolist(),
            raw_period_bias_nm=error[:,0].real.tolist())
    return dict(models=result,periods=raw['periods'],raw_nyquist_hz=raw['nyquist_hz'],
        convention='raw current minus retained-band model; average energy across complete periods, not averaged residual samples; outside-band content includes drive/stiction/noise and is not a full-band native-model validation')
