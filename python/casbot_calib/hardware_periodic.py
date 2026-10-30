"""H4 periodic SDK observations, explicitly on the callback-availability clock.

The shared Fourier implementation also serves qualified S3 data. This adapter
does not promote callback times to acquisition times or supply missing status.
"""
import numpy as np
from .dynamics_problem import fourier_projection,_project


def periodic_problem(obs,elapsed,motion,engine,config,profile,friction,*,offset=None):
    windows=[(motion['knots_s'][i],motion['knots_s'][i+1]) for i,label in enumerate(motion['labels']) if label=='periodic']
    if not windows:raise ValueError('Missing declared periodic SDK window')
    period=config['period_s'];start=windows[0][0];end=windows[-1][1]
    begin=start+config['discard_periods']*period
    indices=np.flatnonzero((elapsed>=begin)&(elapsed<end))
    if len(indices)<2 or np.any(np.diff(indices)!=1):raise ValueError('Contiguous retained SDK periods required')
    times=obs['receive_ns'][indices]
    if np.max(np.diff(times))*1e-9>config['maximum_gap_s']:raise ValueError('SDK callback gap in periodic window')
    # Trapezoidal time weights prevent dense callback bursts from counting as
    # extra elapsed exposure. They do not assert independent acquisitions.
    dt=np.diff(times)*1e-9;weights=np.r_[dt[0]/2,(dt[:-1]+dt[1:])/2,dt[-1]/2]
    origin=int(times[0])-round((elapsed[indices[0]]-start)*1e9)
    h=config['harmonics'];points=config['phase_points']
    qfit,temporal=fourier_projection(times,obs['q'][indices],origin,period,h,points,True,weights=weights,diagnostics=True)
    velocity,_=fourier_projection(times,obs['dq'][indices],origin,period,h,points,weights=weights)
    effort=(obs['effort_raw'][indices]-np.asarray(profile['effort_zero_nm']))*np.asarray(profile['effort_gain'])
    v=obs['dq'][indices];direction=np.sign(v);transition=np.exp(-(v/.005)**2)*direction
    b,fc,fs=(np.asarray(friction[key]) for key in ('viscous','coulomb','static_friction'))
    # Evaluate nonlinear slip features before projection, as in staged_inertia.
    dry=b*v+fc*(direction-transition)+fs*transition
    shift=config['effort_relative_offset_s'] if offset is None else offset
    torque,torque_temporal=fourier_projection(times,effort,origin,period,h,points,weights=weights,phase_offset_s=shift,diagnostics=True)
    fixed,_=fourier_projection(times,dry,origin,period,h,points,weights=weights)
    q,dq,ddq=qfit;native=engine.regressor_batch(q,dq,ddq)
    regression=_project(native['regressor'],h);rotor=_project(native['rotor_torque'],h)
    mismatch=np.sqrt(np.mean((dq-velocity[0])**2,axis=0))
    if np.max(mismatch)>config['maximum_velocity_consistency_rad_s']:raise ValueError('Measured velocity and analytic position derivative disagree')
    return dict(q=q,dq=dq,ddq=ddq,regression=regression,rotor=rotor,friction=fixed[0],
        observed_torque_nm=torque[0],target=torque[0]-rotor-fixed[0],moving=abs(dq)>config['moving_speed_rad_s'],
        preprocessing=dict(time_basis='SDK callback availability, not acquisition',sample_time_known=False,
            independent_acquisition_count=None,relative_channel_alignment_verified=False,
            configured_effort_phase_offset_s=shift,offset_estimated=False,fit_band_hz=[0,h/period],harmonics=h,
            phase_points=points,discard_periods=config['discard_periods'],temporal_rank=temporal['rank'],
            velocity_consistency_rmse_rad_s=mismatch.tolist(),maximum_callback_gap_s=float(dt.max()),
            callback_publications=len(indices),friction_order='measured velocity -> nonlinear slip features -> matched projection',
            position_unmodelled_callback_rmse_rad=temporal['unmodelled_event_signal_rmse'],
            effort_unmodelled_callback_rmse_nm=torque_temporal['unmodelled_event_signal_rmse'],
            limitations=['Unknown cache age, synchronization and effort filtering remain model mismatch; no delay/low-pass equivalence assumed']))


def prediction_metrics(problem,theta,axes,config):
    predicted=problem['regression']@theta+problem['rotor']+problem['friction']
    error=predicted-problem['observed_torque_nm'];rmse=[];normalized=[];counts=[]
    for j in axes:
        use=problem['moving'][:,j];count=int(use.sum());counts.append(count)
        absolute=float(np.sqrt(np.mean(error[use,j]**2))) if count else None
        scale=max(config['torque_floor_nm'],float(np.sqrt(np.mean(problem['observed_torque_nm'][use,j]**2)))) if count else None
        rmse.append(absolute);normalized.append(absolute/scale if count else None)
    return dict(rmse_nm=rmse,normalized_rmse=normalized,moving_phase_points=counts,
        mean_residual_nm=np.mean(error[:,axes],axis=0).tolist(),
        passed=all(n>=16 and r is not None and r<=config['validation_absolute_rmse_nm'] and p<=config['validation_normalized_rmse'] for n,r,p in zip(counts,rmse,normalized)))
