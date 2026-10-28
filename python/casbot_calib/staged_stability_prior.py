"""Cache exact linear projections for the unchanged predecessor development gates."""
from pathlib import Path
import numpy as np
from .staged_identification import recordings, _windows, _residual, friction_pairs, friction_pair_projection


def gravity_regressor(engine,q,axis=None):
    q=np.asarray(q);blocks=[]
    for begin in range(0,len(q),512):
        part=q[begin:begin+512];zero=np.zeros_like(part)
        y=engine.regressor_batch(part,zero,zero)['regressor']
        blocks.append(y if axis is None else y[:,axis,:].copy())
    return np.concatenate(blocks,axis=0)


def prepare_prior_cache(root,engine,theta_g,config):
    gy=[];gt=[];fx=[];fy=[];fg=[];fj=[];onset=[];direct_error=0.
    from .rigid_identification import model_from_parameters
    original=model_from_parameters(engine,theta_g)
    for _,meta,obs in recordings(root,'gravity'):
        if meta['conditions']['batch']!='validation':continue
        _,end=_windows(meta,'static_measure')[0];t=obs['sample_ns']*1e-9
        mask=(t>end-.5)&(t<end-.05)&(np.max(abs(obs['dq']),axis=1)<.002)
        gy.append(obs['q'][mask].mean(axis=0));gt.append(obs['current'][mask].mean(axis=0)*config['drive']['gain_nm_a'])
    gravity=gravity_regressor(engine,np.array(gy))
    for name,meta,obs in recordings(root,'friction'):
        if meta['conditions']['batch']!='validation':continue
        j=meta['conditions']['joint']
        x,current,_=friction_pair_projection(obs,meta,obs['current'][:,j]*config['drive']['gain_nm_a'],config['control_period_ns'])
        _,regression,_=friction_pair_projection(obs,meta,gravity_regressor(engine,obs['q'],j),config['control_period_ns'])
        # Verify the cached matrix projection against the original native-gravity
        # scalar path for every predecessor scan, not merely one round trip.
        direct_x,direct_y,_=friction_pairs(obs,meta,original,config['drive']['gain_nm_a'],config['control_period_ns'])
        error=max(float(np.max(abs(x-direct_x))),float(np.max(abs(current-regression@theta_g-direct_y))))
        direct_error=max(direct_error,error)
        if error>1e-10:raise ValueError('Cached predecessor projection differs from the original gate: '+name)
        fx.append(x);fy.append(current);fg.append(regression);fj.extend([j]*len(x))
    for _,meta,obs in recordings(root,'breakaway'):
        begin,end=_windows(meta,'torque_ramp')[0];t=obs['sample_ns']*1e-9
        onset.append(gravity_regressor(engine,obs['q'][(t>=begin)&(t<=end)][::25]).reshape(-1,140))
    return dict(gravity_regressor=gravity,gravity_target=np.array(gt),friction_features=np.vstack(fx),
        friction_current=np.concatenate(fy),friction_gravity=np.vstack(fg),friction_joint=np.asarray(fj,dtype=np.int64),
        onset_regressor=np.vstack(onset),original_theta=np.asarray(theta_g),direct_projection_maximum_error=np.array(direct_error))


def check_prior_cache(cache,theta,friction,static,acceptance):
    g=cache['gravity_regressor']@theta;target=cache['gravity_target']
    gravity=_residual((g[::2]+g[1::2])/2,(target[::2]+target[1::2])/2)
    j=cache['friction_joint'];features=cache['friction_features']
    error=features[:,0]*friction[j]+features[:,1]*friction[14+j]-cache['friction_current']+cache['friction_gravity']@theta
    rmse=[float(np.sqrt(np.mean(error[j==axis]**2))) for axis in range(14)]
    onset=float(np.max(abs(cache['onset_regressor']@(theta-cache['original_theta']))))
    return dict(passed=bool(gravity['maximum_absolute_rmse_nm']<=acceptance['gravity_pair_rmse_nm'] and max(rmse)<=acceptance['friction_absolute_rmse_nm'] and onset<=1e-5),
        gravity=gravity,moving_friction_rmse_nm=rmse,maximum_breakaway_gravity_change_nm=onset,
        static_friction_frozen=np.asarray(static).tolist(),cached_projection_maximum_error=float(cache['direct_projection_maximum_error']),
        scope='exact same public predecessor development gates via matched linear projection; not final holdout acceptance')
