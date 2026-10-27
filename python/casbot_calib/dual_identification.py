"""Observation-only identification in an explicitly restricted physical subspace."""
import numpy as np
from scipy.optimize import least_squares
from .dual_model import body_basis


def load_observations(folder,expected_names=None):
    import json
    import hashlib
    from pathlib import Path
    folder=Path(folder);schema=json.loads((folder/'observation_schema.json').read_text(encoding='utf-8'))
    if schema.get('schema_version')!=2 or schema.get('units')!={'q':'rad','dq':'rad/s','current':'A','time':'int64 ns'}:
        raise ValueError('Expected dual-arm schema 2 with SI units')
    if expected_names is not None and schema['joint_names']!=expected_names:raise ValueError('Observation joint order mismatch')
    metadata=json.loads((folder/'metadata.json').read_text(encoding='utf-8'))
    if hashlib.sha256((folder/'observations.npz').read_bytes()).hexdigest()!=metadata['raw_sha256']:
        raise ValueError('Raw observation digest mismatch')
    with np.load(folder/'observations.npz',allow_pickle=False) as data:obs={k:data[k] for k in data.files}
    n=len(schema['joint_names']);count=len(obs['sample_ns'])
    if n!=14 or any(obs[k].shape!=(count,n) for k in ('q','dq','current','status')):raise ValueError('Invalid 14-axis observations')
    for key in ('sample_ns','receive_ns','sequence'):
        if obs[key].shape!=(count,) or obs[key].dtype!=np.dtype('int64') or np.any(obs[key]<0):
            raise ValueError('Expected nonnegative int64 acquisition/receive times and sequences')
    if np.any(obs['receive_ns']<obs['sample_ns']) or obs['status'].dtype.kind not in 'iu' or np.any(obs['status']<0):
        raise ValueError('Invalid observation timestamps/status')
    return obs


def valid_observations(obs):
    mask=(obs['status']==0).all(axis=1)
    for key in ('q','dq','current'):mask&=np.isfinite(obs[key]).all(axis=1)
    clean={k:v[mask] for k,v in obs.items()};order=np.argsort(clean['sample_ns'])
    clean={k:v[order] for k,v in clean.items()}
    if len(order)<40 or np.any(np.diff(clean['sample_ns'])<=0):raise ValueError('Insufficient valid, distinct sample timestamps')
    return clean


def periodic_observations(obs,period_s,discard_periods,harmonics=24,phase_points=512):
    """Fit actual positions/current at acquisition time, then analytically differentiate.

    Missing samples enter a timestamped regression; no finite differences cross gaps.
    """
    if period_s<=0 or discard_periods<0 or harmonics<1 or 2*harmonics+1>=phase_points:raise ValueError('Invalid Fourier configuration')
    obs=valid_observations(obs);t=obs['sample_ns']*1e-9;keep=t>=period_s*discard_periods
    t=t[keep];q=obs['q'][keep];current=obs['current'][keep]
    if len(t)<4*(2*harmonics+1) or np.ptp(t)<1.8*period_s:raise ValueError('Need at least two steady periods')
    def design(times,derivative=0):
        columns=[np.ones(len(times)) if derivative==0 else np.zeros(len(times))]
        for k in range(1,harmonics+1):
            w=2*np.pi*k/period_s
            columns.extend([w**derivative*np.sin(w*times+derivative*np.pi/2),w**derivative*np.cos(w*times+derivative*np.pi/2)])
        return np.column_stack(columns)
    basis=design(t);coef=np.linalg.lstsq(basis,np.column_stack([q,current]),rcond=1e-10)[0]
    if np.linalg.matrix_rank(basis)<basis.shape[1]:raise ValueError('Rank-deficient temporal Fourier fit')
    phase=np.arange(phase_points)*period_s/phase_points;n=q.shape[1]
    return dict(q=design(phase)@coef[:,:n],dq=design(phase,1)@coef[:,:n],ddq=design(phase,2)@coef[:,:n],
        current=design(phase)@coef[:,n:],phase_s=phase,harmonics=harmonics)


def feature_matrix(nominal,q,v,a,smoothing_velocity=.03,project_harmonics=None):
    reg=nominal.regressor_batch(q,v,a)['regressor'];mass=reg@body_basis(nominal);n=v.shape[1]
    features=np.zeros((len(q),n,3*n));features[:,:,:n]=mass
    for j in range(n):features[:,j,n+j]=v[:,j];features[:,j,2*n+j]=np.tanh(v[:,j]/smoothing_velocity)
    if project_harmonics is not None:
        frequency=np.fft.rfft(features,axis=0);frequency[project_harmonics+1:]=0.
        features=np.fft.irfft(frequency,n=len(q),axis=0)
    return features,reg


def fit_parameters(features,torque,start,active=None):
    """Scaled SVD plus bounded robust fit. No Mock model/parameter truth inputs."""
    x=features.reshape(-1,features.shape[-1]);y=torque.reshape(-1);start=np.asarray(start,dtype=float)
    active=np.arange(len(start)) if active is None else np.asarray(active)
    fixed=x@start-x[:,active]@start[active];x=x[:,active];y=y-fixed
    scales=np.linalg.norm(x,axis=0);safe=np.where(scales>1e-12,scales,1.);scaled=x/safe
    _,singular,vt=np.linalg.svd(scaled,full_matrices=False);rank=int(np.count_nonzero(singular>singular[0]*1e-7))
    n=len(start)//3;lower=np.r_[np.full(n,.2),np.zeros(2*n)][active];upper=np.r_[np.full(n,3.),np.ones(2*n)][active]
    fit=least_squares(lambda p:x@p-y,np.clip(start[active],lower+1e-10,upper-1e-10),
        jac=lambda p:x,bounds=(lower,upper),loss='soft_l1',f_scale=.02,x_scale=1./safe,max_nfev=100,
        ftol=1e-10,xtol=1e-10,gtol=1e-10)
    parameters=start.copy();parameters[active]=fit.x
    null=vt[rank:] if rank<len(active) else np.empty((0,len(active)))
    return dict(parameters=parameters.tolist(),success=bool(fit.success),rank=rank,columns=len(active),
        singular_values=singular.tolist(),column_scales=scales.tolist(),rmse_nm=float(np.sqrt(np.mean((x@fit.x-y)**2))),
        individually_observable=(np.sum(null**2,axis=0)<1e-6).tolist(),
        identifiability='full_rank_in_restricted_subspace' if rank==len(active) else 'rank_deficient_prediction_candidate_only',
        parameterization='positive body mass/inertia scales with fixed nominal COM/shape; viscous and smooth Coulomb friction')


def residual_metrics(predicted,observed,floor=.2):
    error=predicted-observed;rmse=np.sqrt(np.mean(error**2,axis=0));denom=np.maximum(np.sqrt(np.mean(observed**2,axis=0)),floor)
    return dict(rmse_nm=rmse.tolist(),normalized_rmse=(rmse/denom).tolist(),normalization_floor_nm=floor,
        maximum_normalized_rmse=float(np.max(rmse/denom)))
