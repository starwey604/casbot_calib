"""Physical static-friction intervals from raw, synchronized observations.

Fourier truncation does not preserve pointwise bounds. A zero crossing in a
moving chain is not a static pose: qualify complete fixed-base branches using
measured motion, then predict their gravity with Pinocchio.
"""
import numpy as np
from scipy.ndimage import maximum_filter1d,minimum_filter1d
from .torque_observations import output_torque

POLICY=dict(version='raw_static_branch_windows_v1',window_s=.02,maximum_speed_rad_s=1e-5,
            maximum_position_span_rad=2e-6,minimum_samples_per_axis=32)


def fixed_base_branches(engine):
    n=len(engine.model.joints);parents=list(engine.model.parents) or list(range(-1,n-1));groups={}
    for j in range(n):
        root=j
        while parents[root]>=0:root=parents[root]
        groups.setdefault(root,[]).append(j)
    return list(groups.values())


def observation_problem(engine,observations,gain=None):
    q=np.asarray(observations['q']);v=np.asarray(observations['dq']);ns=np.asarray(observations['sample_ns'])
    n=len(engine.model.joints);groups=fixed_base_branches(engine)
    empty=dict(geometry=engine,q=np.empty((0,n)),observed=np.empty((0,n)),quiet=np.empty((0,n),dtype=bool),
        time_ns=np.empty(0,dtype=np.int64),policy=dict(POLICY),branches=groups,
        windows=[[] for _ in range(n)],counts=[0]*n,eligible=False)
    if len(ns)<4 or q.shape!=(len(ns),n) or v.shape!=q.shape or not np.isfinite(q).all() or not np.isfinite(v).all():
        raise ValueError('Static interval requires finite raw position/velocity observations')
    torque=output_torque(observations,gain)
    if torque.shape!=q.shape or not np.isfinite(torque).all():raise ValueError('Static interval requires one finite output torque observation')
    delta=np.diff(ns)
    if np.any(delta<=0) or np.any(delta!=delta[0]):
        return dict(empty,reason='raw acquisition is not contiguous uniform time; no physical static qualification across gaps')
    if np.any(observations.get('status',0)):return dict(empty,reason='invalid raw observation status')
    width=int(np.ceil(POLICY['window_s']*1e9/int(delta[0])))+1
    if width%2==0:width+=1
    if len(ns)<width:return dict(empty,reason='raw record shorter than stationary window')
    speed=maximum_filter1d(abs(v),size=width,axis=0,mode='constant',cval=np.inf)
    high=maximum_filter1d(q,size=width,axis=0,mode='constant',cval=np.inf)
    low=minimum_filter1d(q,size=width,axis=0,mode='constant',cval=-np.inf)
    axis_still=(speed<=POLICY['maximum_speed_rad_s'])&((high-low)<=POLICY['maximum_position_span_rad'])
    quiet=np.zeros_like(axis_still)
    # A shared moving ancestor makes its entire branch ineligible. Distinct
    # branches are dynamically independent only through the fixed model base;
    # thus a parked arm can still be tested while the other arm moves.
    for group in groups:quiet[:,group]=np.all(axis_still[:,group],axis=1)[:,None]
    selected=np.flatnonzero(np.any(quiet,axis=1));windows=[]
    for j in range(n):
        changes=np.diff(np.r_[False,quiet[:,j],False].astype(int));begins=np.flatnonzero(changes==1);ends=np.flatnonzero(changes==-1)
        windows.append([dict(begin_ns=int(ns[a]),end_ns=int(ns[b-1]),samples=int(b-a)) for a,b in zip(begins,ends)])
    counts=np.sum(quiet,axis=0).astype(int).tolist();eligible=all(c>=POLICY['minimum_samples_per_axis'] for c in counts)
    return dict(empty,q=q[selected].copy(),observed=torque[selected].copy(),quiet=quiet[selected],time_ns=ns[selected],
        windows=windows,counts=counts,eligible=eligible,
        reason='stationary branch windows qualified' if eligible else 'insufficient genuinely stationary branch observations on one or more axes',
        window_span_s=(width-1)*int(delta[0])*1e-9,raw_sample_period_ns=int(delta[0]),
        scope='all raw trajectory holds; complete fixed-base branches must be stationary; no torque filtering or acceleration differentiation')


def metrics(problem,theta,static):
    from .rigid_identification import model_from_parameters
    model=model_from_parameters(problem['geometry'],theta)
    residual=model.gravity_batch(problem['q'])-problem['observed'] if len(problem['q']) else np.empty_like(problem['observed'])
    excess=[]
    for j,count in enumerate(problem['counts']):
        value=residual[problem['quiet'][:,j],j]
        excess.append(float(np.max(np.maximum(abs(value)-static[j],0))) if count else None)
    return dict(stationary_interval_excess_nm=excess,maximum_stationary_interval_excess_nm=max((e for e in excess if e is not None),default=0.),
        stationary_qualified=problem['eligible'],stationary_convention='raw gravity +/- frozen operational static friction on genuinely stationary fixed-base branches',
        stationary_observations={k:v for k,v in problem.items() if k not in ('geometry','q','observed','quiet','time_ns')})
