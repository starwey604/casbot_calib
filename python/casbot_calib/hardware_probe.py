"""Non-reversing SDK wrist observations on callback availability time.

A finite-window Fourier basis includes the nonperiodic position drift. Reported
velocity is fitted before analytic differentiation; cached values are never
finite-differenced or deduplicated. Force and all balance columns share one
projection. No device/ROS I/O is implemented here.
"""
from pathlib import Path
from copy import deepcopy
from math import factorial
import numpy as np
import yaml
from .staged_motion import wrist_probe_motion


def settings(path='configs/sdk_wrist_probe.yaml'):
    c = yaml.safe_load(Path(path).read_text())
    if c.get('schema_version') not in ('casbot_sdk_wrist_probe_v1','casbot_sdk_wrist_probe_v2'):
        raise ValueError('Explicit SDK non-reversing wrist configuration required')
    if type(c['cache_period_ns']) is not int or c['cache_period_ns']<=0:
        raise ValueError('Positive integer cache period required')
    p=c['processing']; probe=c['probe']
    if (p['polynomial_degree']!=2 or not 0<p['fit_band_hz'] or
        not 0<p['edge_trim_s']<probe['duration_s']/2 or
        not 0<p['quadrature_period_s']<.5/p['fit_band_hz'] or
        (legacy_quality(c) and any(not np.isfinite(p[k]) or p[k]<=0 for k in (
            'maximum_callback_gap_s','minimum_signed_velocity_rad_s',
            'maximum_held_velocity_rad_s','maximum_velocity_consistency_rad_s')))):
        raise ValueError('Invalid finite-window wrist processing settings')
    return c



def legacy_quality(config):
    """Schema1 preserves the explicit experimental contract; field schema2 is ungated."""
    return config['schema_version']=='casbot_sdk_wrist_probe_v1'


def field_probe_config(config):
    """New field observations keep numerical processing, not empirical gates."""
    c=deepcopy(config);c['schema_version']='casbot_sdk_wrist_probe_v2'
    for key in ('maximum_callback_gap_s','minimum_signed_velocity_rad_s','maximum_held_velocity_rad_s','maximum_velocity_consistency_rad_s'):
        c['processing'].pop(key,None)
    return c

def make_probe(name, home, pose, joint, direction, *, config=None, transition_s=4., settle_s=3.):
    """Use the existing C3 waveform and its original dedicated probe envelope."""
    c=settings() if config is None else config
    recipe=dict(wrist_probe=c['probe'],transition_s=transition_s,settle_s=settle_s)
    motion=wrist_probe_motion(name,home,pose,recipe,joint,direction,
        dict(refine_position_bounds=True,observation_protocol='sdk_callback_nonreversing_v1'))
    # The native recipe carries a4kHz planning hint. It is NOT an SDK sampling
    # period and must not leak into the callback observation contract.
    motion.conditions.pop('sample_period_ns')
    return motion


def basis(time_s, begin, duration, harmonics, degree=2, derivative=0):
    t=np.asarray(time_s)-begin; u=2*t/duration-1
    columns=[]
    for k in range(degree+1):
        columns.append(np.zeros_like(t) if k<derivative else
            factorial(k)/factorial(k-derivative)*(2/duration)**derivative*u**(k-derivative))
    for k in range(1,harmonics+1):
        omega=2*np.pi*k/duration; angle=omega*t+derivative*np.pi/2
        columns.extend([omega**derivative*np.sin(angle),omega**derivative*np.cos(angle)])
    return np.column_stack(columns)


def exposure(time_s):
    dt=np.diff(time_s)
    if len(dt)==0 or np.any(dt<=0):raise ValueError('Strictly increasing SDK callback times required')
    return np.r_[dt[0]/2,(dt[:-1]+dt[1:])/2,dt[-1]/2]


def least_squares(design, values, weights):
    scale=np.sqrt(weights);x=design*scale[:,None]
    coef,_,rank,singular=np.linalg.lstsq(x,np.asarray(values)*scale[:,None],rcond=1e-10)
    if rank!=design.shape[1]:raise ValueError('Insufficient callback temporal rank for declared probe band')
    return coef,singular


def probe_problem(obs,elapsed,motion,engine,config,profile,*,effort_relative_offset_s=0.):
    """Build the shared rigid/probe fit interface from public SDK feedback only."""
    c=config['processing']; elapsed=np.asarray(elapsed)
    if not np.isfinite(effort_relative_offset_s) or abs(effort_relative_offset_s)>=c['edge_trim_s']:
        raise ValueError('Effort diagnostic offset must remain inside the trimmed measurement window')
    windows=[(motion['knots_s'][i],motion['knots_s'][i+1])
             for i,l in enumerate(motion['labels']) if l=='probe_measure']
    if not windows:raise ValueError('Declared non-reversing probe window required')
    begin,end=windows[0][0],windows[-1][1]; duration=end-begin
    joint=motion['conditions']['joint'];sign=motion['conditions']['direction']
    ids=np.flatnonzero((elapsed>=begin)&(elapsed<end))
    if len(ids)<2 or np.any(np.diff(ids)!=1):raise ValueError('Contiguous public probe window required')
    times=(obs['receive_ns'][ids]-obs['receive_ns'][ids[0]])*1e-9+elapsed[ids[0]]
    w=exposure(times)
    if legacy_quality(config) and (times[0]-begin>c['maximum_callback_gap_s'] or end-times[-1]>c['maximum_callback_gap_s'] or np.max(np.diff(times))>c['maximum_callback_gap_s']):
        raise ValueError('Incomplete SDK callback coverage in non-reversing window')
    rawv=obs['dq'][ids];held=[j for j in range(14) if j!=joint]
    if legacy_quality(config) and np.min(sign*rawv[:,joint])<=c['minimum_signed_velocity_rad_s']:
        raise ValueError('Public reported velocity reversed or stopped in probe window')
    if legacy_quality(config) and np.max(abs(rawv[:,held]))>c['maximum_held_velocity_rad_s']:
        raise ValueError('Other axes moved during single-axis SDK probe')
    h=int(np.floor(c['fit_band_hz']*duration+1e-9)); degree=c['polynomial_degree']
    b=basis(times,begin,duration,h,degree)
    torque=(obs['effort_raw'][ids]-np.asarray(profile['effort_zero_nm']))*np.asarray(profile['effort_gain'])
    channels=np.column_stack([obs['q'][ids],rawv,torque])
    coefficients,singular=least_squares(b,channels,w)
    # Full-window quadrature is a numerical evaluation grid, not new samples.
    grid=np.arange(begin,end,c['quadrature_period_s'])
    be=basis(grid,begin,duration,h,degree);bd=basis(grid,begin,duration,h,degree,1)
    fit=be@coefficients;q=fit[:,:14];v=fit[:,14:28];a=bd@coefficients[:,14:28]
    qvelocity=bd@coefficients[:,:14]
    retain=(grid>=begin+c['edge_trim_s'])&(grid<end-c['edge_trim_s'])
    mismatch=np.sqrt(np.mean((qvelocity[retain]-v[retain])**2,axis=0))
    if legacy_quality(config) and np.max(mismatch)>c['maximum_velocity_consistency_rad_s']:
        raise ValueError('Actual position derivative and reported velocity disagree')
    if legacy_quality(config) and np.min(sign*v[retain,joint])<=c['minimum_signed_velocity_rad_s']:
        raise ValueError('Reconstructed probe did not sustain signed motion')
    # Evaluate the model on the SAME callback rows as force, then use the
    # identical weighted temporal operator. Grid rows are only output quadrature.
    source=b@coefficients
    source_a=basis(times,begin,duration,h,degree,1)@coefficients[:,14:28]
    native=engine.regressor_batch(source[:,:14],source[:,14:28],source_a)
    yc,_=least_squares(b,native['regressor'][:,joint],w);regression=be@yc
    rc,_=least_squares(b,native['rotor_torque'][:,joint,None],w);rotor=(be@rc)[:,0]
    # Nonlinear friction features originate in public velocity BEFORE projection.
    speed=rawv[:,joint];direction=np.sign(speed);decay=np.exp(-(speed/.005)**2)
    friction_basis=np.column_stack([speed,direction*(1-decay),direction*decay])
    fc,_=least_squares(b,friction_basis,w);features=be@fc
    extra=np.zeros((len(grid),28));extra[:,joint]=features[:,0];extra[:,14+joint]=features[:,1]
    gc,_=least_squares(b,engine.gravity_batch(source[:,:14])[:,joint,None],w);gravity=(be@gc)[:,0]
    residual=b@coefficients-channels
    observed=fit[:,28+joint] if effort_relative_offset_s==0 else (
        basis(grid+effort_relative_offset_s,begin,duration,h,degree)@coefficients[:,28+joint])
    return dict(regression=regression[retain],extra=extra[retain],target=(observed-rotor)[retain],
        static_feature=features[retain,2],observed_torque_nm=observed[retain],rotor=rotor[retain],
        projected_gravity_nm=gravity[retain],q=q[retain],dq=v[retain],ddq=a[retain],joint=joint,
        time_s=grid[retain],preprocessing=dict(time_basis='SDK callback availability, not acquisition',
            sample_time_known=False,independent_acquisition_count=None,cache_standard_ns=config['cache_period_ns'],
            cache_age_measured=False,relative_channel_alignment_verified=False,
            imposed_effort_evaluation_offset_s=effort_relative_offset_s,offset_estimated=False,
            temporal_basis='degree2 trend plus complete Fourier band on full non-reversing window',
            frequency_band_hz=[0,h/duration],basis_columns=b.shape[1],temporal_condition=float(singular[0]/singular[-1]),
            matched_balance_operator='identical callback rows, exposure weights and trend/Fourier design for force and all model columns',
            acceleration='analytic derivative of fitted public reported velocity; no cached finite differences',
            maximum_callback_gap_s=float(np.max(np.diff(times))),callback_publications=len(ids),
            empirical_quality_gate_applied=legacy_quality(config),evaluation_grid_is_acquisition=False,edge_trim_s=c['edge_trim_s'],
            target_definition='estimated output Nm minus projected rotor; subtract frozen fs * static_feature before rigid/b-fc fit',
            minimum_signed_velocity_rad_s=float(np.min(sign*rawv[:,joint])),
            maximum_held_velocity_rad_s=float(np.max(abs(rawv[:,held]))),
            velocity_consistency_rmse_rad_s=mismatch.tolist(),
            unmodelled_position_callback_rmse_rad=np.sqrt(np.average(residual[:,:14]**2,axis=0,weights=w)).tolist(),
            unmodelled_velocity_callback_rmse_rad_s=np.sqrt(np.average(residual[:,14:28]**2,axis=0,weights=w)).tolist(),
            unmodelled_effort_callback_rmse_nm=np.sqrt(np.average(residual[:,28:]**2,axis=0,weights=w)).tolist()))


def inertia_balance(problem):
    """Probe diagnostic: infer inertia with velocity, offset and drift nuisance."""
    p=problem;j=p['joint']; t=p['time_s']-np.mean(p['time_s'])
    x=np.column_stack([p['ddq'][:,j],p['dq'][:,j],np.ones(len(t)),t,t*t])
    scales=np.linalg.norm(x,axis=0)
    if np.any(scales==0) or not np.isfinite(scales).all():raise ValueError('Local inertia balance has unidentifiable columns')
    coef=np.linalg.lstsq(x/scales,p['target']-p['projected_gravity_nm'],rcond=1e-10)[0]/scales
    residual=x@coef-(p['target']-p['projected_gravity_nm'])
    return dict(inertia_kg_m2=float(coef[0]),viscous_nm_s_rad=float(coef[1]),
                residual_rms_nm=float(np.sqrt(np.mean(residual**2))),
                method='public acceleration + reported velocity + constant/linear/quadratic nuisance; projected gravity removed')
