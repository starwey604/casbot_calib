"""Offline roll frequency design on public observations; no device/plant I/O.

Complex convention: x(t) = Re[Z exp(i*w*(t-origin))]. A positive delta
advances the effort channel, matching hardware_probe's diagnostic convention.
"""
from copy import deepcopy
import numpy as np
from scipy.optimize import least_squares
from .hardware_probe import exposure, make_probe


def phasors(time_s, values, frequencies, *, origin_s, weights=None):
    t=np.asarray(time_s)-origin_s;y=np.asarray(values)
    x=np.column_stack([np.ones(len(t)),t,t*t,*[a for f in frequencies
        for a in (np.cos(2*np.pi*f*t),np.sin(2*np.pi*f*t))]])
    w=exposure(time_s) if weights is None else np.asarray(weights)
    if w.shape!=(len(t),) or np.any(w<=0):raise ValueError('Positive per-row exposure weights required')
    s=np.sqrt(w)
    coef,_,rank,_=np.linalg.lstsq(x*s[:,None],y*s if y.ndim==1 else y*s[:,None],rcond=1e-10)
    if rank!=x.shape[1]:raise ValueError('Frequency/trend temporal design is rank deficient')
    return coef[3::2]-1j*coef[4::2]


def frozen_single(y, acceleration, known, frequency):
    """Exact positive-J amplitude roots; phase uses its principal branch."""
    a=complex(acceleration);c=complex(known);y=complex(y)
    roots=np.roots([abs(a)**2,2*np.real(a.conjugate()*c),abs(c)**2-abs(y)**2])
    solutions=[]
    for root in roots:
        if abs(root.imag)<1e-12 and root.real>0:
            j=float(root.real);delta=float(np.angle(y/(j*a+c))/(2*np.pi*frequency))
            solutions.append(dict(inertia_kg_m2=j,effective_advance_s=delta))
    return dict(available=len(solutions)==1,solutions=solutions,
        note='Frozen-friction unit-gain pure-phase model; effective phase is not physical SDK latency')


def response(frequencies, velocity, inertia, viscous, advance, stiffness=0., *, gain=1., tau=0.):
    w=2*np.pi*np.asarray(frequencies);v=np.asarray(velocity,dtype=complex)
    return gain*np.exp(1j*w*advance)/(1+1j*w*tau)*v*(viscous+1j*w*inertia+stiffness/(1j*w))


def information(frequencies, velocity, inertia, viscous, stiffness=0.):
    """Scaled local Jacobian; missing parameter directions are retained."""
    w=2*np.pi*np.asarray(frequencies);v=np.asarray(velocity,dtype=complex)
    z=response(frequencies,v,inertia,viscous,0.,stiffness)
    d=np.column_stack([1j*w*v*inertia,v*viscous,1j*w*z*.005])
    real=np.vstack([d.real,d.imag]);s=np.linalg.svd(real,compute_uv=False)
    rank=int(np.linalg.matrix_rank(real));pinv=np.linalg.pinv(real)
    return dict(rank=rank,parameters=3,parameter_scales=[inertia,viscous,.005],
        singular_values_nm=s.tolist(),condition_number=float(s[0]/s[-1]) if rank==3 else None,
        null_dimensions=3-rank,inertia_sensitivity_nm_inverse=pinv[0].tolist() if rank==3 else None)


def fit_components(frequencies, acceleration, velocity, known, observed, inertia_scale, viscous_scale,*,fixed_viscous=None):
    """Fit local J/b/effective phase with frozen complex gravity/rotor terms."""
    f=np.asarray(frequencies);y=np.asarray(observed)
    if len(f)<2 or len(np.unique(f))!=len(f):raise ValueError('Distinct multiple frequencies required for free J/b/phase')
    phase_bound=1/(4*max(f));w=2*np.pi*f
    def residual(x):
        b=x[1]*viscous_scale if fixed_viscous is None else fixed_viscous
        model=x[0]*inertia_scale*np.asarray(acceleration)+b*np.asarray(velocity)+np.asarray(known)
        z=np.exp(1j*w*x[-1]*.005)*model-y
        return np.r_[z.real,z.imag]/max(np.linalg.norm(y),1e-12)
    free=fixed_viscous is None
    candidates=[least_squares(residual,[1.,1.,d/.005] if free else [1.,d/.005],
        bounds=([0.,0.,-phase_bound/.005],[np.inf,np.inf,phase_bound/.005]) if free else ([0.,-phase_bound/.005],[np.inf,phase_bound/.005]),
        ftol=1e-12,xtol=1e-12,gtol=1e-12) for d in (-.005,0.,.005)]
    best=min(candidates,key=lambda x:np.linalg.norm(x.fun));j=best.x[0]*inertia_scale
    b=best.x[1]*viscous_scale if free else fixed_viscous;delta=best.x[-1]*.005
    return dict(inertia_kg_m2=float(j),viscous_nm_s_rad=float(b),effective_advance_s=float(delta),
        residual_complex_norm_nm=float(np.linalg.norm(best.fun)*max(np.linalg.norm(y),1e-12)),
        principal_advance_branch_s=[-phase_bound,phase_bound],solver_converged=bool(best.success))


def fit_response(frequencies, velocity, observed, inertia_scale, viscous_scale, stiffness=0.):
    w=2*np.pi*np.asarray(frequencies);v=np.asarray(velocity,dtype=complex)
    return fit_components(frequencies,1j*w*v,v,stiffness/(1j*w)*v,observed,inertia_scale,viscous_scale)


def multitone_motion(name, home, pose, joint, direction, config, tones, *, transition_s=4.,settle_s=3.):
    """Affine sum of the existing C3 probe primitive, preserving its full path."""
    def primitive(frequency,amplitude):
        c=deepcopy(config);c['probe'].update(frequency_hz=frequency,amplitude_rad=amplitude)
        return make_probe(name,home,pose,joint,direction,config=c,transition_s=transition_s,settle_s=settle_s)
    base=primitive(5.,0.);out=deepcopy(base)
    states=np.asarray(base.states)
    for frequency,amplitude in tones:
        part=primitive(frequency,amplitude)
        if part.knots!=base.knots:raise ValueError('C3 primitive time grids differ')
        states=states+np.asarray(part.states)-np.asarray(base.states)
    out.states=list(states)
    out.conditions.pop('frequency_hz')
    out.conditions.update(tones=[dict(frequency_hz=f,amplitude_rad=a) for f,a in tones],
        observation_protocol='offline_multifrequency_design_only',execution_authorized=False)
    return out


def magnitude_pair(frequencies, velocity, observed, stiffness=0.):
    """Two amplitudes identify positive J/b even with different tone phases.

    Each response may have its own phase. Unit effort gain, known gravity
    stiffness and a frequency-independent viscous coefficient remain required.
    """
    w=2*np.pi*np.asarray(frequencies);v=np.abs(velocity);y=np.abs(observed)
    if len(w)!=2 or w[0]==w[1] or np.any(v<=0):raise ValueError('Two distinct excited frequencies required')
    r=(y/v)**2
    j2=(r[1]-r[0]-stiffness**2*(1/w[1]**2-1/w[0]**2))/(w[1]**2-w[0]**2)
    if j2<=0:return dict(available=False,reason='No positive inertia from amplitude pair')
    j=float(np.sqrt(j2));b2=r[0]-(w[0]*j-stiffness/w[0])**2
    if b2<0:return dict(available=False,reason='No nonnegative viscous coefficient from amplitude pair')
    return dict(available=True,inertia_kg_m2=j,viscous_nm_s_rad=float(np.sqrt(b2)))
