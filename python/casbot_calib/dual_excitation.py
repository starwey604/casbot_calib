"""Consistent analytic q/dq/ddq/jerk references; no measured acceleration side channel."""
import numpy as np


def periodic_reference(t,center,period_s=4.,amplitude=.1,seed=27,validation=False,active='both'):
    t=np.asarray(t);center=np.asarray(center);n=len(center);rng=np.random.default_rng(seed)
    harmonics=np.array([1,3,4,6] if validation else [1,2,3,5])
    weights=np.array([1.,.6,.35,.2]);weights/=weights.sum()
    phase=rng.uniform(-np.pi,np.pi,(n,len(harmonics)))
    amplitude=np.broadcast_to(np.asarray(amplitude,dtype=float),(n,))
    if not np.isfinite(amplitude).all() or np.any(amplitude<0):raise ValueError('Invalid per-joint excitation amplitude')
    amplitudes=amplitude[:,None]*rng.uniform(.7,1.2,(n,len(harmonics)))*weights
    if active=='left':amplitudes[7:]=0.
    elif active=='right':amplitudes[:7]=0.
    elif active!='both':raise ValueError('Unknown active arm group')
    q=np.broadcast_to(center,(len(t),n)).copy();v=np.zeros_like(q);a=v.copy();jerk=v.copy()
    for k,harmonic in enumerate(harmonics):
        w=2*np.pi*harmonic/period_s;angle=w*t[:,None]+phase[:,k]
        q+=amplitudes[:,k]*np.sin(angle);v+=amplitudes[:,k]*w*np.cos(angle)
        a-=amplitudes[:,k]*w*w*np.sin(angle);jerk-=amplitudes[:,k]*w**3*np.cos(angle)
    return q,v,a,jerk


def direction_reference(t,center,amplitude=.45,slot_s=6.,initial_hold_s=3.):
    """14 sequential + / - / return jogs, C3-continuous at every waypoint."""
    t=np.asarray(t);center=np.asarray(center);n=len(center)
    q=np.broadcast_to(center,(len(t),n)).copy();v=np.zeros_like(q);a=v.copy();jerk=v.copy()
    if slot_s<=0 or initial_hold_s<0 or amplitude<=0:raise ValueError('Invalid direction jog configuration')
    knots=np.array([0.,.2,.6,.8,1.])*slot_s;values=np.array([0.,amplitude,-amplitude,0.,0.])
    for j in range(n):
        local=t-initial_hold_s-j*slot_s
        for k in range(4):
            mask=(local>=knots[k])&(local<knots[k+1]);span=knots[k+1]-knots[k]
            u=(local[mask]-knots[k])/span;delta=values[k+1]-values[k]
            q[mask,j]+=values[k]+delta*(35*u**4-84*u**5+70*u**6-20*u**7)
            v[mask,j]=delta*(140*u**3-420*u**4+420*u**5-140*u**6)/span
            a[mask,j]=delta*(420*u**2-1680*u**3+2100*u**4-840*u**5)/span**2
            jerk[mask,j]=delta*(840*u-5040*u**2+8400*u**3-4200*u**4)/span**3
    return q,v,a,jerk


def gravity_reference(t,center,hold_s=2.,poses=8,seed=41):
    """Separate static holds joined by quintic transitions; only settled holds are fitted."""
    rng=np.random.default_rng(seed);center=np.asarray(center);targets=center+rng.uniform(-.25,.25,(poses,len(center)))
    q=np.zeros((len(t),len(center)));v=q.copy();a=q.copy();jerk=q.copy()
    for index,time in enumerate(t):
        pose=min(int(time/hold_s),poses-1);local=time-pose*hold_s
        start=center if pose==0 else targets[pose-1];delta=targets[pose]-start;transition=.5
        if local>=transition:q[index]=targets[pose];continue
        u=local/transition
        q[index]=start+delta*(10*u**3-15*u**4+6*u**5)
        v[index]=delta*(30*u*u-60*u**3+30*u**4)/transition
        a[index]=delta*(60*u-180*u*u+120*u**3)/transition**2
        jerk[index]=delta*(60-360*u+360*u*u)/transition**3
    return q,v,a,jerk
