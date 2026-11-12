"""Matched local force balance on fixed, non-reversing raw sample support."""
import numpy as np
from scipy.sparse import csr_matrix

from .staged_wrist_periodic import AXES

MIN_SPEED = .02
STRIDE = 15


def raw_sustained_support(raw, meta, step):
    period_ns = round(meta['conditions']['period_s']*1e9)
    first = meta['labels'].index('periodic')
    start = round(meta['knots_s'][first]*1e9) + 2*period_ns
    time = raw['sample_ns']
    lo,hi = np.searchsorted(time, [start, start+2*period_ns])
    if not np.array_equal(time[lo:hi],np.arange(start,start+2*period_ns,step,dtype=np.int64)):
        raise ValueError('Expected two complete cycles at the acquisition rate')
    center = np.arange(lo+2,hi-2)
    v = raw['dq']
    ref = v[center][:,AXES]
    moving = (np.abs(ref)>=MIN_SPEED).all(axis=1)
    # Select the common time support first. No acceleration stencil may touch
    # a stopped or transition sample, nor cross any active axis sign reversal.
    for off in (-2,-1,1,2):
        other=v[center+off][:,AXES]
        moving &= ((np.abs(other)>=MIN_SPEED)&(ref*other>0)).all(axis=1)
    return lo,hi,center[moving]


def sustained_samples(raw, meta, step):
    lo,hi,eligible=raw_sustained_support(raw,meta,step)
    time=raw['sample_ns']
    start=time[lo]
    period_ns=(time[hi-1]-start+step)//2
    v=raw['dq']
    keep = eligible[(eligible-lo)%STRIDE==0]
    if len(keep)<16:
        raise ValueError('Insufficient simultaneous sustained-motion samples')
    step_s = step*1e-9
    a = (v[keep-2]-8*v[keep-1]+8*v[keep+1]-v[keep+2])/(12*step_s)
    a3 = (v[keep+1]-v[keep-1])/(2*step_s)
    info = dict(steady_raw_samples=int(hi-lo), eligible_stencil_centers=len(eligible), retained_rows_per_axis=len(keep),
        retained_by_cycle=[int(np.count_nonzero(time[keep]<start+period_ns)),int(np.count_nonzero(time[keep]>=start+period_ns))],
        measured_speed_minimum_rad_s=np.min(abs(v[keep][:,AXES]),axis=0).tolist(),
        maximum_abs_active_acceleration_rad_s2=np.max(abs(a[:,AXES]),axis=0).tolist(),
        stencil_3_vs_5_acceleration_rms_rad_s2=np.sqrt(np.mean((a-a3)**2,axis=0))[AXES].tolist())
    return keep,a,info


def local_kernel(valid, centers, sigma_samples):
    """Centered Gaussian, truncated at 3 sigma and at each contiguous edge."""
    idx=np.searchsorted(valid,centers)
    if not np.array_equal(valid[idx],centers):
        raise ValueError('Frozen fit centers are outside sustained stencil support')
    edges=np.r_[0,np.flatnonzero(np.diff(valid)!=1)+1,len(valid)]
    segment=np.searchsorted(edges[1:],idx,side='right')
    nominal=int(np.ceil(3*sigma_samples))
    radii=np.minimum(nominal,np.minimum(idx-edges[segment],edges[segment+1]-1-idx))
    offset=np.arange(-nominal,nominal+1)
    allowed=np.abs(offset)[None,:]<=radii[:,None]
    row,k=np.nonzero(allowed)
    col=idx[row]+offset[k]
    data=np.exp(-.5*(offset[k]/sigma_samples)**2)
    normalization=np.bincount(row,weights=data,minlength=len(centers))
    matrix=csr_matrix((data/normalization[row],(row,col)),shape=(len(centers),len(valid)))
    if (not np.array_equal(valid[col]-centers[row],offset[k])
            or not np.allclose(np.asarray(matrix.sum(axis=1)).ravel(),1,atol=1e-14,rtol=0)
            or not np.allclose(matrix@valid.astype(float),centers,atol=1e-9,rtol=0)):
        raise ValueError('Local filter is not constant/first-moment preserving within support')
    return matrix,radii,nominal


def raw_rows(nominal,raw,valid,gain,static,step):
    """Evaluate raw balance columns before any smoothing, for the three torque axes."""
    v=raw['dq'];dt=step*1e-9
    acceleration=(v[valid-2]-8*v[valid-1]+8*v[valid+1]-v[valid+2])/(12*dt)
    rows=np.empty((len(valid),len(AXES),169))
    for start in range(0,len(valid),4096):
        sl=slice(start,start+4096); k=valid[sl]
        native=nominal.regressor_batch(raw['q'][k],v[k],acceleration[sl])
        obs=raw['current'][k][:,AXES]*gain
        dv=v[k][:,AXES];sign=np.sign(dv);decay=np.exp(-(dv/.002)**2)
        rows[sl,:,:140]=native['regressor'][:,AXES]
        rows[sl,:,140:168]=0
        for col,j in enumerate(AXES):
            rows[sl,col,140+j]=dv[:,col]
            rows[sl,col,154+j]=sign[:,col]*(1-decay[:,col])
        rows[sl,:,168]=obs-native['rotor_torque'][:,AXES]-sign*decay*static[AXES]
    return rows,acceleration[:,AXES]


def local_problem(nominal,raw,meta,gain,static,step,centers,sigma_ms):
    """Same support and normalized kernel as the offline bandwidth comparison."""
    expected,_,_=sustained_samples(raw,meta,step)
    if not np.array_equal(centers,expected):
        raise ValueError('Frozen left wrist support differs from the diagnosed support')
    _,_,valid=raw_sustained_support(raw,meta,step)
    rows,a=raw_rows(nominal,raw,valid,gain,static,step)
    kernel,_,_=local_kernel(valid,centers,sigma_ms*1e6/step)
    block=(kernel@rows.reshape(len(rows),-1)).reshape(len(centers),3,169)
    reg=np.zeros((len(centers),14,140)); extra=np.zeros((len(centers),14,28))
    torque=np.zeros((len(centers),14));target=torque.copy();ddq=torque.copy()
    reg[:,AXES]=block[:,:,:140];extra[:,AXES]=block[:,:,140:168]
    target[:,AXES]=block[:,:,168]
    torque[:,AXES]=kernel@(raw['current'][valid][:,AXES]*gain)
    ddq[:,AXES]=kernel@a
    moving=np.zeros_like(torque,dtype=bool);moving[:,AXES]=True
    return dict(regression=reg,extra=extra,target=target,current_torque=torque,ddq=ddq,
                moving=moving,local_balance=True)
