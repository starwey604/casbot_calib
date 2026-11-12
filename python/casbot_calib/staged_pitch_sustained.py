"""Frozen train-only pitch and primary-wrist raw sustained preprocessing."""
import numpy as np
from scipy.ndimage import minimum_filter1d

from .staged_wrist_local import local_kernel
from .torque_observations import output_torque

POLICY=dict(joint=12,sigma_ms=1.5,stride=15,minimum_speed_rad_s=.02,same_sign_margin_s=.005,
            support_file='right_pitch_sample_indices.npz',scope='train_primary_only')
WRIST_POLICY={**{k:v for k,v in POLICY.items() if k!='joint'},
              'joints':[4,5,6,11,12,13],'support_file':'wrist_primary_sample_indices.npz'}


def validate_policy(p0):
    if 'right_pitch_train' not in p0 or p0['pitch_sustained_train']!=POLICY:
        raise ValueError('Unsupported sustained right-pitch development policy')


def validate_wrist_policy(p0):
    if 'pitch_sustained_train' not in p0 or p0['wrist_sustained_train']!=WRIST_POLICY:
        raise ValueError('Unsupported sustained wrist-primary development policy')


def support(raw,meta,step,*,joint=12):
    period=round(meta['conditions']['period_s']*1e9)
    start=round(meta['knots_s'][meta['labels'].index('periodic')]*1e9)+2*period
    lo,hi=np.searchsorted(raw['sample_ns'],[start,start+2*period])
    if not np.array_equal(raw['sample_ns'][lo:hi],np.arange(start,start+2*period,step)):
        raise ValueError('Sustained pitch requires two contiguous raw steady cycles')
    v=raw['dq'][:,joint];radius=round(POLICY['same_sign_margin_s']*1e9/step)
    width=2*radius+1
    good=(minimum_filter1d(v,size=width,mode='nearest')>POLICY['minimum_speed_rad_s'])|(
          minimum_filter1d(-v,size=width,mode='nearest')>POLICY['minimum_speed_rad_s'])
    valid=np.flatnonzero(good&(np.arange(len(v))>=lo+radius)&(np.arange(len(v))<hi-radius))
    centers=valid[(valid-lo)%POLICY['stride']==0]
    if len(centers)<32:raise ValueError('Insufficient sustained joint support')
    return valid,centers


def problem(nominal,raw,meta,gain,static,step,centers,*,joint=12):
    valid,expected=support(raw,meta,step,joint=joint)
    if not np.array_equal(centers,expected):
        raise ValueError('Frozen sustained raw support differs from its declared rule')
    v=raw['dq'];dt=step*1e-9
    a=(v[valid-2]-8*v[valid-1]+8*v[valid+1]-v[valid+2])/(12*dt)
    native=nominal.regressor_batch(raw['q'][valid],v[valid],a)
    block=np.zeros((len(valid),169));block[:,:140]=native['regressor'][:,joint]
    speed=v[valid,joint];sign=np.sign(speed);transition=sign*np.exp(-(speed/.002)**2)
    block[:,140+joint]=speed;block[:,154+joint]=sign-transition
    block[:,-1]=output_torque(raw,gain)[valid,joint]-native['rotor_torque'][:,joint]-static[joint]*transition
    kernel,_,_=local_kernel(valid,centers,POLICY['sigma_ms']*1e6/step)
    block=kernel@block
    return dict(regression=block[:,:140],extra=block[:,140:168],target=block[:,-1])
