"""Analytical phase/friction counterexamples and offline C3 design contracts."""
import numpy as np
import pytest
from casbot_calib.hardware_roll_frequency import (
    phasors,frozen_single,response,information,fit_response,magnitude_pair,multitone_motion)
from casbot_calib.hardware_probe import exposure,settings,make_probe
from casbot_calib.staged_motion import derivative_bounds


def test_complex_convention_and_omission_preserves_original_row_weights():
    t=np.arange(0,2,.001);w=exposure(t);x=t-1
    y=.02+.003*x-.001*x*x+.002*np.cos(10*np.pi*x)+.003*np.sin(10*np.pi*x)
    assert phasors(t,y,[5.],origin_s=1)[0]==pytest.approx(.002-.003j,abs=1e-14)
    y+=.001*np.cos(12*np.pi*x);keep=~((t>=.4)&(t<.8))
    X=np.column_stack([np.ones(sum(keep)),x[keep],x[keep]**2,np.cos(10*np.pi*x[keep]),np.sin(10*np.pi*x[keep])])
    sw=np.sqrt(w[keep]);c=np.linalg.lstsq(X*sw[:,None],y[keep]*sw,rcond=1e-10)[0]
    z=phasors(t[keep],y[keep],[5.],origin_s=1,weights=w[keep])[0]
    assert z==pytest.approx(c[-2]-1j*c[-1],abs=1e-14)
    assert abs(z-phasors(t[keep],y[keep],[5.],origin_s=1)[0])>1e-5


@pytest.mark.parametrize('delta',[-.005,0.,.005])
def test_frozen_single_recovers_inertia_and_advance_when_friction_is_correct(delta):
    f=5.;J=2.7e-4;b=.01;v=.12+.02j;K=.016;w=2*np.pi*f
    y=response([f],[v],J,b,delta,K)[0];a=1j*w*v;known=(b+K/(1j*w))*v
    fit=frozen_single(y,a,known,f)
    assert fit['available']
    assert fit['solutions'][0]['inertia_kg_m2']==pytest.approx(J,rel=1e-12)
    assert fit['solutions'][0]['effective_advance_s']==pytest.approx(delta,abs=1e-12)
    wrong=frozen_single(y,a,known+.2*b*v,f)
    assert not wrong['available'] or abs(wrong['solutions'][0]['inertia_kg_m2']/J-1)>.1


def test_single_unknown_friction_has_a_null_direction_and_two_tones_remove_it():
    assert information([5.],[.12],2.7e-4,.01,.016)['null_dimensions']==1
    i=information([2.5,10.],[.118,.043],2.7e-4,.01,.016)
    assert i['rank']==i['parameters']==3 and i['condition_number']>1


def test_two_frequency_amplitudes_do_not_require_a_shared_phase():
    f=np.array([2.5,5.]);v=np.array([.1,.12]);J=2.7e-4;b=.01;K=.016
    y=response(f,v,J,b,0.,K)*np.exp(1j*np.array([.31,-.18]))
    fit=magnitude_pair(f,v,y,K)
    assert fit['available'] and fit['inertia_kg_m2']==pytest.approx(J,rel=1e-12)
    assert fit['viscous_nm_s_rad']==pytest.approx(b,rel=1e-12)


def test_two_tone_complex_fit_and_cached_readout_close_without_fixed_friction():
    f=np.array([2.5,10.]);amps=np.array([.0075,.00068]);J=2.7e-4;b=.012;K=.016;delta=.004
    t=np.arange(0,2,.001);cached=np.floor((t-.0013)/.005)*.005+.0013
    v=np.sum(2*np.pi*f*amps*np.cos(2*np.pi*cached[:,None]*f),axis=1)
    # Independently construct shifted q/v/a torque, not response() roundtrip.
    angle=2*np.pi*(cached[:,None]+delta)*f
    q=np.sum(amps*np.sin(angle),axis=1);tv=np.sum(2*np.pi*f*amps*np.cos(angle),axis=1)
    a=np.sum(-(2*np.pi*f)**2*amps*np.sin(angle),axis=1);y=J*a+b*tv+K*q
    V=phasors(t,v,f,origin_s=0.);Y=phasors(t,y,f,origin_s=0.)
    fit=fit_response(f,V,Y,J,.01,K)
    assert abs(fit['inertia_kg_m2']/J-1)<.005
    assert abs(fit['viscous_nm_s_rad']/b-1)<.005
    assert abs(fit['effective_advance_s']-delta)<2e-5


def test_affine_composition_preserves_original_single_tone_and_probe_envelope():
    c=settings();home=np.zeros(14);pose=home.copy();pose[13]=.3
    kwargs=dict(transition_s=4.,settle_s=3.)
    one=multitone_motion('one',home,pose,13,1,c,[(5.,.004)],**kwargs)
    original=make_probe('one',home,pose,13,1,config=c,**kwargs)
    np.testing.assert_allclose(one.states,original.states,atol=1e-15)
    for direction in [-1,1]:
        m=multitone_motion('two',home,pose,13,direction,c,[(2.5,.0075),(10.,.00068)],**kwargs);curve=m.polynomial()
        peaks=[np.max(derivative_bounds(curve,k)) for k in [1,2,3]]
        assert all(p<limit for p,limit in zip(peaks,[1.,8.,200.]))
        measure=[k for k,l in enumerate(m.labels) if l=='probe_measure']
        assert np.min(direction*curve.derivative().c[:,measure,13])>.02
        assert np.max(abs(np.asarray(m.states)[:,:,0:13]))==0
        assert m.metadata()['initial_final_home'] and 'sample_period_ns' not in m.conditions
