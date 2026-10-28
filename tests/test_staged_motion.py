"""Analytic trajectory/manifest contracts used by every new stage."""
from copy import deepcopy
import numpy as np
import yaml
from casbot_calib.staged_motion import Motion,build_recipes,derivative_bounds


def test_bernstein_motion_has_consistent_derivatives_and_valid_segment_bounds():
    m=Motion('test','oracle',np.zeros(2));m.add(1.,[.3,-.2]).add(.5,[.4,-.1],velocity=[.1,.1]).add(1.,[0,0])
    c=m.polynomial();d=c.derivative();bound=np.max(abs(d.c),axis=0)
    for i,(a,b) in enumerate(zip(m.knots[:-1],m.knots[1:])):
        t=np.linspace(a,b,501);assert np.all(abs(c(t,nu=1))<=bound[i]+1e-12)
        mid=(a+b)/2;eps=1e-5
        np.testing.assert_allclose((c(mid+eps)-c(mid-eps))/(2*eps),c(mid,nu=1),rtol=1e-6,atol=1e-8)
    for i,t in enumerate(m.knots):
        for order in range(4):np.testing.assert_allclose(c(t,nu=order),m.states[i][order],atol=1e-10)
    for order in (1,2,3):
        tightened=derivative_bounds(c,order)
        for i,(a,b) in enumerate(zip(m.knots[:-1],m.knots[1:])):
            assert np.all(abs(c(np.linspace(a,b,501),nu=order))<=tightened[i]+1e-10)
        assert np.max(tightened)<=np.max(abs(c.derivative(order).c))*(1+1e-8)


def test_staged_recipes_cover_return_braking_and_exact_constant_velocity():
    with open('configs/staged_dual_arm.yaml') as stream:config=yaml.safe_load(stream)
    trials=build_recipes(config)
    assert set(m.stage for m in trials)=={'direction','timing','gravity','friction','breakaway','inertia_probe','inertia','control_tuning','control_validation'}
    assert config['control_period_ns']==2000000
    for m in trials:
        assert m.metadata()['initial_final_home']
        for a,b in m.windows('constant_velocity'):
            t=np.linspace(a+.01,b-.01,10);q,v,acc,_=m.at(t);j=m.conditions['joint']
            np.testing.assert_allclose(abs(v[:,j]),m.conditions['speed'],atol=1e-10)
            np.testing.assert_allclose(acc,0,atol=1e-10)
    inertia=next(m for m in trials if m.stage=='inertia' and m.conditions['batch']=='train')
    t=np.linspace(2,20,1000);assert np.ptp(inertia.at(t)[0],axis=0).min()>.15
