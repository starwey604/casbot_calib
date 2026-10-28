"""Analytic 2R gravity oracle for the dedicated Pinocchio batch path."""
import numpy as np
import pytest
from casbot_calib import _core


def test_gravity_batch_matches_independent_planar_formula_with_strided_input():
    model=_core.ArmModel();model.model_id='batch_gravity_2r';model.gravity_world=np.array([0.,-9.81,0.]);joints=[]
    for k,(mass,com) in enumerate([(1.3,.35),(.8,.225)]):
        j=_core.ArmJoint();j.name=f'joint_{k}';j.link_name=f'link_{k}';pose=j.origin;pose.translation=np.array([0. if k==0 else .7,0.,0.]);j.origin=pose
        b=j.body;b.mass=mass;b.com=np.array([com,0.,0.]);j.body=b;joints.append(j)
    model.joints=joints;engine=_core.ArmDynamics(model)
    q=np.array([[.2,.4],[-.7,.9],[.8,-.3]])[::-1]
    expected=np.column_stack([9.81*((1.3*.35+.8*.7)*np.cos(q[:,0])+.8*.225*np.cos(q.sum(axis=1))),9.81*.8*.225*np.cos(q.sum(axis=1))])
    np.testing.assert_allclose(engine.gravity_batch(q),expected,atol=1e-13,rtol=0)
    with pytest.raises(ValueError):engine.gravity_batch(np.zeros((2,3)))
    q=q.copy();q[0,0]=np.nan
    with pytest.raises(ValueError):engine.gravity_batch(q)
