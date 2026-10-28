"""Analytic sustained-motion torque balance, independent of the simulator."""
import numpy as np
from casbot_calib import _core
from casbot_calib.staged_inertia import probe_problem


def test_band_matched_probe_recovers_small_inertia_despite_larger_friction():
    model=_core.ArmModel();model.model_id='independent_rotors';model.model_version='tree_rigid_v1';model.gravity_world=np.zeros(3)
    joints=[]
    for k in range(14):
        j=_core.ArmJoint();j.name=f'axis_{k}';j.link_name=f'body_{k}';b=j.body;b.inertia_com=np.eye(3)*5e-5;j.body=b;joints.append(j)
    model.joints=joints;model.parents=[-1]*14;engine=_core.ArmDynamics(model)
    t=np.arange(0,4.,.00025);omega=10*np.pi;q=np.zeros((len(t),14));v=q.copy();a=q.copy()
    q[:,4]=.18*t+.004*np.sin(omega*t);v[:,4]=.18+.004*omega*np.cos(omega*t);a[:,4]=-.004*omega**2*np.sin(omega*t)
    torque=5e-5*a[:,4]+.035*v[:,4]+.04
    current=np.zeros_like(q);current[:,4]=torque/2+np.random.default_rng(18).normal(0,.0002,len(t))
    obs=dict(sample_ns=np.rint(t*1e9).astype(np.int64),q=q,dq=v,current=current)
    meta=dict(conditions=dict(joint=4,direction=1,frequency_hz=5.),labels=['initial','probe_measure','final'],knots_s=[0.,1.,3.,4.])
    p=probe_problem(engine,obs,meta,2.)
    x=np.column_stack([p['ddq'][:,4],p['dq'][:,4],np.ones(len(p['q']))]);value=np.linalg.lstsq(x,p['target'],rcond=None)[0]
    assert abs(value[0]/5e-5-1)<.05 and abs(value[1]/.035-1)<.05 and abs(value[2]/.04-1)<.05
