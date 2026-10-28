"""Independent physical parameter coordinates and constrained optimisation checks."""
import numpy as np
import pytest
pytest.importorskip('clarabel')
from casbot_calib.rigid_identification import pseudo_inertia,fit_rigid,model_from_parameters
from casbot_calib.dual_model import nominal_dual_arm


def box_parameters(mass,half,center):
    # Analytic uniform cuboid, independent of Pinocchio and pseudo_inertia().
    half=np.asarray(half);c=np.asarray(center)
    second=mass*(np.diag(half**2/3)+np.outer(c,c))
    inertia=np.trace(second)*np.eye(3)-second
    return np.r_[mass,mass*c,inertia[0,0],inertia[0,1],inertia[1,1],inertia[0,2],inertia[1,2],inertia[2,2]],second


def test_pseudo_inertia_matches_analytic_second_moments_and_solver_recovers_full_rank():
    t,s=box_parameters(1.7,[.2,.1,.08],[.03,-.04,.11]);p=pseudo_inertia(t)
    np.testing.assert_allclose(p[:3,:3],s,atol=1e-16)
    np.testing.assert_allclose(p[:3,3],t[1:4],atol=1e-16)
    rng=np.random.default_rng(18);a=rng.normal(size=(120,10));prior=t.copy();prior[[0,1,4]]*=[1.2,.8,1.1]
    fit=fit_rigid(a,a@t,prior,robust_scale=None)
    assert fit['rank']==10 and fit['unique_body_parameters']
    np.testing.assert_allclose(fit['parameters'],t,rtol=1e-6,atol=1e-8)


def test_static_mass_com_ambiguity_and_frozen_stage_equalities_are_reported():
    t,_=box_parameters(1.,[.2,.1,.08],[.03,-.04,.11]);a=np.zeros((50,10));a[:,1]=np.linspace(-9.,9.,50)
    fit=fit_rigid(a,a@t,t)
    assert fit['rank']==1 and fit['unobservable_dimension']==9 and not fit['unique_body_parameters']
    rng=np.random.default_rng(24);dynamic=rng.normal(size=(80,10));target=dynamic@t+.05
    frozen=np.zeros((1,10));frozen[0,1]=1
    updated=fit_rigid(dynamic,target,t,equalities=(frozen,frozen@t))
    assert abs(updated['parameters'][1]-t[1])<1e-9


def test_physical_coordinate_roundtrip_preserves_native_dynamics_and_rejects_bad_block():
    nominal,_=nominal_dual_arm();theta=nominal.rigid_parameters();candidate=model_from_parameters(nominal,theta)
    np.testing.assert_allclose(candidate.rigid_parameters(),theta,atol=1e-16,rtol=1e-13)
    q=np.linspace(-.3,.3,14);np.testing.assert_allclose(candidate.mass_matrix(q),nominal.mass_matrix(q),atol=1e-14)
    theta[4]=-100
    with pytest.raises(ValueError,match='Nonphysical'):model_from_parameters(nominal,theta)


def test_joint_fit_keeps_auxiliary_parameters_bounded_and_gravity_combination_frozen():
    t,_=box_parameters(1.,[.2,.1,.08],[.03,-.04,.11]);rng=np.random.default_rng(67)
    a=rng.normal(size=(150,10));extra=rng.normal(size=(150,2));z=np.array([.035,.04])
    constraint=np.zeros((1,10));constraint[0,1]=1
    result=fit_rigid(a,a@t+extra@z,t,auxiliary=(extra,[.03,.045],[.02,.03],[.05,.06]),equalities=(constraint,constraint@t),lift_residuals=True,physical_length_scale=.1)
    np.testing.assert_allclose(result['auxiliary_parameters'],z,atol=1e-7)
    assert len(result['parameters'])==10 and result['columns']==12
    assert abs(result['parameters'][1]-t[1])<1e-9
