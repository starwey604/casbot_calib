"""Independent R2 coordinate/observability/operator counterexamples."""
from pathlib import Path
import numpy as np
import pytest
import yaml
from casbot_calib import _core
from casbot_calib.rigid_identification import fit_rigid,model_from_parameters,parameter_scales,pseudo_inertia
from casbot_calib.staged_stability_math import comparison_basis,subspace_change,operator_difference,parameter_difference,physical_margins
from casbot_calib.staged_stability import physical_prior_variants,frozen_group_mask
from casbot_calib.dual_model import nominal_dual_arm


def cube():
    # Independent inertia tensor, positive triangle margins, COM on the axis.
    return np.array([1.,0,0,0,.01,0,.02,0,0,.025])


def policy():
    return dict(mass_diagonal_floor_kg_m2=1e-6,maximum_mass_relative_frobenius=.05,
                maximum_mass_diagonal_relative=.1,maximum_inverse_axis_normalized_rms=.02)


def rotor(theta):
    model=_core.ArmModel();model.model_id='independent_R2_rotor';model.gravity_world=np.zeros(3)
    joint=_core.ArmJoint();joint.name='axis';joint.link_name='body';model.joints=[joint]
    return model_from_parameters(_core.ArmDynamics(model),theta)


def test_fixed_metric_matches_independent_analytic_quadratic_minimizer():
    prior=cube();target=prior*1.05;scale=np.linspace(.8,1.3,10);metric=np.diag(np.linspace(1.,2.,10));ridge=.1
    expected=(target+ridge*np.diag(metric)/scale**2*prior)/(1+ridge*np.diag(metric)/scale**2)
    result=fit_rigid(np.eye(10),target,prior,fixed_scales=scale,regularization_metric=metric,
                    ridge=ridge,torque_scale=1.,robust_scale=None,lift_residuals=True,physical_length_scale=.1)
    np.testing.assert_allclose(result['parameters'],expected,rtol=1e-7,atol=2e-9)
    assert result['coordinate_scale_source']=='explicit_frozen'
    assert result['regularization_metric_source']=='explicit_complete_quadratic'


def test_prior_perturbation_does_not_change_explicit_coordinates_or_frozen_equality():
    prior=cube();other=prior*1.1;scale=parameter_scales(prior);e=np.zeros((1,10));e[0,0]=1
    results=[fit_rigid(np.eye(10),prior,p,fixed_scales=scale,regularization_metric=np.eye(10),
        equalities=(e,e@prior),ridge=1e-3,robust_scale=None,lift_residuals=True) for p in (prior,other)]
    for result in results:
        np.testing.assert_array_equal(result['parameter_scales'],scale)
        assert abs(result['parameters'][0]-prior[0])<1e-8


@pytest.mark.parametrize('fault',['shape','negative_scale','nonsymmetric','indefinite'])
def test_bad_fixed_coordinates_are_rejected(fault):
    s=np.ones(10);m=np.eye(10)
    if fault=='shape':s=np.ones(9)
    elif fault=='negative_scale':s[3]=-1
    elif fault=='nonsymmetric':m[0,1]=1
    else:m[0,0]=-1
    with pytest.raises(ValueError):fit_rigid(np.eye(10),cube(),cube(),fixed_scales=s,regularization_metric=m)


def test_common_basis_removes_gravity_directions_and_reports_friction_coupling():
    design=np.array([[1.,1.,1.],[2.,1.,1.],[3.,2.,2.]])
    e=np.array([[1.,0.]])
    result=comparison_basis(design,np.ones(3),e)
    assert result['equality_rank']==1 and result['rank']==1
    np.testing.assert_allclose(result['rigid_fraction'],[.5],atol=1e-12)
    np.testing.assert_allclose(result['auxiliary_fraction'],[.5],atol=1e-12)
    np.testing.assert_allclose(result['basis'][0],0,atol=1e-12)


def test_rank_lost_by_omitting_a_group_is_an_explicit_failure():
    full=comparison_basis(np.eye(3),np.ones(3),np.zeros((0,3)))
    removed=comparison_basis(np.eye(3)[:2],np.ones(3),np.zeros((0,3)),cutoff=full['threshold'])
    change=subspace_change(full,removed)
    assert not change['passed'] and change['rank_lost']==1


def test_same_zero_training_error_can_hide_large_prior_dependent_mass_change():
    a=cube();b=a.copy();b[-1]=.029
    models=[rotor(p) for p in (a,b)];q=np.linspace(-.3,.3,32)[:,None];v=np.full_like(q,.2);zero=np.zeros_like(q)
    # With constant speed and zero gravity/friction, tau=J*qdd=0 for any J.
    terms=[m.evaluate_batch(q,v,zero) for m in models]
    assert np.max(abs(terms[0]['total']-terms[1]['total']))<1e-14
    result=operator_difference(terms[0]['mass_matrix'],terms[1]['mass_matrix'],terms[0]['total'],terms[1]['total'],policy())
    assert not result['passed'] and not result['checks']['mass_frobenius']


def test_only_nullspace_representative_changes_are_allowed_when_operators_are_identical():
    a=cube();b=a.copy();b[4]+=.002;b[6]+=.002
    ma,mb=rotor(a),rotor(b);q=np.linspace(-.3,.3,32)[:,None];v=np.full_like(q,.2);acc=np.ones_like(q)
    basis=comparison_basis(ma.regressor_batch(q,v,acc)['regressor'].reshape(-1,10),np.ones(10),np.zeros((0,10)))
    ta,tb=(m.evaluate_batch(q,v,acc) for m in (ma,mb))
    result=operator_difference(ta['mass_matrix'],tb['mass_matrix'],ta['total'],tb['total'],policy())
    change=parameter_difference(a,b,np.ones(10),basis['basis'])
    assert result['passed'] and change['observable_scaled_norm']<1e-12 and change['unobservable_scaled_norm']>0


def test_inverse_dynamics_gate_and_mass_diagonal_floor_have_independent_meanings():
    mass=np.tile(np.diag([1.,1e-8]),(8,1,1));other=mass.copy();other[:,1,1]+=2e-7
    tau=np.zeros((8,2));different=tau.copy();different[:,0]=.005
    result=operator_difference(mass,other,tau,different,policy())
    assert result['mass_diagonal_floor_uses']==8
    assert result['checks']['mass_frobenius'] and not result['checks']['mass_diagonal'] and not result['checks']['inverse_dynamics']


def test_physical_prior_perturbations_obey_declared_envelope_and_leave_nominal_unchanged():
    nominal,_=nominal_dual_arm();before=nominal.rigid_parameters().copy()
    config=yaml.safe_load(Path('configs/staged_dual_arm_p0.yaml').read_text());settings=config['p0']['stability']
    variants=physical_prior_variants(nominal,settings)
    np.testing.assert_array_equal(nominal.rigid_parameters(),before)
    assert variants.shape==(4,140)
    for vector in variants:
        model=model_from_parameters(nominal,vector)
        for first,second in zip(nominal.model.joints,model.model.joints):
            a,b=first.body,second.body
            assert abs(b.mass/a.mass-1)<=.1+1e-12
            assert np.max(abs(b.com-a.com))<=.001+1e-12
            ratio=np.linalg.norm(b.inertia_com)/np.linalg.norm(a.inertia_com)
            assert abs(ratio-1)<=.1+1e-10
        assert min(np.linalg.eigvalsh(pseudo_inertia(row)).min() for row in vector.reshape(-1,10))>0


def test_group_deletions_preserve_every_other_row_and_do_not_rescale():
    weights=[dict(rows=2,center_id='c0',acquisition_batch_id='b0'),dict(rows=3,center_id='c1',acquisition_batch_id='b1')]
    np.testing.assert_array_equal(frozen_group_mask(weights,'center_id','c0'),[False,False,True,True,True])
    margins=physical_margins(cube(),np.array([0.,1.]),np.zeros(2),np.ones(2))
    assert margins['auxiliary_lower_boundary_hits']==[0] and margins['auxiliary_upper_boundary_hits']==[1]
    assert not margins['boundary_hits_alone_fail']


def test_friction_projection_commutes_with_linear_combination_and_filters_time_axis():
    from casbot_calib.staged_motion import build_recipes
    from casbot_calib.staged_identification import friction_pair_projection
    config=yaml.safe_load(Path('configs/staged_dual_arm.yaml').read_text())
    motion=next(m for m in build_recipes(config) if m.stage=='friction' and m.conditions['joint']==0)
    times=np.arange(0,round(motion.duration*1e9),250000,dtype=np.int64);t=times*1e-9
    q,v,_,_=motion.at(t);obs=dict(sample_ns=times,q=q,dq=v)
    features=np.column_stack([np.sin(q[:,0]),np.cos(3*q[:,0]),t*.1])
    theta=np.array([.3,-.2,.05])
    x,a,_=friction_pair_projection(obs,motion.metadata(),features,2000000)
    xx,b,_=friction_pair_projection(obs,motion.metadata(),features@theta,2000000)
    np.testing.assert_array_equal(x,xx)
    np.testing.assert_allclose(a@theta,b,rtol=0,atol=1e-12)


def test_regularization_path_compares_to_fixed_reference_not_each_solution_itself(tmp_path):
    from casbot_calib.data import write_json
    from casbot_calib.staged_stability_validation import regularization_path
    basis=np.zeros((10,1));basis[9,0]=1
    np.savez_compressed(tmp_path/'coordinates.npz',scales=np.ones(10))
    np.savez_compressed(tmp_path/'comparison_basis.npz',basis=basis)
    variants=[]
    for ridge,factor in ((1e-5,1.2),(1e-4,1.),(1e-3,1.001)):
        label=f'ridge_{ridge:g}_full';directory=tmp_path/'variants'/label;directory.mkdir(parents=True)
        vector=cube();vector[-1]*=factor
        np.savez_compressed(directory/'solution.npz',vector=vector,mass=np.ones((16,1,1))*.025*factor,torque=np.ones((16,1))*.025*factor)
        variants.append(dict(label=label,ridge=ridge,kind='full',completed=True))
    write_json(tmp_path/'report.json',dict(variants=variants))
    path=regularization_path(tmp_path,policy())
    rows={r['ridge']:r for r in path['comparisons']}
    assert path['reference_ridge']==1e-4 and not path['passed']
    assert not rows[1e-5]['passed'] and rows[1e-5]['maximum_mass_relative_frobenius']==pytest.approx(.2)
    assert rows[1e-4]['passed'] and rows[1e-4]['maximum_mass_relative_frobenius']==0
