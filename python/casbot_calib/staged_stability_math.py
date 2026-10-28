"""R2 comparison coordinates and operator diagnostics; no device/truth access."""
import numpy as np
from scipy.linalg import null_space, subspace_angles
from .rigid_identification import pseudo_inertia


def comparison_basis(design, scales, equality, *, rank_rtol=1e-7, rank_atol=1e-12, cutoff=None):
    design=np.asarray(design,dtype=float);scales=np.asarray(scales,dtype=float)
    e=np.asarray(equality,dtype=float)
    if design.ndim!=2 or scales.shape!=(design.shape[1],) or e.ndim!=2 or e.shape[1]>len(scales) or not np.isfinite(design).all() or not np.isfinite(scales).all() or np.any(scales<=0):
        raise ValueError('Invalid constrained comparison coordinates')
    rigid_columns=e.shape[1]
    e=np.pad(e,((0,0),(0,len(scales)-e.shape[1])))
    free=null_space(e*scales,rcond=rank_rtol) if len(e) else np.eye(len(scales))
    reduced=(design*scales)@free
    _,singular,vt=np.linalg.svd(reduced,full_matrices=False)
    threshold=max(rank_atol,rank_rtol*singular[0]) if cutoff is None and len(singular) else (rank_atol if cutoff is None else cutoff)
    rank=int(np.count_nonzero(singular>threshold));basis=free@vt[:rank].T
    return dict(basis=basis,singular_values=singular,rank=rank,threshold=float(threshold),
        equality_rank=len(scales)-free.shape[1],free_dimension=free.shape[1],
        rigid_fraction=np.sum(basis[:rigid_columns]**2,axis=0),
        auxiliary_fraction=np.sum(basis[rigid_columns:]**2,axis=0))


def subspace_change(reference, variant):
    angles=subspace_angles(reference['basis'],variant['basis']) if reference['rank'] and variant['rank'] else np.array([])
    difference=reference['basis']@reference['basis'].T-variant['basis']@variant['basis'].T
    return dict(passed=variant['rank']>=reference['rank'],reference_rank=reference['rank'],variant_rank=variant['rank'],
        rank_lost=max(0,reference['rank']-variant['rank']),projector_frobenius_difference=float(np.linalg.norm(difference)),
        largest_principal_angle_rad=float(max(angles,default=0.)),
        convention='same full-train absolute cutoff; common constrained scaled coordinates; rank loss is not skipped')


def operator_difference(reference_m, variant_m, reference_tau, variant_tau, policy):
    a,b=np.asarray(reference_m),np.asarray(variant_m);ta,tb=np.asarray(reference_tau),np.asarray(variant_tau)
    if a.ndim!=3 or a.shape!=b.shape or ta.shape!=tb.shape or ta.shape!=a.shape[:2] or a.shape[1]!=a.shape[2] or not all(np.isfinite(x).all() for x in (a,b,ta,tb)):
        raise ValueError('Invalid common operator probe arrays')
    norm=np.linalg.norm(a,axis=(1,2))
    if np.any(norm<=0):raise ValueError('Positive reference mass norm required')
    frobenius=np.linalg.norm(a-b,axis=(1,2))/norm
    diagonal=np.diagonal(a,axis1=1,axis2=2);other=np.diagonal(b,axis1=1,axis2=2)
    floor=policy['mass_diagonal_floor_kg_m2'];denominator=np.maximum(diagonal,floor)
    diagonal_error=abs(other-diagonal)/denominator
    torque_rms=np.sqrt(np.mean((ta-tb)**2,axis=0));torque_scale=np.maximum(np.sqrt(np.mean(ta*ta,axis=0)),.2)
    normalized=torque_rms/torque_scale
    checks=dict(positive_mass_matrices=bool(np.linalg.eigvalsh(a).min()>0 and np.linalg.eigvalsh(b).min()>0),
        mass_frobenius=bool(max(frobenius)<=policy['maximum_mass_relative_frobenius']),
        mass_diagonal=bool(np.max(diagonal_error)<=policy['maximum_mass_diagonal_relative']),
        inverse_dynamics=bool(max(normalized)<=policy['maximum_inverse_axis_normalized_rms']))
    return dict(passed=all(checks.values()),checks=checks,mass_relative_frobenius=frobenius.tolist(),
        maximum_mass_relative_frobenius=float(max(frobenius)),maximum_mass_diagonal_relative=float(np.max(diagonal_error)),
        axis_mass_diagonal_maximum_relative=np.max(diagonal_error,axis=0).tolist(),
        mass_diagonal_floor_uses=int(np.count_nonzero(diagonal<floor)),
        inverse_axis_difference_rms_nm=torque_rms.tolist(),inverse_axis_normalized_rms=normalized.tolist(),
        inverse_axis_normalization_nm=torque_scale.tolist(),maximum_inverse_axis_normalized_rms=float(max(normalized)))


def parameter_difference(reference, variant, scales, common_basis):
    delta=(np.asarray(variant)-reference)/scales;observable=common_basis.T@delta
    null=delta-common_basis@observable
    return dict(observable_scaled_coordinates=observable.tolist(),observable_scaled_norm=float(np.linalg.norm(observable)),
        unobservable_scaled_norm=float(np.linalg.norm(null)),total_scaled_norm=float(np.linalg.norm(delta)),
        interpretation='fixed full-train observable combinations; sensitivity diagnostic, not a confidence interval or unique body recovery')


def physical_margins(theta, friction, lower, upper, *, length=.1):
    d=np.diag([1/length]*3+[1.]);raw=[];scaled=[];margins=[]
    for row in np.asarray(theta).reshape(-1,10):
        p=pseudo_inertia(row);raw.append(float(np.linalg.eigvalsh(p).min()))
        scaled.append(float(np.linalg.eigvalsh(d@p@d).min()))
        margins.append(float(np.linalg.eigvalsh(d@(p-np.eye(4)*1e-10)@d).min()))
    fraction=(np.asarray(friction)-lower)/(upper-lower)
    return dict(minimum_pseudo_inertia_eigenvalues=raw,congruence_length_m=length,
        congruent_minimum_eigenvalues=scaled,solver_psd_boundary_margins=margins,
        auxiliary_lower_boundary_hits=np.flatnonzero(fraction<=1e-4).tolist(),
        auxiliary_upper_boundary_hits=np.flatnonzero(fraction>=1-1e-4).tolist(),
        boundary_hits_alone_fail=False,
        units_note='raw homogeneous pseudo-inertia mixes mass and length; congruence scales spatial coordinates by the declared length')
