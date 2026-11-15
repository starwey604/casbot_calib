"""Full Pinocchio inertial parameters with physically consistent conic fitting.

CVXPY + Clarabel solves the convex quadratic/Huber/PSD problem. NumPy assembles
parameter coordinates and constraints; all robot regressors and dynamics are
Pinocchio. Unobservable components select a prior-near physical representative
and are not reported as independent mass/COM measurements.
"""
from time import perf_counter
import cvxpy as cp
import numpy as np
from scipy import sparse
from . import _core


def pseudo_inertia(theta):
    """4x4 pseudo-inertia from [m,hx,hy,hz,Ixx,Ixy,Iyy,Ixz,Iyz,Izz]."""
    theta = np.asarray(theta, dtype=float)
    if theta.shape != (10,) or not np.isfinite(theta).all():
        raise ValueError('Expected ten finite rigid parameters')
    m = theta[0]
    h = theta[1:4]
    xx, xy, yy, xz, yz, zz = theta[4:]
    inertia = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
    result = np.empty((4, 4))
    result[:3, :3] = 0.5 * np.trace(inertia) * np.eye(3) - inertia
    result[:3, 3] = h
    result[3, :3] = h
    result[3, 3] = m
    return result


def model_from_parameters(
    nominal,
    parameters,
    viscous=0.0,
    coulomb=0.0,
    static=None,
    smoothing_velocity=0.001,
    stribeck_velocity=0.002,
    rotor_inertia=None,
):
    model = nominal.model
    joints = model.joints
    n = len(joints)
    p = np.asarray(parameters, dtype=float).reshape(n, 10)
    b, fc, v0, vs = (
        np.broadcast_to(np.asarray(x, dtype=float), (n,))
        for x in (viscous, coulomb, smoothing_velocity, stribeck_velocity)
    )
    fs = fc if static is None else np.broadcast_to(np.asarray(static, dtype=float), (n,))
    ri = None if rotor_inertia is None else np.broadcast_to(np.asarray(rotor_inertia, dtype=float), (n,))
    for k, (joint, theta) in enumerate(zip(joints, p)):
        if theta[0] <= 0 or np.linalg.eigvalsh(pseudo_inertia(theta)).min() <= 0:
            raise ValueError('Nonphysical rigid parameter block')
        body = joint.body
        body.mass = float(theta[0])
        body.com = theta[1:4] / body.mass
        xx, xy, yy, xz, yz, zz = theta[4:]
        origin = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
        c = body.com
        body.inertia_com = origin - body.mass * (np.dot(c, c) * np.eye(3) - np.outer(c, c))
        joint.body = body
        friction = joint.friction
        friction.viscous = float(b[k])
        friction.coulomb = float(fc[k])
        friction.static_friction = float(fs[k])
        friction.smoothing_velocity = float(v0[k])
        friction.stribeck_velocity = float(vs[k])
        joint.friction = friction
        if ri is not None:
            joint.rotor_inertia = float(ri[k])
    model.joints = joints
    # The native constructor independently validates COM inertia and its triangle
    # inequality. It does not use this module's pseudo-inertia implementation.
    return _core.ArmDynamics(model)


def parameter_scales(prior):
    rows = np.asarray(prior, dtype=float).reshape(-1, 10)
    # SI coordinate scales, not observations or hard manufacturer bounds.
    typical = rows[:, 0, None] * np.array([1.0, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
    return np.maximum(np.abs(rows), typical).ravel()


def _parse_validation_data(validation_data, rigid_count, full_count):
    if not isinstance(validation_data, (tuple, list)) or len(validation_data) not in (2, 3):
        raise ValueError('Expected validation_data as (y_val, target_val[, extra_val])')
    target_val = np.asarray(validation_data[1], dtype=float).ravel()
    if len(target_val) == 0 or not np.isfinite(target_val).all():
        raise ValueError('Invalid validation target data')
    raw_y = np.asarray(validation_data[0], dtype=float)
    if len(validation_data) == 3 and validation_data[2] is not None:
        if raw_y.size != len(target_val) * rigid_count:
            raise ValueError('Invalid validation regressor shape')
        extra_val = np.asarray(validation_data[2], dtype=float).reshape(len(target_val), full_count - rigid_count)
        y_val = np.column_stack([raw_y.reshape(len(target_val), rigid_count), extra_val])
        use_full = True
    elif raw_y.size == len(target_val) * full_count:
        y_val = raw_y.reshape(len(target_val), full_count)
        use_full = True
    elif raw_y.size == len(target_val) * rigid_count:
        y_val = raw_y.reshape(len(target_val), rigid_count)
        use_full = False
    else:
        raise ValueError('Invalid validation regressor shape')
    if not np.isfinite(y_val).all():
        raise ValueError('Non-finite validation regressor')
    return y_val, target_val, use_full


def weighted_rmse(residual, weights, axis=None):
    """RMSE over usable observations; an empty axis has no estimate (NaN)."""
    squared = np.asarray(residual, dtype=float) ** 2
    if weights is None:
        return np.sqrt(np.mean(squared, axis=axis))
    count = np.sum(weights, axis=axis)
    total = np.sum(squared * weights, axis=axis)
    return np.sqrt(np.divide(total, count, out=np.full_like(total, np.nan), where=count > 0))


def fit_rigid(
    regressor,
    target,
    prior,
    *,
    equalities=None,
    auxiliary=None,
    torque_scale=0.2,
    rank_rtol=1e-7,
    robust_scale=0.02,
    ridge=1e-12,
    ridge_grid=None,
    validation_data=None,
    validation_weights=None,
    physical_length_scale=1.0,
    lift_residuals=False,
    fixed_scales=None,
    regularization_metric=None,
    sample_weights=None,
):
    prior = np.asarray(prior, dtype=float)
    if prior.ndim != 1 or len(prior) == 0 or len(prior) % 10 != 0 or not np.isfinite(prior).all():
        raise ValueError('Invalid rigid identification data')
    target = np.asarray(target, dtype=float).ravel()
    raw_reg = np.asarray(regressor, dtype=float)
    if raw_reg.size != len(target) * len(prior):
        raise ValueError('Invalid rigid identification data')
    y = raw_reg.reshape(len(target), len(prior))
    if (
        len(target) == 0
        or not np.isfinite(y).all()
        or not np.isfinite(target).all()
        or not np.isfinite(torque_scale)
        or torque_scale <= 0
        or not np.isfinite(rank_rtol)
        or rank_rtol <= 0
        or (robust_scale is not None and (not np.isfinite(robust_scale) or robust_scale <= 0))
        or not np.isfinite(ridge)
        or ridge <= 0
        or not np.isfinite(physical_length_scale)
        or physical_length_scale <= 0
    ):
        raise ValueError('Invalid rigid identification data')

    scale = parameter_scales(prior)
    rigid_count = len(prior)
    if auxiliary is not None:
        extra, extra_prior, extra_lower, extra_upper = auxiliary
        extra = np.asarray(extra, dtype=float).reshape(len(target), -1)
        extra_prior, extra_lower, extra_upper = (
            np.asarray(v, dtype=float) for v in (extra_prior, extra_lower, extra_upper)
        )
        if (
            extra_prior.ndim != 1
            or extra.shape[1] != len(extra_prior)
            or extra_lower.shape != extra_prior.shape
            or extra_upper.shape != extra_prior.shape
            or not all(np.isfinite(v).all() for v in (extra, extra_prior, extra_lower, extra_upper))
            or np.any(extra_lower > extra_prior)
            or np.any(extra_upper < extra_prior)
            or np.any(extra_lower >= extra_upper)
        ):
            raise ValueError('Invalid bounded auxiliary parameter block')
        y = np.column_stack([y, extra])
        prior = np.r_[prior, extra_prior]
        scale = np.r_[scale, np.maximum(np.abs(extra_prior), 0.001)]

    if fixed_scales is not None:
        scale = np.asarray(fixed_scales, dtype=float).copy()
        if scale.shape != prior.shape or not np.isfinite(scale).all() or np.any(scale <= 0):
            raise ValueError('Expected fixed positive scales for the complete rigid/auxiliary block')

    metric = None
    if regularization_metric is not None:
        metric = np.asarray(regularization_metric, dtype=float).copy()
        if (
            metric.shape != (len(prior), len(prior))
            or not np.isfinite(metric).all()
            or not np.allclose(metric, metric.T, rtol=0, atol=1e-12)
        ):
            raise ValueError('Expected a finite symmetric regularization metric')
        if np.linalg.eigvalsh(metric).min() <= 0:
            raise ValueError('Regularization metric must be positive definite')
    if np.any(scale <= 0):
        raise ValueError('Positive physical prior required')

    w_vec = None
    if sample_weights is not None:
        w_vec = np.asarray(sample_weights, dtype=float).ravel()
        if (
            w_vec.shape != (len(target),)
            or not np.isfinite(w_vec).all()
            or np.any(w_vec < 0)
            or not np.any(w_vec > 0)
        ):
            raise ValueError('Invalid sample weights')

    ridges = None
    if ridge_grid is not None:
        ridges = [float(r) for r in ridge_grid]
        if not ridges or any(not np.isfinite(r) or r <= 0 for r in ridges):
            raise ValueError('Invalid ridge grid')
        if validation_data is None:
            raise ValueError('validation_data required when ridge_grid is provided')

    parsed_val = (
        _parse_validation_data(validation_data, rigid_count, len(prior))
        if validation_data is not None
        else None
    )
    val_weights = None
    if validation_weights is not None:
        val_weights = np.asarray(validation_weights, dtype=float).ravel()
        if (parsed_val is None or val_weights.shape != parsed_val[1].shape
                or not np.isfinite(val_weights).all() or np.any(val_weights < 0)
                or not np.any(val_weights > 0)):
            raise ValueError('Invalid validation weights')

    a = y * scale / torque_scale
    rhs = (target - y @ prior) / torque_scale
    w_sqrt = np.sqrt(w_vec) if w_vec is not None else None
    a_svd = a * w_sqrt[:, None] if w_sqrt is not None else a
    _, singular, vh = np.linalg.svd(a_svd, full_matrices=False)
    rank = int(np.count_nonzero(singular > max(singular[0] * rank_rtol, 1e-12)))
    projector = np.eye(len(prior)) - vh[:rank].T @ vh[:rank]

    n_vars = len(prior)
    x = cp.Variable(n_vars)
    constraints = []

    congruence = np.diag([1.0 / physical_length_scale] * 3 + [1.0])
    cong_sq = congruence @ congruence
    eps = 1e-10
    basis_j = np.column_stack([
        (congruence @ pseudo_inertia(row) @ congruence).reshape(16, order='F')
        for row in np.eye(10)
    ])
    for j in range(rigid_count // 10):
        sec = slice(j * 10, (j + 1) * 10)
        j0_vec = (congruence @ pseudo_inertia(prior[sec]) @ congruence - eps * cong_sq).reshape(16, order='F')
        b_mat = basis_j * scale[sec][None, :]
        constraints.append(cp.reshape(j0_vec + b_mat @ x[sec], (4, 4), order='F') >> 0)

    if auxiliary is not None:
        aux_delta = cp.multiply(scale[rigid_count:], x[rigid_count:])
        constraints.append(aux_delta >= extra_lower - extra_prior)
        constraints.append(aux_delta <= extra_upper - extra_prior)

    e_full = None
    eq_value = None
    if equalities is not None:
        e_mat, eq_value = equalities
        e_mat = np.asarray(e_mat, dtype=float)
        eq_value = np.asarray(eq_value, dtype=float)
        if (
            e_mat.ndim != 2
            or e_mat.shape[1] != rigid_count
            or eq_value.shape != (e_mat.shape[0],)
            or not np.isfinite(e_mat).all()
            or not np.isfinite(eq_value).all()
        ):
            raise ValueError('Invalid frozen-stage equalities')
        e_full = (
            np.column_stack([e_mat, np.zeros((len(e_mat), n_vars - rigid_count))])
            if auxiliary is not None
            else e_mat
        )
        u, s, v = np.linalg.svd(e_full * scale, full_matrices=False)
        r_eq = int(np.count_nonzero(s > max(s[0] * rank_rtol, 1e-12)))
        if r_eq > 0:
            constraints.append(v[:r_eq] @ x == (u[:, :r_eq].T @ (eq_value - e_full @ prior)) / s[:r_eq])

    if robust_scale is None:
        a_weighted = a * w_sqrt[:, None] if w_sqrt is not None else a
        rhs_weighted = rhs * w_sqrt if w_sqrt is not None else rhs
        q_orth, r_tri = np.linalg.qr(a_weighted, mode='reduced')
        compressed = q_orth.T @ rhs_weighted
        data_loss = 0.5 * cp.sum_squares(r_tri @ x - compressed)
    else:
        delta = float(robust_scale / torque_scale)
        a_op = sparse.csc_matrix(a) if np.count_nonzero(a) < 0.5 * a.size else a
        huber_terms = cp.huber(a_op @ x - rhs, delta)
        data_loss = 0.5 * cp.sum(cp.multiply(w_vec, huber_terms) if w_vec is not None else huber_terms)

    lam_sqrt = cp.Parameter(nonneg=True)
    if metric is None:
        eig_p, vec_p = np.linalg.eigh(1e-6 * projector)
        pos_p = eig_p > 1e-18
        reg_loss = 0.5 * cp.sum_squares(lam_sqrt * x)
        if np.any(pos_p):
            p_half = (vec_p[:, pos_p] * np.sqrt(eig_p[pos_p])).T
            reg_loss = 0.5 * cp.sum_squares(p_half @ x) + reg_loss
    else:
        eig_m, vec_m = np.linalg.eigh(metric)
        m_half = (vec_m * np.sqrt(np.maximum(eig_m, 0.0))).T
        reg_loss = 0.5 * cp.sum_squares(lam_sqrt * (m_half @ x))

    prob = cp.Problem(cp.Minimize(data_loss + reg_loss), constraints)
    if not prob.is_dcp(dpp=True):
        raise RuntimeError('Rigid conic problem violates DPP structure')

    solve_kwargs = dict(
        solver=cp.CLARABEL,
        tol_gap_abs=1e-9,
        tol_feas=1e-9,
        tol_gap_rel=1e-9,
        max_iter=200,
    )
    start = perf_counter()
    history = []
    ridge_scan = None

    def _check_candidate(theta_vec):
        eigs = [
            float(np.linalg.eigvalsh(pseudo_inertia(block)).min())
            for block in theta_vec[:rigid_count].reshape(-1, 10)
        ]
        if min(eigs) <= 0:
            return None
        if e_full is not None:
            eq_err = float(np.max(np.abs(e_full @ theta_vec - eq_value), initial=0.0))
            if eq_err > 1e-6 * (1.0 + float(np.max(np.abs(eq_value), initial=0.0))):
                return None
        return eigs

    if ridges is None:
        selected_ridge = float(ridge)
        lam_sqrt.value = float(np.sqrt(selected_ridge))
        prob.solve(**solve_kwargs)
        iters = int(getattr(prob.solver_stats, 'num_iters', 0) or 0)
        history.append(dict(status=str(prob.status), iterations=iters))
        if prob.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE) or x.value is None:
            raise RuntimeError(f'Physical fit failed: {prob.status}')
        theta = prior + scale * np.asarray(x.value, dtype=float).ravel()
        eigen = [
            float(np.linalg.eigvalsh(pseudo_inertia(block)).min())
            for block in theta[:rigid_count].reshape(-1, 10)
        ]
        if min(eigen) <= 0:
            raise RuntimeError('Solver returned a nonphysical candidate')
        if e_full is not None and np.max(np.abs(e_full @ theta - eq_value), initial=0.0) > 1e-6 * (
            1.0 + np.max(np.abs(eq_value), initial=0.0)
        ):
            raise RuntimeError('Frozen-stage equalities were not preserved')
    else:
        y_val, target_val, use_full = parsed_val
        prior_eff = prior if use_full else prior[:rigid_count]
        res_prior = y_val @ prior_eff - target_val
        prior_joint_rmse = (
            weighted_rmse(res_prior.reshape(-1, 14),
                          None if val_weights is None else val_weights.reshape(-1, 14), axis=0)
            if len(target_val) % 14 == 0
            else None
        )
        ridge_scan = []
        valid_candidates = []
        for r_val in ridges:
            lam_sqrt.value = float(np.sqrt(r_val))
            prob.solve(**solve_kwargs)
            iters = int(getattr(prob.solver_stats, 'num_iters', 0) or 0)
            history.append(dict(ridge=r_val, status=str(prob.status), iterations=iters))
            if prob.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE) or x.value is None:
                ridge_scan.append(dict(ridge=r_val, status=str(prob.status), validation_rmse_nm=None))
                continue
            theta_cand = prior + scale * np.asarray(x.value, dtype=float).ravel()
            eigs_cand = _check_candidate(theta_cand)
            if eigs_cand is None:
                ridge_scan.append(dict(ridge=r_val, status='nonphysical_or_equality_violation', validation_rmse_nm=None))
                continue
            theta_eff = theta_cand if use_full else theta_cand[:rigid_count]
            res_val = y_val @ theta_eff - target_val
            val_rmse = float(weighted_rmse(res_val, val_weights))
            max_deg = 0.0
            non_degrading = True
            if prior_joint_rmse is not None:
                joint_rmse = weighted_rmse(res_val.reshape(-1, 14),
                                          None if val_weights is None else val_weights.reshape(-1, 14), axis=0)
                max_deg = float(np.nanmax(joint_rmse - prior_joint_rmse))
                non_degrading = bool(max_deg <= 0.05)
            entry = dict(
                ridge=r_val,
                status=str(prob.status),
                iterations=iters,
                validation_rmse_nm=val_rmse,
                max_joint_degradation_nm=max_deg,
                non_degrading=non_degrading,
            )
            ridge_scan.append(entry)
            valid_candidates.append((non_degrading, val_rmse, max_deg, r_val, theta_cand, eigs_cand))

        if not valid_candidates:
            raise RuntimeError('Physical fit failed across ridge_grid')
        safe_pool = [c for c in valid_candidates if c[0]]
        pool = safe_pool if safe_pool else valid_candidates
        best = min(pool, key=lambda c: (c[1] + max(0.0, c[2] - 0.05), -c[3]))
        _, _, _, selected_ridge, theta, eigen = best

    result = dict(
        parameters=theta[:rigid_count].tolist(),
        auxiliary_parameters=theta[rigid_count:].tolist(),
        rank=rank,
        columns=len(prior),
        singular_values=singular.tolist(),
        parameter_scales=scale.tolist(),
        observable_scaled_basis=vh[:rank].tolist(),
        solver=history,
        minimum_pseudo_inertia_eigenvalues=eigen,
        rmse_nm=float(np.sqrt(np.mean((y @ theta - target) ** 2))),
        wall_s=perf_counter() - start,
        unobservable_dimension=len(prior) - rank,
        scaled_prior_ridge=selected_ridge,
        physical_congruence_length_m=physical_length_scale,
        coordinate_scale_source='prior_dependent_legacy' if fixed_scales is None else 'explicit_frozen',
        regularization_metric_source=(
            'legacy_identity_plus_data_nullspace' if metric is None else 'explicit_complete_quadratic'
        ),
        data_modes_above_ridge=int(np.count_nonzero(singular > np.sqrt(selected_ridge))),
        least_squares_formulation=(
            'QR compressed lifted residuals' if robust_scale is None else 'CVXPY exact conic Huber'
        ),
        identifiability=(
            'observable combinations fitted; nullspace representative selected by physical constraints and nominal prior'
        ),
        unique_body_parameters=rank == len(prior),
    )
    if parsed_val is not None:
        y_val, target_val, use_full = parsed_val
        theta_eff = theta if use_full else theta[:rigid_count]
        result['validation_rmse_nm'] = float(weighted_rmse(y_val @ theta_eff - target_val, val_weights))
    if ridge_scan is not None:
        result['ridge_scan'] = ridge_scan
    return result
