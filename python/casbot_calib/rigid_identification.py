"""Full Pinocchio inertial parameters with physically consistent conic fitting.

Clarabel solves the convex quadratic/PSD problem. NumPy only assembles parameter
coordinates and constraints; all robot regressors and dynamics are Pinocchio.
Unobservable components select a prior-near physical representative and are not
reported as independent mass/COM measurements.
"""
from time import perf_counter
import numpy as np
from scipy import sparse
from . import _core


def pseudo_inertia(theta):
    """4x4 pseudo-inertia from [m,hx,hy,hz,Ixx,Ixy,Iyy,Ixz,Iyz,Izz]."""
    theta=np.asarray(theta,dtype=float)
    if theta.shape!=(10,) or not np.isfinite(theta).all():raise ValueError('Expected ten finite rigid parameters')
    m=theta[0];h=theta[1:4];xx,xy,yy,xz,yz,zz=theta[4:]
    inertia=np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])
    result=np.empty((4,4));result[:3,:3]=.5*np.trace(inertia)*np.eye(3)-inertia
    result[:3,3]=h;result[3,:3]=h;result[3,3]=m
    return result


def _svec(matrix):
    return np.array([matrix[i,j]*(1 if i==j else np.sqrt(2)) for j in range(len(matrix)) for i in range(j+1)])


def model_from_parameters(nominal,parameters,viscous=0.,coulomb=0.,static=None,smoothing_velocity=.001,stribeck_velocity=.002):
    model=nominal.model;joints=model.joints;n=len(joints);p=np.asarray(parameters,dtype=float).reshape(n,10)
    b,fc=(np.broadcast_to(np.asarray(x,dtype=float),(n,)) for x in (viscous,coulomb))
    fs=fc if static is None else np.broadcast_to(np.asarray(static,dtype=float),(n,))
    for k,(joint,theta) in enumerate(zip(joints,p)):
        if theta[0]<=0 or np.linalg.eigvalsh(pseudo_inertia(theta)).min()<=0:raise ValueError('Nonphysical rigid parameter block')
        body=joint.body;body.mass=float(theta[0]);body.com=theta[1:4]/body.mass
        xx,xy,yy,xz,yz,zz=theta[4:];origin=np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])
        c=body.com;body.inertia_com=origin-body.mass*(np.dot(c,c)*np.eye(3)-np.outer(c,c));joint.body=body
        friction=joint.friction;friction.viscous=float(b[k]);friction.coulomb=float(fc[k]);friction.static_friction=float(fs[k])
        friction.smoothing_velocity=smoothing_velocity;friction.stribeck_velocity=stribeck_velocity;joint.friction=friction
    model.joints=joints
    # The native constructor independently validates COM inertia and its triangle
    # inequality. It does not use this module's pseudo-inertia implementation.
    return _core.ArmDynamics(model)


def parameter_scales(prior):
    rows=np.asarray(prior,dtype=float).reshape(-1,10)
    # SI coordinate scales, not observations or hard manufacturer bounds.
    typical=rows[:,0,None]*np.array([1.,.1,.1,.1,.01,.01,.01,.01,.01,.01])
    return np.maximum(np.abs(rows),typical).ravel()


def fit_rigid(regressor,target,prior,*,equalities=None,auxiliary=None,torque_scale=.2,rank_rtol=1e-7,robust_scale=.02,ridge=1e-12,physical_length_scale=1.,lift_residuals=False,fixed_scales=None,regularization_metric=None):
    import clarabel
    prior=np.asarray(prior,dtype=float);y=np.asarray(regressor,dtype=float).reshape(-1,len(prior));target=np.asarray(target,dtype=float).ravel()
    if prior.ndim!=1 or len(prior)==0 or len(prior)%10 or y.shape[0]!=len(target) or len(target)==0 or not np.isfinite(y).all() or not np.isfinite(target).all() or not np.isfinite(prior).all() or torque_scale<=0 or rank_rtol<=0 or (robust_scale is not None and robust_scale<=0) or ridge<=0 or not np.isfinite(ridge) or physical_length_scale<=0 or not np.isfinite(physical_length_scale):
        raise ValueError('Invalid rigid identification data')
    scale=parameter_scales(prior)
    rigid_count=len(prior)
    if auxiliary is not None:
        extra,extra_prior,extra_lower,extra_upper=auxiliary
        extra=np.asarray(extra,dtype=float).reshape(len(target),-1);extra_prior,extra_lower,extra_upper=(np.asarray(x,dtype=float) for x in (extra_prior,extra_lower,extra_upper))
        if extra_prior.ndim!=1 or extra.shape[1]!=len(extra_prior) or extra_lower.shape!=extra_prior.shape or extra_upper.shape!=extra_prior.shape or not all(np.isfinite(x).all() for x in (extra,extra_prior,extra_lower,extra_upper)) or np.any(extra_lower>extra_prior) or np.any(extra_upper<extra_prior) or np.any(extra_lower>=extra_upper):
            raise ValueError('Invalid bounded auxiliary parameter block')
        y=np.column_stack([y,extra]);prior=np.r_[prior,extra_prior];scale=np.r_[scale,np.maximum(abs(extra_prior),.001)]
    if fixed_scales is not None:
        scale=np.asarray(fixed_scales,dtype=float).copy()
        if scale.shape!=prior.shape or not np.isfinite(scale).all() or np.any(scale<=0):
            raise ValueError('Expected fixed positive scales for the complete rigid/auxiliary block')
    metric=None
    if regularization_metric is not None:
        metric=np.asarray(regularization_metric,dtype=float).copy()
        if metric.shape!=(len(prior),len(prior)) or not np.isfinite(metric).all() or not np.allclose(metric,metric.T,rtol=0,atol=1e-12):
            raise ValueError('Expected a finite symmetric regularization metric')
        if np.linalg.eigvalsh(metric).min()<=0:raise ValueError('Regularization metric must be positive definite')
    if np.any(scale<=0):raise ValueError('Positive physical prior required')
    a=y*scale/torque_scale;rhs=(target-y@prior)/torque_scale
    _,singular,vh=np.linalg.svd(a,full_matrices=False);rank=int(np.count_nonzero(singular>max(singular[0]*rank_rtol,1e-12)))
    # full_matrices=False may omit nullspace rows when there are few observations.
    projector=np.eye(len(prior))-vh[:rank].T@vh[:rank]
    congruence=np.diag([1/physical_length_scale]*3+[1.])
    physical=np.column_stack([_svec(congruence@pseudo_inertia(row)@congruence) for row in np.eye(10)])
    rows=[];bounds=[];cones=[];eps=1e-10
    for j in range(rigid_count//10):
        section=slice(j*10,(j+1)*10);block=np.zeros((10,len(prior)));block[:,section]=-physical*scale[section]
        rows.append(block);bounds.append(physical@prior[section]-eps*_svec(congruence@congruence));cones.append(clarabel.PSDTriangleConeT(4))
    if auxiliary is not None:
        block=np.zeros((len(extra_prior),len(prior)));block[:,rigid_count:]=np.diag(scale[rigid_count:])
        rows.extend([block,-block]);bounds.extend([extra_upper-extra_prior,extra_prior-extra_lower]);cones.extend([clarabel.NonnegativeConeT(len(extra_prior))]*2)
    if equalities is not None:
        e,value=equalities;e=np.asarray(e,dtype=float);value=np.asarray(value,dtype=float)
        if e.ndim!=2 or e.shape[1]!=rigid_count or value.shape!=(e.shape[0],) or not np.isfinite(e).all() or not np.isfinite(value).all():raise ValueError('Invalid frozen-stage equalities')
        if auxiliary is not None:e=np.column_stack([e,np.zeros((len(e),len(extra_prior)))])
        # Remove redundant equality rows with SVD to avoid a singular KKT system.
        u,s,v=np.linalg.svd(e*scale,full_matrices=False);r=int(np.count_nonzero(s>max(s[0]*rank_rtol,1e-12)))
        if r:rows.append(v[:r]);bounds.append((u[:,:r].T@(value-e@prior))/s[:r]);cones.append(clarabel.ZeroConeT(r))
    constraints=sparse.csc_matrix(np.vstack(rows));bound=np.concatenate(bounds)
    settings=clarabel.DefaultSettings();settings.verbose=False;settings.max_iter=150
    settings.tol_gap_abs=1e-9;settings.tol_feas=1e-9;settings.tol_gap_rel=1e-9
    weights=np.ones(len(rhs));start=perf_counter();history=[]
    for _ in range(4):
        # An explicit metric is the complete quadratic in the caller's frozen
        # coordinates. Do not change it using a subset-dependent nullspace.
        regularizer=(1e-6*projector+ridge*np.eye(len(prior))) if metric is None else ridge*metric
        if lift_residuals:
            # QR preserves the weighted squared residual up to an irrelevant
            # constant. Explicit residual variables avoid forming A.T@A, which
            # squares the condition number in weak inertial directions.
            square_root=np.sqrt(weights);orthogonal,triangular=np.linalg.qr(a*square_root[:,None],mode='reduced')
            compressed=orthogonal.T@(rhs*square_root);count=triangular.shape[0]
            h=sparse.block_diag((sparse.csc_matrix(regularizer),sparse.eye(count,format='csc')),format='csc')
            lifted=sparse.vstack([sparse.hstack([constraints,sparse.csc_matrix((len(bound),count))]),
                sparse.hstack([-sparse.csc_matrix(triangular),sparse.eye(count,format='csc')])],format='csc')
            solver=clarabel.DefaultSolver(sparse.triu(h).tocsc(),np.zeros(len(prior)+count),lifted,np.r_[bound,-compressed],cones+[clarabel.ZeroConeT(count)],settings)
        else:
            h=a.T@(a*weights[:,None])+regularizer;linear=-(a.T@(weights*rhs))
            solver=clarabel.DefaultSolver(sparse.triu(sparse.csc_matrix(h)).tocsc(),linear,constraints,bound,cones,settings)
        solution=solver.solve();history.append(dict(status=str(solution.status),iterations=solution.iterations))
        if str(solution.status) not in ('Solved','AlmostSolved'):raise RuntimeError(f'Physical fit failed: {solution.status}')
        x=np.asarray(solution.x)[:len(prior)];residual=(a@x-rhs)*torque_scale
        if robust_scale is None:break
        weights=1/np.sqrt(1+(residual/robust_scale)**2)
    theta=prior+scale*x
    eigen=[float(np.linalg.eigvalsh(pseudo_inertia(t)).min()) for t in theta[:rigid_count].reshape(-1,10)]
    if min(eigen)<=0:raise RuntimeError('Solver returned a nonphysical candidate')
    if equalities is not None and np.max(abs(e@theta-value),initial=0.)>1e-6*(1+np.max(abs(value),initial=0.)):
        raise RuntimeError('Frozen-stage equalities were not preserved')
    return dict(parameters=theta[:rigid_count].tolist(),auxiliary_parameters=theta[rigid_count:].tolist(),rank=rank,columns=len(prior),singular_values=singular.tolist(),
        parameter_scales=scale.tolist(),observable_scaled_basis=vh[:rank].tolist(),solver=history,
        minimum_pseudo_inertia_eigenvalues=eigen,rmse_nm=float(np.sqrt(np.mean((y@theta-target)**2))),
        wall_s=perf_counter()-start,unobservable_dimension=len(prior)-rank,
        scaled_prior_ridge=ridge,physical_congruence_length_m=physical_length_scale,
        coordinate_scale_source='prior_dependent_legacy' if fixed_scales is None else 'explicit_frozen',
        regularization_metric_source='legacy_identity_plus_data_nullspace' if metric is None else 'explicit_complete_quadratic',
        data_modes_above_ridge=int(np.count_nonzero(singular>np.sqrt(ridge))),
        least_squares_formulation='QR compressed lifted residuals' if lift_residuals else 'normal-equation quadratic',
        identifiability='observable combinations fitted; nullspace representative selected by physical constraints and nominal prior',
        unique_body_parameters=rank==len(prior))
