"""K2 task IK and full, bounded approach/withdrawal segments; no calibration fitting."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .staged_motion import Motion, derivative_bounds, gate_motion


def solve_position(engine, frame, point, seed, orientation=None):
    active=np.arange(7) if frame.startswith('left_') else np.arange(7,14)
    seed=np.asarray(seed,dtype=float);point=np.asarray(point,dtype=float);reference=engine.frame_pose(seed,frame)[:3,:3] if orientation is None else orientation
    limits=np.array([[j.lower+.02,j.upper-.02] for j in engine.model.joints])[active]
    def residual(values):
        q=seed.copy();q[active]=values;actual=engine.frame_pose(q,frame)
        return np.r_[actual[:3,3]-point,(.1 if orientation is not None else .0002)*Rotation.from_matrix(reference.T@actual[:3,:3]).as_rotvec(),.000005*(values-seed[active])]
    solution=least_squares(residual,np.clip(seed[active],limits[:,0]+1e-8,limits[:,1]-1e-8),bounds=(limits[:,0],limits[:,1]),
        xtol=1e-11,ftol=1e-11,gtol=1e-11,max_nfev=180)
    q=seed.copy();q[active]=solution.x
    if np.linalg.norm(engine.frame_pose(q,frame)[:3,3]-point)>.0001:raise ValueError('No accurate nominal task IK')
    return q


def segment(profile,start,end,config,label,duration=None):
    settings=config['workflow'];duration=duration or settings['transition_s']
    motion=Motion(label,'trical',np.asarray(start)).add(duration,end,label=label)
    curve=motion.polynomial()
    limits=[settings['maximum_velocity_rad_s'],settings['maximum_acceleration_rad_s2'],settings['maximum_jerk_rad_s3']]
    peaks=[float(np.max(derivative_bounds(curve,order))) for order in (1,2,3)]
    if any(a>b for a,b in zip(peaks,limits)):
        factor=max(a/b for a,b in zip(peaks,limits));return segment(profile,start,end,config,label,duration*max(1.2,factor**1.01))
    # Account for declared unknown zero/tracking envelope, not fitted hidden values.
    motion.conditions['position_allowance_rad']=[settings['geometry_allowance_rad']]*14
    travel=np.array([[j.lower,j.upper] for j in profile.engine.model.joints]);allowance=settings['geometry_allowance_rad']
    if np.any(curve.c.min(axis=(0,1))-allowance<travel[:,0]) or np.any(curve.c.max(axis=(0,1))+allowance>travel[:,1]):
        return motion,dict(passed=False,reason='uncertain_position_outside_joint_limits',motion=label)
    from .collision import certify_reference
    gate=gate_motion(profile,motion,settings['clearance_m'])
    if gate['passed']:
        radii=profile.radii@np.full(14,settings['geometry_allowance_rad'])
        probe=profile.check(curve(np.linspace(0,duration,9)))['distances']
        thresholds=np.asarray(getattr(profile,'pair_clearance_m',np.full(len(profile.pairs),settings['clearance_m'])))
        if np.any(probe-radii[None,:]<=thresholds):
            gate['passed']=False;gate['reason']='declared_geometry_envelope_not_certified_at_probe'
            return motion,gate
        expanded=certify_reference(profile,lambda t:curve(t),duration,derivative_bounds(curve)[0],
            clearance_m=settings['clearance_m'],initial_step_s=duration,position_allowance=np.full(14,settings['geometry_allowance_rad']))
        gate['uncertainty_envelope']=expanded;gate['passed']=expanded['passed']
    gate['derivative_peaks']=peaks
    return motion,gate


def local_correction(engine, frame, q, offset_sensor, config):
    settings=config['workflow'];active=np.arange(7) if frame.startswith('left_') else np.arange(7,14)
    transform=engine.frame_pose(q,frame);delta=transform[:3,:3]@np.asarray(offset_sensor)
    delta*=min(1.,settings['maximum_cartesian_step_m']/max(np.linalg.norm(delta),1e-15))
    jacobian=engine.frame_jacobian(q,frame)[:,active]
    # Small position correction, preserve current orientation; regularised least squares.
    design=np.vstack([jacobian[:3],.08*jacobian[3:],.001*np.eye(7)])
    step=np.linalg.lstsq(design,np.r_[delta,np.zeros(10)],rcond=None)[0]
    step*=min(1.,settings['maximum_joint_step_rad']/max(np.max(abs(step)),1e-15))
    result=q.copy();result[active]+=step
    return result
