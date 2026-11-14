"""Reusable scripted arm preparation; only this prefix skips collision checks.

One composed reference keeps the same native owner and enable cycle through
preparation and calibration. No GUI, device access, or encoder-zero writes.
"""
import numpy as np
from .staged_motion import Motion, derivative_bounds
from .static_field import _move


def ready_position():
    q=np.zeros(14);q[1]=np.deg2rad(20.);q[8]=-q[1]
    return q


def preparation(start,engine,maximum_velocity_rad_s=1.5):
    """Rest-to-rest C3 motion, 20/2 PD, zero FF until explicit handover."""
    q=ready_position();motion=Motion('shm_arm_ready','preparation',start)
    motion.add(.5,start,label='ready_start_hold')
    envelope=dict(speed_rad_s=min(.15,maximum_velocity_rad_s),acceleration_rad_s2=.3,jerk_rad_s3=1.)
    _move(motion,q,5.,envelope,'to_ready_pose')
    move_end=round(motion.duration*1e9)
    motion.add(2.,q,label='ready_field_handover')
    handover_end=round(motion.duration*1e9)
    motion.add(1.,q,label='ready_hold')
    motion.knots=[round(t*1e9)*1e-9 for t in motion.knots]
    curve=motion.polynomial();lower=curve.c.min(axis=(0,1));upper=curve.c.max(axis=(0,1))
    joints=engine.model.joints
    travel=bool(all(j.lower<=a and b<=j.upper for a,b,j in zip(lower,upper,joints)))
    speed=derivative_bounds(curve)
    velocity=bool(all(speed[:,i].max()<=min(maximum_velocity_rad_s,j.velocity_limit)*(1+1e-9) for i,j in enumerate(joints)))
    certificate=dict(passed=travel and velocity,collision_check='skipped_user_authorized_preparation_only',
        joint_limits=dict(passed=travel,lower_rad=lower.tolist(),upper_rad=upper.tolist(),method='Bernstein coefficient convex hull'),
        reference_velocity=dict(passed=velocity,maximum_rad_s=float(speed.max())),
        target_rad=q.tolist(),kp=[20.]*14,kd=[2.]*14,move_end_ns=move_end,handover_end_ns=handover_end,
        calibration_begin_ns=round(motion.duration*1e9),
        feedforward='zero during unfolding; explicit smooth handover to selected calibration feedforward at fixed ready reference')
    if not certificate['passed']:raise ValueError('准备姿态连续URDF/参考速度检查未通过；没有发送命令')
    return motion,certificate


def prepend(prefix,formal):
    """Join complete derivative states; no interpolation change or disable gap."""
    if not np.array_equal(prefix.states[-1],formal.states[0]):raise ValueError('准备段与正式参考的q/dq/ddq/jerk不连续')
    joined=Motion(formal.name,formal.stage,prefix.home,dict(formal.conditions))
    offset_ns=round(prefix.duration*1e9)
    joined.knots=prefix.knots[:]+[(offset_ns+round(t*1e9))*1e-9 for t in formal.knots[1:]]
    joined.states=prefix.states[:]+formal.states[1:]
    joined.labels=prefix.labels[:]+formal.labels[:]
    joined.conditions['calibration_begin_s']=offset_ns*1e-9
    if 'comparison_window_s' in joined.conditions:
        joined.conditions['comparison_window_s']=[(offset_ns+round(t*1e9))*1e-9 for t in joined.conditions['comparison_window_s']]
    return joined


def handover_fraction(ns,certificate):
    u=np.clip((np.asarray(ns)-certificate['move_end_ns'])/(certificate['handover_end_ns']-certificate['move_end_ns']),0.,1.)
    return u*u*u*(10.+u*(-15.+6.*u))


def pd_schedule(certificate,kp,kd):
    return [dict(t_ns=0,kp=[20.]*14,kd=[2.]*14),
            dict(t_ns=certificate['move_end_ns'],kp=[20.]*14,kd=[2.]*14),
            dict(t_ns=certificate['handover_end_ns'],kp=list(kp),kd=list(kd))]
