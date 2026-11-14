"""Single-pass saved-zero reference using the existing C3/Coal planning kernel."""
import numpy as np
from .shm_data import NAMES


def plan_zero(start,target,kp,kd,duration_s=5.):
    from .shm_h1 import load_profile,DEFAULT_PROFILE
    from .model_runtime import model_for,collision_for
    from .staged_motion import Motion,gate_motion,derivative_bounds
    from .static_field import _move
    from .shm_reference import export_motion
    start=np.asarray(start,float);target=np.asarray(target,float)
    if start.shape!=(14,) or target.shape!=(14,) or not np.isfinite([start,target]).all() or not np.isfinite(duration_s) or duration_s<=0:
        raise ValueError('需要完整有限起点/零位及正时长')
    config=load_profile(DEFAULT_PROFILE);engine,_=model_for(config)
    motion=Motion('shm_saved_zero_return','position',start)
    motion.add(config['initial_hold_s'],start,label='measured_start')
    _move(motion,target,duration_s,dict(speed_rad_s=min(.3,config['maximum_velocity_rad_s']),acceleration_rad_s2=.5,jerk_rad_s3=2.),'to_saved_zero')
    motion.add(config['final_hold_s'],target,label='saved_zero_final_hold')
    motion.knots=[round(t*1e9)*1e-9 for t in motion.knots]
    gate=gate_motion(collision_for(config,engine),motion,config['scene']['clearance_m'])
    if not gate['passed']:raise ValueError('回零连续参考/URDF/碰撞检查未通过：'+gate.get('reason','clearance'))
    if np.max(derivative_bounds(motion.polynomial()))>config['maximum_velocity_rad_s']:raise ValueError('回零参考速度越限')
    bundle=export_motion(motion,kp=kp,kd=kd)
    bundle.update(purpose='shm_gui_zero_collision_checked',observed_velocity_limit_rad_s=config['maximum_velocity_rad_s'])
    return dict(bundle=bundle,certificate=gate,model_context=config['model_context'])


def validate_zero(value,identity,mock):
    if value.get('schema_version')!='casbot_shm_zero_v1' or value.get('units')!='rad' or value.get('model_context')!=identity or tuple(value.get('joint_names',()))!=NAMES or value.get('scope')!=('private_mock' if mock else 'hardware_shm'):
        raise ValueError('零位格式/模型/轴序/真实或Mock范围不匹配，请重新采集')
    target=np.asarray(value['q_rad'],float)
    if target.shape!=(14,) or not np.isfinite(target).all():raise ValueError('零位缺少完整有限位置')
    return target
