"""Four near-ready targets / eight opposite-approach static windows, offline."""
from copy import deepcopy
from pathlib import Path
import numpy as np
import yaml
from .data import write_json
from .field_artifacts import output_outside,seal,FieldContractError
from .model_runtime import model_for,collision_for
from .shm_data import NAMES,ROOT
from .shm_ready import ready_position
from .static_field import paired_motions
from .staged_motion import gate_motion,derivative_bounds


DEFAULT_SETTINGS=ROOT/'configs/shm/static_small.yaml'


def make_preset(profile,output,*,settings=DEFAULT_SETTINGS):
    from .shm_h1 import load_profile
    config=load_profile(profile) if not isinstance(profile,dict) else profile
    engine,source=model_for(config);geometry=collision_for(config,engine)
    c=yaml.safe_load(Path(settings).read_text()) if not isinstance(settings,dict) else deepcopy(settings)
    if c.get('schema_version')!='casbot_shm_small_static_v1':raise ValueError('需要少量静态预设配置')
    values=[c['target_offset_deg'],c['approach_offset_deg'],*c['timing'].values(),*c['planning_envelope'].values()]
    if not np.isfinite(values).all() or min(values)<=0 or c['target_offset_deg']+c['approach_offset_deg']>20 or c['planning_envelope']['speed_rad_s']>.05:
        raise ValueError('少量静态范围须为正有限值，含接近点不超过±20°，参考速度不超过0.05rad/s')
    c.update(home_rad=ready_position().tolist(),model_context=config['model_context'],command_period_ns=config['command_period_ns'],
        approach_offset_rad=float(np.deg2rad(c['approach_offset_deg'])),
        scope='four shoulder-pitch targets near ready; eight static windows; no full-arm identification coverage')
    root=output_outside(output);root.mkdir(parents=True,exist_ok=False)
    rows=[];order=[];gates=[];duration=0.
    for arm,j in (('left',0),('right',7)):
        for split,angle in (('train',c['target_offset_deg']),('validation',-c['target_offset_deg'])):
            q=ready_position();q[j]=np.deg2rad(angle)
            row=dict(id=f'{arm}_pitch_{"pos" if angle>0 else "neg"}',arm=arm,split=split,
                q_rad=q.tolist(),family='single_axis',active_joints=[j],offsets_deg=[angle],status='selected')
            rows.append(row);trial_ids=[]
            for motion in paired_motions(row,c,split=split):
                motion.knots=[round(t*1e9)*1e-9 for t in motion.knots]
                proof=gate_motion(geometry,motion,config['scene']['clearance_m'])
                speed=float(np.max(derivative_bounds(motion.polynomial())))
                proof['reference_velocity_max_rad_s']=speed
                folder=root/'plans'/motion.name;folder.mkdir(parents=True)
                write_json(folder/'motion.json',motion.metadata());write_json(folder/'collision_reference.json',proof)
                if not proof['passed'] or speed>.05*(1+1e-9):
                    write_json(root/'report.json',dict(passed=False,reason='small_static_reference_rejected',trial=motion.name));seal(root)
                    raise FieldContractError('少量静态参考碰撞/URDF/速度检查未通过；没有连接设备')
                order.append(motion.name);trial_ids.append(motion.name);gates.append(proof);duration+=motion.duration
            row['trial_ids']=trial_ids
    for name,value in (('config.json',c),('scene.json',config['scene']),('candidates.json',rows)):
        write_json(root/name,value)
    report=dict(passed=True,source_model_sha256=source['sha256'],model_context=config['model_context'],joint_names=list(NAMES),
        planned_execution_order=order,scope=c['scope'],target_count=4,measurement_windows=8,
        nominal_measurement_s=8*c['timing']['measure_s'],reference_duration_s=duration,
        minimum_sampled_distance_m=min(g['minimum_distance_m'] for g in gates),
        target_shoulder_pitch_deg=[c['target_offset_deg'],-c['target_offset_deg']],
        including_approach_shoulder_pitch_range_deg=[-(c['target_offset_deg']+c['approach_offset_deg']),c['target_offset_deg']+c['approach_offset_deg']])
    write_json(root/'report.json',report);seal(root);return report
