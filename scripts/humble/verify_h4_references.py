#!/usr/bin/env python3
"""Offline nominal full-14-axis H4 reference audit, without SDK or observations."""
import argparse
from pathlib import Path
import numpy as np
import yaml
from casbot_calib.hardware_inertia import settings
from casbot_calib.hardware_friction import center
from casbot_calib.dual_model import nominal_dual_arm
from casbot_calib.collision import nominal_profile
from casbot_calib.staged_motion import periodic_motion,derivative_bounds,gate_motion
from casbot_calib.data import write_json


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=Path('configs/hardware_inertia.yaml'))
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    engine,_=nominal_dual_arm();names=[j.name for j in engine.model.joints];c=settings(a.config,names)
    scene=yaml.safe_load(Path('configs/collision_scene.yaml').read_text());profile=nominal_profile(scene=scene)
    jobs=[(role,offset,seed) for role in ('train','validation') for offset in c['pose_offsets_rad'][role] for seed in c[role+'_seeds']]
    jobs += [('selection',0.,c['control']['tuning_seed']),('control',0.,c['control']['validation_seed'])]
    recipe=dict(dynamic_period_s=c['period_s'],dynamic_periods=c['periods'],pose_amplitude_rad=c['amplitude_rad'],settle_s=c['settle_s'],
        dynamic_transition_s=c['transition_s'],smooth_periodic_envelope=True,dynamic_ramp_s=c['excitation_ramp_s'])
    reports=[]
    for role,offset,seed in jobs:
        motion=periodic_motion(role+'_'+str(seed)+'_'+str(offset),'inertia',c['home_rad'],center(c,offset),recipe,seed,role!='train',conditions=dict(refine_position_bounds=True))
        gate=gate_motion(profile,motion,scene['clearance_m']);curve=motion.polynomial()
        bounds=[float(np.max(derivative_bounds(curve,k))) for k in (1,2,3)]
        passed=gate['passed'] and bounds[0]<=.75 and bounds[1]<=c['maximum_acceleration_rad_s2'] and bounds[2]<=c['maximum_jerk_rad_s3']
        report=dict(name=motion.name,role=role,passed=bool(passed),maximum_certified_derivatives=bounds,collision=gate)
        write_json(a.output/(motion.name+'.json'),report);reports.append(report)
        print(motion.name,passed,flush=True)
    write_json(a.output/'verification.json',dict(passed=all(r['passed'] for r in reports),references=len(reports),joints=14,device_writes=0,
        scope='complete nominal references from a declared synthetic start, not real start/stop/tracking qualification',reports=reports))
    return 0 if all(r['passed'] for r in reports) else 1


if __name__=='__main__':raise SystemExit(main())
