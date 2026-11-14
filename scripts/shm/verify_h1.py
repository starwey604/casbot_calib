#!/usr/bin/env python3
"""Private numerical H1 acceptance: actual measured snapshot, Coal, native physics and PSD fit."""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'python'))
from casbot_calib.shm_h1 import snapshot,prepare,execute,analyse,load_profile
from casbot_calib.shm_a_evidence import analyse as analyse_a
from casbot_calib.static_field import paired_motions
from casbot_calib.field_artifacts import seal
from casbot_calib.model_runtime import model_for
from casbot_calib.data import write_json


def small_preset(root,config,home):
    """Explicit two-pose test fixture, not the full 64-pose commissioning preset."""
    engine,source=model_for(config);c=yaml.safe_load((ROOT/'configs/static_field_nominal.yaml').read_text())
    c['home_rad']=home;c['approach_offset_rad']=.0005
    c['timing']=dict(initial_hold_s=.05,minimum_transition_s=.15,minimum_paired_approach_s=.1,settle_s=.1,measure_s=.3,final_hold_s=.05)
    root.mkdir();rows=[];order=[]
    for k,split in enumerate(('train','validation')):
        q=np.array(home);q[0]+=.002*(1 if k==0 else -1)
        row=dict(id=f'private_fixture_{k}',arm='left',status='selected',split=split,q_rad=q.tolist(),family='single_axis',active_joints=[0],offsets_deg=[.1])
        rows.append(row)
        for motion in paired_motions(row,c,split=split):
            folder=root/'plans'/motion.name;folder.mkdir(parents=True);write_json(folder/'motion.json',motion.metadata());order.append(motion.name)
    for name,value in (('config.json',c),('candidates.json',rows),('scene.json',config['scene'])):write_json(root/name,value)
    write_json(root/'report.json',dict(passed=True,source_model_sha256=source['sha256'],joint_names=[j.name for j in engine.model.joints],planned_execution_order=order,scope='two-pose private physics fixture only'))
    seal(root)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bin-dir',type=Path,required=True);p.add_argument('--output',type=Path)
    a=p.parse_args();temporary=None
    if a.output:root=a.output.resolve();root.mkdir(parents=True,exist_ok=False)
    else:temporary=tempfile.TemporaryDirectory(prefix='casbot-h1-');root=Path(temporary.name)
    try:
        binary=a.bin_dir.resolve()/'shm_executor';config=yaml.safe_load((ROOT/'configs/shm/hardware_field.yaml').read_text())
        config['collision_scene']=str(ROOT/'configs/collision_scene.yaml');config['initial_hold_s']=config['final_hold_s']=.1;config['anchor_transition_s']=.2
        profile=root/'profile.yaml';profile.write_text(yaml.safe_dump(config))
        start=root/'snapshot';snapshot(profile,start,executor=binary,mock=True)
        jog=root/'jog';prepare(profile,start,jog,executor=binary,amplitude_deg=.1)
        capture=root/'jog_capture';assert execute(jog,capture,executor=binary,mock=True,do_execute=True)==0
        tracking=analyse(jog,capture,root/'tracking');assert tracking['analysis_available']
        duration=json.loads((jog/'plan.json').read_text())['duration_ns']*1e-9
        diagnostic=analyse_a(capture,root/'a',profile=profile,window=(.15,duration-.15),joint_id=12,band=5.)
        # No numerical-quality gate: an unidentifiable A window is an explicit result.
        config=load_profile(profile);home=json.loads((jog/'motion.json').read_text())['states'][0][0]
        preset=root/'preset';small_preset(preset,config,home)
        plan=root/'static';prepare(profile,start,plan,preset=preset,executor=binary)
        static_capture=root/'static_capture';assert execute(plan,static_capture,executor=binary,mock=True,do_execute=True)==0
        result=analyse(plan,static_capture,root/'gravity')
        assert result['execution_completed'] and result['analysis_available'],result
        assert result['fit_succeeded'] and result['passed'] and not result['hardware_calibration_accepted']
        assert json.loads((root/'gravity/urdf_reload.json').read_text())['passed']
        summary=dict(passed=True,scope='H1_private_two_pose_physics_and_PSD_export',hardware_tested=False,
            jog_feedback_rows=tracking['feedback_rows'],static_feedback_rows=result['feedback_rows'],A_available=diagnostic['available'],
            static_windows=4,gravity_fit_succeeded=result['fit_succeeded'])
        write_json(root/'verification.json',summary);print(json.dumps(summary,indent=2))
    finally:
        if temporary:temporary.cleanup()


if __name__=='__main__':main()
