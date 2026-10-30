#!/usr/bin/env python3
"""H4 over actual local ROS SDK transport; observations and evaluation separate."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid
import numpy as np
import yaml
from casbot_calib import hardware_batch as batch
from casbot_calib.hardware_inertia import plan_inertia,candidate_engine
from casbot_calib.sdk_mock import SDKMockSession
from casbot_calib.data import write_json


def truth_denied(root,tag='truth_denied'):
    code='''import os,sys
from pathlib import Path
def guard(event,args):
 if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)) and 'evaluation' in Path(os.fsdecode(args[0])).parts:raise RuntimeError('evaluation forbidden')
sys.addaudithook(guard)
try:open('/__probe__/evaluation/truth','rb')
except RuntimeError:pass
else:raise RuntimeError('guard inactive')
from casbot_calib.cli import main
if main(['hardware-analyse-batch',sys.argv[1],'--output',sys.argv[2]]):raise SystemExit(1)
import numpy as np
from casbot_calib import hardware_batch as batch,hardware_field as field
from casbot_calib.hardware_inertia import inertia_result
from casbot_calib.hardware_friction import gravity_engine,read
from casbot_calib.hardware_periodic import periodic_problem
from casbot_calib.data import write_json
root,p=batch.load_batch(sys.argv[1]);history,_=batch.outcomes(root,p);largest=0.;preprocessing={}
for job in p['jobs']:
 if job['kind']!='inertia':continue
 source=batch.accepted(history[job['id']]);folder=root/source['attempt'];plan=folder/'plan'
 meta,commands,origin,obs,elapsed,collision=field.field_observations(plan,folder/'capture')
 problem=periodic_problem(obs,elapsed,read(plan/'motion.json'),gravity_engine(plan/'gravity'),p['h4'],meta['config'],read(plan/'friction.json'))
 with np.load(folder/'inspection/periodic_problem.npz',allow_pickle=False) as saved:
  for key in saved.files:
   if key=='moving':assert np.array_equal(saved[key],problem[key])
   else:largest=max(largest,float(np.max(abs(saved[key]-problem[key]))))
 preprocessing[job['id']]=problem['preprocessing']
report,bundle=inertia_result(root,p,history);frozen=read(root/'inertia_checkpoint/candidate.json')
difference=float(np.max(abs(np.asarray(bundle['parameters'])-frozen['parameters']))) if bundle else None
passed=report['passed'] and largest<1e-9 and difference is not None and difference<1e-8
write_json(sys.argv[3],dict(passed=passed,raw_observations_reprocessed=True,fresh_fit_executed=True,maximum_feature_difference=largest,
 maximum_frozen_parameter_difference=difference,preprocessing=preprocessing,independent_validation=report.get('validation'),evaluation_access_forbidden=True))
raise SystemExit(0 if passed else 1)
'''
    with (root/(tag+'.log')).open('x') as stream:
        result=subprocess.run([sys.executable,'-c',code,str(root/'batch'),str(root/(tag+'_analysis')),str(root/(tag+'_refit.json'))],stdout=stream,stderr=subprocess.STDOUT,timeout=120)
    return dict(passed=result.returncode==0,device_writes=0,raw_reprocessing_and_fresh_fit_required=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--prior',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--render',action='store_true');a=p.parse_args()
    root=a.output.resolve();root.mkdir(parents=True,exist_ok=False)
    domain=220+int(uuid.uuid4().hex[:2],16)%10;os.environ.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1')
    mock=yaml.safe_load(Path('configs/sdk_mock.yaml').read_text())
    mock['plant'].update(stiction_enabled=True,sample_period_ns=1000000,integration_ns=100000,
        friction_viscous=[.08]+[.01]*13,friction_coulomb=[.03]+[.005]*13,friction_static=[.06]+[.01]*13,
        inertia_scales=[1.8]*14)
    mock_path=root/'mock.yaml';mock_path.write_text(yaml.safe_dump(mock))
    c=yaml.safe_load(Path('configs/hardware_inertia.yaml').read_text())
    c.update(joints=['left_shoulder_pitch_joint'],pose_offsets_rad=dict(train=[-.01],validation=[.01]),
        train_seeds=[401,402],validation_seeds=[501],period_s=8.,periods=3,discard_periods=1,
        amplitude_rad=[.08]*14,harmonics=12,settle_s=3.,transition_s=4.,inter_job_pause_s=.1)
    config=root/'h4.yaml';config.write_text(yaml.safe_dump(c))
    profile=yaml.safe_load(Path('configs/hardware_field.yaml').read_text())
    profile.update(collision_scene=str(Path('configs/collision_scene.yaml').resolve()),initial_hold_s=.15,final_hold_s=.15)
    base=root/'base.yaml';base.write_text(yaml.safe_dump(profile));checks={}
    with SDKMockSession(mock_path,root/'mock',domain,profile=base) as session:
        plan_inertia(config,session.client_profile,a.prior,root/'batch',domain_id=domain)
        first=batch.run_batch(root/'batch',max_jobs=1)
        checks['checkpoint_after_one_inertia_trial']=first['state']=='checkpoint'
        result=batch.run_batch(root/'batch',resume=True) if first['state']=='checkpoint' else first
        write_json(root/'batch_result.json',result);checks['complete_inertia_and_four_control_groups']=result['passed']
    checks['formal_sampling_not_fabricated']=not result['formal_hardware_calibration_accepted']
    if result['inertia']['available']:
        checks['frozen_prior_stages_rechecked']=result['inertia']['prior_stage_acceptance']['passed']
        checks['independent_inertia_prediction']=result['inertia']['checks']['independent_prediction']
    if result['passed']:
        checks['selection_before_independent_control']=not result['control']['selection']['validation_used_for_selection'] and len(result['control']['groups'])==4
        isolated=truth_denied(root);write_json(root/'truth_isolation.json',isolated);checks['fresh_process_truth_denied']=isolated['passed']
        original=json.loads((root/'batch/inertia_checkpoint/candidate.json').read_text())
        replay=json.loads((root/'truth_denied_analysis/inertia/candidate.json').read_text())
        checks['frozen_candidate_identical_without_truth']=original==replay
        from casbot_calib.dual_model import nominal_dual_arm
        nominal,_=nominal_dual_arm();model=nominal.model;joints=model.joints
        for j,joint in enumerate(joints):
            body=joint.body;body.inertia_com=body.inertia_com*mock['plant']['inertia_scales'][j];joint.body=body
        model.joints=joints;plant=type(nominal)(model);candidate=candidate_engine(root/'batch',original)
        q=np.array(c['home_rad']);observed=plant.mass_matrix(q)[0,0];estimated=candidate.mass_matrix(q)[0,0]
        evaluation=root/'evaluation';evaluation.mkdir()
        write_json(evaluation/'mass_matrix_comparison.json',dict(plant_joint_inertia=observed,candidate_joint_inertia=estimated,
            relative_error=abs(estimated-observed)/observed,truth_not_used_for_fitting=True,only_selected_axis_assessed=True))
        checks['independent_selected_axis_mass_matrix']=abs(estimated-observed)/observed<.10
        from verify_hardware_friction import verify_step_convergence
        convergence=verify_step_convergence(root);checks['recorded_command_step_halving']=convergence['passed']
        if a.render:
            from casbot_calib.hardware_batch_viewer import render_batch
            view=render_batch(root/'batch',root/'replay',fps=4,max_seconds_per_attempt=.5)
            checks['geometry_only_sdk_replay']=view['passed'] and view['device_writes']==0
    write_json(root/'verification.json',dict(passed=all(checks.values()),checks=checks,domain=domain,physical_device_tested=False,
        scope='one synthetic joint in full 14-axis model; three inertia, five tuning and four independent control records',
        full14_default_suite_executed=False,raw_sampling_qualified=False))
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
