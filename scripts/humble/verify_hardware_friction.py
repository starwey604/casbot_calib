#!/usr/bin/env python3
"""H3 through unified SDK Mock; internal plant data is for independent audit only."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid
import numpy as np
import yaml
from casbot_calib.sdk_mock import SDKMockSession
from casbot_calib import hardware_batch as batch
from casbot_calib.hardware_friction import plan_friction
from casbot_calib.data import write_json


def truth_denied(root):
    code='''import os,sys
from pathlib import Path
def guard(event,args):
 if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)) and 'evaluation' in Path(os.fsdecode(args[0])).parts:raise RuntimeError('evaluation forbidden')
sys.addaudithook(guard)
try:open('/__probe__/evaluation/truth','rb')
except RuntimeError:pass
else:raise RuntimeError('guard inactive')
from casbot_calib.cli import main
raise SystemExit(main(['hardware-analyse-batch',sys.argv[1],'--output',sys.argv[2]]))
'''
    with (root/'truth_denied.log').open('x') as stream:
        run=subprocess.run([sys.executable,'-c',code,str(root/'batch'),str(root/'truth_denied_analysis')],stdout=stream,stderr=subprocess.STDOUT,timeout=60)
    if run.returncode:return dict(passed=False)
    a=batch.read(root/'batch/latest.json');original=batch.read(root/'batch'/a['report']/'moving_friction.json')
    new=batch.read(root/'truth_denied_analysis/moving_friction.json')
    return dict(passed=original==new,parameter_report_identical=original==new,device_writes=0)



def verify_step_convergence(root):
    from casbot_calib import _core
    from casbot_calib.dual_model import nominal_dual_arm
    from casbot_calib.hardware_friction import read
    from time import perf_counter
    root=Path(root).resolve();folder=root/'step_convergence';folder.mkdir(exist_ok=False)
    _,protocol=batch.load_batch(root/'batch');history,_=batch.outcomes(root/'batch',protocol)
    job=next(j for j in protocol['jobs'] if j['kind']=='moving_friction');source=batch.accepted(history[job['id']])
    trial=root/'batch'/source['attempt'];metadata=read(trial/'plan/plan.json')
    import csv
    with (trial/'capture/execution/commands.csv').open() as stream:rows=list(csv.DictReader(stream))
    origin=int(rows[0]['publish_ns']);times=np.array([int(r['publish_ns'])-origin for r in rows],dtype=np.int64)
    commands=np.empty((len(rows),14,5))
    for k,prefix in enumerate(('q','v','effort','kp','kd')):
        commands[:,:,k]=np.array([[float(r[prefix+str(j)]) for j in range(14)] for r in rows])
    mock=yaml.safe_load((root/'mock.yaml').read_text());settings=mock['plant']
    nominal,_=nominal_dual_arm();model=nominal.model;joints=model.joints
    for j,joint in enumerate(joints):
        f=joint.friction;f.viscous=settings['friction_viscous'][j];f.coulomb=settings['friction_coulomb'][j]
        f.static_friction=settings['friction_static'][j];f.stribeck_velocity=.005;joint.friction=f
    model.joints=joints;plant=_core.ArmDynamics(model);outputs=[];cost=[]
    for step in (settings['integration_ns'],settings['integration_ns']//2):
        config=_core.ArmSimulationConfig();config.gain=np.full(14,2.);config.stiction_enabled=True
        config.integration_ns=step;config.sample_period_ns=1000000;config.command_delay_ns=settings['drive_delay_ns']
        config.feedback_delay_ns=settings['feedback_delay_ns'];config.current_tau=settings['current_tau']
        config.position_noise=0.;config.current_noise=0.
        begin=perf_counter();obs,_=_core.ArmSimulationRunner(plant,config).run(times,commands,int(times[-1])+1000000,np.array(metadata['observed_start_rad']),np.zeros(14))
        outputs.append(obs);cost.append(perf_counter()-begin)
    coarse,fine=outputs
    errors={key:float(np.max(abs(coarse[key]-fine[key]))) for key in ('q','dq','current')}
    scales=dict(q=max(.01,float(np.max(np.ptp(fine['q'],axis=0)))),dq=max(.01,float(np.max(abs(fine['dq'])))),
                current=max(.05,float(np.max(abs(fine['current'])))))
    relative={key:errors[key]/scales[key] for key in errors}
    peaks={key:float(abs(np.max(abs(coarse[key]))-np.max(abs(fine[key])))/scales[key]) for key in errors}
    report=dict(passed=relative['q']<.01 and all(value<.01 for value in peaks.values()),peak_relative_changes=peaks,pointwise_velocity_change_diagnostic=relative['dq'],integration_steps_ns=[settings['integration_ns'],settings['integration_ns']//2],
        maximum_absolute_errors=errors,normalized_errors=relative,normalization=scales,solver_wall_s=cost,
        scope='Independent fixed recorded-command numerical replay; no new SDK execution and no fitting',
        source_attempt_manifest_sha256=source['manifest_sha256'],sample_period_ns=1000000,physical_device_tested=False)
    write_json(folder/'report.json',report);return report


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--gravity',type=Path,required=True);parser.add_argument('--render',action='store_true')
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    domain=220+int(uuid.uuid4().hex[:2],16)%10;os.environ.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1')
    mock=yaml.safe_load(Path('configs/sdk_mock.yaml').read_text())
    mock['plant'].update(stiction_enabled=True,sample_period_ns=1000000,integration_ns=100000,
        friction_viscous=[.08]+[.01]*13,friction_coulomb=[.03]+[.005]*13,friction_static=[.06]+[.01]*13)
    mock_path=root/'mock.yaml';mock_path.write_text(yaml.safe_dump(mock))
    c=yaml.safe_load(Path('configs/hardware_friction.yaml').read_text())
    c.update(joints=['left_shoulder_pitch_joint'],pose_offsets_rad=dict(train=[-.01],validation=[.01]),
        speeds_rad_s=dict(train=[.03,.06],validation=[.045]),transition_s=4.,settle_s=3.,speed_ramp_s=.75,
        plateau_trim_s=.5,inter_job_pause_s=.1)
    c['breakaway'].update(dwell_s=[.4],slopes_nm_s=[.12],ramp_s=1.3,recovery_hold_s=.8)
    config_path=root/'h3.yaml';config_path.write_text(yaml.safe_dump(c))
    profile=yaml.safe_load(Path('configs/hardware_field.yaml').read_text())
    profile.update(collision_scene=str(Path('configs/collision_scene.yaml').resolve()),initial_hold_s=.15,final_hold_s=.15)
    profile_path=root/'base.yaml';profile_path.write_text(yaml.safe_dump(profile))
    checks={}
    with SDKMockSession(mock_path,root/'mock',domain,profile=profile_path) as session:
        plan_friction(config_path,session.client_profile,args.gravity,root/'batch',domain_id=domain)
        first=batch.run_batch(root/'batch',max_jobs=1)
        checks['checkpoint_after_one_trial']=first['state']=='checkpoint'
        result=batch.run_batch(root/'batch',resume=True) if first['state']=='checkpoint' else first
        write_json(root/'batch_result.json',result)
        checks['complete_sdk_friction_and_onset']=result['passed']
        checks['formal_sampling_not_fabricated']=not result['formal_hardware_calibration_accepted'] and not result['moving_friction'].get('sampling_qualification_passed',False)
    if result['moving_friction']['available']:
        axis=result['moving_friction']['axes'][0]
        checks['independent_synthetic_viscous_recovery']=axis['viscous_nm_s_rad'] is not None and abs(axis['viscous_nm_s_rad']-.08)<.025
        checks['independent_synthetic_coulomb_recovery']=axis['coulomb_nm'] is not None and abs(axis['coulomb_nm']-.03)<.005
    if result['breakaway']['available']:
        directions=result['breakaway']['axes'][0]['directions']
        checks['independent_synthetic_static_recovery']=all(abs(d['estimate_nm']-.06)<.012 for d in directions.values())
        checks['callback_intervals_not_physical_intervals']=all(not r['physical_onset_interval_known'] for r in result['breakaway']['ramps'])
    if result['passed']:
        isolated=truth_denied(root);write_json(root/'truth_isolation.json',isolated)
        checks['fresh_process_no_internal_truth']=isolated['passed']
        numerical=verify_step_convergence(root)
        checks['recorded_command_step_halving']=numerical['passed']
        if args.render:
            from casbot_calib.hardware_batch_viewer import render_batch
            replay=render_batch(root/'batch',root/'replay',fps=4,max_seconds_per_attempt=.5)
            checks['geometry_only_sdk_replay']=replay['passed'] and replay['device_writes']==0
    write_json(root/'verification.json',dict(passed=all(checks.values()),checks=checks,domain=domain,
        physical_device_tested=False,local_only=True,plant_truth_used_by_identification=False,
        scope='one synthetic joint, multiple speeds, disjoint posture validation, both ramp directions and independent repeats',
        defaults_14axis_full_field_suite_executed=False))
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
