#!/usr/bin/env python3
"""DVT2 paired SDK server/client, H1 frequency tiers and H2 read-only replay."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml
from casbot_calib.sdk_mock import SDKMockSession
from casbot_calib import hardware_field as field
from casbot_calib.model_context import load_context
from casbot_calib.model_runtime import model_for
from casbot_calib.data import write_json


def verify(output,domain):
    root=Path(output);root.mkdir(parents=True,exist_ok=False)
    context=load_context()
    base=yaml.safe_load(Path('configs/hardware_field.yaml').read_text())
    base['collision_scene']=str(Path('configs/collision_scene.yaml').resolve())
    base.update(initial_hold_s=.2,final_hold_s=.2,anchor_transition_s=.3)
    profile=root/'field.yaml';profile.write_text(yaml.safe_dump(base))
    results=[]
    with SDKMockSession('configs/sdk_mock.yaml',root/'mock',domain,profile=profile) as session:
        config=field.load_profile(session.client_profile)
        initial=field.run_executor(config,root/'initial',domain,snapshot=True)
        if not initial['passed']:raise ValueError('Read-only SDK snapshot failed')
        anchor=field._read_snapshot(root/'initial')
        write_json(root/'initial_binding.json',dict(model_context=context.identity,observed_q=anchor))
        for joint in ('left_shoulder_pitch_joint','right_shoulder_pitch_joint','left_wrist_roll_joint','right_wrist_roll_joint'):
            for hz in ((100,250,500) if 'shoulder' in joint else (500,)):
                name=joint+'_'+str(hz);target=root/name;target.mkdir()
                c=yaml.safe_load(session.client_profile.read_text());c['command_hz']=hz
                c['collision_scene']=str(root.resolve()/'mock/scene.yaml')
                path=target/'profile.yaml';path.write_text(yaml.safe_dump(c))
                loaded=field.load_profile(path)
                snapshot=field.run_executor(loaded,target/'snapshot',domain,snapshot=True)
                if not snapshot['passed']:raise ValueError('Snapshot failed: '+name)
                planned=field.prepare_jog(path,target/'snapshot',target/'plan',joint,.3,anchor_rad=anchor)
                capture=field.execute_field(target/'plan',target/'capture',domain)
                if not capture['passed']:raise ValueError('Execution failed: '+name)
                analysis=field.analyse_field(target/'plan',target/'capture',target/'analysis')
                if not analysis['passed']:raise ValueError('Observed motion missing: '+name)
                results.append(dict(joint=joint,hz=hz,source_model_sha256=planned['source_model_sha256'],
                    passed=analysis['passed'],command_publish_frequency=analysis['command_publish_frequency'],
                    feedback_receive_frequency=analysis['feedback_receive_frequency'],
                    aggregate_response_lag_ms=analysis['aggregate_response_lag_ms'],
                    comparison_reference_sha256=analysis['comparison_reference_sha256'],
                    actual_sample_rate_known=analysis['actual_sample_rate_known']))
                print(name+': PASS',flush=True)
    behavior=json.loads((root/'mock/protocol/behavior.json').read_text())
    engine,source=model_for(base)
    checks=dict(same_model=behavior['model_context']==context.identity and behavior['server_params']['model_id']==engine.model.model_id,
        paired_profile=config['model_context']==context.identity,
        all_source_bindings=all(r['source_model_sha256']==source['sha256'] for r in results),
        same_reference_across_rates=all(len({r['comparison_reference_sha256'] for r in results if r['joint']==joint})==1 for joint in context.joint_names if any(r['joint']==joint for r in results)),
        both_arms_and_light_wrists=len(results)==8 and all(r['passed'] for r in results),
        sample_time_remains_unknown=all(not r['actual_sample_rate_known'] for r in results),
        shutdown=json.loads((root/'mock/shutdown.json').read_text())['returncodes']==[0])
    report=dict(passed=all(checks.values()),checks=checks,model_context=context.identity,results=results,
        scope='localhost SDK messages; default model/configs, shoulders at 100/250/500 Hz and both wrist-roll axes at500 Hz; not full H3/H4 dynamics acceptance or physical network',
        physical_device_tested=False,hardware_executable=False)
    write_json(root/'report.json',report)
    if not report['passed']:raise ValueError('DVT2 SDK probe rejected')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True);parser.add_argument('--domain-id',type=int,default=227)
    args=parser.parse_args();result=verify(args.output,args.domain_id);print(json.dumps(result['checks']))
