#!/usr/bin/env python3
"""Verify saved next-start settings reach only the next owned private Mock."""
import argparse
import json
import os
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--domain-id',type=int,required=True)
    args=p.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    os.environ['CASBOT_FIELDWORK_CACHE']=str(root/'cache')
    from casbot_calib.fieldwork_launch import prepare_gui
    from casbot_calib.fieldwork_control import Workspace
    from casbot_calib.fieldwork_settings import defaults,save_mock_next
    ui=argparse.Namespace(mock=True,mock_diagnostics=False,session=None,recover=None,preview=False,output=None,
        profile=Path('configs/hardware_field.yaml'),domain_id=args.domain_id,scope=None,cache_limit_mib=512)
    checks={}
    with prepare_gui(ui) as (workspace,monitor,mock):
        session=Workspace(workspace);before=session.config['mock_limits'];settings=defaults(session.config,session.names,session.metadata['model_context'])
        settings['mock'].update(effort_limit_nm=150.,kp_limit=2000.,kd_limit=200.,torque_limit_nm=7.,watchdog_s=.2)
        save_mock_next(settings)
        checks['current_instance_unchanged']=session.config['mock_limits']==before and before['torque_limit_nm']==100.
    with prepare_gui(ui) as (workspace,monitor,mock):
        session=Workspace(workspace);actual=session.config['mock_limits']
        behavior=json.loads((workspace.parent/'mock/protocol/behavior.json').read_text())
        params=behavior['server_params']
        checks.update(next_profile_matches=actual==settings['mock'],server_total_request=params['torque_limit_nm']==7.,
            server_effort_acceptance=params['effort_limit_nm']==150.,server_gains=params['kp_limit']==2000. and params['kd_limit']==200.,
            server_watchdog=params['watchdog_ns']==200000000,gravity_still_disabled=not params['internal_gravity'] and not params['protection_gravity'])
    result=dict(passed=all(checks.values()),checks=checks,physical_robot_tested=False)
    (root/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
