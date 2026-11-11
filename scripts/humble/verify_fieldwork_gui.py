#!/usr/bin/env python3
"""Drive actual ImGui widgets against localhost SDK Mock, then freeze the frontend."""
import argparse
import json
import os
from pathlib import Path
import time
import uuid

import yaml


def verify_report_window(session, root):
    """Consume an existing session without starting a Mock or any ROS node."""
    from casbot_calib import fieldwork_gui
    from casbot_calib.fieldwork_analysis import write_json
    from imgui_bundle import imgui
    original = fieldwork_gui.Desk.gui
    args = argparse.Namespace(session=session, output=None, preview=False, profile=None,
        domain_id=None, scope=None, conditions=None, frames=300, screenshot=root/'window.png')
    result = dict(passed=False)
    def draw(desk):
        original(desk)
        if desk.frame == 5:
            imgui.get_io().add_mouse_pos_event(*desk.controls['report'])
            imgui.get_io().add_mouse_button_event(0, True)
        if desk.frame == 7:
            imgui.get_io().add_mouse_button_event(0, False)
        if desk.response and desk.response['action'] == 'report':
            result.update(passed=desk.response['completed'], response=desk.response)
            args.frames = min(args.frames, desk.frame+3)
    fieldwork_gui.Desk.gui = draw
    try: fieldwork_gui.run_gui(args)
    finally: fieldwork_gui.Desk.gui = original
    write_json(root/'verification.json', result)
    return 0 if result['passed'] else 1


def review_interruptions(folders, output):
    """Reuse recorded interruption trials; no new command publication."""
    import csv
    from casbot_calib.fieldwork_analysis import read_json, write_json
    from casbot_calib.hardware_field import check_seal
    cases = []
    for folder in folders:
        before = read_json(folder/'verification.json')
        with (folder/'mock/evaluation/server/server_commands.csv').open() as stream:
            received = len(list(csv.DictReader(stream)))
        captures = list((folder/'desk/attempts').glob('*/capture'))
        no_after = True
        for capture in captures:
            check_seal(capture)
            with (capture/'execution/events.csv').open() as stream: events = list(csv.DictReader(stream))
            with (capture/'execution/commands.csv').open() as stream: commands = list(csv.DictReader(stream))
            stop = [int(row['receive_ns']) for row in events if row['state'] == 'STOPPING']
            no_after &= bool(stop and all(int(row['publish_ns']) < stop[0] for row in commands))
        checks = {k: v for k, v in before['checks'].items() if k != 'actual_sdk_commands'}
        checks.update(actual_sdk_commands=received >= 30, no_commands_after_fault=no_after and bool(captures))
        cases.append(dict(source=str(folder), received_commands=received, checks=checks, passed=all(checks.values())))
    report = dict(passed=all(case['passed'] for case in cases), cases=cases,
        scope='offline review of existing private-localhost GUI Stop/Close trials',
        criterion='Interruption deliberately triggered after 30 commands, not the full workflow >100-command throughput check',
        hardware_tested=False)
    write_json(output/'verification.json', report)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenario', choices=('full', 'stop', 'close'), default='full')
    parser.add_argument('--report-session', type=Path, help='Only test report export on an existing session; no SDK/ROS started')
    parser.add_argument('--review-interruptions', type=Path, nargs='+', help='Offline review of previously recorded Stop/Close trials')
    args = parser.parse_args()
    root = args.output.resolve(); root.mkdir(parents=True, exist_ok=False)
    if args.review_interruptions:
        return review_interruptions(args.review_interruptions, root)
    if args.report_session:
        return verify_report_window(args.report_session, root)
    domain = 180 + int(uuid.uuid4().hex[:2], 16) % 30
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1')
    from casbot_calib.sdk_mock import SDKMockSession
    from casbot_calib.fieldwork_control import create_session, Session
    from casbot_calib.fieldwork_analysis import write_json, read_json
    from casbot_calib import fieldwork_gui
    from imgui_bundle import imgui
    with SDKMockSession('configs/sdk_mock.yaml', root/'mock', domain) as mock:
        profile = yaml.safe_load(mock.client_profile.read_text())
        profile.update(collision_scene=str(root/'mock/scene.yaml'), anchor_transition_s=.5,
                       initial_hold_s=.15, final_hold_s=.15)
        path = root/'profile.yaml'; path.write_text(yaml.safe_dump(profile))
        create_session(path, root/'desk', domain, 'localhost_contract_mock')
        ui_args = argparse.Namespace(session=root/'desk', output=None, preview=False, profile=path,
            domain_id=None, scope=None, conditions=None, frames=50000, screenshot=root/'window.png')
        trace = []
        original_gui = fieldwork_gui.Desk.gui
        state = dict(index=0, stage='select', scheduled=[], frame=0, started=time.monotonic(), frozen=False)
        def set_stage(value):
            state['stage'] = value
            state['stage_started'] = time.monotonic()
            trace.append(dict(stage=value, index=state['index'], time_s=time.monotonic()-state['started']))
            write_json(root/'progress.json', trace[-1])
        def click(desk, key, after, text=None):
            if key not in desk.controls:
                return
            x, y = desk.controls[key]
            io = imgui.get_io()
            low, high = desk.control_limits[key]
            if y < low or y > high:
                io.add_mouse_pos_event(x, (low+high)/2)
                io.add_mouse_wheel_event(0, 4 if y < low else -4)
                return
            io.add_mouse_pos_event(x, y)
            state['scheduled'] = [(1, 'down', None), (3, 'up', None)]
            if text is not None:
                state['scheduled'].append((5, 'text', text))
            state['scheduled'].append((7, 'stage', after))
            state['frame'] = desk.frame
        def driver(desk):
            if time.monotonic()-state['started'] > 600:
                raise TimeoutError('GUI drive timeout: '+state['stage'])
            if state['scheduled']:
                due = desk.frame-state['frame']
                remaining = []
                for frame, kind, value in state['scheduled']:
                    if frame > due:
                        remaining.append((frame, kind, value)); continue
                    if kind in ('down', 'up'):
                        imgui.get_io().add_mouse_button_event(0, kind == 'down')
                    elif kind == 'text':
                        imgui.get_io().add_input_characters_utf8(value)
                    else:
                        set_stage(value)
                state['scheduled'] = remaining
                return
            if desk.response and not desk.response['completed']:
                raise RuntimeError(desk.response['error'])
            if state['stage'] == 'wait_review' and not desk.busy and time.monotonic()-state.get('stage_started', 0) > 2:
                raise RuntimeError('Save review click was not consumed; inspect ui_state.json and window.png')
            stage = state['stage']
            if stage == 'select': click(desk, 'joint_'+str(state['index']), 'snapshot')
            elif stage == 'snapshot': click(desk, 'snapshot', 'wait_snapshot')
            elif stage == 'wait_snapshot' and not desk.busy and desk.status['snapshot']:
                set_stage('physical' if state['index'] < 14 else 'short_mode' if not state['frozen'] else 'plan_freeze')
            elif stage == 'physical': click(desk, 'physical', 'identify', 'Mock joint '+str(state['index']))
            elif stage == 'identify': click(desk, 'identify', 'plan')
            elif stage == 'plan': click(desk, 'plan_jog', 'wait_plan')
            elif stage == 'wait_plan' and not desk.busy and desk.plan: set_stage('confirm')
            elif stage == 'confirm': click(desk, 'execute_confirm', 'execute')
            elif stage == 'execute': click(desk, 'execute', 'wait_execute' if args.scenario == 'full' else 'interrupt_active')
            elif stage == 'interrupt_active' and desk.busy:
                attempt = desk.pending['values']['relative']
                commands = root/'desk'/attempt/'capture/execution/commands.csv'
                if commands.exists() and len(commands.read_text().splitlines()) > 30:
                    state['interrupt_attempt'] = attempt
                    if args.scenario == 'stop':
                        click(desk, 'stop', 'wait_interrupt')
                    else:
                        set_stage('done'); ui_args.frames = desk.frame+2
            elif stage == 'wait_interrupt' and not desk.busy:
                set_stage('done'); ui_args.frames = desk.frame+5
            elif stage == 'wait_execute' and not desk.busy:
                if desk.executed:
                    set_stage('same_axis')
                elif desk.status['blocked_reason']:
                    raise RuntimeError('SDK execution failed: '+str(desk.timeline))
            elif stage == 'same_axis': click(desk, 'same_axis', 'same_direction')
            elif stage == 'same_direction': click(desk, 'same_direction', 'observer' if not desk.observer else 'review')
            elif stage == 'observer': click(desk, 'observer', 'note', 'Mock UI observer')
            elif stage == 'note': click(desk, 'note', 'review', 'Automated software-only widget test; not physical observation')
            elif stage == 'review': click(desk, 'review', 'wait_review')
            elif stage == 'wait_review' and not desk.busy and desk.names[state['index']] in desk.status['accepted_joints']:
                state['index'] += 1
                set_stage('select' if state['index'] < 14 else 'snapshot')
            elif stage == 'short_mode': click(desk, 'short_mode_2', 'plan_short')
            elif stage == 'plan_short': click(desk, 'plan_short', 'wait_short_plan')
            elif stage == 'wait_short_plan' and not desk.busy and desk.plan: set_stage('short_confirm')
            elif stage == 'short_confirm': click(desk, 'execute_confirm', 'short_execute')
            elif stage == 'short_execute': click(desk, 'execute', 'wait_short_execute')
            elif stage == 'wait_short_execute' and not desk.busy:
                if not desk.executed: raise RuntimeError('Short execution failed: '+str(desk.timeline))
                write_json(root/'short_result.json', desk.response)
                state['frozen'] = True
                set_stage('snapshot')
            elif stage == 'plan_freeze': click(desk, 'plan_jog', 'wait_freeze_plan')
            elif stage == 'wait_freeze_plan' and not desk.busy and desk.plan: set_stage('freeze_confirm')
            elif stage == 'freeze_confirm': click(desk, 'execute_confirm', 'freeze_execute')
            elif stage == 'freeze_execute': click(desk, 'execute', 'freeze')
            elif stage == 'freeze' and desk.busy:
                attempt = desk.pending['values']['relative']
                commands = root/'desk'/attempt/'capture/execution/commands.csv'
                if commands.exists() and len(commands.read_text().splitlines()) > 30:
                    time.sleep(3.)  # no GUI frames/heartbeat; backend must stop
                    state['freeze_attempt'] = attempt
                    set_stage('wait_freeze')
            elif stage == 'wait_freeze' and not desk.busy:
                if not desk.status['blocked_reason']: raise RuntimeError('Heartbeat loss did not terminate motion session')
                set_stage('done'); ui_args.frames = desk.frame+5
        def instrumented(desk):
            original_gui(desk)
            write_json(root/'ui_state.json', dict(stage=state['stage'], busy=desk.busy, observer=desk.observer,
                physical_label=desk.physical_label, same_axis=desk.same_axis, same_direction=desk.same_direction,
                executed=desk.executed, plan_kind=desk.plan['kind'] if desk.plan else None, controls=desk.controls))
            try:
                driver(desk)
            except Exception as exc:
                write_json(root/'failure.json', dict(error=str(exc), stage=state['stage'], timeline=list(desk.timeline)))
                desk.close()
                ui_args.frames = desk.frame+2
                state['stage'] = 'failed'
        fieldwork_gui.Desk.gui = instrumented
        try:
            fieldwork_gui.run_gui(ui_args)
        finally:
            fieldwork_gui.Desk.gui = original_gui
            write_json(root/'trace.json', trace)
        status = Session(root/'desk').status()
        checks = dict(completed=state['stage'] == 'done')
        if args.scenario == 'full':
            checks['all14_gui_reviews'] = len(status['accepted_joints']) == 14
        if state.get('freeze_attempt'):
            capture = root/'desk'/state['freeze_attempt']/'capture'
            result = read_json(capture/'result.json')
            checks['heartbeat_terminal'] = result['supervisor_interruption'] == 'frontend_heartbeat_timeout' and bool(status['blocked_reason'])
            checks['physical_stop_not_claimed'] = not result['physical_stop_verified']
        short = read_json(root/'short_result.json')
        if args.scenario == 'full':
            checks['dual_arm_short_passed'] = bool(short and short['result']['passed'])
        if args.scenario != 'full' and state.get('interrupt_attempt'):
            attempt = root/'desk'/state['interrupt_attempt']
            result = read_json(attempt/'capture/result.json')
            checks['controlled_interrupt'] = result['supervisor_interruption'] == ('operator_cancel' if args.scenario == 'stop' else 'frontend_closed')
            checks['not_sdk_fault_resume'] = not status['blocked_reason'] and status['snapshot'] is None
            checks['no_physical_stop_claim'] = not result['physical_stop_verified']
        import csv
        with (root/'mock/evaluation/server/server_commands.csv').open() as stream:
            received = list(csv.DictReader(stream))
        # Interrupt scenarios intentionally stop soon after the first flushed
        # command block; they test interruption, not long-run throughput.
        checks['actual_sdk_commands'] = len(received) > 100 if args.scenario == 'full' else len(received) >= 30
        result = dict(passed=all(checks.values()), checks=checks,
            scope='actual Dear ImGui widgets + unified SDK mechanical Mock on private localhost DDS',
            hardware_tested=False, physical_direction_verified=False)
        write_json(root/'verification.json', result)
        print(json.dumps(result, indent=2))
        return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
