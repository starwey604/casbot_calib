#!/usr/bin/env python3
"""Nonvisual private-Mock limits acceptance, including real ImGui widgets and native ACK."""
import argparse
import copy
import csv
import json
import os
from pathlib import Path
import time


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--domain-id',type=int,required=True)
    args=p.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    os.environ['CASBOT_FIELDWORK_CACHE']=str(root/'cache')
    from casbot_calib import fieldwork_gui as gui,fieldwork_settings as policy
    ui=argparse.Namespace(mock=True,mock_diagnostics=False,session=None,recover=None,preview=False,output=None,
        profile='configs/hardware_field.yaml',domain_id=args.domain_id,scope=None,cache_limit_mib=512,frames=None,screenshot=None)
    original=gui.Desk.gui;original_poll=gui.Desk.poll
    checks={};state={'phase':'ui','tab':0,'click':None};started=time.monotonic();runs=[]
    from imgui_bundle import imgui
    original_tab=imgui.begin_tab_item
    def tab(label,*a,**kw):
        labels=['反馈观察','目标输入','等待与通信','Mock']
        if state['phase']=='ui' and label.startswith(labels[state['tab']]):kw['flags']=imgui.TabItemFlags_.set_selected
        return original_tab(label,*a,**kw)
    imgui.begin_tab_item=tab
    def draw(desk):
        from imgui_bundle import imgui
        desk.limits.open=True
        original(desk)
        now=time.monotonic();phase=state['phase']
        if now-started>100:raise RuntimeError('limits verification timeout: '+phase)
        if phase=='ui':
            tabs=[('observed','limit_observed_velocity'),('input','limit_input_velocity'),('general','limit_start_velocity'),('mock','limit_mock_effort_limit_nm')]
            target,key=tabs[state['tab']]
            if key in desk.controls:
                checks['panel_'+target]=True;state['tab']+=1
                if state['tab']==len(tabs):
                    desk.limits.open=False;desk.connect();state['phase']='connecting';return
                return
            else:return
        elif phase=='connecting' and desk.connected and desk.feedback['kp'] is not None:
            desk.limits.draft['overrides'][desk.names[0]]=dict(observed_upper=-.1)
            desk.limits.draft['general']['start_stable_s']=.2
            desk.limits.apply();state['initial_revision']=desk.limits.settings['revision']
            desk.begin_sending();runs.append(desk.control_root/desk.active_run);state['phase']='warning'
        elif phase=='warning' and desk.operator.current and desk.operator.current['code']=='observed_warning':
            checks['warn_continues']=desk.sending and desk.observing
            checks['initial_native_ack']=desk.limits.ack==state['initial_revision']
            target=desk.commands['position'].copy()
            desk.limits.draft['defaults']['kp_max']=2500.
            desk.limits.draft['overrides'][desk.names[0]]['observed_upper']=1.6
            desk.limits.apply();state['recover_revision']=desk.limits.settings['revision'];state['target']=target
            state['phase']='recover'
        elif phase=='recover' and desk.limits.ack==state['recover_revision'] and desk.operator.current['code']=='observed_recovered':
            checks['live_recovery']=desk.sending
            checks['no_target_clamp']=bool((desk.commands['position']==state['target']).all())
            wire=desk.control_root/desk.active_run/'limits.csv'
            row=list(csv.reader(wire.open()))[0]
            checks['input_pinned']=float(row[1+4*14])==1000.
            bad=copy.deepcopy(desk.limits.settings);bad['revision']+=100
            policy.write_wire(wire,bad,desk.model_lower,desk.model_upper) # illicit live input change must be rejected
            state['phase']='rejected';state['at']=now
        elif phase=='rejected' and '拒绝' in desk.limits.notice:
            checks['native_rejects_unpinned']=desk.sending and desk.limits.ack==state['recover_revision']
            desk.limits.draft['overrides'][desk.names[0]].update(observed_velocity=1e-5,observed_action=1)
            desk.limits.apply();state['end_revision']=desk.limits.settings['revision'];state['phase']='ending'
            desk.change('position',0,float(desk.commands['position'][0])+.02)
        elif phase=='ending' and not desk.busy:
            checks['end_action']=not desk.sending and desk.operator.first_fault and desk.operator.first_fault['code']=='observed_limit'
            checks['observer_after_end']=desk.observing
            checks['ended_session_blocked']=gui.Session(desk.control_root).status()['blocked_reason'] is not None
            desk.refresh_initial();desk.limits.draft['overrides'][desk.names[0]]=dict(observed_upper=1.6)
            desk.limits.draft['general']['maximum_duration_s']=.5;desk.limits.apply()
            desk.begin_sending();runs.append(desk.control_root/desk.active_run);state['phase']='duration'
        elif phase=='duration' and not desk.busy:
            checks['duration_normal_end']=bool(desk.response and desk.response['completed'] and desk.response['result']['controlled_stop'])
            checks['duration_no_fault']=desk.operator.first_fault is None
            checks['duration_session_not_blocked']=gui.Session(desk.control_root).status()['blocked_reason'] is None
            desk.refresh_initial();desk.limits.draft['general'].update(maximum_duration_s=0.,heartbeat_timeout_s=.3,stop_timeout_s=.3)
            desk.limits.apply();desk.begin_sending();runs.append(desk.control_root/desk.active_run);state['phase']='heartbeat_start'
        elif phase=='heartbeat_start' and desk.sending:
            time.sleep(.8);state['phase']='heartbeat_end'
        elif phase=='heartbeat_end' and not desk.busy:
            checks['configured_heartbeat_terminates']=desk.operator.first_fault and desk.operator.first_fault['code']=='frontend_heartbeat_timeout'
            checks['heartbeat_observer_survives']=desk.observing
            desk.save_path=str(root/'saved');desk.begin_save();state['phase']='saving'
        elif phase=='saving' and desk.save_job is None:
            checks['snapshot_saved']=(root/'saved/saved_session.json').exists()
            ui.frames=desk.frame+1;state['phase']='done'
    gui.Desk.gui=draw
    try:gui.run_gui(ui)
    finally:gui.Desk.gui=original;gui.Desk.poll=original_poll;imgui.begin_tab_item=original_tab
    for index,run in enumerate(runs):
        commands=list(csv.DictReader((run/'capture/execution/commands.csv').open()))
        events=list(csv.DictReader((run/'capture/execution/events.csv').open()))
        stop=next(int(row['receive_ns']) for row in events if row['state']=='STOPPING')
        checks[f'run{index}_no_commands_after_stop']=all(int(row['publish_ns'])<stop for row in commands)
        from casbot_calib.raw_archive import capture_raw,iter_records
        raw,start,end=capture_raw(run/'capture')
        checks[f'run{index}_shared_raw_cdr']=any(cdr for _,cdr in iter_records(raw,start_ns=start,end_ns=end))
        settings=list(csv.DictReader((run/'capture/execution/settings_applied.csv').open()))
        checks[f'run{index}_accepted_policy_recorded']=bool(settings)
        if index==0:
            checks['live_full_values_recorded']=len(settings)==3 and float(settings[-1]['observed_action0'])==1 and float(settings[-1]['kp_max0'])==1000.
        elif index==1:checks['next_control_new_input']=float(settings[0]['kp_max0'])==2500.
    result=dict(passed=all(checks.values()),checks=checks,runs=[str(r) for r in runs],visual_inspection=False,physical_robot_tested=False)
    (root/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
