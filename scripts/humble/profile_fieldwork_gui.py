#!/usr/bin/env python3
"""Headless, private Mock GUI CPU benchmark; no screenshots or physical endpoints."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import time


def processes():
    rows={}
    for path in Path('/proc').glob('[0-9]*'):
        try:
            stat=(path/'stat').read_text(); fields=stat[stat.rfind(')')+2:].split()
            rows[int(path.name)]=dict(parent=int(fields[1]),ticks=int(fields[11])+int(fields[12]),
                threads=int(fields[17]),name=stat[stat.index('(')+1:stat.rfind(')')],
                command=(path/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace'))
        except (OSError,ValueError,IndexError):pass
    owned={os.getpid()}
    while True:
        children={pid for pid,row in rows.items() if row['parent'] in owned}
        if children<=owned:break
        owned|=children
    return {pid:rows[pid] for pid in owned if pid in rows}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--domain-id',type=int,required=True)
    parser.add_argument('--seconds',type=float,default=10.)
    parser.add_argument('--short-trajectory',action='store_true',help='exercise GUI-generated 2s small trajectory instead of 5Hz edits')
    parser.add_argument('--exercise-input',action='store_true',help='inject ImGui mouse events without moving the system pointer')
    parser.add_argument('--stall-seconds',type=float,default=0.,help='freeze UI after sending phase, testing heartbeat timeout')
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    from casbot_calib import fieldwork_gui
    ui=argparse.Namespace(mock=True,session=None,recover=None,preview=False,output=None,
        profile='configs/hardware_field.yaml',domain_id=args.domain_id,scope=None,
        cache_limit_mib=512,frames=None,screenshot=None)
    os.environ['CASBOT_FIELDWORK_CACHE']=str(root/'cache')
    original=fieldwork_gui.Desk.gui;start=time.monotonic();state={'phase':'warmup','draw_s':0.};reports=[];graphics={}
    def begin(desk,phase):
        state.update(phase=phase,at=time.monotonic(),before=processes(),frame=desk.frame,
            rendered=desk.rendered_frames,edits=0,draw_start=state['draw_s'])
    def finish(desk):
        elapsed=time.monotonic()-state['at'];after=processes();rows=[]
        for pid,row in after.items():
            old=state['before'].get(pid)
            if old:rows.append(dict(pid=pid,name=row['name'],command=row['command'],threads=row['threads'],cpu_percent=
                (row['ticks']-old['ticks'])/os.sysconf('SC_CLK_TCK')/elapsed*100))
        reports.append(dict(phase=state['phase'],seconds=elapsed,gui_fps=(desk.frame-state['frame'])/elapsed,
            view_fps=(desk.rendered_frames-state['rendered'])/elapsed,processes=rows,
            draw_ms=1000*(state['draw_s']-state['draw_start'])/(desk.frame-state['frame']),
            total_cpu_percent=sum(row['cpu_percent'] for row in rows)))
    def draw(desk):
        if args.exercise_input and state['phase']=='sending':
            from imgui_bundle import imgui
            imgui.get_io().add_mouse_pos_event(1100+desk.frame%80,12) # Empty title margin, not a control.
        before=time.monotonic();original(desk);now=time.monotonic();phase=state['phase']
        state['draw_s']+=now-before
        if desk.short is not None:state.setdefault('short_frames',[]).append(now)
        if not graphics:
            from OpenGL import GL
            graphics.update(renderer=GL.glGetString(GL.GL_RENDERER).decode(),samples=int(GL.glGetIntegerv(GL.GL_SAMPLES)))
        if now-start>150:raise RuntimeError('benchmark timed out: '+phase)
        if phase=='warmup' and now-start>8 and desk.rendered_frames>0:begin(desk,'idle')
        elif phase=='idle' and now-state['at']>=args.seconds:
            finish(desk);desk.connect();state['phase']='connecting'
        elif phase=='connecting' and desk.connected and desk.feedback['kp'] is not None:
            begin(desk,'connected')
        elif phase=='connected' and now-state['at']>=args.seconds:
            finish(desk);desk.begin_sending();state['phase']='starting'
        elif phase=='starting' and desk.sending:
            state['anchor']=float(desk.commands['position'][0]);begin(desk,'sending')
            if args.short_trajectory:
                desk.short=dict(start=time.monotonic(),duration=2.,amplitude=.005,anchor=state['anchor'],joint=0)
        elif phase=='sending':
            if not desk.sending:raise RuntimeError('unexpected control stop: '+str(desk.operator.displayed))
            if now-state['at']>=args.seconds:
                finish(desk)
                if args.stall_seconds:
                    state['before_stall']=desk.receive_ns;time.sleep(args.stall_seconds)
                else:desk.stop()
                state['phase']='stopping'
            else:
                edit=int((now-state['at'])*5)
                if not args.short_trajectory and edit>state['edits']:
                    desk.change('position',0,state['anchor']+.005*math.sin(2*math.pi*(now-state['at'])/4))
                    state['edits']=edit
        elif phase=='stopping' and not desk.busy:
            state['control_response']=desk.response;state['workspace']=str(desk.root)
            state['observer_survived']=desk.observing and (not args.stall_seconds or desk.receive_ns>state['before_stall'])
            state['needs_reinitialize']=desk.needs_reinitialize
            state['visible_code']=(desk.operator.displayed or {}).get('code')
            ui.frames=desk.frame+1;state['phase']='done'
    fieldwork_gui.Desk.gui=draw
    try:fieldwork_gui.run_gui(ui)
    finally:fieldwork_gui.Desk.gui=original
    workspace=Path(state['workspace']);run=next(workspace.glob('controls/*/runs/*'))
    with (run/'capture/execution/commands.csv').open() as stream:commands=list(csv.DictReader(stream))
    with (run/'capture/execution/events.csv').open() as stream:events=list(csv.DictReader(stream))
    stops=[int(row['receive_ns']) for row in events if row['state']=='STOPPING']
    checks=dict(finished=state['phase']=='done',observer_survived=state['observer_survived'],
        explicit_restart_required=state['needs_reinitialize'],published_commands=len(commands)>100,
        no_commands_after_stop=bool(stops and all(int(row['publish_ns'])<stops[0] for row in commands)),
        raw_cdr_preserved=(run/'capture/raw_state/raw/cdr.bin').stat().st_size>0,
        no_gravity_feedforward=all(float(row['effort'+str(j)])==0 for row in commands for j in range(14)))
    if args.stall_seconds:
        checks['heartbeat_cause_visible']=state['visible_code']=='frontend_heartbeat_timeout'
        checks['heartbeat_terminated_control']=not state['control_response']['result']['controlled_stop']
    else:checks['normal_stop']=state['control_response']['result']['controlled_stop']
    short_frames=state.get('short_frames',[])
    short_fps=(len(short_frames)-1)/(short_frames[-1]-short_frames[0]) if len(short_frames)>1 else None
    if args.short_trajectory:
        values=[float(row['q0']) for row in commands]
        checks['short_moved']=max(values)-min(values)>.0075
        checks['short_returned_to_target']=abs(values[-1]-state['anchor'])<1e-7
        checks['short_active_frame_budget']=short_fps is not None and 15<short_fps<=31
    result=dict(short_trajectory_fps=short_fps,phases=reports,finished=state['phase']=='done',checks=checks,passed=all(checks.values()),
        input_exercised=args.exercise_input,stall_seconds=args.stall_seconds,
        control_response=state.get('control_response'),
        graphics=graphics,scope='Xvfb software-rendered GUI + owned private localhost Mock; no screenshot/visual inspection',
        cpu_percent_semantics='100% = one CPU core; owned process tree only',physical_robot_tested=False)
    (root/'profile.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(finished=result['finished'],passed=result['passed'],checks=checks,phases=[{k:v for k,v in row.items() if k!='processes'} for row in reports]),indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
