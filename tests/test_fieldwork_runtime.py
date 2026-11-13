"""Native event consumption, control/observation isolation and pipe lifetime contracts."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest
import yaml

from casbot_calib.fieldwork import clock_id
from casbot_calib.fieldwork_control import Session,Workspace,create_session
from casbot_calib.fieldwork_ipc import PipeHeartbeat,Child
from casbot_calib.fieldwork_status import describe,OperatorStatus
from casbot_calib.hardware_batch import event
from casbot_calib.sdk_process import run_process,recording_process,OutputPump


def pulse(fd,owner='one',**changes):
    row=dict(clock_id=clock_id(),owner_id=owner,monotonic_ns=time.monotonic_ns(),cancel=None)
    row.update(changes);os.write(fd,(json.dumps(row)+'\n').encode())


def test_pipe_heartbeat_cancel_without_recording_files():
    r,w=os.pipe()
    try:
        check=PipeHeartbeat(r,'one');pulse(w);assert check() is None
        pulse(w,cancel='operator_cancel');assert check()=='operator_cancel'
    finally:os.close(r);os.close(w)


def test_readonly_observer_survives_ui_stall_but_not_owner_exit():
    r,w=os.pipe()
    try:
        check=PipeHeartbeat(r,'one',timeout_s=None)
        pulse(w,monotonic_ns=time.monotonic_ns()-10_000_000_000);assert check() is None
        os.close(w);w=None;assert check()=='frontend_closed'
    finally:
        os.close(r)
        if w is not None:os.close(w)


@pytest.mark.parametrize('changes,reason',[
    ({'monotonic_ns':0},'frontend_heartbeat_timeout'),
    ({'owner_id':'two'},'frontend_owner_changed'),
    ({'clock_id':'another'},'frontend_clock_mismatch')])
def test_pipe_heartbeat_detects_real_owner_failure(changes,reason):
    r,w=os.pipe()
    try:
        check=PipeHeartbeat(r,'one');pulse(w,**changes);assert check()==reason
    finally:os.close(r);os.close(w)


def test_control_root_cause_is_not_replaced_by_stop_observation():
    status=OperatorStatus()
    original=describe(dict(code='observed_limit',phase='STREAMING',action='end_control',observations=[
        dict(joint='left_shoulder_roll_joint',field='velocity',value=-.829,threshold=.75,unit='rad/s'),
        dict(joint='right_shoulder_roll_joint',field='velocity',value=.786,threshold=.75,unit='rad/s')]))
    status.accept(original)
    status.accept(describe(dict(code='stop_unconfirmed',phase='FAILED',action='control_ended')))
    assert status.displayed is original
    assert '左肩滚转' in status.displayed['details'][0] and '0.75000' in status.displayed['details'][0]
    assert len(status.displayed['details'])==2 and status.current['code']=='stop_unconfirmed'


def test_cancel_is_normal_and_wait_is_explained():
    wait=describe(dict(code='waiting_stationary',phase='WAIT_FEEDBACK',action='wait',observations=[
        dict(joint='left_shoulder_roll_joint',field='velocity',value=.0278,threshold=.02,unit='rad/s')]))
    assert '条件满足后继续' in wait['effect'] and '0.02000' in wait['details'][0]
    status=OperatorStatus();status.accept(wait)
    status.accept(describe(dict(code='interrupted',action='cancel_or_stop')))
    assert status.first_fault is None


def test_workspace_fault_does_not_destroy_readonly_connection_or_resume_old_control(tmp_path):
    root=Path(__file__).resolve().parents[1]
    config=yaml.safe_load((root/'configs/hardware_field.yaml').read_text())
    config['collision_scene']=str(root/'configs/collision_scene.yaml')
    source=tmp_path/'profile.yaml';source.write_text(yaml.safe_dump(config))
    workspace=tmp_path/'desk';create_session(source,workspace,181,'hardware_position_stream',workspace=True)
    desk=Workspace(workspace);first=desk.new_control()
    event(workspace/first,'terminal_fault',reason='unit_fixture')
    assert Session(workspace/first).status()['blocked_reason']
    assert desk.status()['blocked_reason'] is None
    second=desk.new_control()
    assert first!=second and Session(workspace/second).status()['blocked_reason'] is None
    with pytest.raises(ValueError):Session(workspace)


def test_cancel_before_spawn_cannot_issue_a_late_start(tmp_path):
    marker=tmp_path/'unexpected_start'
    command=[sys.executable,'-c',f'from pathlib import Path;Path({str(marker)!r}).write_text("bad")']
    result=run_process(command,None,tmp_path/'log',cancel_check=lambda:'operator_cancel')
    assert not result.started and result.interruption=='operator_cancel' and not marker.exists()


def test_cancel_during_recorder_startup_does_not_later_start_executor(tmp_path):
    marker=tmp_path/'unexpected_start';recorder_command=[sys.executable,'-c','import time;time.sleep(30)']
    with recording_process(recorder_command,None,tmp_path/'recorder.log',tmp_path/'never_ready',
            cancel_check=lambda:'operator_cancel') as recorder:
        result=run_process([sys.executable,'-c',f'open({str(marker)!r},"w").close()'],None,tmp_path/'exec.log',recorder=recorder)
    assert not result.started and result.interruption=='operator_cancel' and recorder.poll() is not None
    assert not marker.exists()


def test_status_pipe_continues_when_archival_log_fails():
    class FullDisk:
        def write(self,*args):raise OSError('unit simulated full disk')
        def flush(self):raise OSError('unit simulated full disk')
    output=[]
    process=subprocess.Popen([sys.executable,'-c','print("first",flush=True);print("second",flush=True)'],stdout=subprocess.PIPE,text=True)
    pump=OutputPump(process.stdout,FullDisk(),output.append)
    process.wait(timeout=5);pump.join()
    assert pump.log_failed and pump.error is None
    assert any('recording_io_error' in line for line in output)
    assert 'first\n' in output and 'second\n' in output


def test_ui_drains_feedback_without_building_a_backlog(tmp_path):
    source='import json\nfor i in range(300):print("FIELDWORK_IPC "+json.dumps(dict(type="feedback",q=[i])))\n'
    child=Child([sys.executable,'-c',source],tmp_path/'child.log')
    try:
        child.process.wait(timeout=5);child.reader.join(timeout=2)
        rows=child.drain();assert len(rows)==1 and rows[0]['q']==[299]
    finally:child.close()


def test_unexpected_observer_exit_stops_control_with_real_cause():
    from casbot_calib.fieldwork_gui import Desk
    class FakeChild:
        def __init__(self,done):self.finished=done
        def done(self):return self.finished
        def drain(self):return []
        def close(self):pass
    desk=Desk.__new__(Desk)
    desk.observer=FakeChild(True);desk.task=FakeChild(False);desk.observer_cancel=None
    desk.observer_response=dict(completed=False,error='fixture subscription error')
    desk.operator=OperatorStatus();desk.start_requested=True;desk.sending=True
    desk.short=None;desk.q=None;desk.renderer=None;desk.pulse=lambda:None
    desk.poll()
    assert desk.control_cancel=='observation_exited' and not desk.start_requested
    assert desk.operator.first_fault['code']=='observation_exited'
    assert 'fixture subscription error' in desk.message


def test_recording_notice_cancels_short_but_does_not_stop_manual_stream():
    from casbot_calib.fieldwork_gui import Desk
    desk=Desk.__new__(Desk)
    desk.operator=OperatorStatus();desk.short={'running':True};desk.sending=True
    desk.start_requested=False;desk.send=lambda:None
    desk._handle(dict(type='status',event=describe(dict(code='recording_io_error',action='recording_warning',message='异步写盘队列已满，控制CSV暂停'))))
    assert '异步写盘队列已满' in desk.recording_error and desk.short is None and desk.sending
    assert desk.operator.first_fault is None
    desk._handle(dict(type='recording_error',message='EFBIG fixture'))
    assert '当前录制不完整' in desk.recording_error and '停止通道继续' in desk.recording_error
    assert desk.sending


def test_shared_recorder_watches_exact_process_and_blocks_dead_start(tmp_path):
    from casbot_calib.sdk_process import SharedRecorder
    child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'])
    try:
        stat=Path(f'/proc/{child.pid}/stat').read_text();ticks=stat[stat.rfind(')')+2:].split()[19]
        watch=SharedRecorder(dict(recorder_pid=child.pid,process_start_ticks=ticks))
        assert watch.poll() is None
        wrong=SharedRecorder(dict(recorder_pid=child.pid,process_start_ticks='wrong'))
        assert wrong.poll() is not None
        child.terminate();child.wait(timeout=5)
        assert watch.poll() is not None
        marker=tmp_path/'unexpected_start'
        result=run_process([sys.executable,'-c',f'open({str(marker)!r},"w").close()'],None,tmp_path/'log',recorder=watch)
        assert not result.started and result.interruption=='raw_recorder_exited' and not marker.exists()
    finally:
        if child.poll() is None:child.terminate();child.wait(timeout=5)
