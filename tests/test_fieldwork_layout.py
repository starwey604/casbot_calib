"""Real ImGui layout checks without a GL window, screenshots, ROS or hardware."""
import time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest

pytest.importorskip('imgui_bundle')
from imgui_bundle import imgui
from casbot_calib.fieldwork_gui import Desk,_setup_gui
from casbot_calib.fieldwork_status import OperatorStatus,describe,_SUFFIXES


@pytest.fixture
def layout():
    context=imgui.create_context();io=imgui.get_io();io.display_size=imgui.ImVec2(1440,980);io.delta_time=1/60;io.set_ini_filename('')
    desk=Desk.__new__(Desk)
    desk.owner_id='layout';desk.frame=0;desk.controls={};desk.control_bounds={};desk.gui_managed_ticks=True
    desk.preview=desk.read_only=desk.snapshot=False;desk.status=dict(domain_id=181,scope='localhost_contract_mock')
    desk.cache_monitor=None;desk.owned_mock=None;desk.save_open=False;desk.save_job=None;desk.save_message='';desk.recovery_rows=[]
    desk.save_path='';desk.root=Path('/fixture');desk.observer=None;desk.task=None;desk.pending=None
    desk.start_requested=desk.sending=desk.needs_reinitialize=False;desk.last_recording=None
    desk.message='连接只读观测；点击开始后才申请控制并发送五字段。';desk.recording_error=desk.render_error=None
    desk.operator=OperatorStatus();desk.connected_at=None;desk.connection_channels={};desk.config=dict(joint_topic='/fixture/joint_states',max_feedback_age_ns=100000000)
    desk.names=[side+name for side in ('left_','right_') for name in _SUFFIXES];desk.axes=[[1.,0.,0.]]*14;desk.selected=0
    desk.lower=desk.model_lower=np.full(14,-2.);desk.upper=desk.model_upper=np.full(14,2.);desk.velocity_bounds=np.full(14,.75)
    desk.input_tuning=dict(kp_max=np.full(14,1000.),kd_max=np.full(14,100.),effort_limit_nm=np.full(14,10.))
    desk.commands=dict(position=np.zeros(14),velocity=np.zeros(14),kp=np.full(14,20.),kd=np.full(14,2.),effort=np.zeros(14))
    desk.q=np.zeros(14);desk.first_q=np.zeros(14);desk.receive_ns=None;desk.has_feedback_pose=True;desk.mode='READY'
    desk.feedback=dict(dq=[0.]*14,effort_raw=[0.]*14,kp=[20.]*14,kd=[2.]*14);desk.sources=dict(kp='配置默认20',kd='配置默认2')
    desk.short=None;desk.short_amplitude=.25;desk.short_duration=6.;desk.texture=None
    desk.zero_duration=5.
    desk.limits=SimpleNamespace(usable=True,error=None,pinned=None,accepted_observation=None,ack=None,
        settings=dict(general=dict(short_amplitude_deg=1.,short_min_s=2.,short_max_s=12.)),gui=lambda remember:None)
    desk.check_commands=lambda:None;desk.update_texture=lambda:None;desk.pulse=lambda:None
    desk.change=lambda key,index,value:desk.commands[key].__setitem__(index,value)
    _setup_gui(desk);io.fonts.get_tex_data_as_rgba32()
    def frame():
        imgui.new_frame();imgui.set_next_window_pos(imgui.ImVec2(0,0));imgui.set_next_window_size(imgui.ImVec2(1440,980))
        flags=imgui.WindowFlags_.no_title_bar|imgui.WindowFlags_.no_resize|imgui.WindowFlags_.no_saved_settings
        imgui.begin('layout_fixture',flags=flags);desk.gui();imgui.end();imgui.render()
        return {key:tuple(value) for key,value in desk.control_bounds.items()}
    frame();frame() # Resolve first-frame table/scrollbar sizing before comparison.
    try:yield desk,frame
    finally:imgui.destroy_context(context)


def warning(desk):
    return describe(dict(code='observed_warning',phase='STREAMING',action='warning',observations=[
        dict(joint=name,field='velocity',value=1.3,threshold=.75,unit='rad/s') for name in desk.names]))


def test_notices_do_not_move_joint_or_five_field_controls(layout,monkeypatch):
    desk,frame=layout;baseline=frame();keys=[*[f'position_{i}' for i in range(14)],'velocity','kp','kd','effort','model_image']
    assert baseline['zero_duration'][2]<=1440
    assert baseline['short'][3]<=968, baseline['short']
    scroll={};stack=[];begin,end=imgui.begin_child,imgui.end_child
    def child(name,*args,**kwargs):stack.append(name);return begin(name,*args,**kwargs)
    def finish():
        if stack[-1]=='status_band':scroll['max_y']=imgui.get_scroll_max_y()
        end();stack.pop()
    monkeypatch.setattr(imgui,'begin_child',child);monkeypatch.setattr(imgui,'end_child',finish)
    desk.operator.accept(warning(desk));desk.message='五字段发送中；拖动滑条更新目标。'
    frame();bounds=frame()
    assert scroll['max_y']>0 # Long details remain available by scrolling, not deletion.
    assert {k:bounds[k] for k in keys}=={k:baseline[k] for k in keys}
    desk.recording_error='缓存容量已达到上限，已停止追加记录；反馈和手动控制继续。'*8
    desk.render_error='几何视图暂不可用；数值反馈继续。';desk.save_message='保存会话完成：'+('long-path/'*40)
    desk.operator.accept(describe(dict(code='stale_mode',action='end_control',observations=[])))
    desk.operator.accept(describe(dict(code='stop_unconfirmed',action='control_ended',observations=[])))
    frame();bounds=frame()
    assert {k:bounds[k] for k in keys}=={k:baseline[k] for k in keys}
    desk.operator=OperatorStatus();desk.recording_error=desk.render_error=None;desk.save_message='';desk.message='恢复正常'
    desk.limits.usable=False;desk.limits.error='配置错误：'+('long error '*50)
    frame();bounds=frame()
    assert {k:bounds[k] for k in keys}=={k:baseline[k] for k in keys}
    desk.limits.usable=True;desk.observer=SimpleNamespace(done=lambda:False);desk.connected_at=time.monotonic();desk.connection_channels={'joint':dict(publishers=0)}
    desk.operator.accept(describe(dict(code='waiting_feedback',action='wait',since_ns=time.monotonic_ns(),observations=[])))
    frame();bounds=frame()
    assert {k:bounds[k] for k in keys}=={k:baseline[k] for k in keys}


def test_drag_continues_at_same_position_across_warning_and_cache_full(layout):
    desk,frame=layout;baseline=frame();x1,y1,x2,y2=baseline['position_0'];y=(y1+y2)/2;x=(x1+x2)/2
    io=imgui.get_io();io.add_mouse_pos_event(x,y);frame();io.add_mouse_button_event(0,True);frame()
    desk.operator.accept(warning(desk));io.add_mouse_pos_event(x+.08*(x2-x1),y);frame();first=desk.commands['position'][0]
    assert first>.05
    desk.recording_error='缓存已满；发送继续。';io.add_mouse_pos_event(x+.16*(x2-x1),y);frame();second=desk.commands['position'][0]
    assert second>first
    desk.recording_error=None;desk.operator=OperatorStatus();io.add_mouse_pos_event(x+.24*(x2-x1),y);bounds=frame()
    assert desk.commands['position'][0]>second
    assert bounds['position_0']==baseline['position_0']
    assert np.all(desk.commands['position'][1:]==0)
    io.add_mouse_button_event(0,False);frame()


def test_feedback_caption_and_limits_ack_do_not_move_tuning_sliders(layout):
    desk,frame=layout;baseline=frame();keys=['velocity','kp','kd','effort']
    desk.texture=1;desk.receive_ns=time.monotonic_ns();desk.sending=True
    desk.limits.accepted_observation=dict(defaults=dict(input_lower=None,input_upper=None,input_velocity=.75,input_effort=10.,kp_max=1000.,kd_max=100.,
        observed_lower=None,observed_upper=None,observed_velocity=.75,observed_action=0),overrides={},joint_names=desk.names)
    desk.limits.ack=2
    frame();bounds=frame()
    assert {k:bounds[k] for k in keys}=={k:baseline[k] for k in keys}
