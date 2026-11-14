"""GUI control/ImGui draw-data checks without starting a renderer or inspecting visuals."""
import copy
import json
import time
from pathlib import Path
import numpy as np
import pytest
from casbot_calib.shm_gui import Desk, parser, validate_fields, SETTINGS
from casbot_calib.shm_data import ROOT
from test_shm_gui_remote import wait
from test_shm_fieldwork import NATIVE


def test_input_ranges_reject_without_clamping():
    fields=dict(position=[0.]*14,velocity=[0.]*14,effort=[0.]*14,kp=[20.]*14,kd=[2.]*14)
    hello=dict(lower=[-1.]*14,upper=[1.]*14,velocity=[1.5]*14)
    assert validate_fields(fields,hello,SETTINGS)['kp'][0]==20
    fields['position'][0]=2
    with pytest.raises(ValueError,match='ID12'):validate_fields(fields,hello,SETTINGS)
    assert fields['position'][0]==2
    fields['position'][0]=float('nan')
    with pytest.raises(ValueError):validate_fields(fields,hello,SETTINGS)


def test_preview_widgets_without_graphics_or_control(tmp_path):
    from imgui_bundle import imgui
    from casbot_calib.fieldwork_style import setup_gui
    args=parser().parse_args(['--preview']);root=tmp_path/'ui';root.mkdir()
    context=imgui.create_context();imgui.get_io().set_ini_filename(str(tmp_path/'window.ini'))
    desk=Desk(root,args,render=False)
    try:
        setup_gui(desk);io=imgui.get_io();io.display_size=imgui.ImVec2(1440,980);io.delta_time=1/60;io.fonts.get_tex_data_as_rgba32()
        imgui.new_frame();imgui.set_next_window_pos(imgui.ImVec2(0,0));imgui.set_next_window_size(imgui.ImVec2(1440,980));imgui.begin('test');desk.gui();imgui.end();imgui.render()
        assert desk.client is None and desk.frame==1
        assert imgui.get_draw_data().total_vtx_count>0
    finally:desk.close();imgui.destroy_context(context)


@pytest.mark.skipif(not (NATIVE/'shm_gui_owner').exists(),reason='build GUI owner')
def test_desk_control_remote_mock_and_settings(tmp_path):
    from imgui_bundle import imgui
    from casbot_calib.fieldwork_style import setup_gui
    args=parser().parse_args(['--mock','--executor',str(NATIVE/'shm_executor'),'--remote-output',str(tmp_path/'remote')])
    root=tmp_path/'local';root.mkdir();desk=Desk(root,args,render=False);desk.connect()
    context=imgui.create_context();imgui.get_io().set_ini_filename(str(tmp_path/'window.ini'))
    def spin(test,seconds=6):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            desk.poll();desk.pulse()
            if test():return
            assert not desk.client.failure,desk.client.failure
            time.sleep(.01)
        raise AssertionError(desk.message)
    try:
        spin(lambda:desk.connected and desk.client.hello and desk.settings_pending is None)
        setup_gui(desk);io=imgui.get_io();io.display_size=imgui.ImVec2(1440,980);io.delta_time=1/60;io.fonts.get_tex_data_as_rgba32()
        desk.start();spin(lambda:desk.phase==3)
        desk.commands['position'][0]+=.005;desk.dirty=True;spin(lambda:desk.feedback['revision']==1)
        desk.settings_open=True
        imgui.new_frame();imgui.set_next_window_pos(imgui.ImVec2(0,0));imgui.set_next_window_size(imgui.ImVec2(1440,980));imgui.begin('test');desk.gui();imgui.end();imgui.render()
        desk.stop();spin(lambda:desk.phase==-1);assert desk.feedback['disable_ack']
        assert 'rclpy' not in __import__('sys').modules
    finally:desk.close();imgui.destroy_context(context)


def test_per_axis_limits_are_independent_and_inputs_remain_unclipped():
    from casbot_calib.shm_gui import axis_policy
    settings=copy.deepcopy(SETTINGS);settings['axes']['12']=dict(kp_max=0,observed_lower=-.1,input_lower=-.5)
    hello=dict(lower=[-1.]*14,upper=[1.]*14,velocity=[1.5]*14)
    assert axis_policy(settings,hello,0)['kp_max']==0
    assert axis_policy(settings,hello,1)['kp_max']==100
    fields=dict(position=[0.]*14,velocity=[0.]*14,effort=[0.]*14,kp=[20.]*14,kd=[2.]*14)
    with pytest.raises(ValueError,match='ID12 kp'):validate_fields(fields,hello,settings)
    assert fields['kp'][0]==20


def test_old_native_protocol_remains_readonly(tmp_path):
    from types import SimpleNamespace
    root=tmp_path/'old_protocol';root.mkdir();desk=Desk(root,parser().parse_args([]),render=False)
    try:
        desk.client=SimpleNamespace(hello=dict(version=1,hand_background=True),failure=None)
        desk.model_ok=True;desk.feedback=dict(receive_ns=1,phase=-1)
        assert desk.connected and not desk.control_available
        desk.start();assert '只读' in desk.message
    finally:desk.client=None;desk.close()
