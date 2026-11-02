"""Shared policies and process failures exercised without ROS/device access."""
import sys
from pathlib import Path
import numpy as np
import pytest
import yaml
from casbot_calib.control_policy import PD_FACTORS,CONTROL_GROUPS,scale_pd,select_pd_factor,comparison_reductions
from casbot_calib.sdk_process import ros_arguments,recording_process,run_process
from casbot_calib.sdk_mock import mock_config


def test_shared_pd_choice_keeps_input_order_for_ties_and_excludes_inadmissible():
    order=[1.,.5,.75,1.25,1.5]
    rows=[(factor,factor!=1.5,1.) for factor in order]
    assert select_pd_factor(rows)==1.
    rows[-1]=(1.5,False,0.)
    assert select_pd_factor(rows)==1.
    with pytest.raises(ValueError):select_pd_factor([(f,False,1.) for f in PD_FACTORS])
    with pytest.raises(ValueError):select_pd_factor([(1.,True,1.)]*5)


def test_gain_scaling_and_control_differences_preserve_old_numeric_meaning():
    assert CONTROL_GROUPS==(('prior','original'),('prior','tuned'),('identified','original'),('identified','tuned'))
    kp=np.array([2.,4.]);kd=np.array([.1,.2]);factor=.75
    p,d=scale_pd(kp,kd,factor)
    np.testing.assert_array_equal(p,kp*factor);np.testing.assert_array_equal(d,kd*np.sqrt(factor))
    assert comparison_reductions(2.,1.,1.5,.8)==(1-1/2,1-1.5/2,1-.8/1)
    assert comparison_reductions(0.,0.,0.,0.,denominator_floor=1e-15)==(1.,1.,1.)


def test_sdk_mock_local_pd_is_a_confirmed_contract_not_a_switch_to_host_feedback(tmp_path):
    source=Path(__file__).resolve().parents[1]/'configs/sdk_mock.yaml'
    config=yaml.safe_load(source.read_text());config['plant']['device_pd']=False
    path=tmp_path/'invalid.yaml';path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError,match='device-local PD'):mock_config(path)


def test_common_ros_arguments_preserve_empty_optional_values_and_remaps():
    result=ros_arguments('/binary',dict(stop_service='',kp=[0.,1.],enabled=False),{'/joint_states':'/private/joint_states'})
    assert result==['/binary','--ros-args','-p',"stop_service:=''",'-p','kp:=[0.0, 1.0]','-p','enabled:=false','-r','/joint_states:=/private/joint_states']


def test_failed_recorder_never_yields_to_the_executor(tmp_path):
    entered=False
    with pytest.raises(RuntimeError,match='before executor'):
        with recording_process([sys.executable,'-c','raise SystemExit(2)'],None,tmp_path/'recorder.log',tmp_path/'ready',ready_timeout_s=1):
            entered=True
    assert not entered


def test_recorder_exit_stops_owned_executor(tmp_path):
    ready=tmp_path/'ready'
    code='from pathlib import Path; import sys,time; Path(sys.argv[1]).write_text("ready"); time.sleep(.3)'
    with recording_process([sys.executable,'-c',code,str(ready)],None,tmp_path/'recorder.log',ready) as recorder:
        outcome=run_process([sys.executable,'-c','import time; time.sleep(30)'],None,tmp_path/'executor.log',recorder=recorder,timeout_s=2)
    assert outcome.interruption=='raw_recorder_exited' and outcome.returncode!=0
    assert recorder.poll() is not None


def test_executor_timeout_terminates_and_reaps_only_its_child(tmp_path):
    outcome=run_process([sys.executable,'-c','import time; time.sleep(30)'],None,tmp_path/'executor.log',timeout_s=.1)
    assert outcome.interruption=='execution_timeout' and outcome.returncode!=0
