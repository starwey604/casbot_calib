"""Field operational limits are independent of tracking performance and S2."""
import json
from pathlib import Path

import yaml

from casbot_calib import hardware_field as field, fieldwork_settings as policy


def test_field_profiles_and_executor_parameters_have_no_tracking_gate():
    for path in ('configs/hardware_field.yaml','configs/dvt2/hardware_field.yaml'):
        config = field.load_profile(path)
        engine, _ = field.model_for(config); names = [j.name for j in engine.model.joints]
        assert config['maximum_velocity_rad_s'] == 1.5
        for manual in (False,True):
            params = field.params(config,names,manual=manual)
            assert params['maximum_velocity_rad_s'] == 1.5
            for key in ('tracking_limit_rad','position_allowance_rad'):
                assert key not in config and key not in params
        settings = policy.defaults(config,names,config['model_context'])
        assert settings['defaults']['observed_velocity'] == 1.5
        assert settings['defaults']['input_velocity'] == 1.5
    for path in ('configs/hardware_inertia.yaml','configs/dvt2/hardware_inertia.yaml'):
        assert 'maximum_speed_rad_s' not in yaml.safe_load(Path(path).read_text())['control']


def test_cli_position_warning_has_axis_value_threshold_and_action(capsys):
    field.print_field_limit('FIELDWORK_LIMIT '+json.dumps(dict(joint='left_wrist_pitch_joint',
        field='position_upper',value=1.1,threshold=1.04719755,unit='rad',action='warn')))
    text = capsys.readouterr().out
    assert '左腕俯仰' in text and '1.100000' in text and '1.047198' in text and '仅记录，继续发送' in text
    field.print_field_limit('FIELDWORK_LIMIT '+json.dumps(dict(joint='left_shoulder_pitch_joint',
        field='velocity',value=1.6,threshold=1.5,unit='rad/s',action='end_control')))
    text = capsys.readouterr().out
    assert '1.500000 rad/s' in text and '已停止发送' in text
    field.print_field_limit('FIELDWORK_LIMIT {"action":"recovered"}')
    assert '反馈已回到观察范围；发送继续。' in capsys.readouterr().out


def test_legacy_s2_still_owns_its_original_tracking_configuration():
    from casbot_calib.vendor_position import position_config
    config = position_config('configs/vendor_s2.yaml')
    assert config['tracking_guard_rad'] == .002
    assert config['collision_allowance_rad'] == .003
