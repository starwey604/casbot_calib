"""Manual policies resolve independently of legacy qualification/connection profiles."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from casbot_calib import fieldwork_settings as policy

NAMES=['axis'+str(i) for i in range(14)]
CONTEXT={'fixture':'model'}
LOWER=np.linspace(-3,-1,14);UPPER=np.linspace(1,3,14)
CONFIG=dict(maximum_velocity_rad_s=.75,max_feedback_age_ns=100000000,max_mode_age_ns=500000000,transition_timeout_ns=3000000000)


def settings():return policy.defaults(CONFIG,NAMES,CONTEXT)
def valid(s):return policy.validate(s,NAMES,CONTEXT,LOWER,UPPER)


def test_default_warn_and_distinct_velocity_contracts():
    s=valid(settings());a=policy.resolve(s,LOWER,UPPER)
    assert np.all(a['observed_action']==0)
    s['defaults']['input_velocity']=3.;s['defaults']['observed_velocity']=.2
    lo,hi,tuning,v=policy.command_bounds(valid(s),LOWER,UPPER)
    assert np.all(v==3.) and s['general']['start_velocity']==.02
    np.testing.assert_array_equal(lo,LOWER);np.testing.assert_array_equal(hi,UPPER)


def test_axis_override_inherits_and_explicit_zero_lower():
    s=settings();s['overrides'][NAMES[2]]=dict(input_lower=0.,observed_action=1,kp_max=2500.)
    a=policy.resolve(valid(s),LOWER,UPPER)
    assert a['input_lower'][2]==0 and a['input_lower'][1]==LOWER[1]
    assert a['kp_max'][2]==2500 and a['observed_action'][2]==1


@pytest.mark.parametrize('change',[
    lambda s:s['defaults'].update(input_lower=4.),lambda s:s['defaults'].update(kp_max=-1),
    lambda s:s['defaults'].update(observed_action=2),lambda s:s['general'].update(short_min_s=13),
    lambda s:s['general'].update(feedback_timeout_s=0),lambda s:s['general'].update(heartbeat_timeout_s=float('nan')),
    lambda s:s['mock'].update(watchdog_s=.001),lambda s:s['overrides'].update(unknown={}),
    lambda s:s.update(joint_names=list(reversed(NAMES))),lambda s:s.update(model_context={}),
])
def test_invalid_draft_is_rejected(change):
    s=settings();change(s)
    with pytest.raises(ValueError):valid(s)


def test_policy_versions_persist_and_do_not_rewrite_previous(tmp_path):
    s=settings();one=policy.save(tmp_path,valid(s));first=(tmp_path/'settings/versions/00000001.json').read_text()
    s['defaults']['observed_velocity']=2;two=policy.save(tmp_path,valid(s))
    assert one['revision']==1 and two['revision']==2
    assert policy.load(tmp_path,s)==two
    assert (tmp_path/'settings/versions/00000001.json').read_text()==first


def test_live_wire_only_changes_observation_and_logs_revision(tmp_path):
    old=valid(settings());old['revision']=1;new=copy.deepcopy(old);new['revision']=2
    new['defaults'].update(input_velocity=5.,kp_max=2400.,observed_velocity=4.,observed_action=1)
    new['general']['stop_velocity']=.4
    path=tmp_path/'limits.csv';runtime=policy.write_wire(path,valid(new),LOWER,UPPER,pinned=old)
    row=np.loadtxt(path,delimiter=',');assert row.shape==(154,) and row[0]==2
    assert np.all(runtime['axis']['input_velocity']==.75) and np.all(runtime['axis']['observed_velocity']==4)
    assert runtime['general']['stop_velocity']==.02
    assert not path.with_suffix('.pending').exists()


def test_profile_manual_does_not_require_h1_static_limits(tmp_path):
    import yaml
    from casbot_calib.hardware_field import load_profile,params
    config=yaml.safe_load(Path('configs/hardware_field.yaml').read_text());config['collision_scene']=str(Path('configs/collision_scene.yaml').resolve())
    for key in ('static_velocity_rad_s','static_position_range_rad','static_max_gap_s','gravity_validation_rmse_nm'):
        assert key not in config  # These field quality limits were removed, not made optional.
    for key in ('initial_hold_s','final_hold_s','anchor_transition_s','start_tolerance_rad'):
        config.pop(key)
    path=tmp_path/'profile.yaml';path.write_text(yaml.safe_dump(config))
    loaded=load_profile(path,manual=True);v=params(loaded,NAMES,manual=True)
    assert 'tracking_limit_rad' not in v and 'gravity_feedforward_enabled' not in v
    with pytest.raises(KeyError):load_profile(path)


def test_editor_does_not_move_targets_and_bad_draft_does_not_replace(tmp_path,monkeypatch):
    from casbot_calib.fieldwork_settings_gui import SettingsEditor
    monkeypatch.setenv('CASBOT_FIELDWORK_CACHE',str(tmp_path/'cache'))
    desk=SimpleNamespace(root=tmp_path,config=CONFIG,names=NAMES,status={'model_context':CONTEXT},model_lower=LOWER,model_upper=UPPER,
        read_only=False,preview=False,busy=False,commands={'position':np.ones(14)},sync_bounds=lambda:None)
    editor=SettingsEditor(desk);old=copy.deepcopy(editor.settings)
    editor.draft['defaults']['input_upper']=-5;editor.apply()
    assert editor.error and editor.settings==old
    editor.draft=copy.deepcopy(old);editor.draft['defaults']['input_upper']=.5;editor.apply()
    assert not editor.error and np.all(desk.commands['position']==1.)
    assert editor.bounds()[1][0]==.5


def test_corrupt_policy_requires_explicit_repair(tmp_path):
    from casbot_calib.fieldwork_settings_gui import SettingsEditor
    directory=tmp_path/'settings/versions';directory.mkdir(parents=True);(directory/'00000001.json').write_text('{broken')
    desk=SimpleNamespace(root=tmp_path,config=CONFIG,names=NAMES,status={'model_context':CONTEXT},model_lower=LOWER,model_upper=UPPER,read_only=False,preview=False,busy=False)
    editor=SettingsEditor(desk)
    assert not editor.usable and editor.error
    with pytest.raises(ValueError):editor.prepare()


def test_mock_next_persistence_is_explicit_and_model_bound(tmp_path,monkeypatch):
    monkeypatch.setenv('CASBOT_FIELDWORK_CACHE',str(tmp_path))
    s=settings();s['mock']['torque_limit_nm']=150;policy.save_mock_next(s)
    row=json.loads(policy.mock_next_path().read_text())
    assert row['model_context']==CONTEXT and row['limits']['torque_limit_nm']==150


def test_old_ack_does_not_mark_new_revision_as_applied(tmp_path):
    from casbot_calib.fieldwork_settings_gui import SettingsEditor
    desk=SimpleNamespace(root=tmp_path,config=CONFIG,names=NAMES,status={'model_context':CONTEXT},model_lower=LOWER,model_upper=UPPER,read_only=False,preview=False,busy=False)
    editor=SettingsEditor(desk);initial=editor.prepare();editor.waiting=initial['revision']+1
    editor.handle(dict(code='settings_applied',revision=initial['revision']))
    assert editor.waiting==initial['revision']+1 and editor.accepted_observation==initial
