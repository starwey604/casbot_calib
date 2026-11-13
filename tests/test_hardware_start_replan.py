"""Pre-control replans preserve references and never recover execution faults."""
import csv
import json
from pathlib import Path
import shutil

import numpy as np
import pytest
import yaml

from casbot_calib import hardware_field as field
from casbot_calib.data import write_json
from test_hardware_field import planned, profile, make_capture


def rejection(**changes):
    return dict(dict(passed=False, reason='start_pose_changed', fault='', requires_start_replan=True,
        mode_requested=False, published_commands=0, start_pose_error_rad=[.012]+[0.]*13), **changes)


def moved_snapshot(seed, output, config, domain=21, shift=.012):
    shutil.copytree(seed, output)
    path = output/'execution/snapshot.csv'
    with path.open() as stream: rows = list(csv.DictReader(stream))
    rows[0]['q0'] = str(float(rows[0]['q0'])+shift)
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    write_json(output/'request.json', dict(config=config, domain_id=domain))
    field.seal(output)
    return field._read_snapshot(output)


@pytest.mark.parametrize('change', [dict(mode_requested=True), dict(published_commands=1),
    dict(fault='stale_feedback'), dict(supervisor_interruption='user_interrupt'),
    dict(requires_start_replan=False), dict(reason='mode_rejected'), dict(passed=True),
    dict(mode_requested=None), dict(published_commands=None)])
def test_only_explicit_pre_control_rejection_can_replan(change):
    assert field.start_replan_required(rejection())
    assert not field.start_replan_required(rejection(**change))
    assert not field.start_replan_required(dict(passed=False, fault='start_pose_changed'))


def test_static_recipe_is_self_contained_and_keeps_targets_and_roles(planned, tmp_path):
    original, metadata = planned; original_bytes = (original/'manifest.json').read_bytes()
    snapshot = tmp_path/'fresh'; q = moved_snapshot(tmp_path/'snapshot', snapshot, metadata['config'])
    # Replanning uses the selected sealed recipe, not a changed/missing source preset.
    shutil.rmtree(tmp_path/'preset')
    rebuilt = tmp_path/'rebuilt'; result = field.replan_field(original, snapshot, rebuilt)
    np.testing.assert_array_equal(result['observed_start_rad'], q)
    assert result['replan_recipe'] == metadata['replan_recipe']
    assert [r['conditions'] for r in result['trials']] == [r['conditions'] for r in metadata['trials']]
    positions = np.loadtxt(rebuilt/'position_plan.csv', delimiter=',', skiprows=1)
    np.testing.assert_allclose(positions[[0,-1],1:15], np.tile(q,(2,1)), atol=1e-12)
    assert json.loads((rebuilt/'collision_reference.json').read_text())['passed']
    assert json.loads((rebuilt/'collision_command_interpolant.json').read_text())['passed']
    assert (original/'manifest.json').read_bytes() == original_bytes
    field.check_seal(original); field.check_seal(rebuilt)


@pytest.mark.parametrize('fixed_anchor', [False, True])
def test_jog_keeps_relative_amplitude_or_fixed_frequency_anchor(planned, profile, tmp_path, fixed_anchor):
    _, metadata = planned
    anchor = metadata['observed_start_rad'] if fixed_anchor else None
    original = tmp_path/'jog'; before = field.prepare_jog(profile, tmp_path/'snapshot', original,
        'left_shoulder_pitch_joint', .1, anchor_rad=anchor)
    q = moved_snapshot(tmp_path/'snapshot', tmp_path/'fresh', before['config'])
    result = field.replan_field(original, tmp_path/'fresh', tmp_path/'rebuilt')
    np.testing.assert_array_equal(result['observed_start_rad'], q)
    if fixed_anchor:
        assert result['replan_recipe']['anchor_rad'] == anchor
        a = json.loads((original/'motion.json').read_text())['conditions']
        b = json.loads((tmp_path/'rebuilt/motion.json').read_text())['conditions']
        assert a['comparison_reference_sha256'] == b['comparison_reference_sha256']
    else:
        positions = np.loadtxt(tmp_path/'rebuilt/position_plan.csv', delimiter=',', skiprows=1)
        assert np.ptp(positions[:,1]) == pytest.approx(np.deg2rad(.2), abs=1e-5)
        assert (positions[:,1].min()+positions[:,1].max())/2 == pytest.approx(q[0], abs=1e-5)


@pytest.mark.parametrize('kind', ['jog','static'])
def test_h1_repeated_drift_archives_attempts_and_old_analysis_entry_still_works(planned, profile, tmp_path, monkeypatch, kind):
    original = planned[0]
    if kind == 'jog':
        original = tmp_path/'jog'; field.prepare_jog(profile, tmp_path/'snapshot', original, 'left_shoulder_pitch_joint', .1)
    original_bytes = (original/'manifest.json').read_bytes(); calls = []
    def run(config, output, domain, *, snapshot=False, plan=None, **kwargs):
        output = Path(output); calls.append(snapshot)
        if snapshot:
            moved_snapshot(tmp_path/'snapshot', output, config, domain, shift=.012*sum(calls))
            return dict(passed=True, snapshot_only=True)
        if calls.count(False) <= 2:
            (output/'execution').mkdir(parents=True)
            result = rejection(); write_json(output/'result.json', result)
            (output/'execution/commands.csv').write_text('planned_index,planned_ns,publish_ns,phase\n')
            field.seal(output); return result
        plan = Path(plan).parent; metadata = json.loads((plan/'plan.json').read_text())
        make_capture(plan, metadata, output)
        return dict(passed=True, reason='completed_at_observed_start')
    monkeypatch.setattr(field, 'run_executor', run)
    capture = tmp_path/'capture'; result = field.execute_field(original, capture, 21)
    assert result['passed'] and calls == [False,True,False,True,False]
    assert result['effective_plan'] == 'start_replans/attempt_002/plan'
    field.check_seal(capture)
    archive = capture/'start_replans/attempt_000/capture'; field.check_seal(archive)
    assert json.loads((archive/'result.json').read_text()) == rejection()
    assert (original/'manifest.json').read_bytes() == original_bytes
    assert field.analyse_field(original, capture, tmp_path/'analysis')['passed']
    unrelated = tmp_path/'other'; field.prepare_jog(profile, tmp_path/'snapshot', unrelated, 'left_shoulder_pitch_joint', .2)
    with pytest.raises(ValueError, match='different original plan'):
        field.resolve_field_capture(unrelated, capture)


def test_cancel_during_refresh_does_not_send_another_attempt(planned, tmp_path, monkeypatch):
    original, _ = planned; calls = []
    def run(config, output, domain, *, snapshot=False, **kwargs):
        output = Path(output); output.mkdir(parents=True); calls.append(snapshot)
        result = dict(passed=False, fault='interrupted', supervisor_interruption='user_interrupt') if snapshot else rejection()
        write_json(output/'result.json', result); field.seal(output); return result
    monkeypatch.setattr(field, 'run_executor', run)
    result = field.execute_field(original, tmp_path/'capture', 21)
    assert not result['passed'] and calls == [False,True]
    assert result['supervisor_interruption'] == 'user_interrupt'
    field.check_seal(tmp_path/'capture')


def test_refresh_does_not_bypass_collision_gate(planned, tmp_path):
    original, metadata = planned
    snapshot = tmp_path/'fresh'; moved_snapshot(tmp_path/'snapshot', snapshot, metadata['config'])
    path = snapshot/'execution/snapshot.csv'
    with path.open() as stream: rows = list(csv.DictReader(stream))
    for j in range(14): rows[0]['q'+str(j)] = '0'
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    field.seal(snapshot)
    with pytest.raises(ValueError, match='rejected'):
        field.replan_field(original, snapshot, tmp_path/'collision_plan')
    assert not (tmp_path/'collision_plan/position_plan.csv').exists()


def test_new_profiles_default_to_2s_3s_but_explicit_settings_are_preserved(tmp_path):
    from casbot_calib import fieldwork_settings as policy
    for path in ('configs/hardware_field.yaml', 'configs/dvt2/hardware_field.yaml'):
        config = field.load_profile(path)
        assert config['max_feedback_age_ns'] == 2_000_000_000
        assert config['max_mode_age_ns'] == 3_000_000_000
        engine, _ = field.model_for(config); names = [j.name for j in engine.model.joints]
        values = policy.defaults(config, names, config['model_context'])
        assert values['general']['feedback_timeout_s'] == 2.
        assert values['general']['mode_timeout_s'] == 3.
        values['general'].update(feedback_timeout_s=.1, mode_timeout_s=.5)
        saved = policy.save(tmp_path/path.split('/')[-2], values)
        assert policy.load(tmp_path/path.split('/')[-2], {}) == saved
