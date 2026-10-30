"""Static field reference planning: role isolation, nominal gates and portability."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
import yaml
from casbot_calib.dual_model import nominal_dual_arm
from casbot_calib.static_field import (candidate_pool, paired_motions, validate_config,
    plan_static_field, verify_static_field, information)
from casbot_calib.staged_motion import derivative_bounds
from casbot_calib.vendor_identity import sha256

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def config():
    config = yaml.safe_load((ROOT / 'configs/static_field_nominal.yaml').read_text())
    config['collision_scene'] = str(ROOT / 'configs/collision_scene.yaml')
    return config


def test_candidate_grid_is_named_relative_seven_axis_and_deduplicates_neutral(config):
    engine, source = nominal_dual_arm(); validate_config(config, engine, source)
    rows, count = candidate_pool(config, engine)
    assert count == 2 * (7 * 7 + 3 ** 5) and len(rows) == 570
    home = np.asarray(config['home_rad'])
    for arm in ('left', 'right'):
        subset = [r for r in rows if r['arm'] == arm]
        assert len({tuple(r['q_rad']) for r in subset}) == len(subset)
        neutral = next(r for r in subset if np.array_equal(r['q_rad'], home))
        assert len(neutral['aliases']) == 8  # seven neutral jogs and neutral combination
        row = next(r for r in subset if r['family'] == 'single_axis' and r['offsets_deg'] == [75])
        q = np.array(row['q_rad']); joint = row['active_joints'][0]
        assert q[joint] - home[joint] == pytest.approx(np.deg2rad(75))
        np.testing.assert_array_equal(np.delete(q, joint), np.delete(home, joint))


@pytest.mark.parametrize('change', ['mode', 'model', 'joint', 'home', 'period', 'budget', 'nan'])
def test_unqualified_or_ambiguous_configuration_is_rejected(config, change):
    c = deepcopy(config); engine, source = nominal_dual_arm()
    if change == 'mode': c['mode'] = 'hardware'
    if change == 'model': c['source_model_sha256'] = 'evt6'
    if change == 'joint': c['arms']['left']['combination_joints'][0] = 'right_shoulder_roll_joint'
    if change == 'home': c['home_rad'][0] = 100.
    if change == 'period': c['command_period_ns'] = 2e6
    if change == 'budget': c['selection']['train_poses_per_arm'] = 13
    if change == 'nan': c['single_axis_offsets_deg'][0] = float('nan')
    with pytest.raises(ValueError): validate_config(c, engine, source)


def test_paired_trials_hold_other_arm_have_true_measure_windows_and_continuous_bounds(config):
    engine, _ = nominal_dual_arm(); rows, _ = candidate_pool(config, engine)
    row = next(r for r in rows if r['family'] == 'single_axis' and r['offsets_deg'] == [-75])
    motions = paired_motions(row, config, split='train'); target = np.array(row['q_rad'])
    for motion, sign in zip(motions, (-1, 1)):
        curve = motion.polynomial(); assert motion.metadata()['initial_final_home']
        a, b = motion.windows('static_measure')[0]; settle = motion.windows('settle')[0]
        assert b - a == pytest.approx(1.) and a - settle[0] == pytest.approx(2.)
        np.testing.assert_allclose(curve((a + b) / 2), target, atol=1e-14)
        near_index = motion.labels.index('approach') + 1
        near = motion.states[near_index][0]
        np.testing.assert_allclose(near[row['active_joints']] - target[row['active_joints']], sign * config['approach_offset_rad'])
        t = np.linspace(0, motion.duration, 100)
        np.testing.assert_allclose(curve(t)[:, 7:], np.broadcast_to(config['home_rad'][7:], (100, 7)), atol=1e-13)
        for k, limit in enumerate(config['planning_envelope'].values(), 1):
            assert np.max(derivative_bounds(curve, k)) <= limit * (1 + 1e-8)
        for point in motion.knots:
            for k in (1, 2, 3): np.testing.assert_allclose(curve(point, nu=k), 0., atol=1e-10)


def test_static_information_does_not_claim_full_rigid_observability(config):
    engine, _ = nominal_dual_arm(); rng = np.random.default_rng(91)
    q = np.asarray(config['home_rad']) + rng.uniform(-.1, .1, (16, 14)); zero = np.zeros_like(q)
    y = engine.regressor_batch(q, zero, zero)['regressor']
    report, _ = information(y, np.ones(140), 1e-7)
    assert report['rank'] == 28 and report['columns'] == 140
    assert not report['unique_link_mass_com_claimed']
    # Rotational-inertia coordinates cannot be excited by static gravity.
    np.testing.assert_allclose(y.reshape(16, 14, 14, 10)[..., 4:], 0., atol=1e-14)


@pytest.fixture(scope='module')
def plan(tmp_path_factory, config):
    root = tmp_path_factory.mktemp('static-field'); path = root / 'preset.yaml'
    path.write_text(yaml.safe_dump(config)); output = root / 'plan'
    report = plan_static_field(path, output)
    path.unlink()  # frozen verification must not reopen the original preset
    return output, report


def test_complete_native_plan_has_disjoint_roles_gated_pairs_and_rejections(plan):
    root, report = plan; assert report['passed']
    assert not any(report[k] for k in ('hardware_executable', 'observations_qualified', 'parameters_identified'))
    assert report['selected_poses'] == 64 and report['selected_trials'] == 128
    rows = json.loads((root / 'candidates.json').read_text()); selected = [r for r in rows if r['status'] == 'selected']
    assert len({tuple(r['q_rad']) for r in selected}) == 64
    assert report['candidate_status_counts']['rejected'] > 0
    assert all(not r['endpoint_gate_only'] and len(r['trial_ids']) == 2 for r in selected)
    for arm in report['arms'].values():
        assert arm['training_information']['rank'] == arm['validation_information']['rank'] == arm['endpoint_pool_information']['rank'] == 14
        assert not arm['missing_signed_diagnostics']
    for name in report['planned_execution_order']:
        assert json.loads((root / 'plans' / name / 'collision_reference.json').read_text())['passed']


def test_frozen_bundle_rechecks_without_source_config_or_observations(plan, tmp_path):
    root, _ = plan; result = verify_static_field(root, tmp_path / 'verify')
    assert result['passed'] and result['reference_trials_rechecked'] == 128
    assert result['maximum_regressor_error'] < 1e-12 and not result['hardware_executable']
    assert result['arms']['right']['signed_diagnostics'] == 14


def test_manifest_tampering_and_resealed_hardware_promotion_are_rejected(plan, tmp_path):
    import shutil
    original, _ = plan; root = tmp_path / 'copy'; shutil.copytree(original, root)
    target = root / 'report.json'; report = json.loads(target.read_text()); report['hardware_executable'] = True
    target.write_text(json.dumps(report))
    with pytest.raises(ValueError, match='input changed'): verify_static_field(root, tmp_path / 'verify1')
    manifest = json.loads((root / 'manifest.json').read_text()); manifest['files']['report.json'] = sha256(target)
    (root / 'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='cannot qualify hardware'): verify_static_field(root, tmp_path / 'verify2')


def test_existing_output_is_never_overwritten(plan, config, tmp_path):
    root, _ = plan; path = tmp_path / 'preset.yaml'; path.write_text(yaml.safe_dump(config))
    before = sha256(root / 'manifest.json')
    with pytest.raises(FileExistsError): plan_static_field(path, root)
    assert sha256(root / 'manifest.json') == before


def test_review_cannot_write_into_its_frozen_input_bundle(plan):
    root, _ = plan; before = sha256(root / 'manifest.json')
    with pytest.raises(ValueError, match='outside the frozen'):
        verify_static_field(root, root / 'review')
    assert not (root / 'review').exists() and sha256(root / 'manifest.json') == before


def test_clear_endpoints_cannot_bypass_a_rejected_continuous_path(config, tmp_path, monkeypatch):
    import casbot_calib.static_field as module
    original = module.gate_motion
    def reject_active(profile, motion, margin):
        if motion.name == 'home': return original(profile, motion, margin)
        return dict(passed=False, reason='injected_interior_collision')
    monkeypatch.setattr(module, 'gate_motion', reject_active)
    c = deepcopy(config); c['combination_offsets_deg'] = [-20, 20]
    path = tmp_path / 'preset.yaml'; path.write_text(yaml.safe_dump(c))
    output = tmp_path / 'rejected'; report = plan_static_field(path, output)
    assert not report['passed'] and report['selected_trials'] == 0
    assert all(r['continuous_path_rejections'] and r['missing_signed_diagnostics'] for r in report['arms'].values())
    rows = json.loads((output / 'candidates.json').read_text())
    assert any(r['reason'] == 'continuous_reference_rejected' for r in rows)
