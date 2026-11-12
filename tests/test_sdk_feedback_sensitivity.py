"""Read-only paired perturbations, identifiable models and honest tolerances."""
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
import pytest
from casbot_calib.hardware_probe import field_probe_config, settings, probe_problem
from casbot_calib.sdk_feedback_sensitivity_math import (
    default_cases, perturb, estimate_a, estimate_b, crossing_summary, compare_to_baseline)
from casbot_calib.sdk_feedback_sensitivity import evaluate_record, analytic_controls, assess
from test_hardware_probe import fixture, rotor


def case(kind, value, *, phase=0., seed=11):
    return dict(id=kind, family=kind, severity=abs(value), unit='ms',
                operations=[dict(kind=kind, value=value, phase=phase, seed=seed)])


def record(*, dual=True):
    t = np.arange(0., 4., .001); sample = np.floor(t/.005)*.005
    f = np.array([2.5, 10.] if dual else [5.]); amp = np.array([.0075, .00068] if dual else [.004])
    angle = 2*np.pi*sample[:, None]*f
    q = .18*sample+np.sum(amp*np.sin(angle), axis=1)
    v = .18+np.sum(2*np.pi*f*amp*np.cos(angle), axis=1)
    a = -np.sum((2*np.pi*f)**2*amp*np.sin(angle), axis=1)
    arrays = {}
    for key, value in [('q', q), ('dq', v), ('effort_raw', 5e-5*a+.01*v+.005)]:
        arrays[key] = np.zeros((len(t), 14)); arrays[key][:, 4] = value
    arrays['receive_ns'] = np.int64(10**15)+np.rint(t*1e9).astype(np.int64)
    motion = dict(conditions=dict(joint=4, direction=1), labels=['initial', 'probe_measure', 'final'], knots_s=[0., 1., 3., 4.])
    return dict(obs=arrays, elapsed=t, window=(1., 3.), motion=motion, engine=rotor(),
        config=field_probe_config(settings()), profile=dict(effort_gain=1., effort_zero_nm=0.),
        prior=dict(viscous=[.01]*14, coulomb=[.005]*14, static_friction=[.005]*14),
        frequencies=f.tolist(), scales=(5e-5, .01), metadata=dict(job=dict(id='analytic', direction=1), source='fixture'))


def test_grid_frozen_unique_and_original_threshold():
    a, b = default_cases(), default_cases()
    assert a == b and a[0]['id'] == 'baseline'
    assert len({c['id'] for c in a}) == len(a)
    assert {c['family'] for c in a} >= {'effort_advance', 'shared_hold', 'effort_hold', 'arrival_gap', 'arrival_jitter', 'effort_noise', 'combined'}


@pytest.mark.parametrize('kind,value', [('shared_hold', 5), ('effort_hold', 5), ('shared_irregular_hold', 5),
    ('effort_irregular_hold', 5), ('arrival_gap', 20), ('arrival_jitter', 10), ('shared_stall', 100),
    ('effort_stall', 50), ('effort_noise', .1), ('effort_lowpass', 5), ('effort_advance', -5)])
def test_perturbations_are_deterministic_and_never_mutate_inputs(kind, value):
    obs, t, _ = fixture(); before = deepcopy(obs); original_t = t.copy()
    profile = dict(effort_gain=2., effort_zero_nm=.04)
    a, ta, audit = perturb(obs, t, (1., 3.), profile, case(kind, value, phase=.5))
    b, tb, _ = perturb(obs, t, (1., 3.), profile, case(kind, value, phase=.5))
    for key in a: np.testing.assert_array_equal(a[key], b[key])
    for key in obs: np.testing.assert_array_equal(obs[key], before[key])
    np.testing.assert_array_equal(t, original_t); np.testing.assert_array_equal(ta, tb)
    assert np.all(np.diff(a['receive_ns']) > 0) and np.all(np.diff(ta) > 0)
    np.testing.assert_allclose(ta-ta[0], (a['receive_ns']-a['receive_ns'][0])*1e-9, atol=2e-15)
    assert audit['extra_degradation_only']


def test_hold_keeps_clocks_and_common_channels_but_never_selects_future():
    obs, t, _ = fixture()
    obs['q'][:, 0] = t; obs['dq'][:, 0] = 2*t; obs['effort_raw'][:, 0] = 3*t
    a, ta, _ = perturb(obs, t, (1, 3), dict(effort_gain=1., effort_zero_nm=0.), case('shared_hold', 7, phase=.5))
    np.testing.assert_array_equal(ta, t); np.testing.assert_array_equal(a['receive_ns'], obs['receive_ns'])
    assert np.all(a['q'][:, 0] <= t)
    np.testing.assert_allclose(a['dq'][:, 0], 2*a['q'][:, 0]); np.testing.assert_allclose(a['effort_raw'][:, 0], 3*a['q'][:, 0])


def test_effort_only_preserves_motion_and_scale_operates_in_calibrated_nm():
    obs, t, _ = fixture(); profile = dict(effort_gain=2., effort_zero_nm=.04)
    a, _, _ = perturb(obs, t, (1, 3), profile, case('effort_gain', 10))
    np.testing.assert_array_equal(a['q'], obs['q']); np.testing.assert_array_equal(a['dq'], obs['dq'])
    np.testing.assert_allclose((a['effort_raw']-.04)*2., (obs['effort_raw']-.04)*2.*1.1, atol=1e-16)


def test_arrival_gap_removes_rows_stall_does_not():
    obs, t, _ = fixture(); profile = dict(effort_gain=1., effort_zero_nm=0.)
    stall, _, sa = perturb(obs, t, (1, 3), profile, case('shared_stall', 50, phase=.5))
    gap, _, ga = perturb(obs, t, (1, 3), profile, case('arrival_gap', 50, phase=.5))
    assert sa['removed_publications'] == 0 and ga['removed_publications'] == 50
    assert len(stall['q'])-len(gap['q']) == 50


def test_baseline_and_two_routes_share_the_same_problem_and_known_inertia():
    r = record(); result = evaluate_record(r, default_cases()[:1])
    for route in ('A', 'B'):
        fit = result['baseline'][route]
        assert fit['available'] and abs(fit['inertia_kg_m2']/5e-5-1) < .02
        assert result['results'][0]['routes'][route]['change_from_own_baseline_relative'] == 0
    assert not result['absolute_inertia_error_known']


def test_relative_lag_is_not_shared_cache_and_b_handles_pure_phase_better():
    r = record()
    rows = evaluate_record(r, [default_cases()[0], case('shared_hold', 5), case('effort_advance', -2)])['results']
    assert rows[1]['routes']['A']['change_from_own_baseline_relative'] < .03
    assert rows[2]['routes']['A']['change_from_own_baseline_relative'] > .1
    assert rows[2]['routes']['B']['change_from_own_baseline_relative'] < .03


def test_single_tone_does_not_fabricate_a_second_frequency():
    result = evaluate_record(record(dual=False), default_cases()[:1])
    assert result['baseline']['A']['available']
    assert not result['baseline']['B']['available'] and result['baseline']['B']['structural_rank'] == 2


def test_analytical_filter_and_gain_counterexamples_are_not_declared_accuracy():
    controls = {r['case']: r for r in analytic_controls()['cases']}
    assert controls['clean']['A_known_error_relative'] < 1e-10
    assert controls['clean']['B_known_error_relative'] < 1e-8
    assert controls['lag_5ms']['A_known_error_relative'] > .1
    assert controls['lag_5ms']['B_known_error_relative'] < 1e-8
    assert controls['gain_minus10pct']['B_known_error_relative'] > .08
    assert controls['lowpass_10ms']['B_known_error_relative'] > .1


def test_crossing_is_discrete_with_missing_and_nonmonotone_cases():
    baseline = dict(available=True, inertia_kg_m2=.001)
    rows = []
    for level, ratio in [(1, 1.02), (2, 1.11), (3, 1.01), (4, None)]:
        fit = dict(available=ratio is not None, **({'inertia_kg_m2':.001*ratio} if ratio else {}))
        rows.append(dict(case=case('effort_hold', level), routes={k:compare_to_baseline(fit, baseline) for k in ('A', 'B')}))
    for r in crossing_summary(rows):
        assert r['first_observed_exceedance'] == 2 and r['last_consecutive_tested_level_within'] == 1
        assert r['first_unavailable_or_branch_boundary'] == 4 and not r['continuous_tolerance_certified']


def test_evaluator_writes_complete_report_outside_source_without_truth(tmp_path, monkeypatch):
    import casbot_calib.sdk_feedback_sensitivity as module
    source = tmp_path/'input'; source.mkdir(); saved = source/'untouched'; saved.write_text('keep')
    r = record(); r['metadata'].update(source=str(source), joint=4, frequencies_hz=[2.5, 10.])
    monkeypatch.setattr(module, 'load_records', lambda _: iter([r]))
    monkeypatch.setattr(module, 'default_cases', lambda: default_cases()[:3])
    report = assess([source], tmp_path/'output')
    assert report['completed'] and not report['formal_test_consumed'] and not report['source_modified']
    assert saved.read_text() == 'keep'
    assert {p.name for p in (tmp_path/'output').iterdir()} >= {'report.json', 'report.md', 'case_results.csv', 'protocol.json'}
    with pytest.raises(ValueError, match='outside'): assess([source], source/'new')


def test_cli_guard_blocks_truth_process_network_and_source_writes(tmp_path):
    source = tmp_path/'source'; source.mkdir(); (source/'keep').write_text('yes')
    code = '''
import socket,subprocess
from pathlib import Path
from casbot_calib.sdk_feedback_sensitivity import install_readonly_guard
root=Path(__import__('sys').argv[1]);install_readonly_guard([root])
actions=[lambda:(root/'keep').write_text('no'),lambda:(root/'keep').unlink(),
 lambda:open('/missing/evaluation/truth'),lambda:subprocess.run(['true']),
 lambda:socket.socket().connect(('127.0.0.1',1))]
for action in actions:
 try:action()
 except RuntimeError:pass
 else:raise AssertionError('Read-only guard missed operation')
assert (root/'keep').read_text()=='yes'
'''
    subprocess.run([sys.executable, '-c', code, str(source)], check=True)


def test_field_cli_routes_without_ros_or_rich(monkeypatch, tmp_path):
    from casbot_calib import fieldwork_cli, sdk_feedback_sensitivity
    seen = []
    monkeypatch.setattr(sdk_feedback_sensitivity, 'main', lambda args: seen.extend(args) or 0)
    assert fieldwork_cli.main(['sensitivity', '--source', 'a', '--source', 'b', '--output', str(tmp_path/'out')]) == 0
    assert seen == ['--source', 'a', '--source', 'b', '--output', str(tmp_path/'out')]


def test_reference_summary_keeps_direction_failure_when_sensitivity_is_small():
    from casbot_calib.sdk_feedback_sensitivity import reference_comparisons
    row = dict(case=case('shared_hold', 5), routes={k:dict(change_from_own_baseline_relative=.02) for k in ('A','B')})
    report = dict(grouped_thresholds=[dict(source='s')], records=[dict(metadata=dict(source='s'), results=[row])],
        direction_comparisons=[dict(source='s', case_id='shared_hold', routes={k:dict(direction_difference_relative=.13) for k in ('A','B')})])
    summary = reference_comparisons(report)[0]
    assert summary['routes']['B']['maximum_change_relative'] == .02
    assert summary['routes']['B']['maximum_direction_difference_relative'] == .13
