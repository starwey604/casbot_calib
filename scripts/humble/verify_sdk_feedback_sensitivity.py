#!/usr/bin/env python3
"""Read-only baseline replay checks and independent phase-profile verification."""
import argparse
import csv
from pathlib import Path
import numpy as np
from casbot_calib.data import write_json
from casbot_calib.sdk_feedback_sensitivity import read, install_readonly_guard, load_records
from casbot_calib.sdk_feedback_sensitivity_math import default_cases, perturb
from casbot_calib.hardware_probe import probe_problem
from casbot_calib.hardware_roll_frequency import phasors
from verify_sdk_roll_dual_capture import profiled_fit


def check_cross_session(root, report):
    """Independent arithmetic audit of every cached old/new signed pairing."""
    cross=read(Path(root)/'cross_session/report.json')
    previous=read(Path(cross['reference_assessment'])/'report.json')
    maximum=0.;count=0;all_available_flags=True;all_case_bindings=True
    for group in cross['groups']:
        old={r['metadata']['job']['direction']:r for r in previous['records'] if r['metadata']['source']==group['reference_source']}
        new={r['metadata']['job']['direction']:r for r in report['records'] if r['metadata']['source']==group['repeat_source']}
        for index,row in enumerate(group['cases']):
            for route in ('A','B'):
                for signed in row['routes'][route]['directions']:
                    direction=signed['direction'];a=old[direction]['results'][index];b=new[direction]['results'][index]
                    all_case_bindings &= a['case']==b['case']==row['case']
                    av,bv=a['routes'][route],b['routes'][route]
                    available=av['available'] and bv['available'];all_available_flags &= signed['available']==available
                    if available:
                        expected=abs(bv['inertia_kg_m2']-av['inertia_kg_m2'])/max(abs(av['inertia_kg_m2']),1e-6)
                        maximum=max(maximum,abs(expected-signed['cross_session_inertia_difference_relative']))
                    else:all_available_flags &= signed['cross_session_inertia_difference_relative'] is None
                    count+=1
    return dict(passed=maximum<1e-12 and all_case_bindings and all_available_flags,
        same_case_bindings=all_case_bindings,availability_flags_correct=all_available_flags,
        signed_route_comparisons=count,maximum_relative_arithmetic_difference=maximum)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); root = args.source.resolve(); out = args.output.resolve()
    report = read(root/'report.json'); sources = [Path(p) for p in report['protocol']['sources']]
    if any(out == p or p in out.parents for p in [root]+sources): raise ValueError('Verification output must be outside inputs')
    install_readonly_guard([root]+sources)
    out.mkdir(parents=True, exist_ok=False)
    checks = dict(completed=report['completed'], fixed_grid=report['protocol']['cases']==default_cases(),
        original_threshold=report['protocol']['diagnostic_threshold']==.1 and report['protocol']['inertia_denominator_floor_kg_m2']==1e-6,
        no_formal_test_or_candidate=not report['formal_test_consumed'] and not report['protocol']['candidate_written'])
    baseline_difference = 0.; fitted = 0; numerical_unavailable = 0; structural_unavailable = 0
    for record in report['records']:
        original = read(Path(record['metadata']['source'])/'analysis/report.json')
        previous = next(r for r in original['records'] if r['job']['id']==record['metadata']['job']['id'])
        baseline_difference = max(baseline_difference, abs(record['baseline']['A']['inertia_kg_m2']-previous['inertia_diagnostic']['inertia_kg_m2']))
        if record['baseline']['B']['available']:
            baseline_difference = max(baseline_difference, abs(record['baseline']['B']['inertia_kg_m2']-previous['frequency_analysis']['full']['free_b_complex']['inertia_kg_m2']))
        assert [r['case'] for r in record['results']] == default_cases()
        for row in record['results']:
            assert set(row['routes']) == {'A', 'B'}
            for fit in row['routes'].values():
                if fit['available']: fitted += 1
                elif fit.get('structural_rank') == 2: structural_unavailable += 1
                else: numerical_unavailable += 1
    checks['historical_unperturbed_parameters_preserved'] = baseline_difference < 1e-12
    with (root/'case_results.csv').open() as stream: count = len(list(csv.DictReader(stream)))
    checks['complete_paired_csv'] = count == len(report['records'])*len(default_cases())*2
    # Rebuild representative public perturbations. A uses a direct linear solve;
    # B is checked by independent 1D phase profiling + nonnegative linear solve.
    multi = next(r for r in report['records'] if len(r['metadata']['frequencies_hz']) >= 2)
    source = next(load_records(multi['metadata']['source']))
    cases = ['baseline', 'effort_advance_-5_s0_p0', 'effort_advance_5_s0_p0',
             'effort_lowpass_5_s0_p0', 'shared_hold_5_s0_p0.5', 'combined_5', 'effort_noise_0.1_s11_p0']
    verification = []; max_a = 0.; max_b = 0.; max_phase = 0.
    for identifier in cases:
        recorded = next(r for r in multi['results'] if r['case']['id']==identifier)
        obs, t, _ = perturb(source['obs'], source['elapsed'], source['window'], source['profile'], recorded['case'])
        problem = probe_problem(obs, t, source['motion'], source['engine'], source['config'], source['profile'])
        p = problem; j = p['joint']; t = p['time_s']; centered = t-t.mean()
        x = np.column_stack([p['ddq'][:, j], p['dq'][:, j], np.ones(len(t)), centered, centered**2])
        coefficients = np.linalg.lstsq(x, p['target']-p['projected_gravity_nm'], rcond=1e-12)[0]
        known = p['projected_gravity_nm']+p['rotor']+source['prior']['coulomb'][j]*p['extra'][:, 14+j]+source['prior']['static_friction'][j]*p['static_feature']
        z = phasors(t, np.column_stack([p['observed_torque_nm'], p['ddq'][:, j], p['dq'][:, j], known]), source['frequencies'], origin_s=float(t.mean()))
        y, a, v, c = z.T
        independent = profiled_fit(source['frequencies'], a, v, c, y)
        da = abs(coefficients[0]-recorded['routes']['A']['inertia_kg_m2'])
        db = abs(independent['inertia_kg_m2']-recorded['routes']['B']['inertia_kg_m2'])
        dp = abs(independent['effective_advance_s']-recorded['routes']['B']['effective_advance_s'])
        max_a = max(max_a, da); max_b = max(max_b, db); max_phase = max(max_phase, dp)
        verification.append(dict(case_id=identifier, A_inertia_difference_kg_m2=float(da),
            B_inertia_difference_kg_m2=float(db), B_phase_difference_s=float(dp)))
    checks.update(independent_A_agrees=max_a<1e-10, independent_B_agrees=max_b<1e-9,
        independent_phase_agrees=max_phase<1e-6)
    cross_check=check_cross_session(root,report) if report.get('cross_session_report') else None
    if cross_check is not None:checks['all_cross_session_signed_comparisons_agree']=cross_check['passed']
    result = dict(passed=all(checks.values()), checks=checks, baseline_max_difference_kg_m2=baseline_difference,
        representative_checks=verification, records=len(report['records']), cases_per_record=len(default_cases()),
        paired_csv_rows=count, fits_available=fitted, structural_unavailable=structural_unavailable,
        numerical_unavailable=numerical_unavailable, cross_session_check=cross_check,
        physical_device_connected=False, formal_test_consumed=False)
    write_json(out/'verification.json', result)
    print(result)
    if not result['passed']: raise SystemExit(1)


if __name__ == '__main__': main()
