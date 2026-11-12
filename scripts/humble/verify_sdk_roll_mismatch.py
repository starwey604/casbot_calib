#!/usr/bin/env python3
"""Independently profile saved counterfactuals, retaining original Nm weights."""
import argparse
from pathlib import Path
import numpy as np
from casbot_calib.data import write_json
from casbot_calib.sdk_feedback_sensitivity import read, install_readonly_guard
from casbot_calib.sdk_roll_mismatch import PARAMETERS, complex_columns, normalized, hybrid
from verify_sdk_roll_dual_capture import profiled_fit


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('analysis', type=Path); p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); report = read(args.analysis/'report.json')
    install_readonly_guard([args.analysis, *report['sources']])
    try:
        open('/__guard_probe__/evaluation/private', 'rb')
    except RuntimeError:
        guard = True
    else:
        guard = False
    f = report['frequencies_hz']; spectra = []; columns = []
    replay = np.zeros(3); profile_error = np.zeros(3); hybrid_error = np.zeros(3)
    records = []
    for row in report['records']:
        saved = read(Path(row['source'])/'analysis/report.json')
        original = next(r for r in saved['records'] if r['job'] == row['job'])
        z = complex_columns(original['frequency_analysis']['full'])
        spectra.append(normalized(z)); columns.append(z)
        replay = np.maximum(replay, [abs(row['baseline_replay'][k]-row['baseline'][k]) for k in PARAMETERS])
        independent = []
        for test in row['filter_profile']:
            y, a, v, c = z; transfer = 1/(1+1j*2*np.pi*np.array(f)*test['tau_s'])
            result = profiled_fit(f, transfer*a, transfer*v, transfer*c, y)
            profile_error = np.maximum(profile_error, [abs(result[k]-test[k]) for k in PARAMETERS])
            independent.append(dict(tau_s=test['tau_s'], fit=result))
        records.append(independent)
    closure = np.zeros(3); independent_hybrids = []
    for pair in report['pairs']:
        sums = np.sum([[row[k] for k in PARAMETERS] for row in pair['contributions']], axis=0)
        closure = np.maximum(closure, abs(sums-np.array([pair['total_change'][k] for k in PARAMETERS])))
        left, right = [spectra[pair[k]] for k in ('reference_index', 'repeat_index')]
        checked = []
        for mask in (0, 1, 2, 4, 8, 16, 32, 64, 127):
            y, a, v, c = hybrid(left, right, mask)
            actual = profiled_fit(f, a, v, c, y); saved = pair['all_subset_fits'][mask]
            hybrid_error = np.maximum(hybrid_error, [abs(actual[k]-saved[k]) for k in PARAMETERS])
            checked.append(dict(mask=mask, fit=actual))
        independent_hybrids.append(checked)
    tolerances = np.array([1e-9, 1e-7, 1e-6])
    checks = dict(four_records=len(records) == 4, five_pairs=len(report['pairs']) == 5,
        evaluation_guard_active=guard, original_baseline_replayed=bool(np.all(replay < tolerances)),
        exact_attribution_closes=bool(np.all(closure < 1e-12)),
        twenty_filter_profiles_agree=bool(np.all(profile_error < tolerances)),
        fortyfive_counterfactual_profiles_agree=bool(np.all(hybrid_error < tolerances)),
        original_10_percent=report['threshold'] == .1,
        no_calibration_promotion=not any(report[k] for k in ('formal_test_consumed', 'physical_delay_identified',
            'candidate_updated', 'collection_performed', 'full_p0_r2_performed')),
        all_counterfactual_solvers_converged=all(fit['solver_converged'] for p in report['pairs'] for fit in p['all_subset_fits']))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.exists():
        raise FileExistsError(args.output)
    write_json(args.output, dict(passed=all(checks.values()), checks=checks,
        parameter_order=list(PARAMETERS), maximum_baseline_error=replay.tolist(),
        maximum_filter_error=profile_error.tolist(), maximum_hybrid_error=hybrid_error.tolist(),
        maximum_closure_error=closure.tolist(), independent_filters=records, independent_hybrids=independent_hybrids))
    print(checks)
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
