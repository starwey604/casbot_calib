"""Paired, deterministic public-feedback perturbations and local A/B estimates.

No device, filesystem, or truth access. Perturbations are EXTRA degradation of
already delivered observations, never reconstruction of acquisition clocks.
"""
from copy import deepcopy
import numpy as np
from .hardware_probe import inertia_balance
from .hardware_roll_frequency import phasors, fit_components

THRESHOLD = .10
FLOOR = 1e-6


def default_cases():
    cases = [dict(id='baseline', family='baseline', severity=0., unit='', operations=[])]
    def add(family, values, unit, *, seeds=(0,), phases=(0.,), signs=(1,)):
        for value in values:
            for seed in seeds:
                for phase in phases:
                    for sign in signs:
                        op = dict(kind=family, value=float(sign*value), seed=seed, phase=phase)
                        cases.append(dict(id=f'{family}_{sign*value:g}_s{seed}_p{phase:g}',
                            family=family, severity=float(value), unit=unit, operations=[op]))
    add('effort_advance', [.5, 1, 2, 3, 5, 8, 10, 15], 'ms', signs=(-1, 1))
    add('shared_hold', [1, 2, 3, 5, 8, 10, 15, 20, 30, 50], 'ms', phases=(0., .5))
    add('effort_hold', [3, 5, 10, 20], 'ms', phases=(0., .5))
    add('velocity_lag', [1, 3, 5, 10], 'ms')
    add('position_lag', [1, 3, 5, 10], 'ms')
    add('shared_irregular_hold', [2, 5, 10, 20], 'ms', seeds=(11, 23, 37))
    add('effort_irregular_hold', [2, 5, 10, 20], 'ms', seeds=(11, 23, 37))
    add('arrival_jitter', [1, 3, 5, 10], 'ms', seeds=(11, 23, 37))
    # phase here denotes the fractional location within the measurement window.
    add('shared_stall', [5, 10, 20, 50, 100], 'ms', phases=(.25, .5, .75))
    add('effort_stall', [5, 10, 20, 50, 100], 'ms', phases=(.25, .5, .75))
    add('arrival_gap', [5, 20, 50, 100], 'ms', phases=(.25, .5, .75))
    add('effort_lowpass', [1, 2, 5, 8, 10, 15, 20], 'ms')
    add('effort_gain', [1, 2, 5, 10, 20], '%', signs=(-1, 1))
    add('effort_noise', [.02, .05, .1, .2], 'mNm', seeds=(11, 23, 37))
    for lag in (1, 3, 5):
        cases.append(dict(id=f'combined_{lag}', family='combined', severity=float(lag), unit='ms', operations=[
            dict(kind='shared_hold', value=5., phase=.5, seed=0),
            dict(kind='effort_advance', value=-float(lag), phase=0., seed=0),
            dict(kind='effort_lowpass', value=5., phase=0., seed=0),
            dict(kind='effort_gain', value=-5., phase=0., seed=0)]))
    return cases


def _hold_indices(t, period, phase, rng=None):
    if rng is None:
        ticks = t[0] + (np.floor((t-t[0])/period-phase)+phase)*period
    else:
        updates = [t[0]]
        while updates[-1] < t[-1]:
            updates.append(updates[-1] + period*rng.uniform(.5, 1.5))
        updates = np.asarray(updates)
        ticks = updates[np.searchsorted(updates, t, side='right')-1]
    return np.maximum(np.searchsorted(t, ticks, side='right')-1, 0)


def perturb(obs, elapsed, window, profile, case):
    """One memory copy feeds BOTH methods. Source arrays/timestamps stay intact.

    Shifts interpolate the received waveform (not true samples). Holds select
    past received rows; jitter adds bounded FIFO delivery delays; gaps remove
    rows. Low-pass is exact first-order ZOH integration on the receive grid.
    Noise is a 5ms-correlated block process, not independent per publication.
    """
    t = np.asarray(elapsed, dtype=float).copy()
    result = {k: np.asarray(obs[k]).copy() for k in ('receive_ns', 'q', 'dq', 'effort_raw')}
    if len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError('Strictly increasing public receive times required')
    zero = np.broadcast_to(profile['effort_zero_nm'], (14,))
    gain = np.broadcast_to(profile['effort_gain'], (14,))
    if not np.isfinite(gain).all() or np.any(gain == 0):
        raise ValueError('Finite nonzero input effort gain required')
    result['effort_raw'] = (result['effort_raw']-zero)*gain
    initial_rows = len(t)
    for op in case['operations']:
        kind, value = op['kind'], op['value']
        seconds = value*.001
        rng = np.random.default_rng(op['seed'])
        keys = ('q', 'dq', 'effort_raw') if kind.startswith('shared_') else ('effort_raw',)
        if kind.endswith('_hold'):
            indices = _hold_indices(t, seconds, op['phase'], rng if 'irregular' in kind else None)
            for key in keys:
                result[key] = result[key][indices]
        elif kind in ('effort_advance', 'velocity_lag', 'position_lag'):
            key = dict(effort_advance='effort_raw', velocity_lag='dq', position_lag='q')[kind]
            target = t + (seconds if kind == 'effort_advance' else -seconds)
            result[key] = np.column_stack([np.interp(target, t, result[key][:, j]) for j in range(14)])
        elif kind.endswith('_stall'):
            begin = window[0] + op['phase']*(window[1]-window[0])
            mask = (t >= begin) & (t < begin+seconds)
            anchor = max(np.searchsorted(t, begin, side='left')-1, 0)
            for key in keys:
                result[key][mask] = result[key][anchor]
        elif kind == 'arrival_gap':
            begin = window[0] + op['phase']*(window[1]-window[0])
            keep = ~((t >= begin) & (t < begin+seconds))
            result = {k: v[keep] for k, v in result.items()}
            t = t[keep]
        elif kind == 'arrival_jitter':
            # FIFO queue: later messages may bunch up but cannot overtake.
            delays = np.rint(rng.uniform(0., seconds, len(t))*1e9).astype(np.int64)
            ns = result['receive_ns'] + delays
            index = np.arange(len(ns), dtype=np.int64)
            ns = np.maximum.accumulate(ns-index)+index  # strict by at least1ns
            t += (ns-result['receive_ns'])*1e-9
            result['receive_ns'] = ns
        elif kind == 'effort_lowpass':
            raw = result['effort_raw']; filtered = raw.copy()
            decay = np.exp(-np.diff(t)/seconds)
            for k in range(1, len(t)):
                filtered[k] = decay[k-1]*filtered[k-1]+(1-decay[k-1])*raw[k-1]
            result['effort_raw'] = filtered
        elif kind == 'effort_gain':
            result['effort_raw'] *= 1+value/100
        elif kind == 'effort_noise':
            indices = np.floor((t-t[0])/.005).astype(int)
            noise = rng.normal(0., value*.001, (int(indices[-1])+1, 14))
            result['effort_raw'] += noise[indices]
        else:
            raise ValueError('Unknown sensitivity operation: '+kind)
    result['effort_raw'] = result['effort_raw']/gain+zero
    return result, t, dict(input_rows=initial_rows, retained_publication_rows=len(t),
        removed_publications=initial_rows-len(t), extra_degradation_only=True)


def estimate_a(problem):
    """Original zero-phase local time-domain balance; no phase correction."""
    try:
        p = problem; j = p['joint']; t = p['time_s']-np.mean(p['time_s'])
        x = np.column_stack([p['ddq'][:, j], p['dq'][:, j], np.ones(len(t)), t, t*t])
        scale = np.linalg.norm(x, axis=0)
        if np.any(scale == 0):
            raise ValueError('A temporal balance has zero columns')
        normalized = x/scale
        if np.linalg.matrix_rank(normalized, tol=1e-10) != 5:
            raise ValueError('A temporal balance is rank deficient')
        fit = inertia_balance(problem)
        available = all(np.isfinite(v) for k, v in fit.items() if k != 'method') and fit['inertia_kg_m2'] > 0
        return dict(available=bool(available), **fit, scaled_condition_number=float(np.linalg.cond(normalized)),
                    reason=None if available else 'Nonpositive or nonfinite local inertia')
    except (ValueError, np.linalg.LinAlgError) as exc:
        return dict(available=False, reason=str(exc))


def estimate_b(problem, prior, frequencies, scales):
    """Existing dual-tone free-J/b/effective-phase model, same processed data."""
    if len(set(frequencies)) < 2:
        return dict(available=False, reason='Single tone: two real observations cannot identify J/b/phase',
                    structural_rank=2, parameters=3)
    try:
        p = problem; j = p['joint']; t = p['time_s']
        known = (p['projected_gravity_nm']+p['rotor']+prior['coulomb'][j]*p['extra'][:, 14+j]
                 +prior['static_friction'][j]*p['static_feature'])
        z = phasors(t, np.column_stack([p['observed_torque_nm'], p['ddq'][:, j], p['dq'][:, j], known]),
                    frequencies, origin_s=float(t.mean()))
        y, a, v, c = z.T
        fit = fit_components(frequencies, a, v, c, y, *scales)
        w = 2*np.pi*np.asarray(frequencies)
        model = fit['inertia_kg_m2']*a+fit['viscous_nm_s_rad']*v+c
        d = np.exp(1j*w*fit['effective_advance_s'])[:, None]*np.column_stack([scales[0]*a, scales[1]*v, 1j*w*model*.005])
        matrix = np.vstack([d.real, d.imag]); s = np.linalg.svd(matrix, compute_uv=False)
        rank = int(np.count_nonzero(s > s[0]*1e-10))
        finite = all(np.isfinite(fit[k]) for k in ('inertia_kg_m2', 'viscous_nm_s_rad', 'effective_advance_s', 'residual_complex_norm_nm'))
        available = finite and fit['solver_converged'] and fit['inertia_kg_m2'] > 0 and rank == 3
        bound = 1/(4*max(frequencies))
        return dict(available=bool(available), **fit, observed_rank=rank,
            scaled_condition_number=float(s[0]/s[-1]) if rank == 3 else None,
            phase_branch_boundary=bool(abs(fit['effective_advance_s']) >= bound-1e-6),
            frequency_residual_over_observed=float(fit['residual_complex_norm_nm']/max(np.linalg.norm(y), 1e-15)),
            reason=None if available else 'Nonpositive/nonfinite inertia, deficient rank or solver failure')
    except (ValueError, np.linalg.LinAlgError) as exc:
        return dict(available=False, reason=str(exc))


def compare_to_baseline(fit, baseline):
    result = deepcopy(fit)
    result['change_from_own_baseline_relative'] = None
    result['within_10_percent_change'] = None
    if fit['available'] and baseline['available']:
        delta = abs(fit['inertia_kg_m2']-baseline['inertia_kg_m2'])/max(abs(baseline['inertia_kg_m2']), FLOOR)
        result.update(change_from_own_baseline_relative=float(delta), within_10_percent_change=bool(delta <= THRESHOLD))
    return result


def crossing_summary(rows):
    """Discrete observed crossing; never interpolate or assume monotonic safety."""
    families = sorted({r['case']['family'] for r in rows}-{ 'baseline', 'combined' })
    output = []
    for family in families:
        subset = [r for r in rows if r['case']['family'] == family]
        for route in ('A', 'B'):
            points = []
            for level in sorted({r['case']['severity'] for r in subset}):
                selected = [r for r in subset if r['case']['severity'] == level]
                fits = [r['routes'][route] for r in selected]
                comparable = [f for f in fits if f['change_from_own_baseline_relative'] is not None]
                values = [f['change_from_own_baseline_relative'] for f in comparable]
                points.append(dict(severity=level, unit=selected[0]['case']['unit'], comparisons=len(fits),
                    comparable=len(comparable), unavailable=len(fits)-len(comparable),
                    boundary_fits=sum(bool(f.get('phase_branch_boundary', False)) for f in fits),
                    maximum_change_relative=max(values) if values else None,
                    all_within=bool(len(comparable)==len(fits) and max(values, default=np.inf)<=THRESHOLD
                                    and not any(f.get('phase_branch_boundary', False) for f in fits))))
            exceed = next((p['severity'] for p in points if p['maximum_change_relative'] is not None and p['maximum_change_relative']>THRESHOLD), None)
            unresolved = next((p['severity'] for p in points if p['unavailable'] or p['boundary_fits']), None)
            prefix = []
            for p in points:
                if not p['all_within']: break
                prefix.append(p['severity'])
            output.append(dict(family=family, route=route, points=points,
                first_observed_exceedance=exceed, first_unavailable_or_branch_boundary=unresolved,
                last_consecutive_tested_level_within=prefix[-1] if prefix else None,
                continuous_tolerance_certified=False))
    return output
