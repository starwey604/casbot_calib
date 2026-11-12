"""Read-only cross-frequency diagnosis of two sealed dual-tone sessions.

Counterfactuals explain differences between measured complex responses, not
physical causality. None of these diagnostic fits is a calibration candidate.
"""
import argparse
import csv
from math import factorial
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from .data import write_json
from .hardware_roll_frequency import fit_components, phasors
from .sdk_feedback_sensitivity import read, install_readonly_guard
from .sdk_feedback_repeat import recipe

PARAMETERS = ('inertia_kg_m2', 'viscous_nm_s_rad', 'effective_advance_s')
FACTORS = ('low_magnitude', 'low_phase', 'high_magnitude', 'high_phase',
           'acceleration_response', 'known_response', 'velocity_weight')
LABELS = ('低频力矩／速度幅比', '低频力矩／速度相位', '高频力矩／速度幅比',
          '高频力矩／速度相位', '加速度／速度关系', '已知重力等项', '速度频点权重')


def complex_columns(row):
    z = np.asarray(row['components_real_imag'])
    return (z[:, :, 0] + 1j*z[:, :, 1]).T


def normalized(columns):
    y, a, v, c = columns
    if np.any(abs(v) < 1e-12):
        raise ValueError('Excited velocity at both frequencies is required')
    return dict(h=y/v, d=a/v, k=c/v, w=abs(v))


def hybrid(left, right, mask):
    h = np.array([(abs(right['h'][i]) if mask & (1 << (2*i)) else abs(left['h'][i])) *
                  np.exp(1j*(np.angle(right['h'][i]) if mask & (1 << (2*i+1)) else np.angle(left['h'][i])))
                  for i in range(2)])
    d, k, w = [right[key] if mask & (1 << bit) else left[key]
               for key, bit in [('d', 4), ('k', 5), ('w', 6)]]
    return np.array([h*w, d*w, w, k*w])


def fit(columns, frequencies):
    y, a, v, c = columns
    return fit_components(frequencies, a, v, c, y, .00027, .01)


def shapley(values):
    """Exact interaction allocation; rows enumerate all binary subsets."""
    values = np.asarray(values)
    n = (len(values)-1).bit_length()
    if len(values) != 2**n or n == 0:
        raise ValueError('Values must enumerate all subsets of at least one factor')
    out = np.zeros((n, *values.shape[1:]))
    for bit in range(n):
        for mask in range(len(values)):
            if mask & (1 << bit):
                continue
            count = mask.bit_count()
            weight = factorial(count)*factorial(n-count-1)/factorial(n)
            out[bit] += weight*(values[mask | (1 << bit)]-values[mask])
    return out


def attribute(left, right, frequencies):
    fits = [fit(hybrid(left, right, mask), frequencies) for mask in range(128)]
    if not all(row['solver_converged'] for row in fits):
        raise ValueError('Counterfactual fit did not converge')
    values = np.array([[r[k] for k in PARAMETERS] for r in fits])
    contributions = shapley(values)
    return dict(reference=dict(zip(PARAMETERS, values[0].tolist())),
                repeat=dict(zip(PARAMETERS, values[-1].tolist())),
                total_change=dict(zip(PARAMETERS, (values[-1]-values[0]).tolist())),
                contributions=[dict(factor=name, label=label,
                    **dict(zip(PARAMETERS, row.tolist())),
                    inertia_change_percentage_points=float(100*row[0]/values[0, 0]))
                    for name, label, row in zip(FACTORS, LABELS, contributions)],
                closure_error=(contributions.sum(axis=0)-(values[-1]-values[0])).tolist(),
                all_subset_fits=fits)


def filter_fit(columns, frequencies, tau):
    """Fix hypothetical first-order tau; retain original Nm residual weighting."""
    y, a, v, c = columns
    w = 2*np.pi*np.asarray(frequencies)
    scales = np.array([.00027, .01, .005])
    bound = 1/(4*max(frequencies))/.005
    def residual(x):
        j, b, delta = x*scales
        error = np.exp(1j*w*delta)/(1+1j*w*tau)*(j*a+b*v+c)-y
        return np.r_[error.real, error.imag]/max(np.linalg.norm(y), 1e-12)
    solutions = [least_squares(residual, [1, 1, delta],
                    bounds=([0, 0, -bound], [np.inf, np.inf, bound]),
                    ftol=1e-12, xtol=1e-12, gtol=1e-12) for delta in (-1, 0, 1)]
    best = min(solutions, key=lambda r: np.linalg.norm(r.fun))
    return dict(tau_s=tau, **dict(zip(PARAMETERS, (best.x*scales).tolist())),
                residual_complex_norm_nm=float(np.linalg.norm(best.fun)*max(np.linalg.norm(y), 1e-12)),
                solver_converged=bool(best.success))


def local_information(frequencies, tau, *, free_gain=False):
    """Dimension/rank illustration, unit velocity, not an excitation optimizer."""
    w = 2*np.pi*np.asarray(frequencies)
    j, b, stiffness = .00027, .01, .016
    transfer = 1/(1+1j*w*tau)
    z = transfer*(1j*w*j+b+stiffness/(1j*w))
    columns = [transfer*1j*w*j, transfer*b, 1j*w*z*.005,
               -1j*w/(1+1j*w*tau)*z*.005]
    if free_gain:
        columns.append(z)
    matrix = np.vstack([np.array(columns).T.real, np.array(columns).T.imag])
    s = np.linalg.svd(matrix, compute_uv=False)
    rank = int(np.linalg.matrix_rank(matrix))
    return dict(frequencies_hz=list(frequencies), tau_s=tau, free_gain=free_gain,
                rank=rank, parameters=len(columns), real_observations=len(w)*2,
                null_dimensions=len(columns)-rank, singular_values=s.tolist(),
                condition_number=float(s[0]/s[-1]) if rank == len(columns) else None,
                phase_filter_column_sum_norm=float(np.linalg.norm(matrix[:, 2]+matrix[:, 3])))


def window_diagnostics(root, row, problem, columns, frequencies):
    from .hardware_field import resolve_field_capture, read_feedback, check_seal
    from .model_runtime import model_for
    job = row['job']['id']
    plan, capture = resolve_field_capture(root/job/'plan', root/job/'capture')
    for folder in (plan, capture):
        check_seal(folder)
    meta = read(plan/'plan.json'); profile = meta['config']
    engine, _ = model_for(profile)
    model = engine.model; model.gravity_world = np.asarray(profile['gravity_world']); engine = type(engine)(model)
    obs = read_feedback(capture/'execution/feedback.csv', require_effort=True)
    with (capture/'execution/commands.csv').open() as stream:
        command = next(csv.DictReader(stream))
    origin = int(command['planned_ns'])-int(command['planned_index'])*meta['command_period_ns']
    elapsed = (obs['receive_ns']-np.int64(origin))*1e-9
    protocol = read(root/'protocol/experiment.json'); j = protocol['joint']
    prior = protocol['fixture_friction_prior']
    # Raw acceleration is not finite-differenced: steady-tone A=iwV is an
    # explicit alternative readout. Existing captures have zero rotor inertia.
    if np.any(problem['rotor'] != 0):
        raise ValueError('Raw-tone diagnostic currently supports zero saved rotor contribution only')
    begin, end = row['retained_window_s']; mid = (begin+end)/2
    windows = [('matched_raw', (begin, end)), ('whole_measurement', row['measurement_window_s']),
               ('first_half', (begin, mid)), ('second_half', (mid, end))]
    results = []
    for name, (lo, hi) in windows:
        use = (elapsed >= lo) & (elapsed < hi); t = elapsed[use]
        v = obs['dq'][use, j]
        # The same public Nm conversion and frozen Coulomb/static terms.
        y = ((obs['effort_raw'][use]-profile['effort_zero_nm'])*profile['effort_gain'])[:, j]
        decay = np.exp(-(v/.005)**2)
        c = engine.gravity_batch(obs['q'][use])[:, j] + np.sign(v)*(
            prior['coulomb'][j]*(1-decay)+prior['static_friction'][j]*decay)
        z = phasors(t, np.column_stack([y, v, c]), frequencies, origin_s=t.mean())
        Y, V, C = z.T; A = 1j*2*np.pi*np.asarray(frequencies)*V
        results.append(dict(window=name, interval_s=[lo, hi], callback_rows=len(t),
                            fit=fit([Y, A, V, C], frequencies)))
    # Public nominal model tests omitted coupling, Coriolis and changing Mjj;
    # this is a counterfactual, not access to the synthetic plant parameters.
    nominal_j = float(np.mean([engine.mass_matrix(q)[j, j] for q in problem['q']]))
    omitted = problem['regression']@engine.rigid_parameters()-problem['projected_gravity_nm']-nominal_j*problem['ddq'][:, j]
    z = phasors(problem['time_s'], omitted, frequencies, origin_s=problem['time_s'].mean())
    y, a, v, c = columns
    return dict(raw_windows=results, nominal_omitted_dynamic_tone_magnitude_nm=abs(z).tolist(),
                nominal_omitted_dynamic_fit=fit([y, a, v, c+z], frequencies),
                nominal_diagonal_inertia_kg_m2=nominal_j)


def diagnose(reference, repeat, output):
    from .hardware_field import check_seal
    roots = [Path(reference).resolve(), Path(repeat).resolve()]
    out = Path(output).resolve()
    if roots[0] == roots[1] or any(out == r or r in out.parents for r in roots):
        raise ValueError('Distinct sessions and separate output required')
    if recipe(roots[0]) != recipe(roots[1]):
        raise ValueError('A same-recipe comparison is required')
    rows = []; spectra = []
    for session, root in zip(('reference', 'repeat'), roots):
        check_seal(root/'protocol'); check_seal(root/'analysis')
        protocol = read(root/'protocol/experiment.json')
        frequencies = [tone[0] for tone in protocol['tones']]
        if len(frequencies) != 2 or frequencies != sorted(set(frequencies)):
            raise ValueError('Two ordered distinct tones required')
        saved = sorted(read(root/'analysis/report.json')['records'], key=lambda r: r['job']['direction'])
        if [r['job']['direction'] for r in saved] != [-1, 1]:
            raise ValueError('One negative and one positive record required')
        for row in saved:
            columns = complex_columns(row['frequency_analysis']['full'])
            spectrum = normalized(columns); spectra.append(spectrum)
            baseline = row['frequency_analysis']['full']['free_b_complex']
            with np.load(root/'analysis'/(row['job']['id']+'_problem.npz'), allow_pickle=False) as data:
                problem = {key: data[key] for key in data.files}
            checks = window_diagnostics(root, row, problem, columns, frequencies)
            rows.append(dict(session=session, source=str(root), job=row['job'], baseline=baseline,
                baseline_replay=fit(columns, frequencies),
                normalized_torque_magnitude=abs(spectrum['h']).tolist(),
                normalized_torque_phase_deg=np.angle(spectrum['h'], deg=True).tolist(),
                acceleration_relative_error=abs(spectrum['d']/(1j*2*np.pi*np.array(frequencies))-1).tolist(),
                leave_block_out_fits=[r['free_b_complex'] for r in row['frequency_analysis']['leave_block_out']],
                filter_profile=[filter_fit(columns, frequencies, tau) for tau in (0., .002, .005, .01, .02)],
                **checks))
    pairs = []
    for i, j in ((0, 2), (1, 3), (0, 1), (2, 3), (0, 3)):
        pair = attribute(spectra[i], spectra[j], frequencies)
        pair.update(reference_index=i, repeat_index=j,
            tone_response_phase_change_as_advance_s=(np.angle(spectra[j]['h']/spectra[i]['h'])/(2*np.pi*np.array(frequencies))).tolist())
        pairs.append(pair)
    report = dict(schema_version='casbot_sdk_roll_frequency_mismatch_v1', completed=True,
        frequencies_hz=frequencies, sources=[str(r) for r in roots], records=rows, pairs=pairs,
        information=[local_information(f, tau, free_gain=gain)
            for f in (frequencies, [frequencies[0], 5., frequencies[1]])
            for tau in (0., .005) for gain in (False, True)],
        information_scope='5 Hz is a rank illustration only, not a selected collection recipe',
        attribution_scope='Exact seven-factor Shapley of observed complex responses; not physical causal identification',
        raw_window_scope='Raw joint tone+quadratic readout, A=iwV; shorter windows are sensitivity checks, not independent repeats',
        filter_scope='Fixed hypothetical tau, original Nm residual weights; no filter selected or correction applied',
        threshold=.1, formal_test_consumed=False, physical_delay_identified=False,
        candidate_updated=False, collection_performed=False, full_p0_r2_performed=False)
    out.mkdir(parents=True, exist_ok=False)
    write_json(out/'report.json', report)
    (out/'report.md').write_text(markdown_report(report), encoding='utf-8')
    return report


def markdown_report(report):
    lines = ['# 四条双频记录：跨频失配离线诊断', '',
        '固定既有四条记录和原 10% 数值；所有替换、窗口和滤波拟合仅作诊断，不更新候选或观测时间。', '',
        '## 原 B 结果与公开原始读出', '',
        '| 记录 | B J（10⁻⁶ kg·m²） | B 相位 ms | 原始同窗 J（10⁻⁶ kg·m²） | 原始同窗相位 ms | 前／后半窗相位 ms |',
        '|---|---:|---:|---:|---:|---:|']
    for i, r in enumerate(report['records']):
        b = r['baseline']; raw = r['raw_windows']; a = raw[0]['fit']
        lines.append(f"| {i} {r['session']} {r['job']['direction']:+d} | {b[PARAMETERS[0]]*1e6:.3f} | {b[PARAMETERS[2]]*1e3:.3f} | {a[PARAMETERS[0]]*1e6:.3f} | {a[PARAMETERS[2]]*1e3:.3f} | {raw[2]['fit'][PARAMETERS[2]]*1e3:.3f}／{raw[3]['fit'][PARAMETERS[2]]*1e3:.3f} |")
    lines += ['', '## 幅值／相位替换归因', '',
        '用 Y/V、A/V、C/V 和 |V| 保持原 Nm 残差权重。穷举七因素的 128 种组合，按 Shapley 平分交互项。百分点以每对起始记录 J 为分母；这解释数值差异，不证明硬件因果。']
    for pair in report['pairs']:
        lines += ['', f"### 记录 {pair['reference_index']} → {pair['repeat_index']}", '',
            '| 因素 | J 变化贡献（百分点） | 等效相位贡献 ms |', '|---|---:|---:|']
        for c in pair['contributions']:
            lines.append(f"| {c['label']} | {c['inertia_change_percentage_points']:+.3f} | {c['effective_advance_s']*1e3:+.4f} |")
        lines += ['', '两个频点的实测 Y/V 相位变化折算时移：'+
                  '／'.join(f'{x*1e3:+.4f}ms' for x in pair['tone_response_phase_change_as_advance_s'])+'。']
    lines += ['', '## 固定假设低通时间常数的剖面', '',
        '| 记录 | τ ms | J（10⁻⁶ kg·m²） | 等效相位 ms | 复残差范数 mNm |', '|---|---:|---:|---:|---:|']
    for i, r in enumerate(report['records']):
        for p in r['filter_profile']:
            lines.append(f"| {i} | {p['tau_s']*1e3:g} | {p[PARAMETERS[0]]*1e6:.3f} | {p[PARAMETERS[2]]*1e3:.3f} | {p['residual_complex_norm_nm']*1e3:.5f} |")
    lines += ['', '## 判断', '',
        '拟合相位漂移与 Y/V 逐频相位变化须分开比较；幅值变化也能推动相位估计。同带宽处理和公开名义耦合项对照见 JSON。',
        '双频只有四个实数约束，J／b／相位占三个；再自由拟合一阶低通 τ 后没有剩余跨频检验约束。τ=0 时相位与 τ 的一阶导数严格相反，增加频点也不能消除这一局部退化。',
        '建议先保留逐窗稳定性诊断，再离线设计一个额外验证频点及幅值重分配；第三频点用于检验模型预测，不用于把双频零残差当作通过。当前数据不足以选择实际滤波器或物理延迟。',
        '没有补采、消费正式 test、重跑完整 R2 或写回候选。']
    return '\n'.join(lines)+'\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--repeat', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    install_readonly_guard([args.reference, args.repeat])
    report = diagnose(args.reference, args.repeat, args.output)
    print(f"完成 {len(report['records'])} 条记录、{len(report['pairs'])} 组归因：{args.output}")
    return 0
