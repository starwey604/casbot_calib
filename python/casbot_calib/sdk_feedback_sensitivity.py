"""Read-only SDK feedback sensitivity assessment on sealed public probe records.

This module never starts a collector, writes a candidate, or changes source
records. The CLI additionally forbids subprocess/network access and truth reads.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import sys
import numpy as np
from .data import write_json
from .hardware_probe import probe_problem, field_probe_config
from .sdk_feedback_sensitivity_math import (
    THRESHOLD, FLOOR, default_cases, perturb, estimate_a, estimate_b,
    compare_to_baseline, crossing_summary)
from .run_marker import run_marker

SCHEMAS = ('casbot_sdk_wrist_probe_pair_v1', 'casbot_sdk_wrist_roll_screen_v1',
           'casbot_sdk_wrist_roll_dual_v1', 'casbot_sdk_wrist_roll_dual_v2')
LABELS = dict(effort_advance='力矩相对时移（±）', shared_hold='额外共享固定缓存',
    effort_hold='额外力矩独立缓存', velocity_lag='速度字段额外滞后', position_lag='位置字段额外滞后',
    shared_irregular_hold='共享随机刷新', effort_irregular_hold='力矩随机刷新',
    arrival_jitter='FIFO 到达抖动', shared_stall='共同停更', effort_stall='力矩停更',
    arrival_gap='接收空窗', effort_lowpass='力矩一阶低通', effort_gain='力矩增益偏差（±）',
    effort_noise='5ms 相关力矩噪声', combined='组合退化')


def read(path):
    return json.loads(Path(path).read_text())


def install_readonly_guard(sources):
    """CLI lifetime guard. Existing sealed-source integrity checks remain intact."""
    roots = [Path(p).resolve() for p in sources]
    def protected(path):
        if not isinstance(path, (str, bytes, os.PathLike)): return False
        path = Path(os.fsdecode(path)).resolve()
        return any(path == root or root in path.parents for root in roots)
    def audit(event, args):
        if event == 'open' and isinstance(args[0], (str, bytes, os.PathLike)):
            path = Path(os.fsdecode(args[0])).resolve()
            if 'evaluation' in path.parts or path.name.startswith('evaluation_truth'):
                raise RuntimeError('Sensitivity assessment cannot read private evaluation truth')
            mode, flags = args[1:3]
            writing = (isinstance(mode, str) and any(c in mode for c in 'wax+')) or (isinstance(flags, int) and bool(flags & (os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_TRUNC)))
            if writing and protected(path):
                raise RuntimeError('Sensitivity inputs are read-only')
        if event in ('os.remove', 'os.rmdir', 'os.mkdir', 'os.chmod', 'os.truncate') and protected(args[0]):
            raise RuntimeError('Sensitivity inputs are read-only')
        if event in ('os.rename', 'os.link', 'os.symlink') and any(protected(p) for p in args[:2]):
            raise RuntimeError('Sensitivity inputs are read-only')
        if event in ('subprocess.Popen', 'os.system', 'os.posix_spawn', 'socket.connect', 'socket.bind'):
            raise RuntimeError('Sensitivity assessment cannot launch processes or connect devices')
    sys.addaudithook(audit)


def load_records(source):
    from . import hardware_field as field
    from .model_runtime import model_for
    source = Path(source).resolve()
    field.check_seal(source/'protocol')
    protocol = read(source/'protocol/experiment.json')
    if protocol.get('schema_version') not in SCHEMAS:
        raise ValueError('Expected sealed SDK single/dual-tone probe protocol; arbitrary GUI recordings need a declared probe recipe')
    # New assessment semantics only. Original schema1 quality policy is untouched.
    config = field_probe_config(protocol['probe_config'])
    tones = [x[0] for x in protocol['tones']] if 'tones' in protocol else [protocol['probe_config']['probe']['frequency_hz']]
    policy = protocol.get('observation_policy', {})
    scales = (policy.get('inertia_scale_kg_m2', .0003), policy.get('viscous_scale_nm_s_rad', .01))
    for job in protocol['jobs']:
        if Path(job['id']).name != job['id']:
            raise ValueError('Probe job must name a direct child directory')
        trial = source/job['id']
        plan, capture = field.resolve_field_capture(trial/'plan', trial/'capture')
        metadata, _, _, obs, elapsed, collision = field.field_observations(plan, capture)
        motion = read(plan/'motion.json'); profile = metadata['config']
        if motion['conditions']['joint'] != protocol['joint'] or motion['conditions']['direction'] != job['direction']:
            raise ValueError('Delivered probe axis/direction differs from protocol')
        delivered = [x['frequency_hz'] for x in motion['conditions']['tones']] if 'tones' in motion['conditions'] else [motion['conditions']['frequency_hz']]
        if delivered != tones:
            raise ValueError('Delivered frequencies differ from protocol')
        engine, _ = model_for(profile)
        model = engine.model; model.gravity_world = np.asarray(profile['gravity_world']); engine = type(engine)(model)
        windows = [(motion['knots_s'][i], motion['knots_s'][i+1]) for i, label in enumerate(motion['labels']) if label == 'probe_measure']
        window = (windows[0][0], windows[-1][1])
        active = (elapsed >= window[0]) & (elapsed < window[1])
        public = np.column_stack([obs[k][active] for k in ('q', 'dq', 'effort_raw')])
        yield dict(obs=obs, elapsed=elapsed, motion=motion, engine=engine, config=config,
            profile=profile, prior=protocol['fixture_friction_prior'], frequencies=tones, scales=scales, window=window,
            metadata=dict(source=str(source), job=job, joint=protocol['joint'], joint_name=metadata['joint_names'][protocol['joint']],
                source_schema=protocol['schema_version'], plan=str(plan), capture=str(capture),
                scope=profile['connection']['scope'], model_context=profile.get('model_context'),
                prior_status=protocol['prior_status'], frequencies_hz=tones, window_s=window,
                source_cache_standard_ns=protocol['probe_config']['cache_period_ns'],
                source_received_collision_passed=collision['passed'],
                callback_publications=int(active.sum()), maximum_callback_gap_ms=float(np.max(np.diff(elapsed[active]))*1000),
                identical_adjacent_fraction=float(np.mean(np.all(public[1:]==public[:-1], axis=1))),
                independent_acquisition_count=None, sample_time_known=False))


def evaluate_record(record, cases):
    rows = []; baseline = None
    for case in cases:
        obs, elapsed, audit = perturb(record['obs'], record['elapsed'], record['window'], record['profile'], case)
        try:
            problem = probe_problem(obs, elapsed, record['motion'], record['engine'], record['config'], record['profile'])
            estimates = dict(A=estimate_a(problem), B=estimate_b(problem, record['prior'], record['frequencies'], record['scales']))
            prep = problem['preprocessing']
            diagnostic = {key: prep[key] for key in ('maximum_callback_gap_s', 'callback_publications', 'temporal_condition',
                          'minimum_signed_velocity_rad_s', 'maximum_held_velocity_rad_s', 'velocity_consistency_rmse_rad_s')}
        except (ValueError, np.linalg.LinAlgError) as exc:
            estimates = {key: dict(available=False, reason='Common preprocessing: '+str(exc)) for key in ('A', 'B')}
            diagnostic = dict(unavailable=str(exc))
        if baseline is None:
            if case['id'] != 'baseline': raise ValueError('First case must be the unmodified baseline')
            baseline = estimates
        rows.append(dict(case=case, perturbation=audit, preprocessing=diagnostic,
            routes={k: compare_to_baseline(estimates[k], baseline[k]) for k in ('A', 'B')}))
    return dict(metadata=record['metadata'], baseline=baseline, results=rows,
        thresholds=crossing_summary(rows), source_modified=False, absolute_inertia_error_known=False)


def direction_comparisons(records):
    output = []
    sources = sorted({r['metadata']['source'] for r in records})
    for source in sources:
        pair = sorted([r for r in records if r['metadata']['source']==source], key=lambda r:r['metadata']['job']['direction'])
        if len(pair)!=2 or [r['metadata']['job']['direction'] for r in pair] != [-1, 1]:
            continue
        for neg, pos in zip(pair[0]['results'], pair[1]['results']):
            assert neg['case']==pos['case']
            routes = {}
            for route in ('A', 'B'):
                a, b = neg['routes'][route], pos['routes'][route]
                available = a['available'] and b['available']
                delta = abs(b['inertia_kg_m2']-a['inertia_kg_m2'])/max(abs(a['inertia_kg_m2']), FLOOR) if available else None
                routes[route] = dict(available=available, direction_difference_relative=delta,
                    within_10_percent_direction_difference=bool(delta<=THRESHOLD) if delta is not None else None)
            output.append(dict(source=source, case_id=neg['case']['id'], routes=routes))
    return output


def analytic_controls():
    """Known mathematical responses, separate from recorded-data sensitivity."""
    from .hardware_roll_frequency import fit_response
    f = np.array([2.5, 10.]); w = 2*np.pi*f; v = np.array([.118, .043])
    j, b, stiffness = .00027, .01, .016
    a = 1j*w*v; c = stiffness/(1j*w)*v
    rows = []
    conditions = [('clean', 0., 1., 0.), ('advance_5ms', .005, 1., 0.),
                  ('lag_5ms', -.005, 1., 0.), ('lowpass_5ms', 0., 1., .005),
                  ('lowpass_10ms', 0., 1., .010), ('gain_minus10pct', 0., .9, 0.),
                  ('gain_plus10pct', 0., 1.1, 0.)]
    for name, delta, gain, tau in conditions:
        # Construct known response explicitly; no private Mock model involved.
        y = gain*np.exp(1j*w*delta)/(1+1j*w*tau)*(j*a+b*v+c)
        x = np.vstack([np.column_stack([a, v]).real, np.column_stack([a, v]).imag])
        coef = np.linalg.lstsq(x, np.r_[(y-c).real, (y-c).imag], rcond=None)[0]
        complex_fit = fit_response(f, v, y, j, b, stiffness)
        rows.append(dict(case=name, A_frequency_zero_phase_inertia=float(coef[0]),
            B_inertia=complex_fit['inertia_kg_m2'], A_known_error_relative=float(abs(coef[0]/j-1)),
            B_known_error_relative=float(abs(complex_fit['inertia_kg_m2']/j-1)),
            B_residual_complex_norm_nm=complex_fit['residual_complex_norm_nm']))
    return dict(scope='known two-tone linear analytical control; A is its frequency-domain zero-phase analogue',
        true_inertia_kg_m2=j, viscous_nm_s_rad=b, stiffness_nm_rad=stiffness,
        truth_from_physical_device=False, recorded_data_used_for_parameter_choice=False, cases=rows)


def reference_comparisons(report):
    """Keep sensitivity and signed-direction consistency visible side by side."""
    output = []
    for group in report['grouped_thresholds']:
        source = group['source']
        records = [r for r in report['records'] if r['metadata']['source']==source]
        for family, severity in [('shared_hold', 5), ('effort_advance', 5), ('effort_lowpass', 5),
                                 ('effort_gain', 10), ('combined', 5)]:
            selected = [row for r in records for row in r['results']
                        if row['case']['family']==family and row['case']['severity']==severity]
            if not selected: continue
            ids = {row['case']['id'] for row in selected}; routes = {}
            pairs = [r for r in report['direction_comparisons'] if r['source']==source and r['case_id'] in ids]
            for route in ('A', 'B'):
                changes = [r['routes'][route]['change_from_own_baseline_relative'] for r in selected]
                directions = [r['routes'][route]['direction_difference_relative'] for r in pairs]
                routes[route] = dict(
                    maximum_change_relative=max(changes) if all(x is not None for x in changes) else None,
                    maximum_direction_difference_relative=max(directions) if directions and all(x is not None for x in directions) else None)
            output.append(dict(source=source, family=family, severity=severity, unit=selected[0]['case']['unit'], routes=routes))
    return output


def assess(sources, output, *, reference_assessment=None):
    sources = [Path(p).resolve() for p in sources]
    out = Path(output).resolve()
    if len(set(sources)) != len(sources): raise ValueError('Duplicate source directory')
    protected = list(sources)
    if reference_assessment is not None:
        protected += [Path(reference_assessment).resolve()]
        protected += [Path(p).resolve() for p in read(Path(reference_assessment)/'report.json')['protocol']['sources']]
    if any(out == p or p in out.parents for p in protected):
        raise ValueError('Output must be outside every input session')
    # Register all cases before opening observations. No data-dependent tuning.
    cases = default_cases()
    out.mkdir(parents=True, exist_ok=False)
    protocol = dict(schema_version='casbot_sdk_feedback_sensitivity_protocol_v1', sources=[str(s) for s in sources],
        diagnostic_threshold=THRESHOLD, inertia_denominator_floor_kg_m2=FLOOR, cases=cases,
        route_A='existing inertia_balance, zero-phase time-domain acceleration/velocity/quadratic nuisance',
        route_B='existing fit_components, declared tones, free positive J/b and one effective phase',
        common_processing='same perturbed public arrays, callback rows, full-window degree2/Fourier projection; original band and trim',
        perturbation_scope='extra degradation of delivered feedback; unknown original sampling/cache/filter are retained',
        threshold_scope='10% change from own unperturbed estimate; direction difference separate; not absolute error or P0 R2',
        stochastic_scope='three fixed seeds and discrete cache/stall phases; finite stress set, not probabilistic confidence',
        recommendations_are_collection_gates=False, automatic_correction=False, candidate_written=False,
        formal_test_consumed=False, **run_marker())
    if reference_assessment is not None:
        protocol['reference_assessment'] = str(Path(reference_assessment).resolve())
    write_json(out/'protocol.json', protocol)
    records = []
    for source in sources:
        for record in load_records(source):
            print('评估 '+source.name+'/'+record['metadata']['job']['id'], file=sys.stderr, flush=True)
            records.append(evaluate_record(record, cases))
            write_json(out/'progress.json', dict(completed_records=len(records), last_record=record['metadata']))
    if not records: raise ValueError('No supported probe records')
    groups = []
    for source in sources:
        group = [r for r in records if r['metadata']['source']==str(source)]
        groups.append(dict(source=str(source), joint=group[0]['metadata']['joint'],
            frequencies_hz=group[0]['metadata']['frequencies_hz'],
            thresholds=crossing_summary([row for record in group for row in record['results']])))
    report = dict(schema_version='casbot_sdk_feedback_sensitivity_report_v1', completed=True,
        protocol=protocol, records=records, grouped_thresholds=groups, direction_comparisons=direction_comparisons(records),
        analytic_controls=analytic_controls(),
        interpretation='Use per-family discrete sensitivity, direction and independent repeatability together; stable bias is not detectable accuracy',
        physical_sampling_delay_estimated=False, formal_hardware_calibration_accepted=False,
        full_p0_r2_performed=False, independent_session_repeatability_tested=False,
        empirical_quality_gate_applied=False, formal_test_consumed=False, source_modified=False)
    report['reference_conditions'] = reference_comparisons(report)
    if reference_assessment is not None:
        from .sdk_feedback_repeat import compare_assessments
        comparison = compare_assessments(reference_assessment, report, out/'cross_session')
        report['independent_session_repeatability_tested'] = comparison['independent_mock_sessions_verified']
        report['cross_session_report'] = 'cross_session/report.md'
    write_json(out/'report.json', report)
    write_csv(report, out/'case_results.csv')
    (out/'report.md').write_text(markdown_report(report), encoding='utf-8')
    return report


def write_csv(report, path):
    fields = ['source', 'job', 'joint', 'case', 'family', 'severity', 'unit', 'route', 'available',
              'inertia_kg_m2', 'viscous_nm_s_rad', 'effective_advance_s', 'change_from_own_baseline_relative',
              'within_10_percent_change', 'phase_branch_boundary', 'reason']
    with path.open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for record in report['records']:
            for row in record['results']:
                for route, fit in row['routes'].items():
                    data = {k: fit.get(k) for k in fields if k in fit}
                    data.update(source=record['metadata']['source'], job=record['metadata']['job']['id'],
                        joint=record['metadata']['joint'], case=row['case']['id'], family=row['case']['family'],
                        severity=row['case']['severity'], unit=row['case']['unit'], route=route)
                    writer.writerow(data)


def markdown_report(report):
    p = report['protocol']; lines = ['# SDK 反馈—标定敏感性评估', '',
        f"已处理 {len(report['records'])} 条现有记录，每条 {len(p['cases'])} 个预先登记的条件。",
        '**这里的 10% 表示相对各路线未扰动结果的惯量变化。** 正反差另列；现有记录的绝对误差未知，未进行完整 R2。', '',
        '## 怎么用结果', '',
        '- A：同带宽、零相位的局部惯量拟合。B：相同观测上的多频 J／粘性／等效相位联合拟合。',
        '- 共同缓存、字段错位、滤波和尺度分开看。所有扰动都是在已有反馈上额外施加，不是实测缓存年龄。',
        '- 优先使用在所关心退化范围内稳定的简单路线 A；A 对相位敏感而 B 稳定时，优先做 B 的独立会话复测。',
        '- 两者都敏感或不可辨识时，针对该轴补充观测／尺度证据；不能凭更小残差挑一个结果。',
        '- 接收节奏不能约束未知字段延迟／滤波。没有外部界限时，本报告提供条件化选择，不自动宣布实机可用。',
        '- 这些结论不停止采集、不改 H1–H4 流程、不写回模型或时间戳。', '',
        '## 未扰动记录', '', '| 来源／记录 | A 惯量 kg·m² | B 惯量 kg·m² | B 等效相位 ms |', '|---|---:|---:|---:|']
    for record in report['records']:
        a, b = record['baseline']['A'], record['baseline']['B']
        def number(fit, key, factor=1.):
            return f"{fit[key]*factor:.7g}" if fit['available'] else '不可辨识／不可用'
        lines.append(f"| {Path(record['metadata']['source']).name} / {record['metadata']['job']['id']} | {number(a,'inertia_kg_m2')} | {number(b,'inertia_kg_m2')} | {number(b,'effective_advance_s',1000)} |")
    lines += ['', '### 未扰动正反方向差', '', '| 来源 | A | B |', '|---|---:|---:|']
    for row in report['direction_comparisons']:
        if row['case_id'] != 'baseline': continue
        cells = []
        for route in ('A', 'B'):
            x = row['routes'][route]['direction_difference_relative']
            cells.append(f'{100*x:.3f}%' if x is not None else '不可辨识／不可用')
        lines.append(f"| {Path(row['source']).name} | {' | '.join(cells)} |")
    lines += ['', '## 重点条件：自身稳定不等于正反一致', '',
        '同一双频记录同时比较两条路线。每列取所列条件的正反记录／相位／正负扰动最差值；两种百分数分母不同。', '',
        '| 额外退化 | A 自身变化 | B 自身变化 | A 正反差 | B 正反差 |', '|---|---:|---:|---:|---:|']
    multi_sources = {g['source'] for g in report['grouped_thresholds'] if len(g['frequencies_hz']) >= 2}
    for row in report.get('reference_conditions', []):
        if row['source'] not in multi_sources: continue
        cells = []
        for key in ('maximum_change_relative', 'maximum_direction_difference_relative'):
            for route in ('A', 'B'):
                value = row['routes'][route][key]
                cells.append(f'{100*value:.3f}%' if value is not None else '不可用')
        lines.append(f"| {Path(row['source']).name}：{LABELS[row['family']]} {row['severity']:g}{row['unit']} | {' | '.join(cells)} |")
    lines += ['', '任一方向一致性已超 10% 时，不能仅凭“自身变化未超 10%”建议直接使用；仍需独立复测定位。']
    lines += ['', '## 首次观测到超过 10% 的扰动档位', '',
        '合并同组正反向、正负扰动、种子和位置，取最差值。只说明已测离散点；“未越界”不保证中间或更大值安全。',
        '随机刷新单位为平均刷新周期；FIFO 抖动为注入延迟上限；噪声为 5ms 相关块的标准差。', '']
    for group in report['grouped_thresholds']:
        lines += [f"### {Path(group['source']).name}", '', '| 额外退化 | A 首次越界 | B 首次越界 |', '|---|---:|---:|']
        by_family = {}
        for row in group['thresholds']: by_family.setdefault(row['family'], {})[row['route']] = row
        for family, routes in by_family.items():
            cells = []
            for route in ('A', 'B'):
                r = routes[route]; unit = r['points'][0]['unit']; cross = r['first_observed_exceedance']
                if not any(x['comparable'] for x in r['points']): text = '不可辨识／不可用'
                elif cross is not None: text = f'{cross:g} {unit}'
                else: text = f"至 {r['points'][-1]['severity']:g} {unit} 未观测越界"
                if r['first_unavailable_or_branch_boundary'] is not None and any(x['comparable'] for x in r['points']):
                    text += f"；{r['first_unavailable_or_branch_boundary']:g} {unit} 起出现不可用／相位边界"
                cells.append(text)
            lines.append(f"| {LABELS[family]} | {' | '.join(cells)} |")
        lines.append('')
    lines += ['', '## 组合退化与稳定偏差', '',
        '组合条件固定为：额外共享缓存 5ms、力矩低通 5ms、增益 −5%，再叠加 1／3／5ms 力矩滞后。',
        '以下取同组正反向最差相对变化，不能由单项结果相加替代。', '',
        '| 来源 | 力矩滞后 ms | A 最差变化 | B 最差变化 |', '|---|---:|---:|---:|']
    for group in report['grouped_thresholds']:
        records = [r for r in report['records'] if r['metadata']['source']==group['source']]
        for level in (1, 3, 5):
            cells = []
            for route in ('A', 'B'):
                fits = [row['routes'][route] for r in records for row in r['results'] if row['case']['id']==f'combined_{level}']
                values = [f['change_from_own_baseline_relative'] for f in fits if f['change_from_own_baseline_relative'] is not None]
                cells.append(f'{100*max(values):.3f}%' if len(values)==len(fits) and values else '不可辨识／不可用')
            lines.append(f"| {Path(group['source']).name} | {level} | {' | '.join(cells)} |")
    lines += ['', '### 已知数学模型对照：绝对误差另算', '',
        '下表是真值已知的双频线性模型；A 使用其频域零相位对应式，不是现场真实误差。', '',
        '| 数学条件 | A 已知惯量误差 | B 已知惯量误差 | B 复数残差 mNm |', '|---|---:|---:|---:|']
    for row in report['analytic_controls']['cases']:
        lines.append(f"| {row['case']} | {100*row['A_known_error_relative']:.3f}% | {100*row['B_known_error_relative']:.3f}% | {1000*row['B_residual_complex_norm_nm']:.5f} |")
    lines += ['', '## 数据范围与后续动作', '',
        '- 单频记录不给 B 伪造第二个激励频点；单频无法同时辨识 J、粘性和相位。',
        '- 当前比较是局部单轴惯量，不是原 128 点删构型／删批次的完整刚体稳定性。',
        '- 窗口内重复和三种随机种子不是独立会话复测；不把它们计作设备独立采样。',
        '- 保留原频带、时间窗、先验和 10% 数值；本次分析使用质量只报告的处理口径，未改旧协议。',
        '- `case_results.csv` 给出每条记录／扰动／路线的数值；`report.json` 保留原因、离散曲线、方向差和输入路径；`protocol.json` 保存完整扰动配方。',
        '- 现场通用 GUI 记录尚需声明探针窗口、激励频点及重力／摩擦先验后才能适配；当前入口读取已有封存单／双频探针会话。', '']
    if report.get('cross_session_report'):
        lines += ['## 独立会话复测', '',
                  '[跨会话结果](cross_session/report.md)给出旧／新同方向惯量差、各会话正反差、四条全配对差异和粘性／等效相位变化。旧评估结果直接复用，未重复拟合。', '']
    return '\n'.join(lines)


def parser():
    p = argparse.ArgumentParser(description='只读比较 SDK 反馈退化下 A/B 惯量敏感性；不启动 ROS 或采集')
    p.add_argument('--source', type=Path, action='append', required=True, help='封存 SDK 探针会话，可重复指定')
    p.add_argument('--output', type=Path, required=True, help='全部源会话之外的新目录')
    p.add_argument('--reference-assessment', type=Path, help='已完成的旧评估目录；复用同配方结果做跨会话比较')
    return p


def main(argv=None):
    a = parser().parse_args(argv)
    try:
        protected = list(a.source)
        if a.reference_assessment is not None:
            protected += [a.reference_assessment]
            protected += [Path(p) for p in read(a.reference_assessment/'report.json')['protocol']['sources']]
        install_readonly_guard(protected)
        report = assess(a.source, a.output, reference_assessment=a.reference_assessment)
    except (OSError, ValueError, RuntimeError) as exc:
        print('敏感性评估未完成：'+str(exc), file=sys.stderr)
        return 1
    print(f"评估完成：{len(report['records'])} 条记录；报告：{a.output.resolve()/'report.md'}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
