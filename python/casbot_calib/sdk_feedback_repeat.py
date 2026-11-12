"""Cross-session comparisons of completed sensitivity reports, without refitting."""
from itertools import combinations
from pathlib import Path
import csv
import json
from .data import write_json
from .hardware_probe import field_probe_config
from .sdk_feedback_sensitivity_math import FLOOR, THRESHOLD


def read(path):
    return json.loads(Path(path).read_text())


def recipe(source):
    p = read(Path(source)/'protocol/experiment.json')
    return {**{k:p[k] for k in ('joint', 'pose_rad', 'holding', 'transition_s', 'settle_s',
                               'fixture_friction_prior', 'command_hz')},
            'probe_config':field_probe_config(p['probe_config']),
            'tones':p.get('tones'), 'observation_policy':p.get('observation_policy')}


def public_binding(record):
    m = record['metadata']; plan = read(Path(m['plan'])/'plan.json'); c = plan['config']
    # Existing plan identity plus actual first receive clock distinguish runtime
    # histories. No private plant diagnostics or evaluation data are consumed.
    with (Path(m['capture'])/'execution/feedback.csv').open() as stream:
        first = next(csv.DictReader(stream))
    return dict(model_context=c.get('model_context'), source_model_sha256=plan['source_model_sha256'],
        joint_names=plan['joint_names'], gravity_world=c['gravity_world'], effort_gain=c['effort_gain'],
        effort_zero_nm=c['effort_zero_nm'], gravity_feedforward=c['gravity_feedforward'],
        pd=c['breakaway_parameters'], command_period_ns=plan['command_period_ns'],
        endpoint=c['connection']['endpoint_root'], scope=c['connection']['scope'],
        first_receive_ns=int(first['receive_ns']), executor_sha256=plan['executor_sha256'])


def compare_pair(reference_records, repeat_records):
    """Old session is denominator; same signs/cases pair, never sign averages."""
    old = sorted(reference_records, key=lambda r:r['metadata']['job']['direction'])
    new = sorted(repeat_records, key=lambda r:r['metadata']['job']['direction'])
    if len(old)!=2 or len(new)!=2 or any([r['metadata']['job']['direction'] for r in pair]!=[-1,1] for pair in (old,new)):
        raise ValueError('Each comparison needs one negative and one positive record')
    cases = old[0]['results']; comparisons = []
    for r in old+new:
        if [x['case'] for x in r['results']] != [x['case'] for x in cases]:
            raise ValueError('Cross-session perturbation grids differ')
    for index, row in enumerate(cases):
        routes = {}
        for route in ('A', 'B'):
            fits = [r['results'][index]['routes'][route] for r in old+new]
            signs = []
            for k, direction in enumerate((-1, 1)):
                a, b = fits[k], fits[k+2]
                available = a['available'] and b['available']
                difference = abs(b['inertia_kg_m2']-a['inertia_kg_m2'])/max(abs(a['inertia_kg_m2']), FLOOR) if available else None
                signs.append(dict(direction=direction, available=available,
                    reference_inertia_kg_m2=a.get('inertia_kg_m2'), repeat_inertia_kg_m2=b.get('inertia_kg_m2'),
                    cross_session_inertia_difference_relative=difference,
                    within_10_percent=bool(difference<=THRESHOLD) if available else None,
                    reference_viscous_nm_s_rad=a.get('viscous_nm_s_rad'), repeat_viscous_nm_s_rad=b.get('viscous_nm_s_rad'),
                    viscous_difference_nm_s_rad=(b['viscous_nm_s_rad']-a['viscous_nm_s_rad']) if available else None,
                    reference_effective_advance_s=a.get('effective_advance_s'), repeat_effective_advance_s=b.get('effective_advance_s'),
                    effective_advance_difference_s=(b['effective_advance_s']-a['effective_advance_s']) if available and route=='B' else None))
            complete = all(f['available'] for f in fits)
            values = [f['inertia_kg_m2'] for f in fits] if complete else []
            within = [abs(values[1]-values[0])/max(abs(values[0]), FLOOR),
                      abs(values[3]-values[2])/max(abs(values[2]), FLOOR)] if complete else [None, None]
            all_pairs = max(abs(a-b)/max(min(abs(a),abs(b)), FLOOR) for a,b in combinations(values,2)) if complete else None
            cross_pairs = [dict(reference_direction=(-1,1)[i], repeat_direction=(-1,1)[j],
                                difference_relative=abs(values[j+2]-values[i])/max(abs(values[i]),FLOOR))
                           for i in range(2) for j in range(2)] if complete else []
            worst_cross = max(cross_pairs,key=lambda p:p['difference_relative']) if complete else None
            routes[route] = dict(available=complete, directions=signs,
                maximum_same_direction_cross_session_relative=max(s['cross_session_inertia_difference_relative'] for s in signs) if complete else None,
                reference_direction_difference_relative=within[0], repeat_direction_difference_relative=within[1],
                all_four_pairwise_maximum_relative=all_pairs,
                worst_cross_session_including_cross_direction=worst_cross,
                cross_session_within_10_percent=bool(complete and all(s['within_10_percent'] for s in signs)),
                all_four_within_10_percent=bool(complete and all_pairs<=THRESHOLD),
                phase_boundary_present=any(f.get('phase_branch_boundary',False) for f in fits))
        comparisons.append(dict(case=row['case'], routes=routes))
    summaries = []
    for family in sorted({r['case']['family'] for r in comparisons}):
        rows = [r for r in comparisons if r['case']['family']==family]
        routes = {}
        for route in ('A','B'):
            fits = [r['routes'][route] for r in rows]; valid = [f for f in fits if f['available']]
            routes[route] = dict(conditions=len(rows), unavailable=len(fits)-len(valid),
                maximum_same_direction_cross_session_relative=max((f['maximum_same_direction_cross_session_relative'] for f in valid), default=None),
                maximum_all_four_pairwise_relative=max((f['all_four_pairwise_maximum_relative'] for f in valid), default=None),
                cross_session_within_10_percent_count=sum(f['cross_session_within_10_percent'] for f in fits),
                all_four_within_10_percent_count=sum(f['all_four_within_10_percent'] for f in fits))
        summaries.append(dict(family=family, routes=routes))
    return dict(reference_source=old[0]['metadata']['source'], repeat_source=new[0]['metadata']['source'],
        baseline=comparisons[0], cases=comparisons, by_family=summaries)


def compare_assessments(reference_path, repeat_report, output):
    root = Path(reference_path).resolve(); reference = read(root/'report.json')
    if reference.get('schema_version')!='casbot_sdk_feedback_sensitivity_report_v1' or not reference.get('completed'):
        raise ValueError('Completed reference sensitivity report required')
    if any(reference['protocol'][key]!=repeat_report['protocol'][key] for key in
           ('diagnostic_threshold','inertia_denominator_floor_kg_m2','cases','route_A','route_B','common_processing')):
        raise ValueError('Reference/repeat analysis contracts differ; do not reuse incompatible fits')
    out = Path(output).resolve()
    if root == out or root in out.parents:
        raise ValueError('Comparison output must be outside reference report')
    groups = []; mismatches = []
    for new_source in repeat_report['protocol']['sources']:
        new = [r for r in repeat_report['records'] if r['metadata']['source']==new_source]
        for old_source in reference['protocol']['sources']:
            if old_source==new_source: continue
            if recipe(old_source)!=recipe(new_source):
                mismatches.append(dict(reference=old_source, repeat=new_source, reason='different excitation/processing recipe')); continue
            old = [r for r in reference['records'] if r['metadata']['source']==old_source]
            pair = compare_pair(old, new)
            old_bind = [public_binding(r) for r in sorted(old,key=lambda r:r['metadata']['job']['direction'])]
            new_bind = [public_binding(r) for r in sorted(new,key=lambda r:r['metadata']['job']['direction'])]
            ignored = ('endpoint','first_receive_ns','executor_sha256')
            differences = [{k:dict(reference=a[k],repeat=b[k]) for k in a if k not in ignored and a[k]!=b[k]}
                           for a,b in zip(old_bind,new_bind)]
            if any(differences):
                raise ValueError('Matched recipe has different effective model/scale/PD bindings: '+str(differences))
            distinct = all(a['endpoint']!=b['endpoint'] and a['first_receive_ns']!=b['first_receive_ns'] and
                           a['scope']==b['scope']=='localhost_contract_mock' for a,b in zip(old_bind,new_bind))
            pair.update(independent_mock_runtime_evidence=distinct, reference_bindings=old_bind, repeat_bindings=new_bind,
                executor_changed=any(a['executor_sha256']!=b['executor_sha256'] for a,b in zip(old_bind,new_bind)),
                runtime_scope='new localhost endpoint and receive history; same configured plant/seed, not independent hardware population')
            groups.append(pair)
    if not groups: raise ValueError('No same-recipe reference/repeat pairs')
    report = dict(schema_version='casbot_sdk_feedback_repeat_report_v1', completed=True, groups=groups,
        unmatched_sources=mismatches, reference_assessment=str(root), reference_fits_reused_without_refitting=True,
        threshold=THRESHOLD, denominator_floor_kg_m2=FLOOR,
        same_direction_metric='abs(Jrepeat-Jreference)/max(abs(Jreference),1e-6)',
        cross_session_any_direction_metric='four old-to-new pairs, abs(Jnew-Jold)/max(abs(Jold),1e-6)',
        all_four_metric='maximum abs(Ji-Jj)/max(min(abs(Ji),abs(Jj)),1e-6), six unordered pairs; diagnostic only',
        physical_delay_estimated=False, full_p0_r2_performed=False, formal_test_consumed=False,
        independent_mock_sessions_verified=all(g['independent_mock_runtime_evidence'] for g in groups))
    out.mkdir(parents=True,exist_ok=False); write_json(out/'report.json',report)
    (out/'report.md').write_text(markdown_report(report),encoding='utf-8')
    return report


def markdown_report(report):
    from .sdk_feedback_sensitivity import LABELS
    def pct(x):return f'{100*x:.3f}%' if x is not None else '不可用'
    def num(x, scale=1):return f'{x*scale:.7g}' if x is not None else '不可用'
    lines=['# 双频右腕 roll：A／B 跨会话复测','',
        '原会话作为分母，同方向配对；原 10% 数值与 1e-6 kg·m² 分母下限不变。不平均正反结果来掩盖差异。',
        '旧评估结果直接复用，不重新拟合；每个扰动条件在两会话中使用相同配方、种子及处理。','']
    for group in report['groups']:
        lines += [f"## {Path(group['reference_source']).name} → {Path(group['repeat_source']).name}",'',
            f"独立 Mock 运行证据：{'不同私有端点及接收时间历史' if group['independent_mock_runtime_evidence'] else '未建立'}。执行器版本{'有变化' if group['executor_changed'] else '相同'}；有效模型、尺度、PD 与波形绑定相同。",'',
            '### 未扰动惯量与同方向复测','',
            '| 路线 | 方向 | 原 J kg·m² | 新 J kg·m² | 跨会话差 |','|---|---|---:|---:|---:|']
        for route in ('A','B'):
            for row in group['baseline']['routes'][route]['directions']:
                lines.append(f"| {route} | {'负向' if row['direction']<0 else '正向'} | {num(row['reference_inertia_kg_m2'])} | {num(row['repeat_inertia_kg_m2'])} | {pct(row['cross_session_inertia_difference_relative'])} |")
        lines += ['', '| 路线 | 原会话正反差 | 新会话正反差 | 跨会话含跨方向最差差异 |','|---|---:|---:|---:|']
        for route in ('A','B'):
            row=group['baseline']['routes'][route]
            worst=row['worst_cross_session_including_cross_direction']
            lines.append(f"| {route} | {pct(row['reference_direction_difference_relative'])} | {pct(row['repeat_direction_difference_relative'])} | {pct(worst['difference_relative'] if worst else None)} |")
        lines += ['', '跨会话含跨方向比较四种旧→新配对，始终用对应旧记录作分母。JSON 另存四条全配对的对称差异（较小 J 作分母）；该附加度量不能替代原方向差门槛或原 128 点 R2。','',
            '### 粘性项与等效相位是否重复','',
            '| 路线／方向 | 原 b Nm·s/rad | 新 b Nm·s/rad | 原等效相位 ms | 新等效相位 ms |','|---|---:|---:|---:|---:|']
        for route in ('A','B'):
            for row in group['baseline']['routes'][route]['directions']:
                phase=[num(row[key],1000) if route=='B' else '不估计' for key in ('reference_effective_advance_s','repeat_effective_advance_s')]
                lines.append(f"| {route}／{'负' if row['direction']<0 else '正'} | {num(row['reference_viscous_nm_s_rad'])} | {num(row['repeat_viscous_nm_s_rad'])} | {' | '.join(phase)} |")
        lines += ['', '### 相同扰动下的跨会话最差差异','',
            '每类取正反方向和全部已测档位／种子／相位的最差值。强退化导致不可用另计；不因两会话同向偏移而宣称准确。','',
            '| 扰动族 | A 最差跨会话差 | B 最差跨会话差 | A／B 不可用条件数 |','|---|---:|---:|---:|']
        for row in group['by_family']:
            a,b=row['routes']['A'],row['routes']['B']
            lines.append(f"| {LABELS.get(row['family'],'未扰动')} | {pct(a['maximum_same_direction_cross_session_relative'])} | {pct(b['maximum_same_direction_cross_session_relative'])} | {a['unavailable']}／{b['unavailable']} |")
    lines += ['', '## 解释范围','',
        '- 相同模型与种子的独立进程复测主要检验运行／缓存相位／调度重复性，不代表热漂移、独立设备或跨日实机分布。',
        '- 等效相位不是硬件采样时刻；b 仅用于诊断，不更新 H3 候选。',
        '- 与各自基线的扰动敏感性继续看主评估报告；这里增加跨会话一致性，不选择新频带、时移或摩擦先验。',
        '- 原始数据未修改；无实机连接、正式测试或完整刚体 R2。','']
    return '\n'.join(lines)
