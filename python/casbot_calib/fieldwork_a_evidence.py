"""Read saved fieldwork CSV/CDR JSON prefixes into an A-first evidence report.

No collector, motion, candidate update or automatic route promotion. Optional
window diagnostics reuse the existing A estimator on the original time axis.
"""
from pathlib import Path
import numpy as np
import yaml
from .data import write_json
from .sdk_feedback_sensitivity import read, install_readonly_guard


def cadence(ns):
    ns = np.asarray(ns, dtype=np.int64)
    if len(ns) < 2:
        return dict(rows=len(ns), hz=None, nonincreasing_intervals=0)
    dt = np.diff(ns); positive = dt[dt > 0]*1e-6
    return dict(rows=len(ns), hz=float((len(ns)-1)*1e9/(ns[-1]-ns[0])) if ns[-1] > ns[0] else None,
                nonincreasing_intervals=int(np.sum(dt <= 0)),
                interval_ms={key: float(np.quantile(positive, q)) for key, q in
                             [('p50', .5), ('p95', .95), ('p99', .99), ('maximum', 1.)]} if len(positive) else {})


def find_recordings(root):
    """Read the single authoritative execution tables, never derived exports."""
    paths = [root/'commands.csv'] if (root/'commands.csv').is_file() else sorted(root.rglob('execution/commands.csv'))
    return [p.parent for p in paths if 'evaluation' not in p.relative_to(root).parts
            and (p.parent/'feedback.csv').is_file()]


def profile_for(directory):
    for ancestor in (directory, *directory.parents):
        path = ancestor/'protocol/profile.yaml'
        if path.is_file():
            return yaml.safe_load(path.read_text()), str(path)
        path = ancestor/'plan/plan.json'
        if path.is_file():
            value = read(path)
            if 'config' in value:
                return value['config'], str(path)
    raise ValueError('此记录缺少配套profile／plan；先用不带--window的命令整理通信证据')


def imu_projection(problem, imu, *, interval=None):
    """Sensitivity to measured base-motion regressors, not a force correction."""
    from .hardware_probe import basis, exposure, least_squares
    t = problem['time_s']; j = problem['joint']
    lo, hi = interval if interval is not None else (float(t[0]), float(t[-1]+np.median(np.diff(t))))
    ti, values = imu
    if ti[0] > t[0] or ti[-1] < t[-1]:
        return dict(available=False, next_action='IMU未覆盖这个运动窗；下一段同时保留IMU与控制记录')
    use = (ti >= lo) & (ti < hi); ti = ti[use]; values = values[use]
    if len(ti) < 2 or not np.isfinite(values).all():
        return dict(available=False, next_action='该窗IMU有效值不足；检查记录中的协方差缺失标记和实际发布')
    duration = hi-lo
    harmonics = int(np.floor(problem['preprocessing']['frequency_band_hz'][1]*duration))
    try:
        x = basis(ti, lo, duration, harmonics)
        coefficients, _ = least_squares(x, values, exposure(ti))
    except (ValueError, np.linalg.LinAlgError) as exc:
        return dict(available=False, next_action='IMU不能支持本窗处理带宽：'+str(exc))
    u = t-t.mean(); a = problem['ddq'][:, j]
    nuisance = np.column_stack([problem['dq'][:, j], np.ones(len(t)), u, u*u])
    remove = lambda z: z-nuisance@np.linalg.lstsq(nuisance, z, rcond=1e-10)[0]
    ap = remove(a); y = remove(problem['target']-problem['projected_gravity_nm'])
    j0 = float(ap@y/(ap@ap)); rows = []
    for shift in (-.005, 0., .005):
        projected = basis(t+shift, lo, duration, harmonics)@coefficients
        alpha = basis(t+shift, lo, duration, harmonics, derivative=1)@coefficients[:, :3]
        omega = projected[:, :3]
        # Local base-motion nuisance: omega, alpha, specific force and
        # centrifugal products. All terms remain diagnostics, never subtracted.
        terms = np.column_stack([projected, alpha, *[omega[:, k]*omega[:, l] for k in range(3) for l in range(k, 3)]])
        z = remove(terms); scales = np.linalg.norm(z, axis=0)
        active = scales > 1e-10
        if not np.any(active):
            rows.append(dict(imu_offset_s=shift, inertia_kg_m2=j0, change_relative=0., nuisance_rank=0)); continue
        U, singular, _ = np.linalg.svd(z[:, active]/scales[active], full_matrices=False)
        rank = int(np.sum(singular > singular[0]*1e-8)); U = U[:, :rank]
        aq = ap-U@(U.T@ap); remaining = float(np.linalg.norm(aq)/np.linalg.norm(ap))
        if remaining < 1e-8:
            rows.append(dict(imu_offset_s=shift, unavailable='该窗身体运动与惯量激励共线', acceleration_remaining_fraction=remaining)); continue
        estimate = float(aq@y/(aq@aq))
        rows.append(dict(imu_offset_s=shift, inertia_kg_m2=estimate, change_relative=abs(estimate-j0)/max(abs(j0), 1e-6),
                         nuisance_rank=rank, acceleration_remaining_fraction=remaining))
    valid = [r for r in rows if 'change_relative' in r]
    return dict(available=len(valid)==len(rows), cases=rows,
        maximum_inertia_change_relative=max((r['change_relative'] for r in valid), default=None),
        interpretation='IMU局部干扰项的拟合敏感性；与惯量共线时调整激励频带或姿态，不将额外参数收益当作已修正力矩')


def window_a(obs, elapsed, interval, joint, profile, band, *, imu=None):
    from .hardware_probe import settings, field_probe_config, probe_problem, inertia_balance
    from .model_runtime import model_for
    lo, hi = interval
    if not np.isfinite([lo, hi, band]).all() or lo >= hi or band <= 0:
        raise ValueError('窗口须START<END，带宽须为正数')
    use = (elapsed >= lo) & (elapsed < hi)
    if np.sum(use) < 2:
        raise ValueError('指定窗口内没有足够回调行')
    if lo < elapsed[0] or hi > elapsed[-1]:
        raise ValueError('窗口超出实际反馈覆盖范围')
    if not all(np.isfinite(obs[k][use]).all() for k in ('q', 'dq', 'effort_raw')):
        raise ValueError('窗口内q／dq／effort有缺失或非有限值')
    engine, _ = model_for(profile)
    model = engine.model; model.gravity_world = np.asarray(profile['gravity_world']); engine = type(engine)(model)
    config = field_probe_config(settings())
    config['probe']['duration_s'] = hi-lo
    trim = min(.2, (hi-lo)/10)
    config['processing'].update(fit_band_hz=band, edge_trim_s=trim,
        quadrature_period_s=min(.002, 1/(10*band)))
    direction = 1 if np.mean(obs['dq'][use, joint]) >= 0 else -1
    motion = dict(knots_s=[lo, hi], labels=['probe_measure'], conditions=dict(joint=joint, direction=direction))
    problem = probe_problem(obs, elapsed, motion, engine, config, profile)
    # Rank is mathematical information, not a data-quality threshold.
    t = problem['time_s']-np.mean(problem['time_s'])
    a = problem['ddq'][:, joint]; v = problem['dq'][:, joint]
    nuisance = np.column_stack([v, np.ones(len(t)), t, t*t])
    x = np.column_stack([a, nuisance]); scale = np.linalg.norm(x, axis=0)
    if np.any(scale == 0) or np.linalg.matrix_rank(x/scale) < x.shape[1]:
        raise ValueError('该窗口的加速度与速度／慢漂移不可分离；改取含加减速的完整单向片段')
    base = inertia_balance(problem)
    a_perp = a-nuisance@np.linalg.lstsq(nuisance, a, rcond=1e-10)[0]
    independent_accel_rms = float(np.sqrt(np.mean(a_perp*a_perp)))
    changes = []
    for ms in (-5., -2., -1., 1., 2., 5.):
        if abs(ms)*.001 >= trim:
            continue
        shifted = probe_problem(obs, elapsed, motion, engine, config, profile, effort_relative_offset_s=ms*.001)
        fit = inertia_balance(shifted)
        changes.append(dict(extra_effort_advance_ms=ms, **fit,
            inertia_change_relative=float(abs(fit['inertia_kg_m2']-base['inertia_kg_m2'])/max(abs(base['inertia_kg_m2']), 1e-6))))
    q = obs['q'][use]; dq = obs['dq'][use]; held = [j for j in range(14) if j != joint]
    return dict(available=True, interval_s=list(interval), band_hz=band, baseline=base,
        positive_inertia=bool(base['inertia_kg_m2'] > 0),
        body_motion=imu_projection(problem, imu, interval=interval) if imu is not None else dict(available=False, next_action='下一轮同步记录IMU，以比较自平衡运动加入模型前后的惯量变化'),
        signal=dict(target_q_span_rad=float(np.ptp(q[:, joint])),
            target_velocity_min_rad_s=float(np.min(dq[:, joint])), target_velocity_max_rad_s=float(np.max(dq[:, joint])),
            held_maximum_velocity_rad_s=float(np.max(abs(dq[:, held]))),
            acceleration_after_nuisance_rms_rad_s2=independent_accel_rms,
            torque_rms_budget_for_10_percent_J_nm=.1*base['inertia_kg_m2']*independent_accel_rms if base['inertia_kg_m2'] > 0 else None),
        preprocessing=problem['preprocessing'], perturbations=changes,
        maximum_extra_5ms_inertia_change_relative=max(r['inertia_change_relative'] for r in changes if abs(r['extra_effort_advance_ms']) == 5.) if trim > .005 else None,
        interpretation='固定名义重力下原A的局部诊断；额外错位敏感性与10%力矩预算，不是实测缓存年龄或真实误差上界')


def imu_summary(root, *, scratch=None, spool_windows=False):
    from .fieldwork_streaming import AnalysisScratch,StreamingCadence,RunningVector
    from .raw_archive import raw_directories,iter_records
    if scratch is None:
        with AnalysisScratch() as temporary:return imu_summary(root,scratch=temporary)
    results=[]
    for raw in raw_directories(root):
        path=raw/('format.json' if (raw/'format.json').exists() else 'packets.jsonl')
        metadata=read(raw/'recorder.json') if (raw/'recorder.json').exists() else {}
        timing=StreamingCadence(scratch);frames=set();clocks=set()
        stats={key:RunningVector() for key in ('angular_velocity','linear_acceleration')}
        source=dict(id=scratch.series(),path=str(path),clock_id=metadata.get('clock_id'),count=0,first=None,last=None)
        for row,_ in iter_records(raw):
            if row.get('channel')!='imu':continue
            ns=int(row['receive_ns']);timing.add(ns);m=row['message']
            frames.add(m.get('header',{}).get('frame_id',''));clocks.add(row.get('clock_id',metadata.get('clock_id','')))
            valid={}
            for key,stat in stats.items():
                value=m.get(key);cov=m.get(key+'_covariance')
                if cov and cov[0]==-1:continue
                if isinstance(value,list) and len(value)==3 and all(isinstance(x,(int,float)) for x in value) and np.isfinite(value).all():
                    stat.add(value);valid[key]=value
            if len(valid)==2:
                if spool_windows:scratch.add_imu(source['id'],source['count'],ns,valid['angular_velocity']+valid['linear_acceleration'])
                if source['first'] is None:source['first']=ns
                source['last']=ns;source['count']+=1
        if timing.count:
            results.append(dict(path=str(path),receive=timing.result(),frame_ids=sorted(frames),clock_ids=sorted(clocks),
                channels={k:v.result() for k,v in stats.items() if v.count}))
            if spool_windows:scratch.imu_sources.append(source)
    return results



def run(source, output, *, joint=13, window=None, band=30.):
    root = Path(source).resolve(); out = Path(output).resolve()
    if not root.is_dir() or root == out or root in out.parents or out in root.parents:
        raise ValueError('输入须为已保存目录，输出须位于输入之外')
    if not 0 <= joint < 14:
        raise ValueError('joint须为0..13')
    # A specific run still consumes its workspace's continuous IMU recording.
    imu_root = root
    for ancestor in (root, *root.parents):
        meta = ancestor/'protocol/session.json'
        if meta.is_file() and read(meta).get('schema_version') == 'casbot_fieldwork_workspace_v1':
            imu_root = ancestor; break
    if out == imu_root or imu_root in out.parents or out in imu_root.parents:
        raise ValueError('输出须位于保存工作区之外')
    install_readonly_guard([root, imu_root])
    from .fieldwork_streaming import AnalysisScratch,scan_commands,scan_feedback
    out.mkdir(parents=True,exist_ok=False)
    records=[];unavailable=[]
    with AnalysisScratch(out) as scratch:
        imu=imu_summary(imu_root,scratch=scratch,spool_windows=window is not None)
        for path in find_recordings(root):
            try:
                command=scan_commands(path/'commands.csv',scratch)
                feedback,obs,contiguous=scan_feedback(path/'feedback.csv',scratch,command['first_publish_ns'],window)
            except ValueError as exc:
                unavailable.append(dict(directory=str(path),reason=str(exc)));continue
            origin=command['first_publish_ns']
            row=dict(directory=str(path),**command,**feedback,axis=joint)
            if window is not None:
                try:
                    profile,binding=profile_for(path);row['profile']=binding
                    clock_id=None
                    from .raw_archive import capture_raw
                    for ancestor in (path,*path.parents):
                        raw,_,_=capture_raw(ancestor/'capture');recorder=raw/'recorder.json'
                        if recorder.exists():clock_id=read(recorder).get('clock_id');break
                        recorder=ancestor/'raw_state/raw/recorder.json'
                        if recorder.exists():clock_id=read(recorder).get('clock_id');break
                    if not np.isfinite([*window,band]).all() or window[0]>=window[1] or band<=0:
                        raise ValueError('窗口须START<END，带宽须为正数')
                    lo_ns=origin+round(window[0]*1e9);hi_ns=origin+round(window[1]*1e9)
                    matching=[s for s in scratch.imu_sources if clock_id and s['clock_id']==clock_id and s['count']>=2
                        and s['first']<=lo_ns and s['last']>=hi_ns]
                    signal=None
                    if matching:
                        selected=max(matching,key=lambda s:s['count']);row['imu_path']=selected['path']
                        ns,values=scratch.imu_window(selected,lo_ns,hi_ns)
                        signal=((ns-origin)*1e-9,values)
                    if not contiguous:raise ValueError('Contiguous public probe window required')
                    row['A']=window_a(obs,(obs['receive_ns']-origin)*1e-9,window,joint,profile,band,imu=signal)
                except (ValueError,KeyError,np.linalg.LinAlgError) as exc:
                    row['A']=dict(available=False,next_action=str(exc))
            records.append(row)
    report = dict(schema_version='casbot_fieldwork_a_evidence_v1', source=str(root), imu_source=str(imu_root), records=records, imu=imu,
        unavailable_records=unavailable,
        summary=f'已整理{len(records)}段指令／反馈、{len(imu)}份IMU记录', joint=joint,
        window_s=window, band_hz=band, threshold=.1,
        decision_method='按fieldwork/README.md的A优先判断流程，逐轴、逐发送档位与运动频带比较',
        decision_scope='证据摘要；窗口A采用封存名义模型，身体运动/尺度/模型误差由独立对照评估',
        source_modified=False, sample_time_known=False, automatic_calibration_acceptance=False)
    if not records:
        report['next_action'] = '当前目录没有配对commands.csv／feedback.csv；请传GUI保存目录或具体recording目录，只读observe仅提供接收观测'
    write_json(out/'report.json', report)
    (out/'report.md').write_text(markdown(report), encoding='utf-8')
    return report


def markdown(report):
    def n(x): return '—' if x is None else f'{x:.5g}'
    lines = ['# A路线现场数据摘要', '', report['summary'], '',
        '先比较相同运动的发送档位，再比较同一发送档位下的运动带宽。', '',
        '| 记录 | 实际发送 Hz | 发送间隔 p99／最大 ms | 反馈接收 Hz | 反馈间隔 p99／最大 ms |', '|---|---:|---:|---:|---:|']
    for row in report['records']:
        a, b = row['sender'], row['receiver']
        def pair(r):
            z = r.get('interval_ms', {}); return n(z.get('p99'))+'／'+n(z.get('maximum'))
        lines.append(f"| {row['directory']} | {n(a['hz'])} | {pair(a)} | {n(b['hz'])} | {pair(b)} |")
    if report.get('next_action'): lines += ['', report['next_action']]
    for row in report['unavailable_records']:
        lines += ['', row['reason']]
    for row in report['records']:
        if 'A' not in row: continue
        lines += ['', '## '+row['directory'], '']
        a = row['A']
        if not a['available']:
            lines += ['下一步：'+a['next_action']]; continue
        lines += [f"窗口 {a['interval_s']}s；共同处理 {a['band_hz']:g}Hz。",
            f"A 局部惯量：{a['baseline']['inertia_kg_m2']:.8g}kg·m²；粘性项：{a['baseline']['viscous_nm_s_rad']:.7g}Nm·s/rad。",
            f"额外 ±5ms 力矩错位的最大惯量变化：{100*a['maximum_extra_5ms_inertia_change_relative']:.3f}%。" if a['maximum_extra_5ms_inertia_change_relative'] is not None else '窗口过短，未计算±5ms。',
            f"原10%惯量变化对应的总力矩扰动RMS充分预算：{a['signal']['torque_rms_budget_for_10_percent_J_nm']:.6g}Nm。" if a['positive_inertia'] else 'A拟合惯量非正：先核对力矩符号、模型和加速度信号，再计算惯量误差预算。',
            '该预算须与独立记录预测误差、时序及身体运动模型对照；本窗口拟合残差不能充当总扰动上界。', '',
            '| 额外力矩超前 ms | J kg·m² | 相对A基线变化 |', '|---:|---:|---:|']
        lines += [f"| {x['extra_effort_advance_ms']:+g} | {x['inertia_kg_m2']:.8g} | {100*x['inertia_change_relative']:.3f}% |" for x in a['perturbations']]
        body = a['body_motion']; value = body.get('maximum_inertia_change_relative')
        if body['available']:
            lines += ['', f'加入IMU局部干扰项后，J最大变化 {100*value:.3f}%（含IMU额外±5ms对照）。']
        else:
            lines += ['', '身体运动下一步：'+body.get('next_action', '身体运动与惯量激励共线；改变运动频带或姿态后重复比较。')]
    lines += ['', '## IMU与下一步', '']
    if report['imu']:
        for r in report['imu']:
            lines += [f"- {r['path']}：{r['receive']['rows']}条，{n(r['receive']['hz'])}Hz；frame_id={r['frame_ids']}。"]
    else:
        lines += ['本次存档没有IMU；下一轮GUI连接会同步记录 /motion/imu，或用 observe --imu-topic 显式指定话题。']
    lines += ['', '按 README 的判断表把结果落到“沿用A／降低发送率／降低运动带宽／优化SDK并保留B”之一。力矩尺度与身体运动影响单独列项，不用发送或接收Hz替代惯量证据。']
    return '\n'.join(lines)+'\n'
