"""Session-level calibration delivery bundle: URDF + runtime sidecar + provenance + summary."""
import json
from pathlib import Path
import numpy as np
from ..data import write_json
from ..dynamic_export import check_reload, write_inertial_urdf
from ..model_runtime import model_for, urdf_for
from ..rigid_identification import model_from_parameters
from ..shm_data import NAMES, PARTS
from ..stages.dynamics.common import _find_upstream_calibration, _load_valid_calibration


def _load_stage_json(path):
    path = Path(path)
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None


def _stage_rmse_pair(stage_name, report):
    if not report or not report.get('fit_succeeded'):
        return None, None
    if stage_name == 'h1':
        return report.get('overall_nominal_validation_rmse_nm'), report.get('overall_validation_rmse_nm')
    if stage_name == 'h2':
        return report.get('zero_friction_validation_rmse_nm'), report.get('validation_rmse_nm')
    if stage_name == 'h3':
        return report.get('prior_validation_rmse_nm'), report.get('validation_rmse_nm')
    return None, None


def build_session_delivery(session, robot_config):
    """Assemble verified final calibration artifacts under `<session>/delivery/`."""
    delivery_dir = session.root / 'delivery'
    delivery_dir.mkdir(parents=True, exist_ok=True)

    nominal, model_source = model_for(robot_config)
    model = nominal.model
    model.gravity_world = np.asarray(robot_config['gravity_world'], dtype=float)
    nominal = type(nominal)(model)
    nominal_rotor = [float(j.rotor_inertia) for j in nominal.model.joints]

    stage_summaries = {}
    session_accepted = {}
    for st in ('h1', 'h2', 'h3'):
        rec = session.stage_record(st)
        if not rec:
            continue
        adir = session.root / rec['analysis_dir'] if rec.get('analysis_dir') else (session.root / st / 'analysis')
        pdir = session.root / rec['plan_dir'] if rec.get('plan_dir') else (session.root / st / 'plan')
        report = _load_stage_json(adir / 'report.json')
        plan_meta = _load_stage_json(pdir / 'plan.json')
        cal = _load_valid_calibration(adir / 'calibration.json', robot_config)
        fit_ok = bool(rec.get('fit_succeeded') and report and report.get('fit_succeeded'))
        acc_dict = (report or {}).get('acceptance')
        accepted = bool(fit_ok and rec.get('accepted', True) is not False and (acc_dict or {}).get('accepted', True) is not False and cal is not None)
        base_rmse, cand_rmse = _stage_rmse_pair(st, report)
        stage_summaries[st] = dict(
            status=rec.get('status'),
            mode=(report or {}).get('mode') or (plan_meta or {}).get('mode'),
            fit_succeeded=fit_ok,
            accepted=accepted,
            acceptance=acc_dict,
            baseline_rmse_nm=base_rmse,
            validation_rmse_nm=cand_rmse,
        )
        if accepted:
            session_accepted[st] = cal

    selected_cal = None
    selected_source = 'nominal'
    for st in ('h3', 'h2', 'h1'):
        if st in session_accepted:
            selected_cal = session_accepted[st]
            selected_source = f'session_{st}'
            break

    if selected_cal is None:
        upstream_cal, upstream_src, _ = _find_upstream_calibration(session, ('h3', 'h2', 'h1'), robot_config)
        if upstream_cal is not None:
            selected_cal = upstream_cal
            selected_source = upstream_src

    if selected_cal is not None:
        parameters = np.asarray(selected_cal['parameters'], dtype=float)
        viscous = np.asarray(selected_cal.get('viscous', [0.0] * 14), dtype=float)
        coulomb = np.asarray(selected_cal.get('coulomb', [0.0] * 14), dtype=float)
        static_friction = np.asarray(selected_cal.get('static_friction', coulomb), dtype=float)
        smoothing_raw = selected_cal.get('smoothing_velocity', 0.001)
        smoothing = (
            np.asarray(smoothing_raw, dtype=float)
            if isinstance(smoothing_raw, (list, tuple, np.ndarray))
            else float(smoothing_raw)
        )
        stribeck_raw = selected_cal.get('stribeck_velocity', 0.005)
        stribeck = (
            np.asarray(stribeck_raw, dtype=float)
            if isinstance(stribeck_raw, (list, tuple, np.ndarray))
            else float(stribeck_raw)
        )
        rotor_inertia = np.asarray(selected_cal.get('rotor_inertia', nominal_rotor), dtype=float)
    else:
        parameters = np.asarray(nominal.rigid_parameters(), dtype=float)
        viscous = np.zeros(14, dtype=float)
        coulomb = np.zeros(14, dtype=float)
        static_friction = np.zeros(14, dtype=float)
        smoothing = 0.001
        stribeck = 0.005
        rotor_inertia = np.asarray(nominal_rotor, dtype=float)

    gravity_source = (
        'session_h1'
        if 'h1' in session_accepted
        else (selected_cal.get('prior_source', 'nominal') if selected_cal and selected_source in ('session_h2', 'session_h3') and 'h1' in str(selected_cal.get('prior_source', '')) else ('nominal' if selected_source == 'nominal' else selected_source))
    )
    friction_source = (
        'session_h2'
        if 'h2' in session_accepted
        else (selected_cal.get('prior_source', 'nominal') if selected_cal and selected_source == 'session_h3' and 'h2' in str(selected_cal.get('prior_source', '')) else 'nominal')
    )
    inertia_source = 'session_h3' if 'h3' in session_accepted else gravity_source
    rotor_source = 'session_h3' if 'h3' in session_accepted else 'nominal'
    provenance = dict(
        gravity=gravity_source,
        friction=friction_source,
        inertia=inertia_source,
        rotor_inertia=rotor_source,
    )

    delivered_engine = model_from_parameters(
        nominal,
        parameters,
        viscous=viscous,
        coulomb=coulomb,
        static=static_friction,
        smoothing_velocity=smoothing,
        stribeck_velocity=stribeck,
        rotor_inertia=rotor_inertia,
    )

    source_urdf = urdf_for(robot_config)
    calibrated_urdf = delivery_dir / 'calibrated.urdf'
    tmp_urdf = delivery_dir / 'calibrated.urdf.tmp'
    if tmp_urdf.exists():
        tmp_urdf.unlink()
    write_inertial_urdf(source_urdf, delivered_engine, tmp_urdf)
    reload_report = check_reload(tmp_urdf, delivered_engine, source_urdf)
    tmp_urdf.replace(calibrated_urdf)

    runtime_sidecar = dict(
        schema_version='casbot_shm_runtime_dynamics_v1',
        joint_names=list(NAMES),
        gravity_world=list(robot_config['gravity_world']),
        rotor_inertia=[float(j.rotor_inertia) for j in delivered_engine.model.joints],
        friction={
            key: [float(getattr(j.friction, key)) for j in delivered_engine.model.joints]
            for key in ('viscous', 'coulomb', 'static_friction', 'smoothing_velocity', 'stribeck_velocity')
        },
    )
    write_json(delivery_dir / 'runtime_dynamics.json', runtime_sidecar)

    bundle = dict(
        schema_version='casbot_shm_delivery_bundle_v1',
        robot_id=session.robot_id,
        session_dir=str(session.root),
        model_context=robot_config['model_context'],
        source_model_sha256=model_source['sha256'],
        selected_calibration_source=selected_source,
        provenance=provenance,
        stages=stage_summaries,
        joint_names=list(NAMES),
        gravity_world=list(robot_config['gravity_world']),
        parameters=parameters.tolist(),
        rotor_inertia=runtime_sidecar['rotor_inertia'],
        friction=runtime_sidecar['friction'],
        urdf_reload_passed=reload_report['passed'],
        urdf_reload=reload_report,
        files=dict(
            calibrated_urdf='calibrated.urdf',
            runtime_dynamics='runtime_dynamics.json',
            summary_md='summary.md',
        ),
    )
    write_json(delivery_dir / 'calibration_bundle.json', bundle)

    stage_titles = {'h1': 'H1 静态重力', 'h2': 'H2 关节摩擦', 'h3': 'H3 惯量与转子'}
    lines = [
        '# CASBOT 双臂动力学标定交付总表',
        '',
        f'- **机器编号**：`{session.robot_id}`',
        f'- **会话目录**：`{session.root}`',
        f'- **采纳标定源**：`{selected_source}`（重力：`{provenance["gravity"]}`，摩擦：`{provenance["friction"]}`，惯量：`{provenance["inertia"]}`，转子惯量：`{provenance["rotor_inertia"]}`）',
        '- **交付产物**：`delivery/calibrated.urdf` + `delivery/runtime_dynamics.json` + `delivery/calibration_bundle.json`',
        '',
        f'- URDF 重加载复核：{"通过" if reload_report["passed"] else "发现差异，见 calibration_bundle.json"}',
        '',
        '## 1. 阶段拟合与留出验证',
        '',
        '| 阶段 | 模式 | 先验基线 RMSE (Nm) | 候选留出 RMSE (Nm) | 验收判定 | 说明 |',
        '| --- | :---: | ---: | ---: | :---: | --- |',
    ]
    for st in ('h1', 'h2', 'h3'):
        if st not in stage_summaries:
            continue
        info = stage_summaries[st]
        b_str = f'{info["baseline_rmse_nm"]:.4f}' if info['baseline_rmse_nm'] is not None else 'n/a'
        c_str = f'{info["validation_rmse_nm"]:.4f}' if info['validation_rmse_nm'] is not None else 'n/a'
        if not info['fit_succeeded']:
            verdict = '未拟合'
            note = str(info['mode'] or info['status'])
        elif info['accepted']:
            verdict = '采纳'
            note = ''
        else:
            verdict = '回退先验'
            note = (info.get('acceptance') or {}).get('reason', 'validation_degraded')
        lines.append(f'| {stage_titles[st]} | `{info["mode"] or "-"}` | {b_str} | {c_str} | **{verdict}** | {note} |')

    lines.extend([
        '',
        '## 2. 14 轴交付动力学参数一览',
        '',
        '| SHM ID | 关节 | 质量 (kg) | 粘性 b (Nm·s/rad) | 库仑 fc (Nm) | 静摩擦 fs (Nm) | 转子惯量 Ir (kg·m²) |',
        '| ---: | --- | ---: | ---: | ---: | ---: | ---: |',
    ])
    for j, jt in enumerate(delivered_engine.model.joints):
        label = ('左' if j < 7 else '右') + PARTS[j % 7]
        lines.append(
            f'| {j + 12} | {label} (`{NAMES[j]}`) | {jt.body.mass:.4f} '
            f'| {jt.friction.viscous:.4f} | {jt.friction.coulomb:.4f} | {jt.friction.static_friction:.4f} '
            f'| {jt.rotor_inertia:.5f} |'
        )

    summary_path = delivery_dir / 'summary.md'
    summary_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return dict(
        dir='delivery',
        selected_calibration_source=selected_source,
        provenance=provenance,
        calibrated_urdf='delivery/calibrated.urdf',
        runtime_dynamics='delivery/runtime_dynamics.json',
        calibration_bundle='delivery/calibration_bundle.json',
        summary_md='delivery/summary.md',
    )
