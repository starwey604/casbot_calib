"""Static window diagnostics and PSD rigid-body gravity candidate export."""
import csv
from pathlib import Path
import numpy as np
from .data import write_json
from .model_runtime import model_for, urdf_for


def write_static_diagnostics(means, engine, output, names):
    """Compare measured window effort against nominal gravity at measured mean q."""
    output = Path(output)
    windows = []
    grouped = {}
    for row in means:
        item = dict(
            name=row['name'],
            conditions=row['conditions'],
            available=row.get('q') is not None and row.get('torque') is not None,
            paired_approach=row.get('paired_approach'),
        )
        if item['available']:
            q = np.asarray(row['q'])
            torque = np.asarray(row['torque'])
            nominal = engine.gravity(q)
            item.update(
                q_rad=q.tolist(),
                estimated_effort_nm=torque.tolist(),
                nominal_gravity_nm=nominal.tolist(),
                residual_nm=(torque - nominal).tolist(),
                maximum_received_speed_rad_s=row.get('maximum_received_speed_rad_s'),
            )
            c = row['conditions']
            grouped.setdefault((c['batch'], c['pose']), []).append(item)
        else:
            item['unavailable_reason'] = row.get('unavailable_reason', 'no window mean')
        windows.append(item)

    pairs = []
    for (split, pose), rows in grouped.items():
        minus = next((r for r in rows if r['conditions']['approach'] == -1), None)
        plus = next((r for r in rows if r['conditions']['approach'] == 1), None)
        item = dict(batch=split, pose=pose, available=minus is not None and plus is not None)
        if item['available']:
            item.update(
                q_plus_minus_rad=(np.asarray(plus['q_rad']) - minus['q_rad']).tolist(),
                torque_half_difference_nm=((np.asarray(plus['estimated_effort_nm']) - minus['estimated_effort_nm']) / 2).tolist(),
                residual_half_difference_nm=((np.asarray(plus['residual_nm']) - minus['residual_nm']) / 2).tolist(),
            )
        pairs.append(item)

    available = [row for row in windows if row['available']]
    summary = dict(
        available_windows=len(available),
        total_windows=len(windows),
        paired_targets=sum(row['available'] for row in pairs),
        nominal_gravity_rmse_nm=(
            np.sqrt(np.mean(np.square([row['residual_nm'] for row in available]), axis=0)).tolist()
            if available else None
        ),
    )
    write_json(
        output / 'static_diagnostics.json',
        dict(joint_names=list(names), **summary, windows=windows, pairs=pairs),
    )
    with (output / 'static_comparison.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['trial', 'batch', 'pose', 'approach', 'joint_id', 'joint', 'q_rad', 'estimated_effort_nm', 'nominal_gravity_nm', 'residual_nm'])
        for row in available:
            c = row['conditions']
            for j, name in enumerate(names):
                writer.writerow([
                    row['name'], c['batch'], c['pose'], c['approach'], j + 12, name,
                    row['q_rad'][j], row['estimated_effort_nm'][j], row['nominal_gravity_nm'][j], row['residual_nm'][j],
                ])
    return summary


def write_gravity_candidate(means, config, output, metadata):
    """Fit PSD rigid-body gravity parameters on paired windows and export candidate URDF."""
    from .rigid_identification import fit_rigid, model_from_parameters
    from .dynamic_export import write_inertial_urdf, check_reload

    output = Path(output)
    engine, _ = model_for(config)
    model = engine.model
    model.gravity_world = np.asarray(config.get('gravity_world', model.gravity_world))
    engine = type(engine)(model)

    batches = {'train': [], 'validation': []}
    by_pose = {}
    spreads = []
    for row in means:
        if row.get('q') is None or row.get('torque') is None:
            continue
        c = row['conditions']
        by_pose.setdefault((c['batch'], c['pose']), []).append(row)
    for (split, pose), rows in by_pose.items():
        q = np.array([r['q'] for r in rows])
        z = np.zeros_like(q)
        y = engine.regressor_batch(q, z, z)['regressor'].mean(axis=0)
        torque = np.mean([r['torque'] for r in rows], axis=0)
        spreads.append(np.ptp(np.array([r['torque'] for r in rows]), axis=0) / 2)
        batches[split].append((y, torque, pose))
    if not all(batches.values()):
        raise ValueError('需要独立的训练与验证静态姿态集')

    y = np.array([r[0] for r in batches['train']])
    target = np.array([r[1] for r in batches['train']])
    try:
        fit = fit_rigid(y, target, engine.rigid_parameters(), robust_scale=0.1)
    except RuntimeError as error:
        raise ValueError('重力数值拟合不可用：' + str(error)) from error

    vy = np.array([r[0] for r in batches['validation']])
    vt = np.array([r[1] for r in batches['validation']])
    prediction = vy @ np.asarray(fit['parameters'])
    rmse = np.sqrt(np.mean((prediction - vt) ** 2, axis=0))
    nominal_rmse = np.sqrt(np.mean((vy @ engine.rigid_parameters() - vt) ** 2, axis=0))

    candidate = model_from_parameters(engine, fit['parameters'])
    urdf_path = output / 'candidate.urdf'
    write_json(
        output / 'calibration.json',
        dict(
            stage='h1_gravity',
            parameters=fit['parameters'],
            joint_names=metadata['joint_names'],
            gravity_world=np.asarray(engine.model.gravity_world),
            source_model_sha256=metadata['source_model_sha256'],
            effort_gain=config['effort_gain'],
            effort_zero_nm=config['effort_zero_nm'],
            model_context=config.get('model_context'),
        ),
    )
    with (output / 'bodies.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['joint', 'mass_kg', 'com_x_m', 'com_y_m', 'com_z_m'])
        writer.writerows([j.name, j.body.mass, *j.body.com] for j in candidate.model.joints)

    source_path = urdf_for(config)
    aggregation = write_inertial_urdf(source_path, candidate, urdf_path)
    reload_report = check_reload(urdf_path, candidate, source_path)
    write_json(output / 'urdf_reload.json', reload_report)
    return dict(
        passed=bool(reload_report['passed']),
        fit_succeeded=True,
        stage='gravity',
        fit=fit,
        candidate_urdf=str(urdf_path.name),
        validation_rmse_nm=rmse,
        nominal_validation_rmse_nm=nominal_rmse,
        overall_validation_rmse_nm=float(np.sqrt(np.mean((prediction - vt) ** 2))),
        overall_nominal_validation_rmse_nm=float(np.sqrt(np.mean((vy @ engine.rigid_parameters() - vt) ** 2))),
        paired_approach_half_spread_nm=np.max(spreads, axis=0),
        training_poses=[r[2] for r in batches['train']],
        validation_poses=[r[2] for r in batches['validation']],
        aggregation=aggregation,
    )
