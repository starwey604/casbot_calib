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
    from .rigid_identification import fit_rigid, model_from_parameters, parameter_scales
    from .dynamic_export import write_inertial_urdf, check_reload

    output = Path(output)
    engine, _ = model_for(config)
    model = engine.model
    model.gravity_world = np.asarray(config.get('gravity_world', model.gravity_world))
    engine = type(engine)(model)

    by_pose = {}
    train_rows = []
    for row in means:
        if row.get('q') is None or row.get('torque') is None:
            continue
        c = row['conditions']
        by_pose.setdefault((c['batch'], c['pose']), []).append(row)
        if c['batch'] == 'train':
            train_rows.append(row)

    spreads = []
    train_poses = []
    val_batches = []
    for (split, pose), rows in by_pose.items():
        spreads.append(np.ptp(np.array([r['torque'] for r in rows], dtype=float), axis=0) / 2.0)
        if split == 'train':
            train_poses.append(pose)
        elif split == 'validation':
            q_v = np.array([r['q'] for r in rows], dtype=float)
            z_v = np.zeros_like(q_v)
            y_pose = engine.regressor_batch(q_v, z_v, z_v)['regressor'].mean(axis=0)
            t_pose = np.mean([r['torque'] for r in rows], axis=0)
            val_batches.append((y_pose, t_pose, pose))

    if not train_rows or not val_batches:
        raise ValueError('需要独立的训练与验证静态姿态集')

    w_tr = len(train_rows)
    q_train = np.array([r['q'] for r in train_rows], dtype=float)
    tau_train = np.array([r['torque'] for r in train_rows], dtype=float)
    s_train = np.zeros((w_tr, 14), dtype=float)
    w_train = np.ones((w_tr, 14), dtype=float)
    stuck_train = np.zeros((w_tr, 14), dtype=bool)
    thresh = float(np.deg2rad(0.3))

    for idx, row in enumerate(train_rows):
        pa = row.get('paired_approach')
        if pa and pa.get('available') and pa.get('observed_displacement_rad') is not None:
            dq_obs = np.asarray(pa['observed_displacement_rad'], dtype=float)
            dq_ref = np.asarray(pa['reference_displacement_rad'], dtype=float)
            for j in range(14):
                if abs(dq_obs[j]) >= thresh:
                    s_train[idx, j] = float(np.sign(dq_obs[j]))
                elif abs(dq_ref[j]) >= thresh:
                    stuck_train[idx, j] = True
                    s_train[idx, j] = 0.0
                    w_train[idx, j] = 0.5
        else:
            app = float(row['conditions'].get('approach', 0))
            act = row['conditions'].get('active_joints', range(14))
            for j in act:
                s_train[idx, j] = -float(np.sign(app))

    y_win = engine.regressor_batch(q_train, np.zeros_like(q_train), np.zeros_like(q_train))['regressor']
    extra_tr = (np.eye(14)[None, :, :] * s_train[:, :, None]).reshape(-1, 14)
    prior = engine.rigid_parameters()
    auxiliary = (extra_tr, np.zeros(14), np.zeros(14), np.full(14, 3.0))
    fixed_scales = np.r_[parameter_scales(prior), np.ones(14)]

    vy = np.array([r[0] for r in val_batches], dtype=float)
    vt = np.array([r[1] for r in val_batches], dtype=float)

    try:
        fit = fit_rigid(
            y_win.reshape(-1, 140),
            tau_train.ravel(),
            prior,
            auxiliary=auxiliary,
            fixed_scales=fixed_scales,
            sample_weights=w_train.ravel(),
            robust_scale=0.1,
            ridge_grid=[1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 5e-1],
            validation_data=(vy.reshape(-1, 140), vt.ravel()),
        )
    except RuntimeError as error:
        raise ValueError('重力数值拟合不可用：' + str(error)) from error

    static_hysteresis_nm = list(fit.get('auxiliary_parameters', fit.get('auxiliary', [0.0] * 14)))
    fit.setdefault('auxiliary', static_hysteresis_nm)
    deadband_stuck_count = int(np.sum(stuck_train))

    prediction = vy @ np.asarray(fit['parameters'])
    rmse = np.sqrt(np.mean((prediction - vt) ** 2, axis=0))
    nominal_rmse = np.sqrt(np.mean((vy @ prior - vt) ** 2, axis=0))
    overall_val_rmse = float(np.sqrt(np.mean((prediction - vt) ** 2)))
    overall_nom_rmse = float(np.sqrt(np.mean((vy @ prior - vt) ** 2)))

    active_set = set()
    for row in means:
        if row.get('q') is not None and row.get('torque') is not None:
            acts = row.get('conditions', {}).get('active_joints')
            if acts is not None:
                active_set.update(int(j) for j in acts)
    if not active_set:
        active_set = set(range(len(metadata['joint_names'])))

    candidate = model_from_parameters(engine, fit['parameters'])
    urdf_path = output / 'candidate.urdf'
    source_path = urdf_for(config)
    aggregation = write_inertial_urdf(source_path, candidate, urdf_path)
    reload_report = check_reload(urdf_path, candidate, source_path)
    write_json(output / 'urdf_reload.json', reload_report)

    from .stages.dynamics.common import evaluate_stage_acceptance
    acceptance = evaluate_stage_acceptance(
        overall_nom_rmse,
        overall_val_rmse,
        axis_rows=[
            (metadata['joint_names'][j], float(nominal_rmse[j]), float(rmse[j]))
            for j in sorted(active_set)
        ],
        reload_passed=bool(reload_report['passed']),
    )

    write_json(
        output / 'calibration.json',
        dict(
            stage='h1_gravity',
            accepted=acceptance['accepted'],
            acceptance=acceptance,
            parameters=fit['parameters'],
            static_hysteresis_nm=static_hysteresis_nm,
            deadband_stuck_count=deadband_stuck_count,
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

    return dict(
        passed=bool(reload_report['passed']),
        fit_succeeded=True,
        accepted=acceptance['accepted'],
        acceptance=acceptance,
        stage='gravity',
        fit=fit,
        candidate_urdf=str(urdf_path.name),
        static_hysteresis_nm=static_hysteresis_nm,
        deadband_stuck_count=deadband_stuck_count,
        validation_rmse_nm=rmse,
        nominal_validation_rmse_nm=nominal_rmse,
        overall_validation_rmse_nm=overall_val_rmse,
        overall_nominal_validation_rmse_nm=overall_nom_rmse,
        paired_approach_half_spread_nm=np.max(spreads, axis=0),
        training_poses=train_poses,
        validation_poses=[r[2] for r in val_batches],
        aggregation=aggregation,
    )
