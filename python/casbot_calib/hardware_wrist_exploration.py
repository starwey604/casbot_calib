"""Local-only SDK wrist experiment, separate from qualified H4/H3 acceptance.

Collection uses the ordinary field executor. Analysis uses only its public SDK
feedback; the declared nominal/friction fixture is not an accepted H3 result.
"""
import json
from pathlib import Path
import numpy as np
import yaml
from .data import write_json
from . import hardware_field as field
from .hardware_inertia import settings, gravity_constraints
from .hardware_friction import holding_gains
from .hardware_periodic import periodic_problem, prediction_metrics
from .dynamics_problem import _project
from .model_runtime import model_for
from .rigid_identification import fit_rigid, model_from_parameters
from .staged_motion import periodic_motion, derivative_bounds
from .sdk_connection import connection_remaps
from .sdk_mock import SDKMockSession
from .run_marker import run_marker

WRISTS = [4, 5, 6, 11, 12, 13]
FLAGS = dict(physical_device_tested=False, sample_time_known=False,
             sampling_qualification_passed=False, formal_hardware_calibration_accepted=False,
             dedicated_high_bandwidth_wrist_validation_passed=False)


def read(path):
    return json.loads(Path(path).read_text())


def require_local(profile, domain):
    """Never permit the hardware/default endpoint route from this experiment."""
    if profile.get('connection', {}).get('scope') != 'localhost_contract_mock':
        raise ValueError('Wrist experiment requires an isolated local SDK Mock connection')
    return connection_remaps(profile, domain)


def design(output, probes):
    root = Path(output).resolve(); root.mkdir(parents=True, exist_ok=False)
    protocol = root/'protocol'; protocol.mkdir()
    mock = yaml.safe_load(Path('configs/sdk_mock.yaml').read_text())
    # An explicitly declared fixture prior, NOT an accepted/forged H3 batch.
    friction = dict(viscous=[.01]*14, coulomb=[.005]*14, static_friction=[.01]*14)
    mock['plant'].update(stiction_enabled=True, sample_period_ns=1000000,
                         friction_viscous=friction['viscous'], friction_coulomb=friction['coulomb'],
                         friction_static=friction['static_friction'], inertia_scales=[1.8]*14)
    from .hardware_probe import settings as probe_settings
    cache_standard = probe_settings()['cache_period_ns']
    base = yaml.safe_load(Path('configs/hardware_inertia.yaml').read_text())
    engine, _ = model_for(mock); names = [j.name for j in engine.model.joints]
    base.update(joints=[names[j] for j in WRISTS], periods=3, discard_periods=1)
    base['control']['enabled'] = False
    base['holding']['hold_minimum_kp'] = 30.  # Synthetic stiction fixture only.
    path = protocol/'h4.yaml'; path.write_text(yaml.safe_dump(base))
    config = settings(path, names)
    home = np.asarray(config['home_rad']); poses = {}
    for name, pitch, roll in [('c00', .20, .30), ('c01', -.20, .15), ('validation', 0., .225)]:
        pose = home.copy(); pose[[5, 12]] = pitch; pose[[6, 13]] = roll; poses[name] = pose.tolist()
    jobs = []
    for batch, train_seed, val_seed in [('b00', 401, 501), ('b01', 402, 502)]:
        for center in ['c00', 'c01', 'validation']:
            jobs.append(dict(id=f'{center}_{batch}', center=center, batch=batch,
                             split='validation' if center=='validation' else 'train',
                             seed=val_seed if center=='validation' else train_seed))
    with np.load(probes, allow_pickle=False) as data:
        arrays = {k: data[k] for k in ('q', 'dq', 'ddq')}
    if any(v.shape != (128, 14) or not np.isfinite(v).all() for v in arrays.values()):
        raise ValueError('Original 128 finite public operator probes required')
    np.savez_compressed(protocol/'operator_probes.npz', **arrays)
    variations = [dict(id='baseline', harmonics=config['harmonics'])]
    variations += [dict(id=f'effort_shift_{s:+g}ms', harmonics=config['harmonics'], shift_s=s*.001)
                   for s in [-15, -5, 5, 15]]
    variations += [dict(id=f'effort_lowpass_{s}ms', harmonics=config['harmonics'], effort_tau_s=s*.001)
                   for s in [5, 20]]
    variations += [dict(id=f'band_{h}', harmonics=h) for h in [8, 12]]
    record = dict(schema_version='casbot_local_sdk_wrist_exploration_v1', h4=config,
                  poses=poses, jobs=jobs, friction_fixture_prior=friction, variations=variations,
                  cases=[dict(id='cache_3ms', cache_period_ns=3000000),
                         dict(id='cache_5ms', cache_period_ns=cache_standard)],
                  operator_probe_source=str(Path(probes).resolve()), operator_probe_count=128,
                  comparison_mass_diagonal_floor_kg_m2=1e-6, diagnostic_threshold=.10,
                  prior_status='declared synthetic fixture; H3 acceptance not tested or granted',
                  sensitivity_used_for_selection=False, command_control_comparison=False,
                  preprocessing='existing H4 callback-time Fourier projection, not 4kHz sustained wrist processing',
                  **FLAGS, **run_marker())
    write_json(protocol/'experiment.json', record)
    for case in record['cases']:
        cfg = dict(mock, cache_period_ns=case['cache_period_ns'])
        cfg['assumptions'] = dict(mock['assumptions'], cache=f"synthetic fixed {case['cache_period_ns']}ns shared cache, NOT measured device behavior")
        (protocol/(case['id']+'.yaml')).write_text(yaml.safe_dump(cfg))
    field.seal(protocol)
    # Check the actual reference envelopes before opening any Mock endpoint.
    for job in jobs:
        make_motion(home, np.asarray(poses[job['center']]), config, job)
    return record


def make_motion(start, pose, config, job):
    amplitude = np.zeros(14); amplitude[WRISTS] = np.asarray(config['amplitude_rad'])[WRISTS]
    recipe = dict(dynamic_period_s=config['period_s'], dynamic_periods=config['periods'],
                  pose_amplitude_rad=amplitude, settle_s=config['settle_s'],
                  dynamic_transition_s=config['transition_s'], smooth_periodic_envelope=True,
                  dynamic_ramp_s=config['excitation_ramp_s'])
    motion = periodic_motion(job['id'], 'inertia', start, pose, recipe, job['seed'],
                             job['split']!='train', 'both', dict(job, refine_position_bounds=True))
    curve = motion.polynomial()
    if (np.max(derivative_bounds(curve, 2)) > config['maximum_acceleration_rad_s2'] or
        np.max(derivative_bounds(curve, 3)) > config['maximum_jerk_rad_s3']):
        raise ValueError('Original H4 continuous acceleration/jerk envelope exceeded')
    return motion


def collect(root, domain):
    """No fault resume: any failed snapshot/execution ends this invocation."""
    root = Path(root).resolve(); field.check_seal(root/'protocol')
    protocol = read(root/'protocol/experiment.json'); c = protocol['h4']
    for case in protocol['cases']:
        folder = root/case['id']; folder.mkdir()
        with SDKMockSession(root/'protocol'/(case['id']+'.yaml'), folder/'mock', domain) as session:
            profile = field.load_profile(session.client_profile); require_local(profile, domain)
            for job in protocol['jobs']:
                trial = folder/job['id']; trial.mkdir()
                print(f"COLLECT {case['id']} {job['id']}", flush=True)
                result = field.snapshot_field(session.client_profile, trial/'snapshot', domain)
                if not result['passed']: raise RuntimeError('Snapshot failed; entire experiment terminated')
                start = field._read_snapshot(trial/'snapshot'); pose = np.asarray(protocol['poses'][job['center']])
                motion = make_motion(start, pose, c, job)
                kp, kd = holding_gains(pose, c['holding'], profile)
                profile = dict(profile, breakaway_parameters=dict(command_kp=kp.tolist(), command_kd=kd.tolist()))
                require_local(profile, domain)
                plan = trial/'plan'; plan.mkdir()
                field.finish_plan(plan, profile, motion, [], dict(**field.snapshot_binding(trial/'snapshot', profile),
                    local_exploration=True, fixture_prior_not_h3_accepted=True, job=job))
                result = field.execute_field(plan, trial/'capture', domain)
                if not result['passed']: raise RuntimeError('SDK execution fault; no automatic recovery or resume')
                write_json(trial/'completed.json', dict(passed=True, **FLAGS))
    write_json(root/'collection.json', dict(completed=True, records=len(protocol['cases'])*len(protocol['jobs']), **FLAGS))


def transfer(signal, period, *, shift_s=0., tau_s=0.):
    """Periodic post-projection perturbation: advance or causal effort low-pass.

    These are imposed stress tests, never estimates of physical SDK delay/filter.
    """
    signal = np.asarray(signal); freq = np.fft.rfftfreq(len(signal), period/len(signal))
    gain = np.exp(2j*np.pi*freq*shift_s)/(1+2j*np.pi*freq*tau_s)
    return np.fft.irfft(np.fft.rfft(signal, axis=0)*gain[:, None], n=len(signal), axis=0)


def perturb(problem, period, variation):
    p = dict(problem)
    p['observed_torque_nm'] = transfer(problem['observed_torque_nm'], period,
        shift_s=variation.get('shift_s', 0.), tau_s=variation.get('effort_tau_s', 0.))
    p['target'] = p['observed_torque_nm']-p['rotor']-p['friction']
    return p


def mass_diagonal(engine, q):
    return np.array([np.diag(engine.mass_matrix(row)) for row in q])


def mass_difference(reference, variant, floor):
    relative = abs(variant-reference)/np.maximum(reference, floor)
    axis = np.max(relative, axis=0); point, joint = np.unravel_index(np.argmax(relative), relative.shape)
    return dict(maximum_relative=float(relative[point, joint]), axis_maximum_relative=axis.tolist(),
                worst_point=int(point), worst_joint=int(joint), wrist_maximum_relative=float(np.max(axis[WRISTS])))


def analyse(root, output):
    """Read public SDK captures only; all fitting/sensitivity inputs preregistered."""
    root, out = Path(root).resolve(), Path(output).resolve()
    field.output_outside(out, root/'protocol'); out.mkdir(parents=True, exist_ok=False)
    field.check_seal(root/'protocol'); p = read(root/'protocol/experiment.json'); c = p['h4']
    with np.load(root/'protocol/operator_probes.npz', allow_pickle=False) as data: qprobe = data['q']
    # Keep the original P0/R2 denominator floor. A v1 local design used
    # 1e-4 by mistake; preserve it but correct the read-only comparison, with
    # both values in the analysis report. Motion/observations are unchanged.
    floor = 1e-6
    engine, _ = model_for(c); prior = engine.rigid_parameters(); equalities = gravity_constraints(engine, c['home_rad'])
    names = [j.name for j in engine.model.joints]; records = {}; public_audit = []
    # Cache baseline projections by declared bandwidth. Never infer acquisition
    # rates from equal-value runs, nor drop equal-valued publications.
    for case in p['cases']:
        for job in p['jobs']:
            folder = root/case['id']/job['id']
            plan,captured=field.resolve_field_capture(folder/'plan',folder/'capture')
            meta, commands, origin, obs, elapsed, collision = field.field_observations(plan,captured)
            require_local(meta['config'], read(captured/'request.json')['domain_id'])
            motion = read(plan/'motion.json'); problems = {}
            for h in sorted(set(v['harmonics'] for v in p['variations'])):
                problems[h] = periodic_problem(obs, elapsed, motion, engine, dict(c, harmonics=h),
                    meta['config'], p['friction_fixture_prior'])
            records[(case['id'], job['id'])] = problems
            baseline = problems[c['harmonics']]
            windows = [(motion['knots_s'][i], motion['knots_s'][i+1]) for i,l in enumerate(motion['labels']) if l=='periodic']
            retained = (elapsed>=windows[0][0]+c['discard_periods']*c['period_s']) & (elapsed<windows[-1][1])
            values = np.column_stack([obs[k][retained] for k in ('q', 'dq', 'effort_raw')])
            nonactive = [j for j in range(14) if j not in WRISTS]
            public_audit.append(dict(case=case['id'], job=job, preprocessing=baseline['preprocessing'],
                identical_adjacent_public_value_fraction=float(np.mean(np.all(values[1:]==values[:-1],axis=1))),
                identical_values_are_not_frame_age_evidence=True, independent_acquisition_count=None,
                maximum_other_axes_speed_rad_s=float(np.max(abs(obs['dq'][retained][:,nonactive]))),
                wrist_velocity_rms_rad_s=np.sqrt(np.mean(baseline['dq'][:,WRISTS]**2,axis=0)).tolist(),
                wrist_acceleration_rms_rad_s2=np.sqrt(np.mean(baseline['ddq'][:,WRISTS]**2,axis=0)).tolist(),
                wrist_observed_effort_rms_nm=np.sqrt(np.mean(baseline['observed_torque_nm'][:,WRISTS]**2,axis=0)).tolist(),
                wrist_dynamic_torque_rms_nm=np.sqrt(np.mean((baseline['observed_torque_nm']-baseline['friction']-
                    _project(engine.gravity_batch(baseline['q']), c['harmonics']))[:,WRISTS]**2,axis=0)).tolist(),
                received_collision_passed=collision['passed']))
    write_json(out/'public_capture_audit.json', public_audit)
    reports = []; baseline_masses = {}
    groups = dict(full=lambda j:True, delete_c00=lambda j:j['center']!='c00',
                  delete_c01=lambda j:j['center']!='c01', delete_b00=lambda j:j['batch']!='b00',
                  delete_b01=lambda j:j['batch']!='b01')
    for case in p['cases']:
        for variation in p['variations']:
            problems = {job['id']:perturb(records[(case['id'],job['id'])][variation['harmonics']], c['period_s'], variation)
                        for job in p['jobs']}
            row = dict(case=case['id'], variation=variation, fits={}, all_fits_completed=False, deletion_within_10_percent=False, **FLAGS)
            full_mass = None
            for group, include in groups.items():
                train = [j for j in p['jobs'] if j['split']=='train' and include(j)]
                features, target = [], []
                for job in train:
                    prob = problems[job['id']]; mask = np.zeros_like(prob['observed_torque_nm'],dtype=bool); mask[:,WRISTS] = True
                    n = int(mask.sum())
                    if n==0: raise ValueError('No numerical SDK phase rows')
                    weight = np.sqrt(c['phase_points']*len(WRISTS)/n)
                    features.append(prob['regression'][mask]*weight); target.append(prob['target'][mask]*weight)
                try:
                    fit = fit_rigid(np.vstack(features), np.concatenate(target), prior, equalities=equalities,
                        ridge=c['ridge'], physical_length_scale=.1, lift_residuals=True)
                    theta = np.asarray(fit['parameters']); candidate = model_from_parameters(engine, theta)
                    mass = mass_diagonal(candidate, qprobe)
                    validations = {j['id']:prediction_metrics(problems[j['id']], theta, WRISTS, c)
                                   for j in p['jobs'] if j['split']=='validation'}
                    f = dict(completed=True, fit=fit, validation=validations,
                             independent_prediction_available=all(v['available'] for v in validations.values()))
                    if group=='full':
                        full_mass = mass
                        if variation['id']=='baseline': baseline_masses[case['id']] = mass
                    else:
                        f['operator_difference_from_full'] = mass_difference(full_mass, mass, floor)
                    row['fits'][group] = f
                except (ValueError, RuntimeError) as exc:
                    row['fits'][group] = dict(completed=False, error=str(exc))
                    if group=='full': break
            if full_mass is not None:
                reference = baseline_masses.get(case['id'])
                row['perturbation_difference_from_baseline'] = (mass_difference(reference, full_mass, floor)
                    if reference is not None else dict(available=False, reason='baseline_fit_rejected'))
                completed = len(row['fits'])==len(groups) and all(f['completed'] for f in row['fits'].values())
                row['all_fits_completed'] = completed
                row['deletion_within_10_percent'] = bool(completed and all(
                    f['operator_difference_from_full']['maximum_relative']<=p['diagnostic_threshold']
                    for g,f in row['fits'].items() if g!='full'))
            reports.append(row)
            write_json(out/'progress.json', dict(completed_variations=len(reports), total_variations=len(p['cases'])*len(p['variations'])))
            print('ANALYSED',case['id'],variation['id'],flush=True)
    summary = dict(schema_version='casbot_local_sdk_wrist_exploration_report_v2',empirical_quality_gate_applied=False,
        cases=reports, records=len(public_audit), operator_probe_count=len(qprobe), joint_names=names,
        diagnostic_threshold=p['diagnostic_threshold'], prior_status=p['prior_status'],
        all_variations_predeclared=True, validation_used_for_selection=False,
        cache_comparison_cases=[case['id'] for case in p['cases']],
        cache_case_difference=(mass_difference(baseline_masses[p['cases'][0]['id']], baseline_masses[p['cases'][1]['id']], floor)
                               if len(baseline_masses)==2 else dict(available=False, reason='baseline_fit_rejected')),
        comparison_mass_diagonal_floor_kg_m2=floor,
        source_protocol_mass_diagonal_floor_kg_m2=p['comparison_mass_diagonal_floor_kg_m2'],
        analysis_floor_correction_before_fitting=p['comparison_mass_diagonal_floor_kg_m2']!=floor,
        no_h3_acceptance_or_control_comparison_claimed=True, **FLAGS)
    write_json(out/'report.json', summary); field.seal(out)
    return summary
