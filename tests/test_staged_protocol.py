"""P0 R0 contracts; manufactured tiny recordings are not dynamics acceptance."""
from copy import deepcopy
import json
from pathlib import Path
import shutil
import numpy as np
import pytest
import yaml
from casbot_calib.data import write_json
from casbot_calib.staged_protocol import (
    IDENTITY, authorize_role, build_split_manifest, digest, file_hash,
    freeze_candidate, load_frozen_candidate, load_protocol, p0_status,
    prepare_protocol, role_recordings, seal_role_records, validate_split_manifest,
)


@pytest.fixture(scope='module')
def config():
    return yaml.safe_load(Path('configs/staged_dual_arm_p0.yaml').read_text())


@pytest.fixture(scope='module')
def base_root(tmp_path_factory, config):
    root = tmp_path_factory.mktemp('p0') / 'contract'
    report = prepare_protocol(root, config)
    assert report['contract_passed'] and not report['passed']
    return root


@pytest.fixture
def root(tmp_path, base_root):
    destination = tmp_path / 'session'
    shutil.copytree(base_root, destination)
    return destination


def read(path):
    return json.loads(Path(path).read_text())


def tiny_records(root, splits):
    """Stand-in R1 artifacts for exercising I/O contracts, no physics claim."""
    from casbot_calib.staged_motion import Motion
    protocol, config, manifest = load_protocol(root)
    names = read(root / 'session.json')['joint_names']
    slots = [r for r in manifest['trials'] if r['split'] in splits]
    selected = read(root / 'plan_report.json')['selected_trials']
    write_json(root / 'plan_report.json', dict(passed=True, selected_trials=selected + [r['trial_id'] for r in slots]))
    for slot in slots:
        name = slot['trial_id']; plan = root / 'plans' / name; run = root / 'runs' / name
        plan.mkdir(parents=True); run.mkdir(parents=True)
        conditions = {k: slot[k] for k in IDENTITY}
        motion = Motion(name, slot['stage'], np.asarray(config['home_rad']), conditions)
        motion.add(.01, motion.home)
        write_json(plan / 'motion.json', motion.metadata())
        curve = motion.polynomial()
        np.savez_compressed(plan / 'reference.npz', coefficients=curve.c, breakpoints=curve.x)
        write_json(plan / 'collision_reference.json', dict(passed=True, scope='TEST FIXTURE ONLY'))
        write_json(plan / 'manifest.json', dict(reference_gate_passed=True,
            reference_sha256=file_hash(plan / 'reference.npz'), motion_sha256=file_hash(plan / 'motion.json'),
            collision_sha256=file_hash(plan / 'collision_reference.json')))
        count = 8; shape = (count, 14)
        sample = np.arange(count, dtype=np.int64) * config['inertia_sample_period_ns']
        np.savez_compressed(run / 'observations.npz', sample_ns=sample, receive_ns=sample + 700000,
            sequence=np.arange(count, dtype=np.int64), q=np.zeros(shape), dq=np.zeros(shape),
            current=np.zeros(shape), status=np.zeros(shape, dtype=np.uint32))
        np.savez_compressed(run / 'commands.npz', send_ns=np.array([0], dtype=np.int64), commands=np.zeros((1, 14, 5)))
        write_json(run / 'feedforward.json', {})
        write_json(run / 'collision_actual.json', dict(passed=True, scope='TEST FIXTURE ONLY'))
        meta = dict(schema_version=3, passed=True, stage=slot['stage'], conditions=conditions,
            sample_period_ns=config['inertia_sample_period_ns'], control_period_ns=config['control_period_ns'],
            native_binary_sha256=protocol['native_binary_sha256'], joint_names=names,
            units=dict(q='rad', dq='rad/s', current='A', time='int64 ns'),
            raw_sha256=file_hash(run / 'observations.npz'), command_sha256=file_hash(run / 'commands.npz'),
            feedforward_sha256=file_hash(run / 'feedforward.json'), protocol_sha256=file_hash(root / 'protocol.json'))
        if slot['split'] == 'test' and (root / 'candidate_freeze.json').exists():
            meta['candidate_freeze_sha256'] = file_hash(root / 'candidate_freeze.json')
        write_json(run / 'metadata.json', meta)


def selection_fixture(root, *, score_gap=0., failed=False):
    from casbot_calib.dual_model import nominal_dual_arm
    tiny_records(root, ('train', 'selection'))
    for split in ('train', 'selection'):
        seal_role_records(root, split)
    model, source = nominal_dual_arm()
    bundle = dict(model_version='tree_full_physical_inertia_v1', source_sha256=source['sha256'],
        joint_names=[j.name for j in model.model.joints], gain_status='synthetic_declared',
        parameters=model.rigid_parameters().tolist(), viscous=[.035] * 14,
        coulomb=[.04] * 14, static_friction=[.07] * 14)
    inputs = {s: file_hash(root / f'role_inputs/{s}.json') for s in ('train', 'selection')}
    report = dict(schema_version=1, protocol_sha256=file_hash(root / 'protocol.json'), input_manifests=inputs, candidates=[])
    (root / 'candidates').mkdir(); (root / 'eligibility').mkdir()
    for index, ridge in enumerate((1e-5, 1e-4, 1e-3)):
        path = f'candidates/{index}.json'
        write_json(root / path, {**bundle, 'ridge': ridge})
        evidence = {}
        for kind in ('stability', 'prior_development', 'attribution'):
            ev_path = f'eligibility/{index}_{kind}.json'; evidence[kind] = ev_path
            write_json(root / ev_path, dict(schema_version=1, kind=kind, protocol_sha256=report['protocol_sha256'],
                input_manifests=inputs if kind=='attribution' else {'train':inputs['train']}, candidate_sha256=file_hash(root / path), completed=True, passed=not failed,
                prediction_passed=not failed, qualified_band_non_degradation_passed=not failed,
                scope='manufactured contract fixture, not R2/R3 acceptance'))
        report['candidates'].append(dict(ridge=ridge, candidate=path, candidate_sha256=file_hash(root / path),
            selection_axis_nrmse=np.full((8, 14), .02 + index * score_gap).tolist(), evidence=evidence))
    write_json(root / 'selection.json', report)
    return report


def test_manifest_counts_roles_and_deterministic_schedule(config):
    manifest = build_split_manifest(config)
    assert manifest == build_split_manifest(config)
    assert len(manifest['trials']) == 88  # 32 main periodic + 56 wrist scans
    assert [sum(r['split'] == s and r['stage'] == 'inertia' for r in manifest['trials']) for s in ('train', 'selection', 'test')] == [12, 8, 12]
    assert [sum(r['split'] == s and r['stage'] == 'inertia_probe' for r in manifest['trials']) for s in ('train', 'selection', 'test')] == [24, 16, 16]
    assert all(r['center_rad'] is None and r['reference_sha256'] is None for r in manifest['trials'])


@pytest.mark.parametrize('fault', ['duplicate_id', 'duplicate_seed', 'missing_batch', 'center_leak', 'seed_leak', 'duplicate_order', 'wrong_group', 'wrong_wrist'])
def test_manifest_rejects_leakage_and_missing_evidence(config, fault):
    manifest = build_split_manifest(config); rows = manifest['trials']
    if fault == 'duplicate_id': rows[1]['trial_id'] = rows[0]['trial_id']
    elif fault == 'duplicate_seed': rows[1]['acquisition_seed'] = rows[0]['acquisition_seed']
    elif fault == 'missing_batch': rows.pop(0)
    elif fault == 'center_leak':
        for row in rows:
            if row['center_id'] == 'selection_c00': row['center_id'] = 'train_c00'
    elif fault == 'seed_leak':
        for row in rows:
            if row['reference_id'] == 'selection_c00_00': row['reference_seed'] = rows[0]['reference_seed']
    elif fault == 'duplicate_order': manifest['planned_execution_order']['train'][0] = manifest['planned_execution_order']['train'][1]
    elif fault == 'wrong_group':
        for row in rows:
            if row['reference_id'] == 'test_c00_00': row['active_arm_group'] = 'left'
    elif fault == 'wrong_wrist':
        for row in rows:
            if row['reference_id'] == 'train_c00_02': row['joint'] = 5
    with pytest.raises(ValueError): validate_split_manifest(manifest, config)


def test_public_protocol_is_truth_independent_and_is_not_motion_acceptance(root):
    before = load_protocol(root)
    shutil.rmtree(root / 'evaluation')
    assert load_protocol(root) == before
    assert 'mock' not in before[1]
    report = read(root / 'plan_report.json')
    assert not report['passed'] and not report['p0_acceptance_passed']
    assert report['reference_collision_status'] == 'not_executed'
    with pytest.raises(FileExistsError): prepare_protocol(root, before[1])


@pytest.mark.parametrize('filename', ['session.json', 'split_manifest.json', 'qualification/report.json', 'protocol.json'])
def test_changed_protocol_inputs_rejected(root, filename):
    path = root / filename
    path.write_text(path.read_text() + '\n')
    with pytest.raises(ValueError, match='changed'): load_protocol(root)


@pytest.mark.parametrize('split,purpose', [('test', 'fit'), ('test', 'select'), ('selection', 'fit'), ('train', 'select'), ('train', 'final_test'), ('test', None)])
def test_role_authorization_denies_test_in_selection(split, purpose):
    with pytest.raises(ValueError, match='Role leakage'): authorize_role(split, purpose)


def test_role_reader_does_not_open_unrequested_observations(root, monkeypatch):
    tiny_records(root, ('train',)); seal_role_records(root, 'train')
    original = Path.open
    def guarded(path, *args, **kwargs):
        if 'runs' in path.parts and path.parts[path.parts.index('runs') + 1].startswith(('test_', 'selection_')):
            pytest.fail('Reader touched an unrequested role')
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', guarded)
    rows = list(role_recordings(root, 'inertia', split='train', purpose='fit'))
    assert len(rows) == 12


@pytest.mark.parametrize('fault', ['raw', 'missing', 'repeated_sequence', 'identity', 'collision'])
def test_bad_role_records_rejected_before_analysis(root, fault):
    tiny_records(root, ('train',))
    slot = load_protocol(root)[2]['trials'][0]; run = root / 'runs' / slot['trial_id']
    if fault == 'raw': (run / 'observations.npz').write_bytes(b'changed')
    elif fault == 'missing': shutil.rmtree(run)
    elif fault == 'repeated_sequence':
        with np.load(run / 'observations.npz') as z: obs = {k: z[k] for k in z.files}
        obs['sequence'][3] = obs['sequence'][2]
        np.savez_compressed(run / 'observations.npz', **obs)
        meta = read(run / 'metadata.json'); meta['raw_sha256'] = file_hash(run / 'observations.npz'); write_json(run / 'metadata.json', meta)
    elif fault == 'identity':
        meta = read(run / 'metadata.json'); meta['conditions']['split'] = 'test'; write_json(run / 'metadata.json', meta)
    elif fault == 'collision': write_json(run / 'collision_actual.json', dict(passed=False))
    with pytest.raises((ValueError, FileNotFoundError)): seal_role_records(root, 'train')
    assert not (root / 'role_inputs/train.json').exists()


def test_post_seal_tamper_rejected(root):
    tiny_records(root, ('train',)); seal_role_records(root, 'train')
    name = load_protocol(root)[2]['trials'][0]['trial_id']
    path = root / 'runs' / name / 'metadata.json'; path.write_text(path.read_text() + '\n')
    with pytest.raises(ValueError, match='changed'): list(role_recordings(root, 'inertia', split='train', purpose='fit'))


def test_freeze_selects_stronger_regularization_on_frozen_tie_rule(root):
    selection_fixture(root)
    frozen = freeze_candidate(root, 'candidates/2.json', 'selection.json')
    assert frozen['ridge'] == 1e-3 and not frozen['test_consumed'] and not frozen['final_test_passed']
    assert load_frozen_candidate(root) == frozen
    with pytest.raises(FileExistsError): freeze_candidate(root, 'candidates/2.json', 'selection.json')
    tiny_records(root, ('test',)); seal_role_records(root, 'test')
    assert len(list(role_recordings(root, 'inertia', split='test', purpose='final_test'))) == 12
    path = root / 'candidates/2.json'; path.write_text(path.read_text() + '\n')
    with pytest.raises(ValueError, match='changed'): list(role_recordings(root, 'inertia', split='test', purpose='final_test'))


def test_freeze_uses_selection_score_not_largest_ridge(root):
    selection_fixture(root, score_gap=.001)
    with pytest.raises(ValueError, match='winner'): freeze_candidate(root, 'candidates/2.json', 'selection.json')
    assert freeze_candidate(root, 'candidates/0.json', 'selection.json')['ridge'] == 1e-5


@pytest.mark.parametrize('fault', ['failed', 'test_metric', 'test_input', 'missing_candidate', 'stale_evidence', 'seen_test'])
def test_freeze_rejects_failed_leaked_or_incomplete_selection(root, fault):
    report = selection_fixture(root, failed=fault == 'failed')
    if fault == 'test_metric': report['test_rmse'] = .00001
    elif fault == 'test_input': report['input_manifests']['test'] = '0' * 64
    elif fault == 'missing_candidate': report['candidates'].pop()
    elif fault == 'stale_evidence':
        path = root / report['candidates'][0]['evidence']['stability']; ev = read(path); ev['candidate_sha256'] = '0' * 64; write_json(path, ev)
    elif fault == 'seen_test':
        slot = next(r for r in load_protocol(root)[2]['trials'] if r['split'] == 'test')
        (root / 'runs' / slot['trial_id']).mkdir()
    write_json(root / 'selection.json', report)
    with pytest.raises(ValueError): freeze_candidate(root, 'candidates/2.json', 'selection.json')
    assert not (root / 'candidate_freeze.json').exists()


def test_test_access_and_acquisition_seal_require_freeze(root):
    with pytest.raises(FileNotFoundError): list(role_recordings(root, 'inertia', split='test', purpose='final_test'))
    with pytest.raises(FileNotFoundError): seal_role_records(root, 'test')


@pytest.mark.parametrize('field',['prediction_passed','qualified_band_non_degradation_passed','passed'])
def test_attribution_completion_alone_cannot_qualify_a_candidate(root,field):
    report=selection_fixture(root)
    for row in report['candidates']:
        path=root/row['evidence']['attribution'];ev=read(path);ev[field]=False;write_json(path,ev)
    with pytest.raises(ValueError,match='No acceptable'):freeze_candidate(root,'candidates/2.json','selection.json')


def test_freeze_pins_underlying_attribution_proof(root):
    report=selection_fixture(root);proof=root/'proof.json';write_json(proof,dict(diagnostic='immutable proof'))
    for row in report['candidates']:
        path=root/row['evidence']['attribution'];ev=read(path);ev['source_files']={'proof.json':file_hash(proof)};write_json(path,ev)
    freeze_candidate(root,'candidates/2.json','selection.json');proof.write_text('{}')
    with pytest.raises(ValueError,match='changed'):load_frozen_candidate(root)


def test_legacy_compatibility_and_p0_cannot_enter_old_pipeline(root, config, tmp_path):
    from casbot_calib.staged_identification import recordings
    from casbot_calib.staged_workflow import run_staged
    from casbot_calib.staged_analysis import identify_staged
    from casbot_calib.staged_validation import verify_staged
    assert p0_status({'schema_version': 1})['status'] == 'not_executed'
    with pytest.raises(ValueError, match='explicit split'): list(recordings(root, 'inertia'))
    for call in (lambda out: run_staged(out, config), lambda out: identify_staged(root, out), lambda out: verify_staged(root, out)):
        out = tmp_path / 'must_not_start'
        with pytest.raises(ValueError, match='contract-only'): call(out)
        assert not out.exists()


def test_legacy_record_reader_still_consumes_schema_one(root):
    from casbot_calib.staged_identification import recordings
    tiny_records(root, ('train',))
    session = read(root / 'session.json'); session['config']['schema_version'] = 1
    session['config'].pop('p0'); write_json(root / 'session.json', session)
    assert len(list(recordings(root, 'inertia'))) == 12
    with pytest.raises(ValueError, match='Legacy'): list(recordings(root, 'inertia', split='train', purpose='fit'))


def test_wrong_acquisition_kernel_and_sampling_refused(root):
    tiny_records(root, ('train',))
    name = load_protocol(root)[2]['trials'][0]['trial_id']; path = root / 'runs' / name / 'metadata.json'
    meta = read(path); meta['native_binary_sha256'] = '0' * 64; write_json(path, meta)
    with pytest.raises(ValueError, match='kernel'): seal_role_records(root, 'train')


def test_fourier_energy_independent_analytic_oracle_and_noise_floor(base_root):
    from casbot_calib.staged_frequency_contract import harmonic_rms
    report = read(base_root / 'qualification/report.json')
    assert report['passed'] and not report['hardware_bandwidth_qualified']
    threshold = np.asarray(report['threshold_bin_rms_nm'])
    with np.load(base_root / 'qualification/noise.npz') as z: noise = z['torque_nm']
    assert threshold.shape == (40, 14)
    assert np.all(threshold >= 10 * np.max(harmonic_rms(noise, 40), axis=0))
    phase = np.arange(1024) * 2 * np.pi / 1024
    signal = (2 + 3 * np.sin(3 * phase) + 4 * np.cos(5 * phase))[:, None]
    spectrum = harmonic_rms(signal, 40)
    assert spectrum[2, 0] == pytest.approx(3 / np.sqrt(2))
    assert spectrum[4, 0] == pytest.approx(4 / np.sqrt(2))
    assert float(np.sum(spectrum**2)) == pytest.approx(float(np.mean((signal - 2)**2)))
