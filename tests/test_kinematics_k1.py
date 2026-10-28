"""Independent geometry checks and counterexamples for the K1 observation contract."""
from copy import deepcopy
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from casbot_calib import _core
from casbot_calib.dual_model import nominal_dual_arm
from casbot_calib.kinematics import (default_config, nominal_kinematics, apply_joint_corrections,
    geometry_manifest, calibration, ball_world, validate_config)
from casbot_calib.kinematics_checks import run_checks, urdf_reference
from casbot_calib.trical_data import (IndicatorStatus, observation_metadata, validate_observations,
    save_observations, load_observations)


def test_k1_report_independent_oracles_and_no_fake_centered_data(tmp_path, monkeypatch):
    import sys
    for name in ('pinocchio', 'coal', 'mujoco'):
        monkeypatch.setitem(sys.modules, name, None)
    report = run_checks(tmp_path / 'k1')
    assert report['passed'] and not report['calibration_performed'] and not report['collision_certified']
    arrays, metadata = load_observations(tmp_path / 'k1/protocol_example')
    assert report['protocol_example']['fully_valid_records'] == 0
    assert np.all(arrays['indicator_status'] == IndicatorStatus.NO_CONTACT)
    assert np.all(arrays['indicator_sample_ns'] == -1)
    assert np.isnan(arrays['indicator_readings_m']).all()
    assert 'truth' not in metadata
    with pytest.raises(FileExistsError):
        run_checks(tmp_path / 'k1')


def test_imported_fixed_frames_match_original_urdf_and_adding_tools_preserves_dynamics():
    nominal, _ = nominal_dual_arm(); engine, manifest = nominal_kinematics(); rng = np.random.default_rng(41)
    for _ in range(5):
        q, v, a = rng.uniform(-.5, .5, (3, 14)); reference = urdf_reference(q, manifest['joint_names'])
        for name, expected in reference.items():
            np.testing.assert_allclose(nominal.frame_pose(q, name), expected, atol=1e-13)
        np.testing.assert_allclose(engine.mass_matrix(q), nominal.mass_matrix(q), atol=0., rtol=0.)
        np.testing.assert_allclose(engine.inverse(q, v, a), nominal.inverse(q, v, a), atol=0., rtol=0.)
        np.testing.assert_allclose(engine.rigid_parameters(), nominal.rigid_parameters(), atol=0., rtol=0.)
    assert len(nominal.fk(np.zeros(14))) == 14
    with pytest.raises(ValueError, match='named frame'):
        engine.tool_jacobian(np.zeros(14))


def test_named_frames_snapshot_batch_workspace_and_fail_closed():
    engine, _ = nominal_kinematics(); q = np.linspace(-.3, .3, 14); workspace = engine.workspace()
    expected = engine.frame_pose(q, 'right_trical_tcp'); jac = engine.frame_jacobian(q, 'right_trical_tcp')
    workspace.mass_matrix(-q); workspace.frame_pose(-q, 'left_trical_tcp')
    np.testing.assert_allclose(workspace.frame_jacobian(q, 'right_trical_tcp'), jac, atol=1e-14)
    np.testing.assert_allclose(workspace.frame_pose(q, 'right_trical_tcp'), expected, atol=1e-14)
    batch = engine.frame_batch(np.vstack([q, -q]), 'right_trical_tcp')
    np.testing.assert_allclose(batch['poses'][0], expected, atol=0., rtol=0.)
    np.testing.assert_allclose(batch['jacobians'][0], jac, atol=0., rtol=0.)
    assert engine.frame_batch(np.empty((0, 14)), 'right_trical_tcp')['poses'].shape == (0, 4, 4)
    for bad in ('missing', 'universe'):
        with pytest.raises(ValueError): engine.frame_pose(q, bad)
        with pytest.raises(ValueError): engine.frame_jacobian(q, bad)
        with pytest.raises(ValueError): engine.frame_batch(np.empty((0, 14)), bad)
    with pytest.raises(ValueError): engine.frame_batch(np.zeros((2, 7)), 'right_trical_tcp')
    with pytest.raises(ValueError): engine.frame_pose(np.full(14, np.nan), 'right_trical_tcp')
    model = engine.model; frames = model.frames; frames[-1].name = frames[-2].name; model.frames = frames
    with pytest.raises(ValueError): _core.ArmDynamics(model)
    model = engine.model; frames = model.frames; frames[-1].parent = 14; model.frames = frames
    with pytest.raises(ValueError): _core.ArmDynamics(model)
    np.testing.assert_allclose(engine.frame_pose(q, 'right_trical_tcp'), expected, atol=0., rtol=0.)


def test_root_base_rotation_and_offset_in_named_frame_jacobian():
    engine, _ = nominal_kinematics(); model = engine.model
    model.base.rotation = Rotation.from_rotvec([.2, -.3, .1]).as_matrix(); model.base.translation = [.4, -.2, .7]
    placed = _core.ArmDynamics(model); q = np.linspace(-.3, .2, 14)
    np.testing.assert_allclose(placed.frame_pose(q, 'base_link'), model.base.matrix(), atol=1e-14)
    np.testing.assert_allclose(placed.frame_jacobian(q, 'base_link'), 0., atol=0.)
    nominal_pose = engine.frame_pose(q, 'left_trical_tcp')
    np.testing.assert_allclose(placed.frame_pose(q, 'left_trical_tcp'), model.base.matrix() @ nominal_pose, atol=1e-14)
    original_j = engine.frame_jacobian(q, 'left_trical_tcp')
    expected = np.vstack([model.base.rotation @ original_j[:3], model.base.rotation @ original_j[3:]])
    np.testing.assert_allclose(placed.frame_jacobian(q, 'left_trical_tcp'), expected, atol=1e-14)


def test_zero_offsets_are_applied_once_and_share_fk_dynamics_convention():
    engine, manifest = nominal_kinematics(); offsets = np.linspace(-.015, .02, 14)
    corrected = apply_joint_corrections(engine, {name: {'zero_offset_rad': offset}
        for name, offset in zip(manifest['joint_names'], offsets, strict=True)})
    q, v, a = np.random.default_rng(42).uniform(-.3, .3, (3, 14))
    for frame in ('left_trical_tcp', 'right_trical_tcp', 'left_palm_base_link', 'right_palm_base_link'):
        np.testing.assert_allclose(corrected.frame_pose(q, frame), engine.frame_pose(q + offsets, frame), atol=1e-13)
        np.testing.assert_allclose(corrected.frame_jacobian(q, frame), engine.frame_jacobian(q + offsets, frame), atol=1e-13)
    np.testing.assert_allclose(corrected.inverse(q, v, a), engine.inverse(q + offsets, v, a), atol=1e-12)
    np.testing.assert_allclose(corrected.rigid_parameters(), engine.rigid_parameters(), atol=0., rtol=0.)
    assert geometry_manifest(corrected, 'corrected')['geometry_sha256'] != manifest['geometry_sha256']
    np.testing.assert_allclose(engine.frame_pose(q, 'left_trical_tcp'), nominal_kinematics()[0].frame_pose(q, 'left_trical_tcp'), atol=0., rtol=0.)


def test_geometry_candidate_conventions_and_redundant_tilt_rejected():
    engine, manifest = nominal_kinematics(); name = manifest['joint_names'][0]; joint = engine.model.joints[0]
    axis = joint.axis; tilt = np.cross(axis, [.1, .2, .3]) * .01; translation = np.array([.001, -.002, .003])
    corrected = apply_joint_corrections(engine, {name: dict(tilt_local_rad=tilt, translation_parent_m=translation)})
    np.testing.assert_allclose(corrected.model.joints[0].origin.translation, joint.origin.translation + translation, atol=1e-15)
    np.testing.assert_allclose(corrected.model.joints[0].origin.rotation, joint.origin.rotation @ Rotation.from_rotvec(tilt).as_matrix(), atol=1e-15)
    for change in ({name: {'tilt_local_rad': axis * .01}}, {name: {'zero_offset_rad': np.nan}},
                   {name: {'unknown': 1.}}, {'unknown': {'zero_offset_rad': .01}}):
        with pytest.raises(ValueError): apply_joint_corrections(engine, change)


def test_mastered_indicator_sign_radius_and_nonorthogonal_axes():
    config = default_config(); cal = calibration(config)
    sensor = _core.ArmPose(); sensor.translation = [1., 2., 3.]
    sensor.rotation = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
    cal.zero = [.00635, .00635, .00635]
    axes = np.array([[-1., 0., 0.], [.02, 1., 0.], [0., .03, 1.]])
    axes /= np.linalg.norm(axes, axis=1)[:, None]; cal.axes = axes
    offset = np.array([.001, -.002, -.003]); ball = [1.002, 2.001, 2.997]
    readings = _core.trical_readings(sensor, ball, cal)
    np.testing.assert_allclose(readings, axes @ offset + cal.zero, atol=1e-14)
    np.testing.assert_allclose(_core.trical_offset(readings, cal), offset, atol=1e-14)
    np.testing.assert_allclose(_core.trical_readings(sensor, sensor.translation, cal), cal.zero, atol=0., rtol=0.)
    cal.axes = np.zeros((3, 3))
    with pytest.raises(ValueError): _core.trical_offset(readings, cal)
    cal = calibration(config); sensor.rotation = np.diag([1., 1., -1.])
    with pytest.raises(ValueError): _core.trical_readings(sensor, ball, cal)


def test_profile_provenance_fixed_ball_frame_and_invalid_overrides(tmp_path):
    config = default_config(Path(__file__).parents[1] / 'configs/kinematics_trical.yaml')
    assert config['control_period_ns'] == 2_000_000
    np.testing.assert_allclose(ball_world(config, 'ball_2'), [.35, .15, .25], atol=1e-15)
    bad = deepcopy(config); bad['fixture']['balls'][3]['center_m'][0] = 0.
    with pytest.raises(ValueError, match='span'): validate_config(bad)
    bad = deepcopy(config); bad['urdf_sha256'] = 'wrong'
    with pytest.raises(ValueError, match='source'): nominal_kinematics(bad)
    bad = deepcopy(config); bad['tools'][1]['parent_frame'] = 'unknown'
    with pytest.raises(ValueError): nominal_kinematics(bad)
    path = tmp_path / 'bad.yaml'; path.write_text('profile: trical_nominal_v1\ncontrol_period_ns: 2000000.0\n')
    with pytest.raises(ValueError): default_config(path)
    with pytest.raises(ValueError): ball_world(config, 'missing')


def example():
    config = default_config(); _, manifest = nominal_kinematics(); count = 3
    t = np.arange(count, dtype=np.int64) * 20_000_000 + 2**54
    arrays = dict(q_rad=np.zeros((count, 14)), dq_rad_s=np.zeros((count, 14)), joint_sequence=np.arange(count, dtype=np.int64),
        joint_sample_ns=t, joint_receive_ns=t + 100_000, joint_sample_known=np.ones(count, dtype=bool),
        indicator_readings_m=np.zeros((count, 3)), indicator_sequence=np.tile(np.arange(count, dtype=np.int64)[:, None], (1, 3)),
        indicator_sample_ns=np.full((count, 3), -1, dtype=np.int64), indicator_receive_ns=np.tile((t + 200_000)[:, None], (1, 3)),
        indicator_sample_known=np.zeros((count, 3), dtype=bool), indicator_status=np.zeros((count, 3), dtype=np.uint8),
        ball_id=np.array(['ball_1'] * count), tool_frame=np.array(['left_trical_tcp'] * count))
    metadata = observation_metadata(manifest, config, run_id='protocol_test', source='test_fixture_not_acquisition',
        clocks=dict(joint_sample='host', indicator_sample=['unknown'] * 3, receive='host'))
    return arrays, metadata


def test_raw_schema_roundtrip_integer_precision_faults_and_digest(tmp_path):
    arrays, metadata = example(); arrays['indicator_status'][1, 2] = IndicatorStatus.DISCONNECTED
    arrays['indicator_readings_m'][1, 2] = np.nan
    summary = save_observations(tmp_path / 'raw', arrays, metadata)
    assert summary['fully_valid_records'] == 2 and not summary['synchronized']
    loaded, _ = load_observations(tmp_path / 'raw')
    np.testing.assert_array_equal(loaded['joint_sample_ns'], arrays['joint_sample_ns'])
    with pytest.raises(FileExistsError): save_observations(tmp_path / 'raw', arrays, metadata)
    path = tmp_path / 'raw/observations.npz'; path.write_bytes(path.read_bytes() + b'tampered')
    with pytest.raises(ValueError, match='digest'): load_observations(tmp_path / 'raw')


@pytest.mark.parametrize('fault', ['units', 'order', 'float_time', 'unknown_time', 'clock', 'range', 'nan', 'status', 'ball', 'channel', 'causality'])
def test_invalid_observation_contracts_rejected(fault):
    arrays, metadata = example()
    if fault == 'units': metadata['units']['position'] = 'mm'
    elif fault == 'order': metadata['joint_names'] = list(reversed(metadata['joint_names']))
    elif fault == 'float_time': arrays['joint_sample_ns'] = arrays['joint_sample_ns'].astype(float)
    elif fault == 'unknown_time': arrays['indicator_sample_ns'][0, 0] = 100
    elif fault == 'clock': arrays['indicator_sample_known'][0, 0] = True; arrays['indicator_sample_ns'][0, 0] = 100
    elif fault == 'range': arrays['indicator_readings_m'][0, 0] = .006
    elif fault == 'nan': arrays['indicator_readings_m'][0, 0] = np.nan
    elif fault == 'status': arrays['indicator_status'][0, 0] = 255
    elif fault == 'ball': arrays['ball_id'][0] = 'missing'
    elif fault == 'channel': metadata['indicator_channels'] = ['z', 'y', 'x']
    elif fault == 'causality': arrays['joint_sample_ns'][0] = arrays['joint_receive_ns'][0] + 1
    with pytest.raises(ValueError): validate_observations(arrays, metadata)
