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
    np.testing.assert_allclose(placed.frame_pose(q, 'calib_world'), model.base.matrix(), atol=1e-14)
    np.testing.assert_allclose(placed.frame_jacobian(q, 'calib_world'), 0., atol=0.)
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
    for frame in ('left_trical_tcp', 'right_trical_tcp', 'left_flange_frame', 'right_flange_frame'):
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
    np.testing.assert_allclose(ball_world(config, 'B'), config['fixture']['balls'][1]['center_m'], atol=1e-15)
    bad = deepcopy(config); bad['fixture']['balls'][3]['center_m'] = bad['fixture']['balls'][0]['center_m'].copy()
    with pytest.raises(ValueError, match='span'): validate_config(bad)
    bad = deepcopy(config); bad['urdf_sha256'] = 'wrong'
    with pytest.raises(ValueError, match='source'): nominal_kinematics(bad)
    bad = deepcopy(config); bad['tools'][1]['parent_frame'] = 'unknown'
    with pytest.raises(ValueError): nominal_kinematics(bad)
    path = tmp_path / 'bad.yaml'; path.write_text('profile: trical_nominal_v1\ncontrol_period_ns: 2000000.0\n')
    with pytest.raises(ValueError): default_config(path)
    with pytest.raises(ValueError): ball_world(config, 'missing')






