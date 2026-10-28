"""K1 correctness oracles only. This does not plan, move, acquire or calibrate."""
from hashlib import sha256
from importlib.resources import files
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation
from . import _core
from .data import write_json
from .kinematics import default_config, nominal_kinematics, calibration
from .trical_data import IndicatorStatus, observation_metadata, save_observations, load_observations


def urdf_reference(q, joint_names):
    """Independent XML transform oracle, confined to verification (not runtime FK)."""
    source = files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf')
    root = ET.fromstring(source.read_bytes()); positions = dict(zip(joint_names, q, strict=True))
    links = {link.get('name') for link in root.findall('link')}
    joints = root.findall('joint'); children = {j.find('child').get('link') for j in joints}
    base, = links - children; frames = {base: np.eye(4)}; pending = list(joints)
    while pending:
        progressed = False
        for joint in list(pending):
            parent = joint.find('parent').get('link')
            if parent not in frames:
                continue
            origin = joint.find('origin'); transform = np.eye(4)
            if origin is not None:
                transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            if joint.get('type') != 'fixed':
                motion = np.eye(4); axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                motion[:3, :3] = Rotation.from_rotvec(axis * positions[joint.get('name')]).as_matrix()
                transform = transform @ motion
            child = joint.find('child').get('link'); frames[child] = frames[parent] @ transform
            pending.remove(joint); progressed = True
        if not progressed:
            raise ValueError('Invalid URDF reference tree')
    return frames


def run_checks(folder, config=None):
    from pathlib import Path
    folder = Path(folder)
    if folder.exists():
        raise FileExistsError(folder)
    config = default_config() if config is None else config
    engine, manifest = nominal_kinematics(config); rng = np.random.default_rng(20261028)
    q = rng.uniform(-.3, .3, (16, 14)); tools = config['tools']; reference_error = 0.; jacobian_error = 0.; branch_error = 0.
    batches = {t['name']: engine.frame_batch(q, t['name']) for t in tools}
    for index, position in enumerate(q):
        oracle = urdf_reference(position, manifest['joint_names'])
        for frame in ('base_link', 'waist_yaw_link', 'left_palm_base_link', 'right_palm_base_link'):
            reference_error = max(reference_error, float(np.max(np.abs(engine.frame_pose(position, frame) - oracle[frame]))))
        for tool in tools:
            relative = np.eye(4); relative[:3, 3] = tool['translation_m']
            relative[:3, :3] = Rotation.from_rotvec(tool['rotation_vector_rad']).as_matrix()
            name = tool['name']; actual = batches[name]['poses'][index]; jac = batches[name]['jacobians'][index]
            reference_error = max(reference_error, float(np.max(np.abs(actual - oracle[tool['parent_frame']] @ relative))))
            numeric = np.empty((6, 14)); h = 1e-6
            for j in range(14):
                axis = np.eye(14)[j]; plus = engine.frame_pose(position + h * axis, name); minus = engine.frame_pose(position - h * axis, name)
                numeric[:3, j] = (plus[:3, 3] - minus[:3, 3]) / (2 * h)
                skew = ((plus[:3, :3] - minus[:3, :3]) / (2 * h)) @ actual[:3, :3].T
                numeric[3:, j] = [skew[2, 1], skew[0, 2], skew[1, 0]]
            jacobian_error = max(jacobian_error, float(np.max(np.abs(jac - numeric))))
            other = slice(7, 14) if name.startswith('left_') else slice(0, 7)
            branch_error = max(branch_error, float(np.max(np.abs(jac[:, other]))))
    # Analytic 90-degree rotation, independent of the robot's FK.
    sensor = _core.ArmPose(); sensor.translation = [1., 2., 3.]
    sensor.rotation = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
    cal = calibration(config); expected = np.array([.001, -.002, -.003])
    readings = _core.trical_readings(sensor, np.array([1.002, 2.001, 2.997]), cal)
    sensor_error = float(np.max(np.abs(readings - cal.zero - expected)))
    checks = dict(urdf_fk=reference_error < 1e-12, frame_jacobian=jacobian_error < 1e-8,
        cross_branch=branch_error < 1e-14, analytic_indicator=sensor_error < 1e-12)
    folder.mkdir(parents=True, exist_ok=False); write_json(folder / 'geometry.json', manifest); write_json(folder / 'profile.json', config)
    # Protocol example: the instruments are NOT touching any ball. No fake centered observations.
    count = 4; times = np.arange(count, dtype=np.int64) * 2_000_000
    arrays = dict(q_rad=q[:count], dq_rad_s=np.zeros((count, 14)), joint_sequence=np.arange(count, dtype=np.int64),
        joint_sample_ns=times, joint_receive_ns=times + 100_000, joint_sample_known=np.ones(count, dtype=bool),
        indicator_readings_m=np.full((count, 3), np.nan), indicator_sequence=np.tile(np.arange(count, dtype=np.int64)[:, None], (1, 3)),
        indicator_sample_ns=np.full((count, 3), -1, dtype=np.int64), indicator_receive_ns=np.tile((times + 200_000)[:, None], (1, 3)),
        indicator_sample_known=np.zeros((count, 3), dtype=bool), indicator_status=np.full((count, 3), IndicatorStatus.NO_CONTACT, dtype=np.uint8),
        ball_id=np.array(['ball_1', 'ball_2', 'ball_3', 'ball_4']), tool_frame=np.array(['left_trical_tcp', 'right_trical_tcp'] * 2))
    metadata = observation_metadata(manifest, config, run_id='k1_protocol_example', source='schema_example_no_contact_no_acquisition',
        clocks=dict(joint_sample='virtual_ns', indicator_sample=['unknown'] * 3, receive='virtual_ns'))
    summary = save_observations(folder / 'protocol_example', arrays, metadata); loaded, _ = load_observations(folder / 'protocol_example')
    checks['observation_roundtrip'] = all(np.array_equal(arrays[key], loaded[key], equal_nan=True) if arrays[key].dtype.kind == 'f'
        else np.array_equal(arrays[key], loaded[key]) for key in arrays)
    report = dict(schema_version=1, stage='K1', passed=all(checks.values()), checks=checks,
        synthetic_only=True, hardware_executable=False, calibration_performed=False, collision_certified=False,
        samples=16, native_sha256=sha256(Path(_core.__file__).read_bytes()).hexdigest(), backend=engine.backend_version(),
        geometry_sha256=manifest['geometry_sha256'], max_fk_error=reference_error,
        max_jacobian_error=jacobian_error, max_other_arm_jacobian=branch_error,
        max_analytic_indicator_error_m=sensor_error, protocol_example=summary,
        pending=['K2: collision-gated approach, indicator Mock faults and autocentering',
                 'K3: observability, parameter Jacobian, MDH/Hayati mapping and identification',
                 'K4: independent calibration acceptance and Chinese video', 'K5: real instrument and ROS adapters'])
    write_json(folder / 'report.json', report)
    return report
