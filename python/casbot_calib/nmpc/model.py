"""DVT2 model export and NumPy kinematics in SHM joint order."""
import numpy as np
from scipy.spatial.transform import Rotation

from ..shm_data import NAMES

FLANGES = ('left_flange_frame', 'right_flange_frame')


def arm_chains(context):
    model = context.engine.model
    joints = model.joints
    expected_parents = [-1, 0, 1, 2, 3, 4, 5, -1, 7, 8, 9, 10, 11, 12]
    if list(model.parents) != expected_parents or [j.name for j in joints] != list(NAMES):
        raise ValueError('Expected two 7-joint chains in DVT2 SHM order')
    frames = {frame.name: frame for frame in model.frames}
    if [frames[name].parent for name in FLANGES] != [6, 13]:
        raise ValueError('Unexpected flange attachment')
    return dict(
        joint_names=list(NAMES),
        bases=[model.base.matrix().tolist() for _ in FLANGES],
        origins=np.array([j.origin.matrix() for j in joints]).reshape(2, 7, 4, 4).tolist(),
        axes=np.array([j.axis for j in joints]).reshape(2, 7, 3).tolist(),
        tools=[frames[name].placement.matrix().tolist() for name in FLANGES],
        bounds=[[j.lower, j.upper] for j in joints],
    )


def forward(model, q):
    """Return joint placements and flange poses in world coordinates."""
    origins = np.asarray(model['origins'])
    axes = np.asarray(model['axes'])
    rotations = Rotation.from_rotvec(axes.reshape(14, 3) * np.asarray(q).reshape(14, 1))
    joint_poses = np.empty((2, 7, 4, 4))
    turns = np.tile(np.eye(4), (14, 1, 1))
    turns[:, :3, :3] = rotations.as_matrix()
    for arm in range(2):
        pose = np.asarray(model['bases'][arm])
        for link in range(7):
            pose = pose @ origins[arm, link] @ turns[7 * arm + link]
            joint_poses[arm, link] = pose
    return joint_poses.reshape(14, 4, 4), joint_poses[:, -1] @ model['tools']


def stopping_maps(steps, dt):
    """Linear maps from [q,v,a] to tail jerk and final resting position."""
    b = np.vstack([np.full(steps, dt), dt**2 * (steps - np.arange(steps) - .5)])
    rhs = np.array([[0., 0., -1.], [0., -1., -steps*dt]])
    jerks = b.T @ np.linalg.solve(b @ b.T, rhs)
    transition = np.array([[1., dt, dt**2/2], [0., 1., dt], [0., 0., 1.]])
    input_map = np.array([dt**3/6, dt**2/2, dt])[:, None]
    state = np.eye(3)
    for row in jerks:
        state = transition @ state + input_map * row
    return jerks, state[0]
