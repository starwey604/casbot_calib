"""Private physical-loop test of bounded range correction against wrist stiction."""
import json
from pathlib import Path
import time
import numpy as np
import pytest

from casbot_calib.model_context import load_context
from casbot_calib.pipeline.robot import load_robot_config
from casbot_calib.shm_data import ROOT
from casbot_calib.shm_mock import SHMMock
from casbot_calib.stages.kinematics.common import load_recipe
from casbot_calib.trical.routes import Routes
from casbot_calib.trical.controller import Executor, correction
from test_shm_fieldwork import NATIVE


@pytest.mark.slow
def test_stiction_bounded_correction_enters_range(tmp_path):
    context = load_context()
    routes = Routes('left', context)
    seed = np.array(routes.seeds['A']['contact'][0]['q'])
    config = json.loads((ROOT/'configs/shm/k_mock.json').read_text())
    config['urdf'] = str(context.urdf)
    config['initial_q'] = seed.tolist()
    config['initial_fields']['position'] = seed.tolist()
    config['initial_fields']['effort'] = [0.]*14
    config['gravity_world'] = [0.,0.,0.]
    config['friction_static'] = [.1,.1,.1,.1,.3,.3,.3]*2
    config['friction_coulomb'] = [.02]*14
    path = tmp_path/'mock.json'; path.write_text(json.dumps(config))
    robot = load_robot_config(); robot['gravity_world']=[0.,0.,0.]
    recipe = load_recipe()
    binary = Path(NATIVE)/'shm_trical_executor'
    with SHMMock(tmp_path/'master', binary, path) as mock:
        client = Executor(binary, mock.endpoint, context, robot, recipe, 'left', tmp_path/'capture/native')
        try:
            frame = 'left_trical_tcp'
            first = context.engine.frame_pose(seed,frame)
            ball = first[:3,3] + first[:3,:3]@np.array([.0028,0,0])
            for _ in range(3):
                end = time.monotonic()+.5
                while time.monotonic()<end: client.pump(); time.sleep(.005)
                pose = context.engine.frame_pose(client.q,frame)
                d = pose[:3,:3].T@(ball-pose[:3,3])
                if np.max(abs(d)) <= .002: break
                target = correction(context.engine,client.q,'left',d,.0015,.012,client.hold)
                assert np.max(abs(target-client.q)) <= .012+1e-12
                for q in routes.check_contact('A',client.hold,target)[1:]: client.move(q,windup=True)
            else:
                end = time.monotonic()+.5
                while time.monotonic()<end: client.pump(); time.sleep(.005)
                pose = context.engine.frame_pose(client.q,frame)
                d = pose[:3,:3].T@(ball-pose[:3,3])
            assert np.max(abs(d)) <= .002
            assert np.max(abs(client.q[:4]-seed[:4])) > 1e-4
            assert np.max(abs(client.q[4:7]-seed[4:7])) < 1e-4
            client.finish()
        finally:
            client.close()
