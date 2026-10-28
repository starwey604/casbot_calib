"""CAD-envelope viewer uses only geometry and agrees with Pinocchio frames."""
import os
import numpy as np
import pytest


@pytest.mark.skipif(os.environ.get('CASBOT_TEST_RENDER')!='1',reason='optional EGL renderer')
def test_staged_envelope_viewer_never_calls_dynamics_and_matches_native_fk(tmp_path,monkeypatch):
    import mujoco
    from casbot_calib.collision import nominal_profile
    from casbot_calib.staged_presentation import write_scene
    from casbot_calib.viewer import JointView
    from casbot_calib.telemetry import StateFrame
    def forbidden(*args,**kwargs):raise AssertionError('Viewer requested MuJoCo physics/collision')
    for name in ('mj_step','mj_forward','mj_inverse','mj_collision'):monkeypatch.setattr(mujoco,name,forbidden)
    profile=nominal_profile();adapter=write_scene(profile,tmp_path/'scene.xml');q=np.array([0,.45,0,-.9,0,0,0,0,-.45,0,-.9,0,0,0])
    with JointView(adapter,tmp_path/'scene.xml',width=320,height=240) as view:
        for k in range(3):
            position=q+np.random.default_rng(k).uniform(-.1,.1,14)
            frame=StateFrame(adapter.robot_model_id,'geometry oracle',k,k*1000000,k*1000000,tuple(j.name for j in profile.engine.model.joints),position,np.zeros(14),np.zeros(14),None)
            rgb=view.render(frame);assert rgb.shape==(240,320,3) and not np.any(view.data.qacc)
            for i,j in enumerate(profile.engine.model.joints):
                body=mujoco.mj_name2id(view.model,mujoco.mjtObj.mjOBJ_BODY,j.name+'_view');expected=profile.engine.fk(position)[i]
                np.testing.assert_allclose(view.data.xpos[body],expected[:3,3],atol=1e-12)
                np.testing.assert_allclose(view.data.xmat[body].reshape(3,3),expected[:3,:3],atol=1e-12)
