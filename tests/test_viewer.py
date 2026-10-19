import ast
from dataclasses import replace
import inspect
import os
from pathlib import Path
import numpy as np
import pytest
from casbot_calib.telemetry import StateFrame
from casbot_calib import viewer


def frame():
    return StateFrame("synthetic_joint_bench_v1","sdk",1,1_000_000,2_000_000,
        ("joint_1",),[.3],[.4],[.2],[7409.72],[.35])


def test_viewer_source_uses_only_geometry_pipeline():
    tree=ast.parse(inspect.getsource(viewer))
    names={node.func.attr for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)
           and node.func.attr.startswith("mj_")}
    assert names<={"mj_name2id","mj_kinematics","mj_camlight"}


def forbid_dynamics(monkeypatch,mj):
    def forbidden(*args,**kwargs):raise AssertionError("Viewer called a dynamics API")
    for name in ("mj_forward","mj_inverse","mj_step","mj_step1","mj_step2","mj_rne","mj_fwdPosition",
                 "mj_fwdVelocity","mj_fwdActuation","mj_fwdAcceleration"):
        monkeypatch.setattr(mj,name,forbidden)


def test_mock_and_external_frames_use_identical_geometry_without_dynamics(monkeypatch):
    mj=pytest.importorskip("mujoco");forbid_dynamics(monkeypatch,mj)
    with viewer.JointView(offscreen=False) as view:
        external=frame();view.update(external);q=view.data.qpos.copy()
        site=mj.mj_name2id(view.model,mj.mjtObj.mjOBJ_SITE,"observed_tip_site")
        np.testing.assert_allclose(view.data.site_xpos[site],[.2*np.cos(.3),.2*np.sin(.3),.13],atol=1e-12)
        view.update(replace(external,source="cpp_mock"))
        np.testing.assert_array_equal(view.data.qpos,q)
        np.testing.assert_array_equal(view.data.qacc,np.zeros(view.model.nv))
        with pytest.raises(ValueError):view.update(replace(external,robot_model_id="evt6"))


@pytest.mark.skipif(os.environ.get("CASBOT_TEST_RENDER")!="1",reason="Explicit GPU/Mesa render smoke opt-in")
def test_offscreen_rasterization_does_not_call_dynamics(monkeypatch):
    mj=pytest.importorskip("mujoco");forbid_dynamics(monkeypatch,mj)
    with viewer.JointView(width=320,height=180) as view:
        image=view.render(frame())
        assert image.shape==(180,320,3) and image.std()>5
        np.testing.assert_array_equal(view.data.qacc,np.zeros(view.model.nv))
