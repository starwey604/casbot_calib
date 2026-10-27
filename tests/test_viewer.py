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


@pytest.mark.skipif(os.environ.get("CASBOT_TEST_RENDER")!="1",reason="Chinese video rendering opt-in")
def test_chinese_captions_have_glyphs_and_wrap_without_spaces():
    pytest.importorskip("PIL")
    from PIL import Image,ImageDraw
    from casbot_calib.presentation import DemoPresentation,_font,STORIES
    font=_font(14)
    missing=bytes(font.getmask(chr(0x10ffff)))
    for char in "中文动力学标定":
        assert bytes(font.getmask(char))!=missing
    presentation=object.__new__(DemoPresentation);presentation.fonts={14:font}
    draw=ImageDraw.Draw(Image.new("RGB",(400,200)))
    text=STORIES["timing"][1]
    lines=presentation._wrap(draw,text,150,14)
    assert len(lines)>1 and "".join(lines)==text
    assert all(draw.textlength(line,font=font)<=150 for line in lines)


@pytest.mark.skipif(os.environ.get("CASBOT_TEST_RENDER")!="1",reason="Explicit GPU/Mesa render smoke opt-in")
def test_seven_joint_snapshot_matches_pinocchio_fk_without_mujoco_dynamics(tmp_path,monkeypatch):
    mj=pytest.importorskip('mujoco');forbid_dynamics(monkeypatch,mj)
    from casbot_calib.arm_model import default_arm_config,load_arm_model
    from casbot_calib import _core
    from casbot_calib.arm_viewer import render_arm_snapshot
    result=render_arm_snapshot(default_arm_config(),_core.ArmDynamics(load_arm_model()),tmp_path)
    assert result['passed'] and len(result['body_errors'])==8
    assert (tmp_path/'arm_geometry.png').exists()
