"""MuJoCo geometry viewer. Deliberately no dynamics, stepping or device control."""
from __future__ import annotations
from importlib.resources import files
import os
from pathlib import Path
import sys
import numpy as np
from .telemetry import StateFrame, TelemetryAdapter


def default_adapter():
    return TelemetryAdapter.load(files("casbot_calib").joinpath("assets/bench_mapping.json"))


class JointView:
    def __init__(self,adapter=None,scene=None,width=800,height=420,*,offscreen=True):
        if offscreen and sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
            os.environ.setdefault("MUJOCO_GL","egl")
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError("Install viewer dependencies: pip install -e '.[viewer]' --no-build-isolation") from exc
        self.mj=mujoco;self.adapter=adapter or default_adapter()
        custom_scene=scene is not None
        scene=scene or files("casbot_calib").joinpath("assets/bench.xml")
        self.model=mujoco.MjModel.from_xml_path(str(scene));self.data=mujoco.MjData(self.model)
        self.addresses=[];self.joint_ids=[]
        for mapping in self.adapter.mappings:
            j=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,mapping.model_joint)
            if j<0 or int(self.model.jnt_type[j])!=int(mujoco.mjtJoint.mjJNT_HINGE):
                raise ValueError(f"The radian telemetry viewer requires a named revolute joint: {mapping.model_joint}")
            self.addresses.append(int(self.model.jnt_qposadr[j]))
            self.joint_ids.append(j)
        self.reference=self._optional_joint("reference_joint")
        self.before=self._optional_joint("before_joint")
        self.camera=mujoco.MjvCamera()
        self.camera.lookat[:]=[.075,0,.09];self.camera.distance=.72
        self.camera.azimuth=110;self.camera.elevation=-42
        if custom_scene:
            self.camera.lookat[:]=self.model.stat.center
            self.camera.distance=max(.3,1.5*self.model.stat.extent)
            self.camera.azimuth=135;self.camera.elevation=-25
        self.renderer=None
        if offscreen:
            self.model.vis.global_.offwidth=max(width,self.model.vis.global_.offwidth)
            self.model.vis.global_.offheight=max(height,self.model.vis.global_.offheight)
            self.renderer=mujoco.Renderer(self.model,height=height,width=width)
        self.last_frame=None
        self._selected_geoms=np.array([],dtype=int)
        self._original_colors=self.model.geom_rgba.copy()
        self.mj.mj_kinematics(self.model,self.data)
        self.mj.mj_camlight(self.model,self.data)

    def _optional_joint(self,name):
        j=self.mj.mj_name2id(self.model,self.mj.mjtObj.mjOBJ_JOINT,name)
        return int(self.model.jnt_qposadr[j]) if j>=0 else None

    def update(self,frame: StateFrame,before: StateFrame | None=None):
        if frame.robot_model_id!=self.adapter.robot_model_id:
            raise ValueError("Viewer and frame model IDs differ")
        if frame.joint_names!=tuple(m.name for m in self.adapter.mappings):
            raise ValueError("Viewer joint order differs from normalized frame")
        self.data.qpos[self.addresses]=frame.q
        self.data.time=max(frame.sample_ns,0)*1e-9
        if self.reference is not None:
            self.data.qpos[self.reference]=frame.q[0] if frame.target_q is None else frame.target_q[0]
            for name in ("reference_lever","reference_tip"):
                g=self.mj.mj_name2id(self.model,self.mj.mjtObj.mjOBJ_GEOM,name)
                if g>=0:self.model.geom_rgba[g,3]=.8 if frame.target_q is not None else 0.
        if self.before is not None:
            if before is not None and (before.robot_model_id!=frame.robot_model_id or before.joint_names!=frame.joint_names):
                raise ValueError("Comparison frame mapping differs")
            self.data.qpos[self.before]=frame.q[0] if before is None else before.q[0]
            for name in ("before_lever","before_tip"):
                g=self.mj.mj_name2id(self.model,self.mj.mjtObj.mjOBJ_GEOM,name)
                if g>=0:self.model.geom_rgba[g,3]=.85 if before is not None else 0.
        # Geometry transforms only. Never call mj_forward, mj_inverse, mj_step,
        # mj_rne, a controller, or a physical-model callback in this module.
        self.mj.mj_kinematics(self.model,self.data)
        self.mj.mj_camlight(self.model,self.data)
        self.last_frame=frame

    def render(self,frame,before=None):
        if self.renderer is None:raise RuntimeError("No offscreen renderer")
        self.update(frame,before)
        self.renderer.update_scene(self.data,camera=self.camera)
        return self.renderer.render().copy()

    def highlight(self,index):
        self.model.geom_rgba[self._selected_geoms]=self._original_colors[self._selected_geoms]
        body=int(self.model.jnt_bodyid[self.joint_ids[index]])
        self._selected_geoms=np.flatnonzero(self.model.geom_bodyid==body)
        self.model.geom_rgba[self._selected_geoms,:3]=[.2,.9,.75]

    def close(self):
        if self.renderer is not None:self.renderer.close();self.renderer=None

    def __enter__(self):return self
    def __exit__(self,*_):self.close()


def show(source,adapter=None,scene=None,*,duration_s=None,loop=True):
    """GLFW view for replay or latest-only telemetry. No passive physics viewer.

    Space pauses replay; R restarts; left drag rotates camera; wheel zooms.
    LatestStateSource keeps updating independently; this function never writes it.
    """
    import time
    import glfw
    with JointView(adapter,scene,offscreen=False) as view:
        if not glfw.init():raise RuntimeError("GLFW needs a desktop display; use video export on headless machines")
        window=glfw.create_window(1100,720,"casbot_calib - read-only joint viewer",None,None)
        if not window:glfw.terminate();raise RuntimeError("Cannot create GLFW window")
        glfw.make_context_current(window);glfw.swap_interval(1)
        mj=view.mj;scene_view=mj.MjvScene(view.model,maxgeom=2000)
        context=mj.MjrContext(view.model,mj.mjtFontScale.mjFONTSCALE_150)
        option=mj.MjvOption();pause=False;cursor=0.;last=time.monotonic();drag=None;selected=0
        def key(_,key,scan,action,mods):
            nonlocal pause,cursor,selected
            if action!=glfw.PRESS:return
            if key==glfw.KEY_ESCAPE:glfw.set_window_should_close(window,True)
            elif key==glfw.KEY_SPACE:pause=not pause
            elif key==glfw.KEY_R:cursor=0.
            elif key==glfw.KEY_TAB:selected=(selected+1)%len(view.adapter.mappings)
        def scroll(_,x,y):view.camera.distance=max(.15,min(5.,view.camera.distance*np.exp(-.1*y)))
        glfw.set_key_callback(window,key);glfw.set_scroll_callback(window,scroll)
        try:
            while not glfw.window_should_close(window):
                now=time.monotonic()
                if not pause:cursor+=now-last
                last=now
                if duration_s is not None and cursor>duration_s:
                    if loop:cursor=0.
                    else:break
                frame=source.read(round(cursor*1e9))
                if frame is not None:view.update(frame)
                view.highlight(selected)
                x,y=glfw.get_cursor_pos(window)
                if glfw.get_mouse_button(window,glfw.MOUSE_BUTTON_LEFT)==glfw.PRESS:
                    if drag is not None:
                        view.camera.azimuth-=(x-drag[0])*.3
                        view.camera.elevation=np.clip(view.camera.elevation+(y-drag[1])*.3,-89,10)
                    drag=(x,y)
                else:drag=None
                width,height=glfw.get_framebuffer_size(window);rect=mj.MjrRect(0,0,width,height)
                mj.mjv_updateScene(view.model,view.data,option,None,view.camera,mj.mjtCatBit.mjCAT_ALL,scene_view)
                mj.mjr_render(rect,scene_view,context)
                if getattr(source,"error",None):status=f"TELEMETRY ERROR: {source.error}"
                elif frame is None:status="WAITING FOR TELEMETRY"
                else:
                    stale=hasattr(source,"is_stale") and source.is_stale()
                    m=view.adapter.mappings[selected];axis=view.model.jnt_axis[view.joint_ids[selected]]
                    status=f"{'STALE' if stale else frame.source} | seq {frame.sequence} | {m.name}\n"
                    raw="unavailable" if frame.raw_position is None else f"{frame.raw_position[selected]:+.3f}"
                    status+=f"raw {raw} | q {frame.q[selected]:+.5f} rad | i {frame.current[selected]:+.3f} A\n"
                    status+=f"map sign {m.direction:+} | scale {m.scale_rad_per_unit:g} rad/unit | zero {m.zero_raw:g} | local axis {axis}"
                mj.mjr_overlay(mj.mjtFontScale.mjFONTSCALE_150,mj.mjtGridPos.mjGRID_TOPLEFT,rect,
                    "READ ONLY | no physics / no motor writes\n"+status,
                    "Tab joint | Space pause | R restart | drag orbit | wheel zoom | Esc quit",context)
                glfw.swap_buffers(window);glfw.poll_events()
        finally:
            context.free();glfw.destroy_window(window);glfw.terminate()
