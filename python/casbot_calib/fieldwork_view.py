"""Shared model-only view construction. No transport, session or device control."""
from pathlib import Path
import numpy as np
from .fieldwork_display import lightweight_scene

def open_view(metadata, directory, *, lightweight=False):
    from .model_context import resolve_context
    from .viewer import JointView
    context = resolve_context(metadata['model_context'])
    directory = Path(directory); directory.mkdir(parents=True, exist_ok=True)
    scene = directory/'scene.xml'
    if scene.exists():
        from .dvt2_bundle import viewer_adapter
        adapter = viewer_adapter(context.bundle)
    else:
        adapter = context.write_view_scene(scene)
    if lightweight:scene=lightweight_scene(context,scene)
    if lightweight:
        from .fieldwork_frames import WIDTH,HEIGHT
        # The live image is displayed at 380x285. Avoid oversampling it, while
        # retaining the canonical full-size geometry/video path.
        view = JointView(adapter,scene,width=WIDTH,height=HEIGHT,offscreen=False)
        view.model.vis.quality.offsamples=0
        from .fieldwork_scene import LiveSceneRenderer
        view.renderer=LiveSceneRenderer(view.model,height=HEIGHT,width=WIDTH)
    else:view = JointView(adapter, scene, width=640, height=480)
    # Preserve CAD shape/scene; avoid expensive shadow/reflection passes in a tuning view.
    for flag in ('mjRND_SHADOW','mjRND_REFLECTION'):
        view.renderer.scene.flags[getattr(view.mj.mjtRndFlag,flag)] = False
    view.camera.lookat[:] = [0, 0, .95]
    view.camera.distance = 2.3; view.camera.azimuth = 160; view.camera.elevation = -8
    return view


def frame(metadata, view, q, receive_ns=0, *, label='接收位置几何回放', clock_domain='sample_unknown'):
    from .telemetry import StateFrame
    return StateFrame(view.adapter.robot_model_id, label, 0, -1, receive_ns,
                      tuple(metadata['joint_names']), np.asarray(q), None, None, None, clock_domain=clock_domain)

