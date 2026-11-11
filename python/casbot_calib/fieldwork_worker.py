"""Native stream supervisor and isolated geometry renderer; no GUI control loop."""
import argparse
from pathlib import Path
import time

import numpy as np

from .fieldwork_analysis import read_json
from .data import write_json
from .fieldwork_control import Session, heartbeat_check
from .hardware_batch import batch_lock
from .fieldwork_display import latest_json, geometry_changed, lightweight_scene, VIEW_FPS


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
    view = JointView(adapter, scene, width=640, height=480)
    # Preserve CAD shape/scene; avoid expensive shadow/reflection passes in a tuning view.
    for flag in ('mjRND_SHADOW','mjRND_REFLECTION'):
        view.renderer.scene.flags[getattr(view.mj.mjtRndFlag,flag)] = False
    view.camera.lookat[:] = [0, 0, .95]
    view.camera.distance = 2.3; view.camera.azimuth = 160; view.camera.elevation = -8
    return view


def frame(metadata, view, q, receive_ns=0):
    from .telemetry import StateFrame
    return StateFrame(view.adapter.robot_model_id, 'SDK接收位置几何回放', 0, -1, receive_ns,
                      tuple(metadata['joint_names']), np.asarray(q), None, None, None, clock_domain='sample_unknown')


def render_live(root, check):
    """Own EGL in a separate process, never interfere with ImGui's OpenGL context."""
    metadata = read_json(root/'view/metadata.json')
    sequence = 0; previous = None
    with open_view(metadata, root/'view',lightweight=True) as view:
        while check() in (None, 'operator_cancel'):
            pose = read_json(root/'view/pose.json')
            if pose and geometry_changed(pose,previous):
                if pose.get('selected') is not None: view.highlight(pose['selected'])
                pixels = view.render(frame(metadata, view, pose['q'], pose.get('receive_ns') or 0))
                temporary = root/'view/frame.pending'
                with temporary.open('wb') as stream: np.save(stream, pixels, allow_pickle=False)
                temporary.replace(root/'view/frame.npy')
                sequence += 1
                latest_json(root/'view/rendered.json', dict(**pose, rendered_frames=sequence))
                previous = pose
            time.sleep(1/VIEW_FPS)


def render_recording(root, relative, cancel_check=None):
    """Video uses recorded feedback and receive-time ZOH, not command targets or truth."""
    import imageio_ffmpeg
    from PIL import Image, ImageDraw
    from .presentation import _font
    font = _font(18)
    root=Path(root).resolve()
    session = Session(root)
    directory = (root/relative).resolve()
    if directory.parent != root/'runs': raise ValueError('运行目录不属于会话')
    feedback=directory/'recording/feedback.csv'
    if not feedback.exists():feedback=directory/'capture/execution/feedback.csv'
    data = np.genfromtxt(feedback, delimiter=',', names=True, dtype=None, encoding='utf-8')
    data = np.atleast_1d(data)
    if len(data) < 2: raise ValueError('记录不足两帧')
    output = directory/'recording/motion.mp4'
    if output.exists(): raise ValueError('视频已存在')
    output.parent.mkdir(exist_ok=True)
    timestamps = data['receive_ns'].astype(np.int64)
    times = (timestamps-timestamps[0])*1e-9
    with open_view(session.metadata, root/'view') as view:
        temporary = output.with_name('motion.pending.mp4')
        writer = imageio_ffmpeg.write_frames(str(temporary), (640,480), fps=20, codec='libx264',
                    pix_fmt_in='rgb24', pix_fmt_out='yuv420p', output_params=['-crf','22','-preset','fast'])
        writer.send(None)
        try:
            for t in np.arange(0, times[-1]+.025, .05):
                reason = cancel_check() if cancel_check else None
                if reason: raise ValueError('视频导出已中断：'+reason)
                i = max(0, np.searchsorted(times,t,side='right')-1)
                q = [data['q'+str(j)][i] for j in range(14)]
                pixels = Image.fromarray(view.render(frame(session.metadata, view, q, int(timestamps[i]))))
                draw = ImageDraw.Draw(pixels); draw.rectangle((0,0,640,32),fill=(235,242,248))
                label = 'SDK Mock' if session.metadata['scope']=='localhost_contract_mock' else '现场SDK反馈'
                draw.text((12,5),f'{label}  {t:.2f} s   已接收位置 / 几何回放',font=font,fill=(24,44,65))
                writer.send(np.asarray(pixels))
        finally: writer.close()
    temporary.replace(output)
    return dict(video=str(output), duration_s=float(times[-1]), source='recorded SDK feedback, receive-time ZOH')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--request', type=Path)
    parser.add_argument('--heartbeat', type=Path, required=True)
    parser.add_argument('--owner-id', required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--render', action='store_true')
    parser.add_argument('--pipe-heartbeat', action='store_true')
    args = parser.parse_args(argv)
    from .fieldwork_ipc import PipeHeartbeat,emit
    request = None if args.render else read_json(args.request)
    control = request is not None and request['action']=='stream'
    check = PipeHeartbeat(0,args.owner_id,timeout_s=2. if control else None) if args.pipe_heartbeat else heartbeat_check(args.heartbeat, expected_owner=args.owner_id)
    if args.render:
        render_live(args.session, check); return 0
    try:
        if check(): raise ValueError(check())
        with batch_lock(args.session):
            if request['action'] == 'stream':
                result = Session(args.session).stream(**request['values'], cancel_check=check,
                    on_event=lambda value:emit('status',event=value))
            elif request['action'] == 'observe':
                from .fieldwork_observer import observe
                result=observe(args.session,**request['values'],cancel_check=check,publish=emit)
            elif request['action'] == 'video': result = render_recording(args.session, **request['values'], cancel_check=check)
            else: raise ValueError('未知操作')
        response = dict(completed=True, action=request['action'], result=result)
    except Exception as exc:
        response = dict(completed=False, error=str(exc), action=request['action'])
    emit('response',response=response)
    try:write_json(args.output, response)
    except OSError as exc:emit('recording_error',message='操作结果写入失败：'+str(exc))
    return 0 if response['completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
