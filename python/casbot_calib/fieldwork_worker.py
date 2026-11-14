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


from .fieldwork_view import open_view, frame


def render_live(root, check, frame_fd=None):
    """Own EGL in a separate process, never interfere with ImGui's OpenGL context."""
    metadata = read_json(root/'view/metadata.json')
    from .fieldwork_frames import FrameBuffer
    frames=FrameBuffer(frame_fd) if frame_fd is not None else None
    sequence = 0; previous = None
    with open_view(metadata, root/'view',lightweight=True) as view:
        while check() in (None, 'operator_cancel'):
            if not (getattr(check,'latest',None) or {}).get('view_visible',True):
                time.sleep(1/VIEW_FPS);continue
            pose = read_json(root/'view/pose.json')
            if pose and geometry_changed(pose,previous):
                if pose.get('selected') is not None: view.highlight(pose['selected'])
                pixels = view.render(frame(metadata, view, pose['q'], pose.get('receive_ns') or 0))
                sequence += 1
                if frames:frames.write(pixels,sequence)
                else:
                    temporary = root/'view/frame.pending'
                    with temporary.open('wb') as stream: np.save(stream,pixels,allow_pickle=False)
                    temporary.replace(root/'view/frame.npy')
                latest_json(root/'view/rendered.json', dict(**pose, rendered_frames=sequence))
                previous = pose
            time.sleep(1/VIEW_FPS)
    if frames:frames.close()


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
    from .fieldwork_control import recording_capture
    feedback=recording_capture(directory)/'execution/feedback.csv'
    from .fieldwork_streaming import VideoFeedback
    feedback_source=VideoFeedback(feedback,cancel_check)
    output = directory/'recording/motion.mp4'
    if output.exists(): raise ValueError('视频已存在')
    output.parent.mkdir(exist_ok=True)
    with open_view(session.metadata, root/'view') as view:
        temporary = output.with_name('motion.pending.mp4')
        writer = imageio_ffmpeg.write_frames(str(temporary), (640,480), fps=20, codec='libx264',
                    pix_fmt_in='rgb24', pix_fmt_out='yuv420p', output_params=['-crf','22','-preset','fast'])
        samples=feedback_source.frames()
        try:
            writer.send(None)
            for t,received,q in samples:
                pixels = Image.fromarray(view.render(frame(session.metadata, view, q, received)))
                draw = ImageDraw.Draw(pixels); draw.rectangle((0,0,640,32),fill=(235,242,248))
                label = 'SDK Mock' if session.metadata['scope']=='localhost_contract_mock' else '现场SDK反馈'
                draw.text((12,5),f'{label}  {t:.2f} s   已接收位置 / 几何回放',font=font,fill=(24,44,65))
                writer.send(np.asarray(pixels))
        finally:
            samples.close();writer.close()
    temporary.replace(output)
    return dict(video=str(output), duration_s=feedback_source.duration_s, source='recorded SDK feedback, receive-time ZOH')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--request', type=Path)
    parser.add_argument('--heartbeat', type=Path, required=True)
    parser.add_argument('--owner-id', required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--render', action='store_true')
    parser.add_argument('--frame-fd',type=int)
    parser.add_argument('--pipe-heartbeat', action='store_true')
    args = parser.parse_args(argv)
    from .fieldwork_ipc import PipeHeartbeat,emit
    request = None if args.render else read_json(args.request)
    control = request is not None and request['action'] in ('stream','zero_return')
    heartbeat_timeout=request['values'].get('settings',{}).get('general',{}).get('heartbeat_timeout_s',2.) if control else None
    check = PipeHeartbeat(0,args.owner_id,timeout_s=heartbeat_timeout) if args.pipe_heartbeat else heartbeat_check(args.heartbeat, expected_owner=args.owner_id,timeout_s=heartbeat_timeout or 2.)
    if args.render:
        render_live(args.session, check,args.frame_fd); return 0
    try:
        if check(): raise ValueError(check())
        with batch_lock(args.session):
            if request['action'] == 'stream':
                result = Session(args.session).stream(**request['values'], cancel_check=check,
                    on_event=lambda value:emit('status',event=value))
            elif request['action'] == 'observe':
                from .fieldwork_observer import observe
                result=observe(args.session,**request['values'],cancel_check=check,publish=emit)
            elif request['action'] == 'zero_return':
                from .fieldwork_zero import return_zero
                from .hardware_batch import event
                session = Session(args.session)
                if session.status()['blocked_reason']:raise ValueError(session.status()['blocked_reason'])
                relative=Path(request['values']['output']).resolve().relative_to(session.root)
                if relative.parent != Path('runs'):raise ValueError('回零运行必须属于本控制会话')
                event(session.root,'zero_started',run=str(relative))
                from .fieldwork_storage import recording_pause_file
                pause=recording_pause_file(session.root.parent.parent)
                result = return_zero(session.config, session.metadata['domain_id'], session.metadata['scope'],
                    **request['values'], **(dict(recording_pause_file=pause) if pause else {}), cancel_check=check, notify=lambda message:emit('zero_status',message=message))
                event(session.root,'zero_finished',run=str(relative),passed=result['passed'])
                controlled = result['result'].get('supervisor_interruption') in ('operator_cancel','frontend_closed','user_interrupt')
                if not result['passed'] and result['result'].get('fault') and not controlled:
                    event(session.root,'terminal_fault',reason=result['result']['fault'])
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
