"""One GUI operation per process; native SDK sender remains the only command loop."""
import argparse
from pathlib import Path

import numpy as np

from .fieldwork_analysis import read_json
from .data import write_json
from .fieldwork_control import Session, heartbeat_check, operate


def render_pose(root, q, selected, output):
    """Reuse the existing geometry-only viewer in this worker's EGL context."""
    session = Session(root)
    render_geometry(session.metadata, q, selected, output)


def render_geometry(metadata, q, selected, output):
    from PIL import Image
    from .model_context import resolve_context
    from .telemetry import StateFrame
    from .viewer import JointView
    context = resolve_context(metadata['model_context'])
    directory = Path(output).parent; directory.mkdir(parents=True, exist_ok=True)
    scene = directory/'scene.xml'
    if scene.exists():
        from .dvt2_bundle import viewer_adapter
        adapter = viewer_adapter(context.bundle)
    else:
        adapter = context.write_view_scene(scene)
    with JointView(adapter, scene, width=640, height=480) as view:
        view.camera.lookat[:] = [0, 0, .95]
        view.camera.distance = 2.3
        view.camera.azimuth = 160
        view.camera.elevation = -8
        if selected in metadata['joint_names']:
            view.highlight(metadata['joint_names'].index(selected))
        frame = StateFrame(adapter.robot_model_id, '已记录位置的几何预览', 0, -1, 0,
                           tuple(metadata['joint_names']), np.array(q), None, None, None, clock_domain='sample_unknown')
        Image.fromarray(view.render(frame)).save(output)


def preview(root, result, action):
    session = Session(root)
    if action == 'snapshot':
        # Do not alter the already sealed snapshot directory.
        path = Path(root)/'view'/('snapshot_'+Path(result['snapshot']).name+'.png')
        path.parent.mkdir(exist_ok=True)
        render_pose(root, result['q_rad'], None, path)
        result['image'] = str(path)
    elif action == 'plan':
        attempt = Path(root)/result['attempt']
        data = np.genfromtxt(attempt/'plan/position_plan.csv', delimiter=',', names=True)
        indices = np.unique(np.linspace(0, len(data)-1, min(240, len(data)), dtype=int))
        result['plot'] = dict(t=(data['relative_ns'][indices]*1e-9).tolist(),
            q={name: data['q'+str(session.names.index(name))][indices].tolist() for name in result['joints']})
        q = np.array(result['q_start_rad'])
        request = read_json(attempt/'input/request.json')
        for name in request['joints']:
            q[session.names.index(name)] += np.deg2rad(request['amplitude_deg'])
        path = Path(root)/'view'/('plan_'+attempt.name+'.png')
        path.parent.mkdir(exist_ok=True)
        render_pose(root, q, result['joints'][0], path)
        result['image'] = str(path)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--request', type=Path, required=True)
    parser.add_argument('--heartbeat', type=Path, required=True)
    parser.add_argument('--owner-id', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    request = read_json(args.request)
    try:
        check = heartbeat_check(args.heartbeat, expected_owner=args.owner_id)
        reason = check()
        if reason:
            raise ValueError(reason)
        result = operate(args.session, request['action'], request.get('values', {}), cancel_check=check)
        if request['action'] in ('snapshot', 'plan'):
            try:
                preview(args.session, result, request['action'])
            except (OSError, RuntimeError, ValueError) as exc:
                result['preview_error'] = str(exc)
        response = dict(completed=True, action=request['action'], result=result, status=Session(args.session).status())
    except Exception as exc:
        response = dict(completed=False, error=str(exc), action=request['action'], status=Session(args.session).status())
    write_json(args.output, response)
    return 0 if response['completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
