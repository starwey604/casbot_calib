"""Local MuJoCo renderer; feedback-only poses over an owned socket, RGB via memfd."""
import argparse
import json
import os
from pathlib import Path
import select
import socket
import time

from .fieldwork_display import geometry_changed, configure_graphics_environment
from .fieldwork_frames import FrameBuffer
from .fieldwork_view import open_view, frame


def main(argv=None):
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--frame-fd',type=int,required=True);p.add_argument('--pose-fd',type=int,required=True)
    args=p.parse_args(argv);configure_graphics_environment()
    metadata=json.loads((args.root/'view/metadata.json').read_text());sequence=0;previous=None;pending=b'';pose=None
    frames=FrameBuffer(args.frame_fd)
    try:
        with socket.socket(fileno=args.pose_fd) as channel,open_view(metadata,args.root/'view',lightweight=True) as view:
            while True:
                start=time.monotonic()
                if select.select([channel],[],[],.03)[0]:
                    data=channel.recv(65536)
                    if not data:break
                    pending+=data
                    if len(pending)>131072:raise ValueError('local_pose_buffer')
                    parts=pending.split(b'\n');pending=parts.pop()
                    if parts:pose=json.loads(parts[-1])
                if pose and pose.get('visible',True) and (geometry_changed(pose,previous) or previous is None or pose.get('camera')!=previous.get('camera')):
                    view.highlight(pose['selected'])
                    if pose.get('camera'):
                        view.camera.azimuth,view.camera.elevation,view.camera.distance=pose['camera']
                    pixels=view.render(frame(metadata,view,pose['q'],pose.get('receive_ns',0),label='SHM实际接收位置',clock_domain='orin_receive_monotonic'))
                    sequence+=1;frames.write(pixels,sequence);previous=pose
                time.sleep(max(0,1/30-(time.monotonic()-start)))
    finally:frames.close()
    return 0


if __name__=='__main__':raise SystemExit(main())
