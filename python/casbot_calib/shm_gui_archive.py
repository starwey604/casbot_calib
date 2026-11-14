"""Save a remote recording prefix and export video locally, off the control transport."""
import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import time
import uuid

from .shm_gui_remote import encode_config
from .shm_gui_storage import snapshot,load_snapshot


def _run(command,timeout,cancel):
    process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    end=time.monotonic()+timeout
    try:
        while True:
            if cancel and cancel():raise ValueError('保存已取消；可以重试下载')
            if time.monotonic()>end:raise TimeoutError('文件操作超时，可以重试')
            try:
                out,error=process.communicate(timeout=.1)
                return subprocess.CompletedProcess(command,process.returncode,out,error)
            except subprocess.TimeoutExpired:continue
    finally:
        if process.poll() is None:
            process.terminate()
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)


def save_workspace(client,output,*,remote_snapshot=None,cancel=None):
    if not client.ready:raise ValueError('尚无远端工作区')
    output=Path(output).resolve()
    if output.exists():raise ValueError('保存目标已存在')
    partial=output.with_name(output.name+'.partial');partial.mkdir(parents=True,exist_ok=True)
    source=client.ready['workspace']
    remote_snapshot=remote_snapshot or str(Path(source).parent/'snapshots'/('prefix_'+uuid.uuid4().hex))
    state=dict(source=source,remote_snapshot=remote_snapshot,output=str(output))
    # This retrievable state permits download-only retries without taking a new prefix.
    retry=output.with_name(output.name+'.download.json')
    if retry.exists():
        state=json.loads(retry.read_text())
        if state['source']!=source or state['output']!=str(output):raise ValueError('下载重试属于另一会话')
        remote_snapshot=state['remote_snapshot']
    else:
        if client.host:
            result=_run(client.command('snapshot','--config',encode_config(dict(source=source,output=remote_snapshot)),channel=2),120,cancel)
            if result.returncode:raise RuntimeError(result.stderr.decode(errors='replace'))
        else:snapshot(source,remote_snapshot,cancel=cancel)
        retry.write_text(json.dumps(state,indent=2)+'\n')
    if client.host:
        options=client.ssh_options(channel=2)
        command=['rsync','--archive','--partial','--protect-args','-e',shlex.join(options),client.host+':'+remote_snapshot+'/',str(partial)+'/']
        result=_run(command,3600,cancel)
        if result.returncode:raise RuntimeError('下载中断，可重试同一目标；远端 '+remote_snapshot+'：'+result.stderr.decode(errors='replace'))
    else:
        from .shm_gui_storage import _bytes_prefix
        def copy_prefix(src,dst):
            target=Path(dst);temporary=target.with_name(target.name+'.download')
            _bytes_prefix(Path(src),temporary,Path(src).stat().st_size,cancel);temporary.replace(target);shutil.copystat(src,dst);return dst
        shutil.copytree(remote_snapshot,partial,dirs_exist_ok=True,copy_function=copy_prefix)
    manifest=load_snapshot(partial);partial.rename(output);retry.unlink(missing_ok=True)
    return dict(output=str(output),remote_snapshot=remote_snapshot,manifest=manifest)


def export_video(source,output,*,cancel_check=None):
    from .fieldwork_streaming import VideoFeedback
    from .fieldwork_view import open_view,frame
    from .shm_gui_storage import load_snapshot
    import imageio_ffmpeg
    source=Path(source);output=Path(output)
    manifest=load_snapshot(source)
    if output.exists():raise ValueError('视频目标已存在')
    output.parent.mkdir(parents=True,exist_ok=True);temporary=output.with_name(output.stem+'.pending.mp4')
    reader=VideoFeedback(source/'native/feedback.csv',cancel_check,receive_column='receive_mono_ns')
    metadata=json.loads((source/'session.json').read_text());render_root=output.parent/('video_view_'+uuid.uuid4().hex[:8])
    writer=None;samples=reader.frames()
    try:
        with open_view(metadata,render_root) as view:
            writer=imageio_ffmpeg.write_frames(str(temporary),(640,480),fps=20,codec='libx264',pix_fmt_in='rgb24',pix_fmt_out='yuv420p',output_params=['-crf','22','-preset','fast'])
            writer.send(None)
            for _,received,q in samples:
                if cancel_check and cancel_check():raise ValueError('视频导出已取消')
                writer.send(view.render(frame(metadata,view,q,received,label='SHM记录反馈',clock_domain='orin_receive_monotonic')))
            writer.close();writer=None
        temporary.replace(output)
    finally:
        samples.close()
        if writer:writer.close()
        temporary.unlink(missing_ok=True)
    return dict(output=str(output),duration_s=reader.duration_s,prefix=manifest['prefix'],source='native_feedback_receive_time_ZOH')


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='action',required=True)
    video=sub.add_parser('video');video.add_argument('--source',type=Path,required=True);video.add_argument('--output',type=Path,required=True)
    args=p.parse_args(argv);print(json.dumps(export_video(args.source,args.output),ensure_ascii=False));return 0


if __name__=='__main__':raise SystemExit(main())
