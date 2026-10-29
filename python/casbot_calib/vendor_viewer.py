"""Vendor pose playback / file-tail live display, without device I/O or physics."""
import json
from pathlib import Path
from time import monotonic_ns, monotonic, sleep
import numpy as np
from .telemetry import StateFrame, TelemetryAdapter, JointMapping
from .vendor_data import normalize_packet
from .vendor_identity import sha256


def pose_frame(config, row, clock):
    if row['q'] is None:
        return None
    return StateFrame(config['model_id'], 'SDK只读观测（采样时刻未知）', row['receive_sequence'], -1,
                      row['receive_ns'], tuple(config['joint_names']), row['q'], row['dq'], None, row['q'],
                      status=None, clock_domain=clock)


class VendorReplaySource:
    def __init__(self, folder):
        root = Path(folder)
        for name, digest in json.loads((root/'manifest.json').read_text()).items():
            if sha256(root/name) != digest:
                raise ValueError('Normalized vendor record changed')
        self.report = json.loads((root/'report.json').read_text())
        self.config = json.loads((root/'session.json').read_text())['config']
        self.scope = json.loads((root/'session.json').read_text())['connection_scope']
        with np.load(root/'observations.npz', allow_pickle=False) as data:
            self.data = {key: data[key] for key in data.files}
        self.origin = int(self.data['receive_ns'][0]); self.duration_ns = int(self.data['receive_ns'][-1])-self.origin
        self._frame = None; self._cursor = 0; self.error = None

    def read(self, presentation_ns):
        self._cursor = int(presentation_ns)
        end = np.searchsorted(self.data['receive_ns'], self.origin+self._cursor, side='right')
        indices = np.flatnonzero(self.data['position_valid'][:end])
        if not len(indices):
            self._frame = None; return None
        i = int(indices[-1]); data = self.data
        row = dict(q=data['q'][i], dq=data['dq'][i] if data['dq_available'][i] else None,
                   receive_ns=int(data['receive_ns'][i]), receive_sequence=int(data['receive_sequence'][i]))
        self._frame = pose_frame(self.config, row, self.report['clock_id'])
        self.error = '最近位置消息无效，保留此前姿态' if end and not data['position_valid'][end-1] else None
        return self._frame

    def is_stale(self):
        return self._frame is None or self.origin+self._cursor-self._frame.receive_ns > self.config['stale_after_ns']


class VendorLiveSource:
    """Tail complete raw lines only; recorder and GUI remain separate processes."""
    def __init__(self, folder):
        self.root = Path(folder)
        session = json.loads((self.root/'session.json').read_text())
        self.config = session['config']; self.scope = session['connection_scope']
        self._stream = None; self._frame = None; self.error = None; self._clock = None

    def read(self, presentation_ns=0):
        path = self.root/'raw/packets.jsonl'
        if self._stream is None:
            if not path.exists() or not (self.root/'raw/recorder.json').exists(): return None
            recorder = json.loads((self.root/'raw/recorder.json').read_text())
            local_clock = 'linux_monotonic:'+Path('/proc/sys/kernel/random/boot_id').read_text().strip()+':'+str(Path('/proc/self/ns/time').readlink())
            if recorder['clock_id'] != local_clock:
                raise ValueError('Live age requires the recorder local monotonic clock; use replay for remote files')
            self._clock = recorder['clock_id']; self._stream = path.open()
        # Bound each render cycle's work. Retain partial lines until completed.
        for _ in range(10000):
            offset = self._stream.tell(); line = self._stream.readline()
            if not line.endswith('\n'):
                self._stream.seek(offset); break
            packet = json.loads(line)
            if packet['channel'] != 'joint': continue
            row = normalize_packet(packet, self.config)
            if row['q'] is None:
                self.error = '位置消息被拒绝：'+','.join(row['errors']); continue
            self._frame = pose_frame(self.config, row, self._clock); self.error = None
        return self._frame

    def is_stale(self):
        return self._frame is None or monotonic_ns()-self._frame.receive_ns > self.config['stale_after_ns']

    def close(self):
        if self._stream is not None: self._stream.close()


def scene_for_vendor(config, output):
    from .staged_presentation import write_scene
    from .collision import nominal_profile
    profile = nominal_profile()
    model_names = [j.name for j in profile.engine.model.joints]
    if config['joint_names'] != model_names:
        raise ValueError('Viewer requires explicit nominal joint names/order; it does not guess a model mapping')
    write_scene(profile, output)
    adapter = TelemetryAdapter(config['model_id'], [JointMapping(n, n) for n in model_names])
    return adapter, profile.engine


def view_vendor(folder, output, *, live=False, window=False, fps=12):
    if live and not window:
        raise ValueError('Live display requires --window; completed normalized sessions support video export')
    source = VendorLiveSource(folder) if live else VendorReplaySource(folder)
    root = Path(folder)
    fingerprints = None if live else {p.name: sha256(p) for p in (root/'observations.npz', root/'report.json', root/'session.json')}
    try:
        result = present_pose_source(source, output, live=live, window=window, fps=fps, source_sha256=fingerprints)
    finally:
        if live: source.close()
    if fingerprints is not None and fingerprints != {p.name: sha256(p) for p in (root/'observations.npz', root/'report.json', root/'session.json')}:
        raise ValueError('Viewer inputs changed')
    return result


def present_pose_source(source, output, *, live=False, window=False, fps=12, duration_s=None, source_sha256=None):
    """Common geometry renderer for replay, file-tail, SDK ROS and native Mock ROS."""
    if not isinstance(fps, int) or fps <= 0:
        raise ValueError('fps must be a positive integer')
    output = Path(output); output.mkdir(parents=True, exist_ok=False)
    adapter, engine = scene_for_vendor(source.config, output/'scene.xml')
    if window:
        from .viewer import show
        show(source, adapter, output/'scene.xml', duration_s=None if live else source.duration_ns*1e-9, loop=not live)
        return dict(read_only=True, live=live)
    if live and (duration_s is None or not np.isfinite(duration_s) or duration_s <= 0):
        raise ValueError('Live video requires a positive duration')
    from PIL import Image, ImageDraw
    import imageio_ffmpeg
    from .viewer import JointView
    from .presentation import _font
    duration_ns = round(duration_s*1e9) if live else source.duration_ns
    count = max(1, int(np.ceil(duration_ns*1e-9*fps))+1); errors = []; frames = 0; displayed = []
    writer = imageio_ffmpeg.write_frames(str(output/'vendor_read_only_zh.mp4'), (1280, 720), fps=fps,
                                        codec='libx264', pix_fmt_in='rgb24', pix_fmt_out='yuv420p', macro_block_size=16)
    writer.send(None)
    try:
        with JointView(adapter, output/'scene.xml', width=900, height=560) as view:
            view.camera.lookat[:] = [0., 0., .18]; view.camera.distance = 1.4
            view.camera.azimuth = 210; view.camera.elevation = -12
            begin = monotonic()
            for k in range(count):
                at = min(duration_ns, round(k/fps*1e9))
                if live: sleep(max(0., begin+at*1e-9-monotonic()))
                frame = source.read(at)
                canvas = Image.new('RGB', (1280,720), (10,17,27)); draw = ImageDraw.Draw(canvas)
                draw.text((24,16), 'CASBOT / 统一只读状态显示', font=_font(26), fill=(228,237,245))
                heading = '独立ROS订阅 / 渲染不参与物理计算' if live else '按本机接收时间回放，采样语义按来源保留'
                draw.text((24,60), heading, font=_font(18), fill=(62,218,189))
                if frame is not None:
                    canvas.paste(Image.fromarray(view.render(frame)), (20,100))
                    expected = engine.fk(frame.q); error = 0.
                    for j, name in enumerate(source.config['joint_names']):
                        body = view.mj.mj_name2id(view.model, view.mj.mjtObj.mjOBJ_BODY, name+'_view')
                        error = max(error, float(np.max(abs(view.data.xpos[body]-expected[j][:3,3]))),
                                    float(np.max(abs(view.data.xmat[body].reshape(3,3)-expected[j][:3,:3]))))
                    errors.append(error)
                    if np.any(view.data.qacc): raise RuntimeError('Viewer computed dynamics')
                    displayed.append(dict(presentation_ns=at, receive_ns=frame.receive_ns, sample_ns=frame.sample_ns,
                                          sequence=frame.sequence, q=frame.q.tolist(), current_available=frame.current is not None))
                labels = [('显示状态', '反馈陈旧，保持最后姿态' if source.is_stale() else '已收到有效位置'),
                          ('记录时间', f'{at*1e-9:.3f} 秒'), ('位置映射', 'nominal名称映射，实机方向待验证'),
                          ('原始电流 / 采样时间', ('有观测' if frame is not None and frame.current is not None else '未知')+' / '+
                           ('来源声明已知' if frame is not None and frame.sample_ns>=0 else '未知')),
                          ('实机控制', '不发命令，不切换模式'), ('消息诊断', source.error or '位置与其他字段独立校验')]
                for j, (label, value) in enumerate(labels):
                    draw.text((930,120+j*78), label, font=_font(16), fill=(151,172,192))
                    # Keep Chinese values within the sidebar.
                    for line, start in enumerate(range(0,len(value),15)):
                        draw.text((930,145+j*78+line*22), value[start:start+15], font=_font(16), fill=(228,237,245))
                scope = getattr(source,'scope','hardware_read_only')
                origin = {'localhost_contract_mock':'本机ROS Mock反馈，具体来源见session',
                          'localhost_ros_mock':'原生Mock经ROS发布，显示端不计算物理'}.get(scope,'硬件域只读反馈，设备身份待核验')
                draw.text((24,680), 'MuJoCo只更新几何 / '+origin+' / 不代表动力学或实机验收',
                          font=_font(14), fill=(245,194,91))
                if k == count//2: canvas.save(output/'vendor_read_only_zh.png')
                writer.send(np.asarray(canvas)); frames += 1
    finally:
        writer.close()
    result = dict(passed=bool(errors and max(errors)<1e-12), read_only=True, angles_amplified=False,
                  frames=frames, max_fk_error=max(errors) if errors else None, source_sha256=source_sha256,
                  mujoco_usage='kinematics/camera/render only', device_writes=0)
    (output/'displayed_frames.json').write_text(json.dumps(displayed, indent=2)+'\n')
    (output/'presentation.json').write_text(json.dumps(result, indent=2)+'\n')
    return result
