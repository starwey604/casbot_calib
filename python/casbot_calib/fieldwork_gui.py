"""Dear ImGui commissioning desk; no SDK publishers or control loop in the UI."""
from collections import deque
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

import numpy as np
import yaml

from .fieldwork import JsonTail, clock_id
from .fieldwork_analysis import finite_array, read_json, write_json

JOINT_LABELS = ['肩俯仰', '肩滚转', '肩偏航', '肘俯仰', '腕偏航', '腕俯仰', '腕滚转']


def claim_frontend(root):
    import fcntl
    lease = (Path(root)/'.frontend.lock').open('a+')
    try:
        fcntl.flock(lease.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lease.close()
        raise ValueError('同一会话已有GUI操作台，不能打开第二个控制窗口') from None
    return lease


class Desk:
    def __init__(self, root, *, preview=False, profile='configs/hardware_field.yaml'):
        from .fieldwork_control import Session
        from .model_runtime import model_for
        self.root = Path(root).resolve()
        self.preview = preview
        self.frontend_lease = None if preview else claim_frontend(root)
        self.owner_id = uuid.uuid4().hex
        if preview:
            from .hardware_field import load_profile
            config = load_profile(profile); engine, _ = model_for(config)
            self.status = dict(joint_names=[j.name for j in engine.model.joints], model_context=config['model_context'],
                accepted_joints=[], blocked_reason=None, snapshot=None, short_enabled=False, site_conditions_ready=False,
                domain_id=None, scope='offline_preview', reviews={})
        else:
            session = Session(root); self.status = session.status()
            config = session.config; engine, _ = model_for(config)
        self.axes = [np.asarray(j.axis).tolist() for j in engine.model.joints]
        self.names = self.status['joint_names']
        self.selected = 0
        self.amplitude = .5
        self.short_mode = 0
        self.observer = self.physical_label = self.note = ''
        self.same_axis = self.same_direction = self.execute_confirmed = False
        self.identify_confirmed = False
        self.plan = None
        self.executed = None
        self.response = None
        self.pending = None
        self.process = None
        self.log = None
        self.controls = {}
        self.control_limits = {}
        self.timeline = deque(maxlen=8)
        self.history = deque(maxlen=600)
        self.q = None
        self.receive_ns = None
        self.mode = '未读取'
        self.tail = None
        self.tail_clock_checked = False
        self.heartbeat = self.root/'frontend_heartbeat.json'
        self.local_clock = clock_id()
        self.frame = 0
        self.cancel_reason = None
        self.image = str(self.root/'view/nominal.png')
        self.image_label = '名义模型示意 / 尚未读取机器人姿态'
        from .fieldwork_worker import render_geometry
        nominal = yaml.safe_load(Path('configs/sdk_mock.yaml').read_text())['initial_q']
        try: render_geometry(self.status, nominal, self.names[0], self.image)
        except BaseException:
            if self.frontend_lease: self.frontend_lease.close()
            raise
        self.recorded_image = self.image
        self.timeline.append('先读取实际起姿，再选择一个关节做方向检查。')

    @property
    def busy(self):
        return self.process is not None

    def pulse(self, cancel=None):
        row = dict(clock_id=self.local_clock, owner_id=self.owner_id, monotonic_ns=time.monotonic_ns(), cancel=cancel)
        pending = self.heartbeat.with_suffix('.pending')
        write_json(pending, row); os.replace(pending, self.heartbeat)

    def start(self, action, **values):
        if self.preview or self.busy:
            raise ValueError('离线预览或已有操作运行中，不能重复发送')
        token = uuid.uuid4().hex[:12]
        directory = self.root/'operations'; directory.mkdir(exist_ok=True)
        request = directory/(token+'_request.json')
        output = directory/(token+'_result.json')
        write_json(request, dict(action=action, values=values))
        self.pulse()
        self.cancel_reason = None
        self.log = (directory/(token+'.log')).open('x')
        self.pending = dict(action=action, values=values, output=output)
        self.process = subprocess.Popen([sys.executable, '-m', 'casbot_calib.fieldwork_worker',
            '--session', str(self.root), '--request', str(request), '--heartbeat', str(self.heartbeat),
            '--owner-id', self.owner_id,
            '--output', str(output)], stdout=self.log, stderr=subprocess.STDOUT)
        self.timeline.append('运行：'+dict(snapshot='只读起姿', plan='参考规划与连续检查', execute='执行本次动作', review='保存人工结论', report='导出现场报告')[action])
        self.execute_confirmed = False
        if action == 'execute':
            path = self.root/values['relative']/'capture/raw_state/raw/packets.jsonl'
            self.tail = JsonTail(path)
            self.tail_clock_checked = False
            self.history.clear()

    def poll(self):
        if self.tail:
            metadata = read_json(self.tail.path.parent/'recorder.json')
            if metadata and not self.tail_clock_checked:
                if metadata['clock_id'] != self.local_clock:
                    self.cancel_reason = 'frontend_clock_mismatch'
                    self.timeline.append('接收钟域不一致，停止发布；不计算数据年龄。')
                    self.tail = None
                else:
                    self.tail_clock_checked = True
        if self.tail and self.tail_clock_checked:
            for packet in self.tail.read():
                if packet['channel'] == 'mode':
                    self.mode = packet['message'].get('data', '未知')
                if packet['channel'] != 'joint':
                    continue
                msg = packet['message']; names = msg.get('name', [])
                if (len(set(names)) == len(names) and set(self.names) <= set(names) and finite_array(msg.get('position'), len(names))):
                    self.q = [msg['position'][names.index(name)] for name in self.names]
                    self.receive_ns = packet['receive_ns']
                    self.history.append(self.q[self.selected])
        if not self.process or self.process.poll() is None:
            return
        self.log.close()
        response = read_json(self.pending['output'])
        action = self.pending['action']
        self.process = self.log = None
        self.response = response
        self.tail = None
        if response is None:
            from .fieldwork_control import Session
            self.status = Session(self.root).status()
            self.timeline.append('操作进程异常退出；保留日志，未封存的执行不得续跑。')
            return
        self.status = response['status']
        if not response['completed']:
            self.timeline.append('操作拒绝：'+response['error'])
            return
        result = response['result']
        if 'image' in result:
            self.image = result['image']
            self.image_label = '已记录起姿 / 非实时模型' if action == 'snapshot' else '计划正端点 / 真实幅度，不放大角度'
            if action == 'snapshot': self.recorded_image = self.image
        if result.get('preview_error'):
            self.timeline.append('几何预览失败：'+result['preview_error'])
        if action == 'snapshot':
            self.q = result['q_rad']; self.plan = self.executed = None
            self.receive_ns = result['receive_ns']
            self.history.clear(); self.tail = None
            self.timeline.append('实际起姿已保存；执行器仍将在发送前复核起姿。')
        elif action == 'plan':
            self.plan = result; self.executed = None
            self.timeline.append(f"完整参考检查通过：{result['duration_s']:.1f}秒；尚未发送命令。")
        elif action == 'execute':
            self.executed = self.pending['values']['relative'] if result['passed'] else None
            self.timeline.append('执行与观测分析通过；请填写实际观察。' if result['passed'] else '本次执行未通过：'+result.get('reason', '查看分析记录'))
        elif action == 'review':
            self.timeline.append('人工结论已绑定本次观测并保存。')
            self.same_axis = self.same_direction = False
        elif action == 'report':
            self.timeline.append('报告已保存：'+str(self.root/result['report_dir']))

    def close(self):
        try:
            if self.process:
                self.pulse('frontend_closed')
                try: self.process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    self.process.terminate(); self.process.wait(timeout=5)
                self.process = None
        finally:
            if self.log: self.log.close(); self.log = None
            if self.frontend_lease: self.frontend_lease.close(); self.frontend_lease = None

    def reset_selection(self, index):
        self.selected = index
        self.plan = self.executed = None
        self.physical_label = ''
        self.identify_confirmed = self.same_axis = self.same_direction = self.execute_confirmed = False
        self.history.clear()
        self.image = self.recorded_image
        self.image_label = '最近起姿记录 / 非实时模型' if self.q is not None else '名义模型示意 / 尚未读取机器人姿态'

    def gui(self):
        from imgui_bundle import imgui, hello_imgui
        self.frame += 1
        self.poll()
        font = imgui.get_font_size()
        width = imgui.get_content_region_avail().x
        self.controls = {}
        self.control_limits = {}
        def remember(key):
            a, b = imgui.get_item_rect_min(), imgui.get_item_rect_max()
            self.controls[key] = ((a.x+b.x)/2, (a.y+b.y)/2)
            pos, size = imgui.get_window_pos(), imgui.get_window_size()
            self.control_limits[key] = (pos.y+font*.6, pos.y+size.y-font*.6)
        def button(key, label, enabled=True):
            imgui.begin_disabled(not enabled)
            clicked = imgui.button(label, imgui.ImVec2(0, font*1.8)); remember(key)
            imgui.end_disabled()
            return clicked
        def checkbox(key, label, value):
            _, value = imgui.checkbox(label, value); remember(key)
            return value
        def input_text(key, label, value):
            imgui.set_next_item_width(-1)
            _, value = imgui.input_text('##'+key, value); remember(key)
            imgui.text_disabled(label)
            return value
        if hasattr(self, 'heading_font'):
            imgui.push_font(self.heading_font)
        imgui.text_colored(imgui.ImVec4(.10, .22, .36, 1), 'CASBOT  现场联调台')
        if hasattr(self, 'heading_font'):
            imgui.pop_font()
        imgui.same_line()
        imgui.text_disabled('离线预览' if self.preview else f"域号 {self.status['domain_id']} / {self.status['scope']}")
        count = len(self.status['accepted_joints'])
        imgui.text(f'实物映射与方向确认  {count} / 14')
        imgui.same_line()
        if button('snapshot', '读取实际起姿', not self.preview and not self.busy):
            self.start('snapshot')
        imgui.same_line()
        if button('stop', '结束试验 / 停止发布', self.busy and self.pending['action'] == 'execute'):
            self.cancel_reason = 'operator_cancel'
            self.pulse('operator_cancel')
            self.timeline.append('已请求停止发布；物理状态由现场确认。')
        imgui.same_line()
        if button('report', '保存现场报告', not self.preview and not self.busy):
            self.start('report')
        imgui.separator()
        available = imgui.get_content_region_avail()
        height = max(font*20, available.y-font*5.8)
        left, middle = width*.23, width*.44
        imgui.begin_child('joint_list', imgui.ImVec2(left, height), imgui.ChildFlags_.border)
        imgui.text('逐轴核对')
        imgui.progress_bar(count/14, imgui.ImVec2(-1, font), f'{count} / 14')
        imgui.begin_disabled(self.busy)
        for index, name in enumerate(self.names):
            if index in (0, 7):
                imgui.spacing(); imgui.text('左臂' if index == 0 else '右臂')
            label = JOINT_LABELS[index%7] + ('   已确认' if name in self.status['accepted_joints'] else '   待核对')
            clicked, _ = imgui.selectable(label+'##'+name, self.selected == index, size=imgui.ImVec2(0, font*1.6))
            remember('joint_'+str(index))
            if clicked:
                self.reset_selection(index)
        imgui.end_disabled()
        imgui.spacing()
        imgui.text_wrapped('SDK使用具名轴，不把数组下标当作电机ID。')
        imgui.end_child(); imgui.same_line()
        imgui.begin_child('geometry', imgui.ImVec2(middle, height), imgui.ChildFlags_.border)
        shown_joints = self.plan['joints'] if self.plan else [self.names[self.selected]]
        imgui.text_wrapped(', '.join(shown_joints))
        imgui.text_disabled(self.image_label)
        image_width = imgui.get_content_region_avail().x
        hello_imgui.image_from_asset(str(Path(self.image).relative_to(self.root)), imgui.ImVec2(image_width, image_width*.75))
        for name in shown_joints:
            index = self.names.index(name)
            imgui.text_wrapped(f'{name}: 局部正轴 {self.axes[index]}，按右手定则旋转。')
        if self.q is not None:
            imgui.text(f'最近记录位置：{self.q[self.selected]:+.5f} rad')
        if self.history:
            imgui.plot_lines('接收位置 / rad', np.array(self.history, dtype=np.float32), graph_size=imgui.ImVec2(-1, font*5))
        elif self.plan:
            data = next(iter(self.plan['plot']['q'].values()))
            imgui.plot_lines('计划位置 / rad', np.array(data, dtype=np.float32), graph_size=imgui.ImVec2(-1, font*5))
            imgui.text_disabled(f"计划时长 {self.plan['duration_s']:.1f}s；完整接近与回位已检查")
        if self.receive_ns is not None:
            imgui.text_disabled(f'最后回调距今 {(time.monotonic_ns()-self.receive_ns)/1e9:.2f}s；非设备数据年龄')
        imgui.end_child(); imgui.same_line()
        imgui.begin_child('actions', imgui.ImVec2(0, height), imgui.ChildFlags_.border)
        blocked = self.status['blocked_reason']
        if blocked:
            imgui.text_colored(imgui.ImVec4(.65, .16, .10, 1), '会话终止')
            imgui.text_wrapped(blocked)
        elif not self.status['site_conditions_ready']:
            imgui.text_wrapped('只读可用；运动等待现场前提证据。见site_conditions模板。')
        imgui.begin_disabled(self.busy)
        imgui.text('单轴方向检查')
        self.physical_label = input_text('physical', '实物关节名称 / 现场填写', self.physical_label)
        self.identify_confirmed = checkbox('identify', '我已辨认所选实物关节', self.identify_confirmed)
        changed, self.amplitude = imgui.input_float('幅度 / 度', self.amplitude, .1, .25, '%.2f')
        if changed:
            self.plan = None; self.execute_confirmed = False
        can_plan = not self.preview and not blocked and self.status['site_conditions_ready'] and self.status['snapshot'] is not None
        if button('plan_jog', '规划单轴正反往返', can_plan and self.identify_confirmed and bool(self.physical_label.strip())):
            self.start('plan', kind='jog', snapshot=self.status['snapshot'], joints=[self.names[self.selected]], amplitude_deg=self.amplitude)
        if self.plan and not self.executed:
            self.execute_confirmed = checkbox('execute_confirm', '已检查预览和现场条件', self.execute_confirmed)
            if button('execute', '执行本次计划', self.execute_confirmed and can_plan):
                self.start('execute', relative=self.plan['attempt'])
        imgui.separator()
        imgui.text('实际观察结论')
        self.observer = input_text('observer', '观察者', self.observer)
        self.same_axis = checkbox('same_axis', '动作发生在预期实物关节', self.same_axis)
        self.same_direction = checkbox('same_direction', '正向与模型预期一致', self.same_direction)
        self.note = input_text('note', '备注 / 可填写照片或视频路径', self.note)
        reviewable = bool(self.executed and self.plan and self.plan['kind'] == 'jog' and self.observer.strip() and self.physical_label.strip())
        if button('review', '保存该轴人工确认', reviewable and self.same_axis and self.same_direction):
            self.start('review', relative=self.executed, observer=self.observer, physical_label=self.physical_label,
                same_axis=True, same_direction=True, note=self.note)
        if button('reject', '映射或方向不一致，终止会话', reviewable and bool(self.note.strip())):
            self.start('review', relative=self.executed, observer=self.observer, physical_label=self.physical_label,
                same_axis=self.same_axis, same_direction=False, note=self.note)
        imgui.separator()
        imgui.text('短轨迹')
        imgui.text_wrapped('全部14轴确认后开放；每次从新实测起姿规划。')
        for index, label in enumerate(['所选单轴', '同侧肩肘', '双臂肩俯仰']):
            if index:
                imgui.same_line()
            if imgui.radio_button(label, self.short_mode == index):
                self.short_mode = index; self.plan = None; self.execute_confirmed = False
            remember('short_mode_'+str(index))
        if button('plan_short', '规划短轨迹', can_plan and self.status['short_enabled']):
            arm = 'left' if self.selected < 7 else 'right'
            joints = [[self.names[self.selected]], [arm+'_shoulder_pitch_joint', arm+'_elbow_pitch_joint'],
                      ['left_shoulder_pitch_joint', 'right_shoulder_pitch_joint']][self.short_mode]
            self.start('plan', kind='short', snapshot=self.status['snapshot'], joints=joints, amplitude_deg=self.amplitude)
        imgui.end_disabled(); imgui.end_child()
        imgui.separator()
        if self.busy:
            imgui.text_colored(imgui.ImVec4(.14, .36, .61, 1), '后台操作运行中；界面不承担命令发送。')
        for message in list(self.timeline)[-3:]:
            imgui.text_wrapped(message)
        if not self.preview:
            self.pulse(getattr(self, 'cancel_reason', None))


def run_gui(args):
    from imgui_bundle import imgui, hello_imgui
    from .fieldwork_control import create_session
    if args.screenshot and args.screenshot.exists():
        raise ValueError('截图输出已存在，请使用新文件')
    if args.frames is not None and args.frames <= 0:
        raise ValueError('--frames必须为正整数')
    if args.preview:
        if args.session:
            raise ValueError('离线预览请使用新output目录，不加载可执行会话')
        if args.output is None:
            raise ValueError('请为离线预览指定新--output目录')
        args.output.mkdir(parents=True, exist_ok=False)
        root = args.output
    elif args.session:
        if args.output is not None or args.domain_id is not None or args.scope is not None:
            raise ValueError('已有会话使用封存的域号、scope和profile；不能同时提供新连接参数')
        root = args.session
    else:
        if args.output is None or args.domain_id is None or args.scope is None:
            raise ValueError('新会话须显式提供--output、--domain-id和--scope')
        create_session(args.profile, args.output, args.domain_id, args.scope, args.conditions)
        root = args.output
    desk = Desk(root, preview=args.preview, profile=args.profile)
    hello_imgui.set_assets_folder(str(Path(root).resolve()))
    params = hello_imgui.RunnerParams()
    params.app_window_params.window_title = 'CASBOT Fieldwork'
    params.app_window_params.window_geometry.size = [1440, 980]
    params.app_window_params.restore_previous_geometry = False
    params.ini_filename = str(Path(root)/'window.ini')
    params.imgui_window_params.show_menu_bar = False
    params.imgui_window_params.show_status_bar = False
    # Hello ImGui expresses these margins in em, not pixels.
    params.imgui_window_params.full_screen_window_margin_top_left = imgui.ImVec2(.75, .5)
    params.imgui_window_params.full_screen_window_margin_bottom_right = imgui.ImVec2(.75, .5)
    params.fps_idling.enable_idling = False
    def fonts():
        from .presentation import _font
        path = _font(18).path
        io = imgui.get_io()
        glyphs = [0x20, 0xFF, 0x2000, 0x206F, 0x3000, 0x30FF, 0x4E00, 0x9FFF, 0xFF00, 0xFFEF, 0]
        io.font_default = io.fonts.add_font_from_file_ttf(path, 18, glyph_ranges_as_int_list=glyphs)
        desk.heading_font = io.fonts.add_font_from_file_ttf(path, 25, glyph_ranges_as_int_list=glyphs)
    def style():
        imgui.style_colors_light()
        s = imgui.get_style(); s.window_padding = imgui.ImVec2(16, 12); s.item_spacing = imgui.ImVec2(10, 8)
        s.frame_padding = imgui.ImVec2(8, 5); s.frame_rounding = 3; s.child_rounding = 4
        for name, color in dict(window_bg=(.94,.96,.98,1), child_bg=(.99,.995,1,1),
            text=(.10,.18,.28,1), text_disabled=(.34,.40,.47,1), border=(.77,.82,.87,1),
            button=(.80,.88,.96,1), button_hovered=(.65,.79,.94,1), button_active=(.48,.68,.89,1),
            check_mark=(.12,.40,.69,1), header=(.75,.85,.94,1), plot_lines=(.12,.40,.69,1)).items():
            s.set_color_(getattr(imgui.Col_, name), imgui.ImVec4(*color))
    params.callbacks.load_additional_fonts = fonts
    params.callbacks.setup_imgui_style = style
    def gui():
        desk.gui()
        if args.frames and desk.frame >= args.frames:
            params.app_shall_exit = True
    params.callbacks.show_gui = gui
    params.callbacks.before_exit = desk.close
    try:
        hello_imgui.run(params)
        if args.screenshot:
            from PIL import Image
            Image.fromarray(hello_imgui.final_app_window_screenshot()).save(args.screenshot)
    finally:
        desk.close()
    return 0
