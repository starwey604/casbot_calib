"""Manual policy editor: draft/apply, native acknowledgement, no target clamping."""
import copy
import numpy as np
from . import fieldwork_settings as policy

AXIS_LABELS=dict(input_lower='目标位置下限 / rad',input_upper='目标位置上限 / rad',input_velocity='目标速度绝对值 / rad/s',
    input_effort='前馈力矩绝对值 / Nm',kp_max='Kp上限 / Nm/rad',kd_max='Kd上限 / Nm·s/rad',
    observed_lower='反馈位置下限 / rad',observed_upper='反馈位置上限 / rad',observed_velocity='反馈速度绝对值 / rad/s')
GENERAL_LABELS=dict(recording_start_timeout_s='记录器启动等待 / s',command_hz='发送频率 / Hz',start_velocity='启动停稳速度 / rad/s',start_stable_s='启动持续稳定 / s',stop_velocity='停止观察速度 / rad/s',
    stop_stable_s='停止持续稳定 / s',stop_timeout_s='停止观察最长等待 / s',feedback_timeout_s='关节回调超时 / s',
    mode_timeout_s='模式回调超时 / s',rpc_timeout_s='模式切换等待 / s',heartbeat_timeout_s='窗口心跳超时 / s',
    maximum_duration_s='控制最长时长 / s（0=不限）',short_amplitude_deg='短轨迹幅度上限 / °',short_min_s='短轨迹最短时长 / s',short_max_s='短轨迹最长时长 / s')
MOCK_LABELS=dict(effort_limit_nm='前馈接受上限 / Nm',kp_limit='接受Kp上限',kd_limit='接受Kd上限',
    torque_limit_nm='物理内核总力矩限幅 / Nm',watchdog_s='断流看门狗 / s')


class SettingsEditor:
    def __init__(self,desk):
        self.desk=desk;self.open=False;self.selected_scope=0;self.error=None;self.usable=True
        self.ack=None;self.waiting=None;self.notice='';self.pinned=None;self.requests={};self.accepted_observation=None
        fallback=policy.defaults(desk.config,desk.names,desk.status['model_context'])
        try:
            self.settings=policy.validate(policy.load(desk.root,fallback),desk.names,desk.status['model_context'],desk.model_lower,desk.model_upper)
            if not desk.read_only and not desk.preview and not self.settings['revision']:self.settings=policy.save(desk.root,self.settings)
        except (OSError,ValueError,KeyError,TypeError) as exc:
            self.error='已保存限制无法读取；恢复默认或修正后应用：'+str(exc);self.settings=fallback;self.usable=False
        self.draft=copy.deepcopy(self.settings)
        self.recorded=bool(list((desk.root/'settings/versions').glob('*.json')))

    def bounds(self):
        s=self.pinned if self.pinned is not None and self.desk.busy else self.settings
        return policy.command_bounds(s,self.desk.model_lower,self.desk.model_upper)

    def prepare(self):
        if not self.usable:raise ValueError(self.error)
        self.pinned=copy.deepcopy(self.settings);self.ack=None;self.waiting=self.settings['revision'];self.accepted_observation=None
        self.requests={self.settings['revision']:copy.deepcopy(self.settings)}
        return copy.deepcopy(self.settings)

    def handle(self,event):
        if event['code']=='settings_applied':
            self.ack=event['revision']
            self.accepted_observation=self.requests.get(self.ack)
            if self.waiting is not None and self.ack>=self.waiting:self.waiting=None
            self.notice=f'发送器已确认观察设置 v{self.ack}；其它设置按本次起始快照执行。'
        else:
            self.waiting=None;self.notice='发送器拒绝更新，保留已确认版本：'+event.get('message','查看日志')

    def apply(self):
        d=self.desk
        try:
            valid=policy.validate(self.draft,d.names,d.status['model_context'],d.model_lower,d.model_upper)
            old_mock=self.settings['mock']
            self.settings=policy.save(d.root,valid);self.draft=copy.deepcopy(self.settings);self.usable=True;self.error=None;self.recorded=True
            if old_mock!=valid['mock'] or valid['mock']!=d.config.get('mock_limits',policy.MOCK_DEFAULTS):policy.save_mock_next(self.settings)
            if d.busy and d.pending['action']=='stream':
                path=d.control_root/d.active_run/'limits.csv'
                if path.exists():
                    policy.write_wire(path,self.settings,d.model_lower,d.model_upper,pinned=self.pinned)
                    self.waiting=self.settings['revision'];self.requests[self.waiting]=copy.deepcopy(self.settings);self.notice=f'已保存 v{self.settings["revision"]}，等待发送器确认观察阈值；其它项下次控制生效。'
                else:self.notice='已保存；发送器尚在启动，本次仍用启动快照，下次控制生效。'
            else:self.notice='设置已保存；下次开始控制生效。'
            d.sync_bounds()
        except (OSError,ValueError,KeyError,TypeError) as exc:self.error='应用未完成：'+str(exc)

    def gui(self,remember):
        if not self.open:return
        from imgui_bundle import imgui
        d=self.desk
        imgui.set_next_window_size(imgui.ImVec2(930,760),imgui.Cond_.first_use_ever)
        visible,self.open=imgui.begin('限制设置',self.open)
        if not visible:imgui.end();return
        imgui.text(f'保存版本 v{self.settings["revision"]}    '+('观察设置确认 v'+str(self.ack) if self.ack is not None else '当前无发送器确认'))
        if self.pinned is not None and d.busy:imgui.text_disabled(f'本次输入/等待/通信快照 v{self.pinned["revision"]}；新值下次控制生效')
        if self.waiting is not None:imgui.text_colored(imgui.ImVec4(.58,.34,.08,1),f'等待发送器确认 v{self.waiting}')
        if d.read_only and not self.recorded:imgui.text_wrapped('旧记录未保存可调策略；下面是编辑器参考默认，不代表旧控制实际使用值。')
        if self.notice:imgui.text_wrapped(self.notice)
        if self.error:imgui.text_colored(imgui.ImVec4(.68,.14,.12,1),self.error)
        imgui.begin_disabled(d.read_only or d.preview)
        if imgui.button('应用设置'):self.apply()
        remember('limits_apply');imgui.same_line()
        if imgui.button('撤销草稿'):self.draft=copy.deepcopy(self.settings);self.error=None
        imgui.same_line()
        if imgui.button('恢复默认到草稿'):
            self.draft=policy.defaults(d.config,d.names,d.status['model_context']);self.error=None
        imgui.end_disabled();imgui.separator()
        _,self.selected_scope=imgui.combo('修改范围',self.selected_scope,['全轴默认','所选轴覆盖：'+d.names[d.selected]])
        def numeric(label,value,key,*,signed=False):
            imgui.push_id(key);imgui.begin_disabled(d.read_only or d.preview)
            lo=min(-1.,float(value)*2) if signed else 0.
            hi=max(1.,abs(float(value))*2)
            imgui.text(label);imgui.set_next_item_width(430)
            changed,new=imgui.slider_float('##slider',float(value),lo,hi,'%.5g');remember('limit_'+key)
            imgui.same_line();imgui.set_next_item_width(210)
            precise,exact=imgui.input_double('##exact',float(new if changed else value),format='%.8g');remember('limit_exact_'+key)
            imgui.end_disabled();imgui.pop_id();return exact if precise else new if changed else value
        def axis_row(key):
            name=d.names[d.selected];default=self.draft['defaults'];overrides=self.draft['overrides']
            row=default if self.selected_scope==0 else overrides.setdefault(name,{})
            imgui.push_id(key);imgui.begin_disabled(d.read_only or d.preview)
            if self.selected_scope:
                own=key in row
                _,choice=imgui.combo('来源',1 if own else 0,['继承全轴默认','此轴单独设置'])
                if choice and not own:row[key]=default[key]
                elif not choice and own:row.pop(key)
                imgui.begin_disabled(not choice)
            value=row.get(key,default[key])
            if key=='observed_action':
                changed,new=imgui.combo('速度越限动作',int(value),['仅告警，继续发送','结束本次控制'])
                remember('limit_observed_action')
                if changed:row[key]=new
            else:
                if key.endswith(('lower','upper')):
                    _,custom=imgui.combo('位置边界来源',0 if value is None else 1,['各轴URDF参考范围','自定义输入值'])
                    if custom and value is None:value=float(d.model_lower[d.selected] if key.endswith('lower') else d.model_upper[d.selected]);row[key]=value
                    elif not custom and value is not None:row[key]=None;value=None
                if value is None:imgui.text_disabled(AXIS_LABELS[key]+'：按各轴URDF')
                else:
                    new=numeric(AXIS_LABELS[key],value,key,signed=key.endswith(('lower','upper')))
                    if new!=value:row[key]=float(new)
            if self.selected_scope:imgui.end_disabled()
            imgui.end_disabled();imgui.pop_id()
        def tab(label,key):
            value=imgui.begin_tab_item(label)[0];remember('limits_tab_'+key);return value
        if imgui.begin_tab_bar('limits_tabs'):
            if tab('反馈观察 · 可在线应用','observed'):
                imgui.text_wrapped('位置越界只告警/记录；结束动作只作用于速度越限。目标速度不是实测速度限制器。')
                for key in policy.OBSERVED:axis_row(key)
                imgui.end_tab_item()
            if tab('目标输入 · 下次控制','input'):
                imgui.text_wrapped('改变范围不会移动或夹取已有目标；范围冲突会列出具体轴和值。')
                for key in policy.AXIS_KEYS[:6]:axis_row(key)
                imgui.end_tab_item()
            if tab('等待与通信 · 下次控制','general'):
                imgui.text_wrapped('启动等到停稳后继续；停止只观察，不自动回位。超时基于本地回调间隔，不是设备采样年龄。')
                imgui.text_disabled(f'本次profile vel_scale={d.config.get("vel_scale",.1):g}；不是已验证的实际限速倍率')
                for key in policy.GLOBAL_KEYS:
                    if key=='command_hz':
                        rates=[100,250,500];imgui.begin_disabled(d.read_only or d.preview);_,index=imgui.combo(GENERAL_LABELS[key],rates.index(int(self.draft['general'][key])),[str(x) for x in rates]);self.draft['general'][key]=float(rates[index]);imgui.end_disabled();continue
                    self.draft['general'][key]=float(numeric(GENERAL_LABELS[key],self.draft['general'][key],key))
                imgui.end_tab_item()
            if tab('Mock · 下次启动','mock'):
                current=d.config.get('mock_limits')
                imgui.text_wrapped('应用后保存到下次 gui --mock；不会重启当前实例。真实设备的接受范围与保护未知。')
                imgui.text_wrapped('本次Mock：'+str(current) if current else '当前连接未声明Mock设备阈值；不推断实机限制。')
                for key in policy.MOCK_DEFAULTS:
                    self.draft['mock'][key]=float(numeric(MOCK_LABELS[key],self.draft['mock'][key],'mock_'+key))
                imgui.end_tab_item()
            imgui.end_tab_bar()
        imgui.end()
