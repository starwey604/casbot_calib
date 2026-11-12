"""One Chinese rendering of native control events, shared by GUI and terminal records."""
JOINT_LABELS = ['肩俯仰','肩滚转','肩偏航','肘俯仰','腕偏航','腕俯仰','腕滚转']
_SUFFIXES = ['shoulder_pitch_joint','shoulder_roll_joint','shoulder_yaw_joint','elbow_pitch_joint',
             'wrist_yaw_joint','wrist_pitch_joint','wrist_roll_joint']
TITLES = {
 'frontend_heartbeat_timeout':'界面超过配置时限未更新存活信号，本次控制已结束',
 'frontend_owner_changed':'控制窗口所有者发生变化，本次控制已结束',
 'frontend_clock_mismatch':'界面与发送器钟域不一致，本次控制已结束',
 'frontend_heartbeat_invalid':'界面存活消息格式无效，本次控制已结束',
 'raw_recorder_exited':'控制记录进程提前退出，本次控制已结束',
 'observation_exited':'只读观测进程意外退出，本次控制已结束',
 'execution_timeout':'本次控制达到配置的最长时长，已结束发送',
 'status_channel_failed':'后台状态输出通道失败，本次控制已结束',
 'startup':'控制进程已启动，正在读取反馈',
 'waiting_feedback':'等待完整关节位置和速度反馈',
 'waiting_mode_feedback':'等待新的模式反馈',
 'waiting_stationary':'等待关节停稳',
 'waiting_start_command':'反馈已就绪，等待开始发送',
 'waiting_start_mode':'等待可切入上身调试的起始模式',
 'waiting_command_owner':'等待其他命令发布者退出',
 'waiting_mode_service':'等待上身模式服务被发现',
 'request_upper_debug':'正在请求上身调试模式',
 'mode_confirmed':'模式已确认，正在发送五字段指令',
 'waiting_stationary_window':'关节速度已符合条件，等待持续稳定窗口',
 'duration_complete':'达到设置的控制时长，已停止发送',
 'observed_warning':'关节反馈超出观察阈值；发送继续',
 'observed_recovered':'反馈已回到观察范围；发送继续',
 'settings_applied':'限制版本已由发送器确认',
 'settings_rejected':'发送器未采纳新限制；保留原设置',
 'observed_limit':'收到的关节反馈超限，已停止发送',
 'stale_feedback':'关节反馈接收超时，已停止发送',
 'stale_mode':'模式消息接收超时，已停止发送',
 'mode_rejected':'SDK拒绝切入上身调试模式，本次控制已结束',
 'mode_transition_timeout':'模式请求后未在规定时间内完成切入，本次控制已结束',
 'competing_command_publisher':'检测到另一个命令发布者，本次控制已结束',
 'control_mode_lost':'运行中失去上身调试模式，本次控制已结束',
 'invalid_feedback':'反馈缺轴、缺速度或含无效数值，本次控制已结束',
 'invalid_manual_command':'发送目标文件格式或数值范围无效，本次控制已结束',
 'interrupted':'已取消启动或停止发送，正在保存记录',
 'observing_stop':'已停止发送，继续观测关节是否停稳',
 'observing_stop_no_feedback':'已停止发送，但没有新的关节反馈可判断是否停稳',
 'stop_unconfirmed':'已停止发送，规定时间内未观测到全部关节持续停稳',
 'stream_stopped_and_feedback_rest_observed':'已停止发送，并观测到全部关节持续停稳',
 'stop_ack_and_observed_rest':'停止请求已应答，并观测到关节停稳',
 'initial_feedback_or_mode_timeout':'初始反馈或模式等待超时',
 'required_service_unavailable':'上身模式服务不可用',
 'unexpected_start_mode':'当前模式不允许切入上身调试',
 'recording_io_error':'录制写入失败；只读反馈仍可用，当前数据未完整保存',
}


def joint_label(name):
    for prefix,side in [('left_','左'),('right_','右')]:
        if name.startswith(prefix) and name[len(prefix):] in _SUFFIXES:
            return side+JOINT_LABELS[_SUFFIXES.index(name[len(prefix):])]
    return name or '连接'


def describe(native):
    code=native['code'];action=native.get('action','')
    result=dict(native,title=TITLES.get(code,'后台状态：'+code))
    result['severity']='error' if action=='end_control' else 'warning' if code in ('stop_unconfirmed','recording_io_error','observed_warning','settings_rejected') else 'info'
    if action=='wait':
        result['effect']='尚未切模式、未发送指令；条件满足后继续待启动请求，可随时取消。'
        if code=='waiting_start_command':result['effect']='尚未发送指令；点击开始发送后才会切模式。'
    elif action=='request_mode':result['effect']='已发出模式请求，等待SDK应答；尚未发送关节指令。'
    elif action=='warning':result['effect']='仅告警，不改变五字段目标；位置越界只记录，速度越限动作可在限制面板调整。'
    elif action=='recording_warning':result['effect']=native.get('message','')
    elif action=='settings':result['effect']=native.get('message','版本 '+str(native.get('revision','')))
    elif action=='sending':result['effect']='五字段发送中；没有隐藏重力补偿。'
    elif action=='end_control':result['effect']='当前控制不会自动恢复；只读连接继续，可以查看反馈和记录。'
    elif action=='observe_after_stop':result['effect']='停止发布不等于物理急停；只读反馈继续更新。'
    elif action=='control_ended':result['effect']='本次控制已结束；只读连接保留。'
    else:result['effect']=''
    result['details']=[]
    for value in native.get('observations',[]):
        number=value.get('value');threshold=value.get('threshold');field=value.get('field');unit=value.get('unit','')
        actual='尚未收到' if number is None else f'{number:.5f}'
        expected=f'{threshold:.5f}' if threshold is not None else '未知'
        if field=='velocity':detail=f"{joint_label(value.get('joint',''))}：反馈速度 {actual} {unit}，|速度|阈值 {expected} {unit}"
        elif field=='position_lower':detail=f"{joint_label(value.get('joint',''))}：位置 {actual} {unit}，下限 {expected} {unit}"
        elif field=='position_upper':detail=f"{joint_label(value.get('joint',''))}：位置 {actual} {unit}，上限 {expected} {unit}"
        elif field in ('feedback_callback_age','mode_callback_age'):detail=f"接收间隔 {actual} {unit}，阈值 {expected} {unit}（不是设备采样年龄）"
        else:detail=f"{field}：{actual}，允许值 {expected} {unit}"
        result['details'].append(detail)
    if code in ('waiting_start_mode','control_mode_lost'):result['details'].append('当前模式：'+native.get('mode','尚未收到'))
    return result


class OperatorStatus:
    def __init__(self):
        self.current=None;self.first_fault=None;self.history=[]
    def accept(self, event):
        self.current=event
        if event.get('action')=='end_control' and self.first_fault is None:self.first_fault=event
        if not self.history or (self.history[-1]['code'],self.history[-1].get('phase'))!=(event['code'],event.get('phase')):
            self.history.append(event)
        else:self.history[-1]=event
    @property
    def displayed(self):return self.first_fault or self.current
