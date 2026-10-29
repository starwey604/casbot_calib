"""S2 clock-honest stream frequency and aggregate position response diagnostics."""
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import BPoly
from scipy.optimize import minimize_scalar
from .vendor_identity import sha256


def rows(path):
    if len(Path(path).read_text().splitlines())<2:return None
    return np.genfromtxt(path,delimiter=',',names=True,dtype=None,encoding='utf-8',ndmin=1)


def rates(times):
    times=np.asarray(times,dtype=np.int64)
    if len(times)<2:return dict(hz=None,interval_ms=None)
    dt=np.diff(times)*1e-6
    return dict(hz=float((len(times)-1)*1e9/(int(times[-1])-int(times[0]))),
                interval_ms={key:float(value) for key,value in zip(('p50','p95','p99','maximum'),[*np.quantile(dt,[.5,.95,.99]),max(dt)])})


def rejected_stop_verified(capture,no_commands_after_fault,bridge_events):
    """Test a negative stop response, independently of which valid guard fired first."""
    return bool(not capture['passed'] and not capture.get('stop_ack') and capture.get('reason')=='stop_unconfirmed'
                and capture.get('fault') in ('control_mode_lost','stale_feedback','tracking_envelope','aged_tracking_envelope')
                and no_commands_after_fault and bridge_events is not None and np.any(bridge_events['event']=='stop_rejected'))


def analyse_position(folder,output):
    from .collision import nominal_profile,certify_polyline
    folder=Path(folder);output=Path(output)
    manifest=json.loads((folder/'manifest.json').read_text())
    for name,digest in manifest.items():
        if sha256(folder/name)!=digest:raise ValueError('Execution raw data changed')
    capture=json.loads((folder/'capture.json').read_text());plan=Path(capture['plan'])
    if sha256(plan/'manifest.json')!=capture['plan_manifest_sha256']:raise ValueError('Plan manifest changed')
    for name,digest in json.loads((plan/'manifest.json').read_text()).items():
        if sha256(plan/name)!=digest:raise ValueError('Plan inputs changed')
    metadata=json.loads((plan/'plan.json').read_text());config=metadata['config']
    commands=rows(folder/'execution/commands.csv');feedback=rows(folder/'execution/feedback.csv');events=rows(folder/'execution/events.csv')
    stream=commands[commands['phase']=='stream'] if commands is not None else None
    report=dict(schema_version='casbot_vendor_s2_analysis_v1',capture=capture,source_sha256=manifest,
                hardware_executable=False,scope='localhost SDK contract + native Mock, position response only',
                configured_command_hz=config['control_hz'],command_publish=rates(stream['publish_ns'] if stream is not None else []),
                feedback_receive=rates(feedback['receive_ns'] if feedback is not None else []),
                sample_time_known=False,command_adoption_hz=None,one_way_network_delay_ms=None,driver_time_constant_s=None,
                mechanical_parameters_refitted=False,response_model='one scalar trajectory shift for diagnostics only; tracking acceptance uses unaligned data',
                timing_scope='client monotonic send and callback times; lag includes control/driver/mechanics, not isolated network delay',
                stop_confirmed=capture.get('stop_ack',False) and capture.get('reason')=='stop_ack_and_observed_rest')
    stopped=events[events['state']=='STOPPING'] if events is not None else []
    report['no_commands_after_fault']=bool(not len(stopped) or commands is None or np.all(commands['publish_ns']<stopped['receive_ns'][0]))
    if feedback is not None and len(feedback)>1:
        q=np.column_stack([feedback['q'+str(j)] for j in range(14)])
        t=(feedback['receive_ns']-feedback['receive_ns'][0])*1e-9
        try:
            report['observed_interpolant']=certify_polyline(nominal_profile(scene=metadata['scene']),q,t,
                    clearance_m=metadata['scene']['clearance_m'],max_gap_s=config['max_feedback_age_ns']*1e-9)
        except ValueError as exc:report['observed_interpolant']=dict(passed=False,reason=str(exc))
        if stream is not None and len(stream)>1:
            origin=int(stream['planned_ns'][0])-int(stream['planned_index'][0])*(1_000_000_000//config['control_hz'])
            with np.load(plan/'reference.npz') as data:curve=BPoly(data['coefficients'],data['breakpoints'])
            executed_end=min(curve.x[-1],(int(stream['planned_index'][-1])+1)/config['control_hz'])
            rel=(feedback['receive_ns']-origin)*1e-9;active=(rel>=0)&(rel<=executed_end);time=rel[active];observed=q[active]
            error=observed-curve(time)
            report['tracking_rmse_rad']=np.sqrt(np.mean(error**2,axis=0)).tolist() if len(time) else None
            if len(time):report['maximum_tracking_error_rad']=float(np.max(abs(error)))
            else:report['tracking_diagnostic_unavailable']='no callback observations during the actually executed prefix'
            # This estimates one aggregate trajectory shift, with no mechanical refit.
            moving=np.ptp(curve(time),axis=0)>.01 if len(time) else np.zeros(14,dtype=bool)
            report['excitation_observed']=bool(np.any(moving))
            if np.any(moving) and len(time)>20:
                objective=lambda lag:float(np.mean((observed[:,moving]-curve(np.clip(time-lag,0,curve.x[-1]))[:,moving])**2))
                fit=minimize_scalar(objective,bounds=(0.,.15),method='bounded',options={'xatol':1e-6})
                report['aggregate_response_lag_ms']=float(fit.x*1000)
                report['response_lag_boundary_limited']=bool(fit.x<.0001 or fit.x>.1499)
            report['active_feedback_receive']=rates(feedback['receive_ns'][active])
    hz=report['command_publish']['hz']
    report['local_frequency_screen_passed']=bool(hz is not None and hz>=.9*config['control_hz'])
    report['passed']=bool(capture['passed'] and report.get('observed_interpolant',{}).get('passed') and
                          report.get('maximum_tracking_error_rad',float('inf'))<=config['tracking_guard_rad'] and
                          report['local_frequency_screen_passed'] and report['no_commands_after_fault'])
    output.mkdir(parents=True,exist_ok=False)
    from .vendor_data import normalize_session
    raw=normalize_session(folder/'raw_state',output/'observations')
    report['raw_vendor_observations']=dict(valid_position_packets=raw['valid_position_packets'],sample_time_known=False,
                                         effort_source=raw['effort_source'],recorder_reason=raw['capture_reason'])
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    for name,digest in manifest.items():
        if sha256(folder/name)!=digest:raise ValueError('Execution raw data changed during analysis')
    return report


def plot_position_summary(report,path):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.font_manager import FontProperties
    from .presentation import _font
    font=FontProperties(fname=_font(12).path)
    normal=[r for r in report['cases'] if not r['case']['fault']]
    figure=Figure(figsize=(14,4.5));FigureCanvasAgg(figure);axes=figure.subplots(1,3)
    x=np.arange(len(normal));labels=[f"{r['case']['hz']} / {r['case']['repeat']+1}" for r in normal]
    axes[0].plot(x,[r['case']['hz'] for r in normal],'.--',label='请求')
    axes[0].plot(x,[r['command_publish']['hz'] for r in normal],'o',label='实际SDK发布')
    axes[0].set_ylabel('赫兹（非设备采用频率）',fontproperties=font);axes[0].legend(prop=font)
    axes[1].bar(x,[r['maximum_tracking_error_rad']*1000 if r['maximum_tracking_error_rad'] is not None else np.nan for r in normal])
    axes[1].axhline(2.,ls='--',color='red');axes[1].set_ylabel('未对齐最大跟踪误差 / 毫弧度',fontproperties=font)
    axes[2].bar(x,[r['response_lag_ms'] if r['response_lag_ms'] is not None else np.nan for r in normal])
    axes[2].set_ylabel('整体轨迹偏移 / 毫秒（非网络单向延迟）',fontproperties=font)
    for axis in axes:
        axis.set_xticks(x,labels,fontproperties=font);axis.set_xlabel('请求频率 / 批次（实际运行顺序）',fontproperties=font);axis.grid(axis='y',alpha=.2)
    figure.suptitle('S2 本机SDK契约与原生Mock：频率、原始跟踪及整体响应分别报告',fontproperties=font)
    figure.tight_layout();figure.savefig(path,dpi=140)
