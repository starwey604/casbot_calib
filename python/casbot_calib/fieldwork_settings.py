"""Versioned manual-desk policy. No change to sealed connection or H1–H4 limits."""
import copy
import json
from pathlib import Path
import numpy as np
from .hardware_batch import atomic_json

AXIS_KEYS=('input_lower','input_upper','input_velocity','input_effort','kp_max','kd_max',
           'observed_lower','observed_upper','observed_velocity','observed_action')
OBSERVED=AXIS_KEYS[6:]
# observed_action applies to speed; position bounds only produce recorded warnings.
GLOBAL_DEFAULTS=dict(start_velocity=.02,start_stable_s=0.,stop_velocity=.02,stop_stable_s=.1,
    stop_timeout_s=3.,feedback_timeout_s=2.,mode_timeout_s=3.,rpc_timeout_s=3.,
    heartbeat_timeout_s=2.,maximum_duration_s=0.,short_amplitude_deg=1.,short_min_s=2.,short_max_s=12.)
WIRE_GLOBAL_KEYS=tuple(GLOBAL_DEFAULTS)
GLOBAL_DEFAULTS.update(recording_start_timeout_s=10.,command_hz=500.)
GLOBAL_KEYS=tuple(GLOBAL_DEFAULTS)
MOCK_DEFAULTS=dict(effort_limit_nm=100.,kp_limit=1000.,kd_limit=100.,torque_limit_nm=100.,watchdog_s=.1)


def defaults(config,names,context):
    from .fieldwork_control import tuning_config
    t=tuning_config(config)
    return dict(schema_version='casbot_fieldwork_settings_v1',revision=0,joint_names=list(names),model_context=context,
        defaults=dict(input_lower=None,input_upper=None,input_velocity=float(config['maximum_velocity_rad_s']),
            input_effort=t['effort_limit_nm'],kp_max=t['kp_max'],kd_max=t['kd_max'],observed_lower=None,
            observed_upper=None,observed_velocity=float(config['maximum_velocity_rad_s']),observed_action=0),
        overrides={},general=dict(GLOBAL_DEFAULTS,feedback_timeout_s=config['max_feedback_age_ns']*1e-9,
            command_hz=float(config.get('command_hz',500)),mode_timeout_s=config['max_mode_age_ns']*1e-9,rpc_timeout_s=config['transition_timeout_ns']*1e-9),
        mock=dict(MOCK_DEFAULTS,**config.get('mock_limits',{})))


def resolve(settings,lower,upper):
    a={key:[] for key in AXIS_KEYS}
    for i,name in enumerate(settings['joint_names']):
        row=dict(settings['defaults'],**settings['overrides'].get(name,{}))
        for key in AXIS_KEYS:
            value=row[key]
            if value is None:value=lower[i] if key.endswith('lower') else upper[i]
            a[key].append(value)
    return {key:np.asarray(value,dtype=float) for key,value in a.items()}


def validate(settings,names,context,lower,upper):
    s=copy.deepcopy(settings)
    if s.get('schema_version')!='casbot_fieldwork_settings_v1' or s.get('joint_names')!=list(names) or s.get('model_context')!=context:
        raise ValueError('限制设置的模型或14轴顺序与工作区不一致')
    if type(s.get('revision')) is not int or not 0<=s['revision']<=2**53-1:raise ValueError('设置版本无效')
    if set(s['defaults'])!=set(AXIS_KEYS) or set(s['general'])!=set(GLOBAL_KEYS) or set(s['mock'])!=set(MOCK_DEFAULTS):raise ValueError('限制字段不完整或含未知字段')
    if set(s['overrides'])-set(names):raise ValueError('逐轴覆盖含未知轴')
    for row in [s['defaults'],*s['overrides'].values()]:
        if set(row)-set(AXIS_KEYS):raise ValueError('逐轴覆盖含未知字段')
        for key,value in row.items():
            if value is None and key in ('input_lower','input_upper','observed_lower','observed_upper'):continue
            if isinstance(value,bool) or not isinstance(value,(float,int)) or not np.isfinite(value):raise ValueError(key+'须为有限数值')
            if key=='observed_action' and value not in (0,1):raise ValueError('越限动作须为告警或结束控制')
    a=resolve(s,lower,upper)
    for lo,hi in [('input_lower','input_upper'),('observed_lower','observed_upper')]:
        if np.any(a[lo]>=a[hi]):raise ValueError(lo+'必须小于对应上限')
    for key in ('input_velocity','input_effort','kp_max','kd_max','observed_velocity'):
        if np.any(a[key]<=0):raise ValueError(key+'须大于0')
    for key,value in s['general'].items():
        if isinstance(value,bool) or not isinstance(value,(float,int)) or not np.isfinite(value) or value<0:raise ValueError(key+'须为非负有限数值')
        if value==0 and key not in ('start_stable_s','stop_stable_s','maximum_duration_s'):raise ValueError(key+'须大于0')
    if any(s['general'][key]<1e-9 for key in ('stop_timeout_s','feedback_timeout_s','mode_timeout_s','rpc_timeout_s','heartbeat_timeout_s')):raise ValueError('等待和通信时间须至少1纳秒')
    if s['general']['command_hz'] not in (100,250,500):raise ValueError('发送频率支持100/250/500 Hz')
    if s['general']['short_min_s']>s['general']['short_max_s']:raise ValueError('短轨迹最短时长不能大于最长时长')
    # These durations are represented by signed native nanoseconds, not arbitrary UI caps.
    if any(v>9e9 for v in s['general'].values()):raise ValueError('时长超出原生纳秒表示范围')
    for key,value in s['mock'].items():
        if isinstance(value,bool) or not isinstance(value,(float,int)) or not np.isfinite(value) or value<=0:raise ValueError('Mock '+key+'须大于0且有限')
    if s['mock']['watchdog_s']<.01 or s['mock']['watchdog_s']>1.:raise ValueError('Mock看门狗支持0.01–1秒')
    return s


def runtime(settings,lower,upper,*,pinned=None):
    """Only observed fields can change during one native control."""
    a=resolve(settings,lower,upper);g=settings['general']
    if pinned is not None:
        old=resolve(pinned,lower,upper)
        for key in AXIS_KEYS[:6]:a[key]=old[key]
        g=pinned['general']
    return dict(revision=settings['revision'],axis=a,general=dict(g))


def write_wire(path,settings,lower,upper,*,pinned=None):
    r=runtime(settings,lower,upper,pinned=pinned)
    values=[r['revision'],*[x for key in AXIS_KEYS for x in r['axis'][key]],*[r['general'][key] for key in WIRE_GLOBAL_KEYS]]
    path=Path(path);tmp=path.with_suffix('.pending')
    tmp.write_text(','.join(format(x,'.17g') for x in values)+'\n');tmp.replace(path)
    return r


def load(root,fallback):
    files=sorted((Path(root)/'settings/versions').glob('*.json'))
    return json.loads(files[-1].read_text()) if files else copy.deepcopy(fallback)


def save(root,settings):
    path=Path(root)/'settings/versions';path.mkdir(parents=True,exist_ok=True)
    files=list(path.glob('*.json'))
    revision=max([0,*[int(f.stem) for f in files]])+1
    result=copy.deepcopy(settings);result['revision']=revision
    atomic_json(path/f'{revision:08d}.json',result)
    return result


def command_bounds(settings,lower,upper):
    a=resolve(settings,lower,upper)
    return a['input_lower'],a['input_upper'],dict(effort_limit_nm=a['input_effort'],kp_max=a['kp_max'],kd_max=a['kd_max']),a['input_velocity']


def mock_next_path():
    from .fieldwork_storage import cache_home
    return cache_home()/'mock_next.json'


def save_mock_next(settings):
    path=mock_next_path();path.parent.mkdir(parents=True,exist_ok=True)
    atomic_json(path,dict(model_context=settings['model_context'],limits=settings['mock']),replace=True)
