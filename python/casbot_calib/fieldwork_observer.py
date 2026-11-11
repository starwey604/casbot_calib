"""Persistent subscription-only observation, independent of a control attempt."""
import json
from pathlib import Path
import yaml

from .fieldwork_control import Workspace
from .sdk_connection import connection_remaps
from .vendor_capture import capture
from .vendor_data import normalize_packet


def observe(root, relative, cancel_check, publish):
    session=Workspace(root)
    directory=(session.root/relative).resolve()
    if directory.parent!=session.root/'observations':raise ValueError('观测目录必须属于本工作区')
    directory.mkdir(parents=True,exist_ok=False)
    config=yaml.safe_load(Path('configs/vendor_s1.yaml').read_text())
    remaps=connection_remaps(session.config,session.metadata['domain_id']) or {}
    config.update(model_context=session.metadata['model_context'],joint_names=session.names,
        model_id=session.metadata['model_context']['model_id'],joint_type=session.config['feedback_type'],
        joint_topic=remaps.get(session.config['joint_topic'],session.config['joint_topic']),
        debug_topic=remaps.get('/motion/debug/joint_state','/motion/debug/joint_state'),
        mode_topic=remaps.get(session.config['mode_topic'],session.config['mode_topic']),robot_state_topic='')
    path=directory/'observation.yaml';path.write_text(yaml.safe_dump(config),encoding='utf-8')
    values=dict(q=None,dq=None,effort_raw=None,kp=None,kd=None,receive_ns=None,mode='尚未收到模式反馈')
    def line(text):
        if text.startswith('FIELDWORK_GRAPH '):
            publish('connection_status',**json.loads(text[len('FIELDWORK_GRAPH '):]));return
        if text.startswith('FIELDWORK_NOTICE '):
            notice=json.loads(text[len('FIELDWORK_NOTICE '):]);publish('recording_error',message=notice['message'],code=notice['code']);return
        if not text.startswith('FIELDWORK_PACKET '):return
        packet=json.loads(text[len('FIELDWORK_PACKET '):])
        if packet['channel']=='mode':
            values['mode']=packet['message'].get('data','未知');values['mode_receive_ns']=packet['receive_ns']
        elif packet['channel'] in ('joint','debug'):
            normalized=normalize_packet(packet,config)
            if packet['channel']=='joint' and normalized['q'] is not None:
                values['q']=normalized['q'];values['receive_ns']=packet['receive_ns']
                values['dq']=normalized['dq'];values['effort_raw']=normalized['effort_raw']
            for name in ('kp','kd'):
                if packet['channel']=='debug' or normalized[name] is not None:values[name]=normalized[name]
        else:return
        publish('feedback',**values)
    publish('observer_started',relative=relative,channels={name:dict(topic=config[key],publishers=None) for name,key in [('joint','joint_topic'),('mode','mode_topic'),('debug','debug_topic')]})
    from .fieldwork_storage import recording_pause_file,cache_for,CacheMonitor
    cache=cache_for(session.root);monitor=CacheMonitor(cache) if cache else None
    try:
        result=capture(path,directory/'capture',duration_s=10.,domain_id=session.metadata['domain_id'],
        scope='localhost_contract_mock' if remaps else 'hardware_read_only',live_callback=line,
        cancel_check=cancel_check,until_stopped=True,tolerate_recording_io_error=True,recording_pause_file=recording_pause_file(session.root))
    finally:
        if monitor:monitor.close()
    return dict(observation=relative,result=result)
