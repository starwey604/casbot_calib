"""Prepare a GUI connection without making archive paths part of the user workflow."""
from contextlib import contextmanager
from pathlib import Path
import sys
import time
import uuid

from .fieldwork import clock_id
from .fieldwork_control import create_session
from .fieldwork_ipc import Child
from .fieldwork_storage import new_cache,cache_for,recover_workspace,CacheMonitor


@contextmanager
def prepare_gui(args):
    mock=None;monitor=None
    owned=getattr(args,'mock',False);recover=getattr(args,'recover',None)
    if getattr(args,'mock_diagnostics',False) and not owned:raise ValueError('--mock-diagnostics只用于自建Mock')
    if owned and (args.session or recover or args.preview):raise ValueError('--mock不能与已有会话、恢复或离线预览同时使用')
    if args.preview and (args.session or recover):raise ValueError('离线预览不加载已有会话；请省略--session/--recover')
    if args.session and recover:raise ValueError('--session与--recover不能同时使用')
    if args.session or recover:
        if args.output is not None or args.domain_id is not None or args.scope is not None:
            raise ValueError('已有会话沿用封存的连接；不同时传入新连接参数')
        root=Path(args.session) if args.session else recover_workspace(recover)
        cache=cache_for(root)
    else:
        if not args.preview and (args.domain_id is None or (args.scope is None and not owned)):
            raise ValueError('实机/已有Mock连接须显式给出--domain-id和--scope；本地自建Mock使用--mock --domain-id')
        cache=new_cache(getattr(args,'cache_limit_mib',512)) if args.output is None or owned else None
        root=Path(args.output).resolve() if args.output is not None else cache/'workspace'
    try:
        if owned:
            if args.scope not in (None,'localhost_contract_mock'):raise ValueError('--mock只支持localhost_contract_mock')
            from .hardware_field import load_profile
            if load_profile(args.profile,manual=True).get('connection'):raise ValueError('--mock使用基础profile，不使用旧Mock实例的client_profile')
            import json,yaml
            from .fieldwork_settings import mock_next_path,MOCK_DEFAULTS
            cfg=yaml.safe_load(Path('configs/fieldwork_mock.yaml').read_text())
            preference=mock_next_path()
            if preference.exists():
                pref=json.loads(preference.read_text())
                if pref['model_context']!=cfg['model_context']:raise ValueError('下次Mock设置模型不一致：'+str(preference))
                limits=pref['limits']
                if set(limits)!=set(MOCK_DEFAULTS):raise ValueError('下次Mock设置不完整：'+str(preference))
                cfg.update(effort_limit_nm=limits['effort_limit_nm'],kp_limit=limits['kp_limit'],kd_limit=limits['kd_limit'],watchdog_ns=round(limits['watchdog_s']*1e9))
                cfg['plant']['torque_limit_nm']=limits['torque_limit_nm']
            mock_config=cache/'mock_config.yaml';mock_config.write_text(yaml.safe_dump(cfg))
            owner=uuid.uuid4().hex
            mock=Child([sys.executable,'-m','casbot_calib.fieldwork_mock','--output',str(cache/'mock'),
                '--config',str(mock_config),'--domain-id',str(args.domain_id),'--owner-id',owner,'--profile',str(args.profile),
                '--pause-file',str(cache/'recording_paused.json'),
                *(['--diagnostics'] if getattr(args,'mock_diagnostics',False) else [])],cache/'mock_launcher.log',pause_file=cache/'recording_paused.json')
            mock.pulse(dict(clock_id=clock_id(),owner_id=owner,monotonic_ns=time.monotonic_ns(),cancel=None))
            deadline=time.monotonic()+45;profile=None
            while profile is None:
                for item in mock.drain():
                    if item['type']=='mock_ready':profile=Path(item['profile'])
                    elif item['type']=='mock_error':raise RuntimeError(item['message'])
                if profile is not None:break
                if mock.done():raise RuntimeError('本地Mock启动失败；日志：'+str(cache/'mock_launcher.log'))
                if time.monotonic()>deadline:raise RuntimeError('本地Mock启动超时；日志：'+str(cache/'mock_launcher.log'))
                time.sleep(.02)
            create_session(profile,root,args.domain_id,'localhost_contract_mock',workspace=True)
        elif not (args.session or recover):
            if args.preview:root.mkdir(parents=True,exist_ok=False)
            else:create_session(args.profile,root,args.domain_id,args.scope,workspace=True)
        if cache:monitor=CacheMonitor(cache)
        print('GUI运行工作区：'+str(root),file=sys.stderr)
        if cache:print('自动缓存已启用；在GUI中点击“保存会话”生成正式记录。',file=sys.stderr)
        yield root,monitor,mock
    finally:
        if mock:
            # Closing the pipe also works after a parent crash; the worker owns
            # its SDK child and never touches any separately started Mock.
            if mock.process.stdin and not mock.process.stdin.closed:mock.process.stdin.close()
            try:mock.process.wait(timeout=12)
            except Exception:
                from .sdk_process import stop_process
                stop_process(mock.process)
            mock.close()
        if monitor:monitor.close()
