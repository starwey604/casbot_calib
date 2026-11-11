"""Recoverable disk spools and explicit, prefix-consistent session snapshots."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import threading
import time
import uuid

from .hardware_batch import atomic_json


def cache_home():
    return Path(os.environ.get('CASBOT_FIELDWORK_CACHE',
        str(Path(os.environ.get('XDG_STATE_HOME', str(Path.home()/'.local/state')))/'casbot-calib/fieldwork'))).expanduser().resolve()


def new_cache(limit_mib=512):
    if not isinstance(limit_mib,(int,float)) or not 32 <= limit_mib <= 65536:
        raise ValueError('缓存容量须为32–65536 MiB')
    identifier=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8]
    root=cache_home()/'sessions'/identifier;root.mkdir(parents=True,exist_ok=False)
    atomic_json(root/'cache.json',dict(id=identifier,created_utc=datetime.now(timezone.utc).isoformat(),limit_bytes=int(limit_mib*1024**2)))
    return root


def cache_for(workspace):
    parent=Path(workspace).resolve().parent
    return parent if (parent/'cache.json').is_file() else None


def recording_pause_file(workspace):
    cache=cache_for(workspace)
    return cache/'recording_paused.json' if cache else None


def cached_sessions():
    """List known spools only; never discover ROS targets or resume commands."""
    result=[]
    for path in sorted((cache_home()/'sessions').glob('*/cache.json'),reverse=True):
        try:
            row=json.loads(path.read_text());workspace=path.parent/'workspace'
            if (workspace/'protocol/session.json').exists():
                import fcntl
                active=False;lock=workspace/'.frontend.lock'
                if lock.exists():
                    with lock.open('r') as lease:
                        try:fcntl.flock(lease.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
                        except BlockingIOError:active=True
                result.append(dict(row,workspace=str(workspace),active=active))
        except (OSError,ValueError):continue
    return sorted(result,key=lambda row:row['created_utc'],reverse=True)


def recover_workspace(identifier='latest'):
    for row in cached_sessions():
        if identifier==row['id'] and row['active']:raise ValueError('该缓存正在另一个GUI窗口中使用')
        if identifier in ('latest',row['id']) and not row['active']:return Path(row['workspace'])
    raise ValueError('没有可恢复的缓存会话')


def directory_bytes(root):
    total=0
    for path in Path(root).rglob('*'):
        try:
            if path.is_file() and not path.is_symlink():total+=path.stat().st_size
        except FileNotFoundError:pass  # Atomic replacement of a live mailbox/frame.
    return total


class CacheMonitor:
    """A soft disk quota checked off the GUI thread, including owned Mock logs.

    Recording producers stop appending on the marker, with at most one polling
    interval of overshoot. Heartbeats/latest targets and final metadata remain usable.
    """
    def __init__(self,cache,interval_s=1.):
        self.root=Path(cache);self.limit=json.loads((self.root/'cache.json').read_text())['limit_bytes']
        self.used=0;self.error=None;self.interval=interval_s;self.stop=threading.Event()
        self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()
    @property
    def paused(self):return (self.root/'recording_paused.json').exists()
    def check(self):
        self.used=directory_bytes(self.root)
        if self.used>=self.limit and not self.paused:
            try:
                atomic_json(self.root/'recording_paused.json',dict(reason='cache_capacity',used_bytes=self.used,
                    limit_bytes=self.limit,event_ns=time.monotonic_ns(),recording_complete=False))
            except FileExistsError:pass # Another monitor/check already latched the same pause.
    def _run(self):
        while not self.stop.is_set():
            try:self.check()
            except OSError as exc:self.error=str(exc)
            self.stop.wait(self.interval)
    def close(self):self.stop.set();self.thread.join(timeout=3)


def _complete_prefix(stream,limit):
    if limit==0:return 0
    position=limit
    while position:
        start=max(0,position-65536);stream.seek(start);data=stream.read(position-start)
        at=data.rfind(b'\n')
        if at>=0:return start+at+1
        position=start
    return 0


def _copy_prefix(source,destination,*,limit=None,text=False):
    destination.parent.mkdir(parents=True,exist_ok=True)
    with source.open('rb') as incoming,destination.open('xb') as outgoing:
        size=os.fstat(incoming.fileno()).st_size
        count=min(size,limit) if limit is not None else size
        if text:count=_complete_prefix(incoming,count)
        incoming.seek(0);remaining=count
        while remaining:
            chunk=incoming.read(min(1024**2,remaining))
            if not chunk:raise OSError('保存期间源文件被截短：'+str(source))
            outgoing.write(chunk);remaining-=len(chunk)
    return dict(bytes=count,source_bytes_at_open=size)


def save_snapshot(workspace,destination):
    """Copy complete lines; paired CDR stops exactly at the last copied packet.

    Every file has its own recorded cut; no shared device sampling instant is
    invented. Live sources, seals and mailbox ownership are never modified.
    """
    source=Path(workspace).resolve();target=Path(destination).expanduser().resolve()
    if target==source or source in target.parents or target in source.parents:
        raise ValueError('保存目录必须在运行工作区之外')
    target.mkdir(parents=True,exist_ok=False)
    atomic_json(target/'saving.json',dict(source=str(source),started_utc=datetime.now(timezone.utc).isoformat()))
    cuts={};paired=set();started=time.monotonic_ns()
    def copy(path,**kwargs):
        relative=path.relative_to(source);cuts[str(relative)]=_copy_prefix(path,target/relative,**kwargs)
    # The recorder flushes CDR before its JSON row. Read the JSON prefix first,
    # then only the CDR bytes referenced by those rows (no unpaired tail bytes).
    root_events=sorted((source/'events').glob('*.json'))
    controls={json.loads(p.read_text())['control'] for p in root_events if json.loads(p.read_text()).get('event')=='control_created'}
    inventory=[p for p in sorted(source.rglob('*'))
        if not (p.relative_to(source).parts[0]=='events' and p.is_file() and p not in root_events)
        and not (len(p.relative_to(source).parts)>1 and p.relative_to(source).parts[0]=='controls' and 'controls/'+p.relative_to(source).parts[1] not in controls)]
    for packet_path in [p for p in inventory if p.name=='packets.jsonl']:
        copy(packet_path,text=True);relative=packet_path.relative_to(source)
        end=0
        with (target/relative).open() as rows:
            for line in rows:
                packet=json.loads(line);offset=packet.get('cdr_offset')
                if offset is not None:end=max(end,offset+packet['cdr_size'])
        cdr=packet_path.with_name('cdr.bin')
        if cdr.exists():
            if cdr.stat().st_size<end:raise ValueError('CDR长度短于已保存消息引用：'+str(cdr))
            copy(cdr,limit=end);paired.add(cdr)
        elif end:raise ValueError('消息引用的CDR缺失：'+str(cdr))
        paired.add(packet_path)
    # No transient target mailboxes, GL resources, locks or pending files in an archive.
    for path in inventory:
        relative=path.relative_to(source)
        if (not path.is_file() or path.is_symlink() or path in paired or relative.parts[0] in ('view',)
            or any(part.startswith('.') for part in relative.parts) or '.pending' in path.name or path.name in ('latest.csv','frontend_heartbeat.json','window.ini','saving.json','saved_session.json')):continue
        try:copy(path,text=path.suffix in ('.csv','.jsonl','.log'))
        except FileNotFoundError:continue  # A not-yet-finished atomic operation file.
    derived=[]
    for control in sorted((target/'controls').glob('*')):
        if not (control/'protocol/manifest.json').exists():continue
        for directory in sorted((control/'runs').glob('*')):
            execution=directory/'capture/execution'
            if (not (directory/'recording/recording.json').exists()
                and all((execution/name).exists() for name in ('commands.csv','feedback.csv','events.csv'))):
                from .fieldwork_control import Session,export_recording
                export_recording(Session(control),directory);derived.append(str(directory.relative_to(target)/'recording'))
    pause=recording_pause_file(source)
    raw_directories=[target/p.relative_to(source).parent for p in inventory if p.name=='packets.jsonl']
    live_prefix=any(not (p/'summary.json').exists() for p in raw_directories)
    report=dict(schema_version='casbot_fieldwork_snapshot_v1',copy_complete=True,read_only=True,
        source_workspace=str(source),created_utc=datetime.now(timezone.utc).isoformat(),
        capture_started_ns=started,capture_finished_ns=time.monotonic_ns(),file_cuts=cuts,derived_recordings=derived,
        recording_capacity_reached=bool(pause and pause.exists()),
        recording_status='capacity_paused' if pause and pause.exists() else 'empty' if not raw_directories else 'live_prefix' if live_prefix else 'finalized_prefix',
        semantics='complete per-file prefixes; SDK stamps are publication, not acquisition; active controls are not resumable')
    atomic_json(target/'saved_session.json',report);(target/'saving.json').unlink()
    cache=cache_for(source)
    if cache:atomic_json(cache/'last_saved.json',dict(destination=str(target),created_utc=report['created_utc']),replace=True)
    return dict(destination=str(target),files=len(cuts))


def last_observation(workspace,names):
    """Read a bounded tail of archived SDK packets, without live-age assumptions."""
    from .vendor_data import normalize_packet
    for path in sorted(Path(workspace).glob('observations/*/capture/raw/packets.jsonl'),key=lambda p:p.stat().st_mtime_ns,reverse=True):
        with path.open('rb') as stream:
            stream.seek(0,2);size=stream.tell();stream.seek(max(0,size-65536));lines=stream.read().splitlines(keepends=True)
        for line in reversed(lines):
            if not line.endswith(b'\n'):continue
            try:packet=json.loads(line)
            except ValueError:continue
            if packet.get('channel')!='joint':continue
            normalized=normalize_packet(packet,dict(joint_names=names))
            if normalized['q'] is not None:return normalized
    return None


class SaveJob:
    def __init__(self,workspace,destination):
        self.result=None;self.error=None
        def work():
            try:self.result=save_snapshot(workspace,destination)
            except Exception as exc:self.error=str(exc)
        self.thread=threading.Thread(target=work,daemon=True);self.thread.start()
    @property
    def done(self):return not self.thread.is_alive()
