"""Recoverable GUI recording prefixes. No SHM connection or control side effects."""
import csv
import json
import os
from pathlib import Path
import shutil
import time

PAIRS=(('feedback',704),('commands',752))


def _csv_prefix(source,target,limit=None,cancel=None):
    """Copy only complete lines visible at one captured byte bound; bounded memory."""
    size=source.stat().st_size;count=0
    with source.open('rb') as src,target.open('wb') as dst:
        header=src.readline()
        if not header.endswith(b'\n') or src.tell()>size:raise ValueError('CSV header incomplete: '+str(source))
        dst.write(header)
        while src.tell()<size and (limit is None or count<limit):
            if cancel and cancel():raise ValueError('保存已取消，已写前缀保留')
            line=src.readline(65536)
            if not line.endswith(b'\n') or src.tell()>size:break
            dst.write(line);count+=1
    return count


def _bytes_prefix(source,target,size,cancel=None):
    with source.open('rb') as src,target.open('wb') as dst:
        remaining=size
        while remaining:
            if cancel and cancel():raise ValueError('保存已取消，已写前缀保留')
            block=src.read(min(1024*1024,remaining))
            if not block:raise ValueError('Source shrank while saving: '+str(source))
            dst.write(block);remaining-=len(block)


def snapshot(source,output,*,cancel=None):
    source=Path(source).resolve();output=Path(output).resolve()
    session=json.loads((source/'session.json').read_text())
    if session['schema_version']!='casbot_shm_gui_workspace_v1':raise ValueError('SHM GUI workspace required')
    if output==source or source in output.parents:raise ValueError('快照须位于运行工作区之外')
    output.mkdir(parents=True,exist_ok=False);native=output/'native';native.mkdir()
    rows={};files={};closed_at_start=(source/'native/result.json').is_file()
    for name,width in PAIRS:
        raw=source/'native'/f'{name}.bin';table=source/'native'/f'{name}.csv'
        if not raw.exists() or not table.exists():continue
        rows[name]=_csv_prefix(table,native/table.name,raw.stat().st_size//width,cancel)
        _bytes_prefix(raw,native/raw.name,rows[name]*width,cancel)
    for name in ('events.csv','clock_pairs.csv','observed_limits.csv','queues.csv'):
        src=source/'native'/name
        if src.exists():_csv_prefix(src,native/name)
    archive=source/'native/controls.jsonl'
    if archive.exists():
        # Complete JSONL records only, no header.
        bound=archive.stat().st_size
        with archive.open('rb') as src,(native/archive.name).open('wb') as dst:
            while src.tell()<bound:
                line=src.readline(65536)
                if not line.endswith(b'\n') or src.tell()>bound:break
                json.loads(line);dst.write(line)
    (output/'session.json').write_text(json.dumps(session,indent=2)+'\n')
    zero=source/'zero.yaml'
    if zero.exists():
        value=json.loads(zero.read_text());(output/'zero.yaml').write_text(json.dumps(value,indent=2)+'\n')
    for name in ('hand_background_source.bin','result.json','recording_stopped.json'):
        src=source/'native'/name
        if src.exists() and (name!='result.json' or closed_at_start):
            if name.endswith('.json'):
                try:value=json.loads(src.read_text())
                except json.JSONDecodeError:continue
                (native/name).write_text(json.dumps(value,indent=2)+'\n')
            else:shutil.copyfile(src,native/name)
    for src in sorted((source/'references').glob('*.json')) if (source/'references').is_dir() else ():
        target=output/'references'/src.name;target.parent.mkdir(exist_ok=True);shutil.copyfile(src,target)
    for path in output.rglob('*'):
        if path.is_file():files[str(path.relative_to(output))]=path.stat().st_size
    manifest=dict(schema_version='casbot_shm_gui_snapshot_v1',source_workspace=str(source),created_wall_ns=time.time_ns(),
                  created_mono_ns=time.monotonic_ns(),model_context=session['model_context'],rows=rows,files=files,
                  prefix=True,complete_session=closed_at_start and (native/'result.json').exists(),sampling='full_native_recording_not_display_stream')
    (output/'snapshot.json').write_text(json.dumps(manifest,indent=2)+'\n')
    for path in output.rglob('*'):
        if path.is_file():path.chmod(0o444)
    return manifest


def load_snapshot(path):
    root=Path(path).resolve();manifest=json.loads((root/'snapshot.json').read_text())
    if manifest['schema_version']!='casbot_shm_gui_snapshot_v1':raise ValueError('需要 SHM GUI 快照')
    actual={str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() and p.name!='snapshot.json'}
    if actual!=set(manifest['files']):raise ValueError('快照文件清单不完整')
    for name,size in manifest['files'].items():
        p=(root/name).resolve()
        if root not in p.parents or p.stat().st_size!=size:raise ValueError('快照文件缺失/长度变化：'+name)
    return manifest


def last_feedback(path):
    """Tail read for opening a large archive, not a full-table scan."""
    with Path(path).open('rb') as stream:
        header=stream.readline().decode().strip().split(',');stream.seek(0,2);size=stream.tell()
        offset=max(0,size-65536);stream.seek(offset)
        if offset:stream.readline()
        lines=stream.read().decode().splitlines()
    for line in reversed(lines):
        row=next(csv.reader([line]))
        if len(row)==len(header) and row!=header:return dict(zip(header,row))
    raise ValueError('快照没有完整反馈')
