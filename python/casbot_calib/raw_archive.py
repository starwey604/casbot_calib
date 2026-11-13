"""Bounded streaming access to lossless JSON+CDR gzip chunks and native S1 raw."""
from contextlib import contextmanager, ExitStack
import fcntl
import gzip
import json
import os
from pathlib import Path
import struct
import zlib

MAGIC=b'CBRAW2\n'
SCHEMA='casbot_raw_gzip_frames_v2'


def raw_directories(root):
    root=Path(root)
    return sorted({p.parent for pattern in ('format.json','packets.jsonl') for p in root.rglob(pattern)
        if 'evaluation' not in p.relative_to(root).parts})


@contextmanager
def chunk_streams(raw, *, latest_only=False):
    """Pin one descriptor at a time; sealed chunks never change or disappear."""
    raw=Path(raw)
    with ExitStack() as stack:
        lock_path=raw/'.archive.lock'
        lock=stack.enter_context(lock_path.open('rb')) if lock_path.exists() else None
        if lock:fcntl.flock(lock.fileno(),fcntl.LOCK_SH)
        try:paths=sorted((raw/'chunks').glob('*.gz'))
        finally:
            if lock:fcntl.flock(lock.fileno(),fcntl.LOCK_UN)
        if latest_only:paths=paths[-2:]
        def pinned():
            for path in paths:
                if lock:fcntl.flock(lock.fileno(),fcntl.LOCK_SH)
                try:
                    # The only legal rename is this same chunk's finalization.
                    if path.name.endswith('.open.gz') and not path.exists():
                        path=path.with_name(path.name.replace('.open.gz','.bin.gz'))
                    stream=path.open('rb');size=os.fstat(stream.fileno()).st_size
                finally:
                    if lock:fcntl.flock(lock.fileno(),fcntl.LOCK_UN)
                try:yield path,stream,size
                finally:stream.close()
        iterator=pinned()
        try:yield iterator
        finally:iterator.close()


def encoded_frames(stream, size, *, sealed):
    """Yield only complete frames; validate CRC/footer for finalized chunks."""
    codec=zlib.decompressobj(31);buffer=bytearray();magic=False;remaining=size
    while remaining:
        data=stream.read(min(65536,remaining))
        if not data:raise ValueError('原始压缩块在读取时被截短')
        remaining-=len(data)
        try:buffer.extend(codec.decompress(data))
        except zlib.error as exc:raise ValueError('原始压缩块损坏：'+str(exc)) from exc
        if not magic:
            if len(buffer)<len(MAGIC):continue
            if buffer[:len(MAGIC)]!=MAGIC:raise ValueError('原始压缩块格式标记无效')
            del buffer[:len(MAGIC)];magic=True
        offset=0
        while len(buffer)-offset>=8:
            nj,nc=struct.unpack_from('<II',buffer,offset);end=offset+8+nj+nc
            if len(buffer)<end:break
            yield bytes(buffer[offset+8:offset+8+nj]),bytes(buffer[offset+8+nj:end])
            offset=end
        if offset:del buffer[:offset]
    if sealed and (not codec.eof or buffer or not magic or codec.unused_data):
        raise ValueError('已封存原始压缩块缺少完整帧或gzip校验尾部')


def iter_records(raw, *, start_ns=None, end_ns=None, latest_only=False):
    raw=Path(raw)
    if (raw/'format.json').exists():
        if json.loads((raw/'format.json').read_text()).get('schema_version')!=SCHEMA:
            raise ValueError('不支持的原始记录格式')
        expected_sequence=expected_offset=None if latest_only else 0
        with chunk_streams(raw,latest_only=latest_only) as chunks:
            for path,stream,size in chunks:
                for encoded,cdr in encoded_frames(stream,size,sealed=path.name.endswith('.bin.gz')):
                    row=json.loads(encoded)
                    if row['cdr_size']!=len(cdr):raise ValueError('原始帧CDR长度不匹配')
                    if expected_sequence is not None and (row['receive_sequence']!=expected_sequence or row['cdr_offset']!=expected_offset):
                        raise ValueError('原始帧序号或CDR逻辑偏移不连续，可能缺少压缩块')
                    expected_sequence=row['receive_sequence']+1;expected_offset=row['cdr_offset']+len(cdr)
                    ns=row['receive_ns']
                    if start_ns is not None and ns<start_ns:continue
                    if end_ns is not None and ns>=end_ns:continue
                    yield row,cdr
    else:
        # Existing S1/H1–H4 producers retain their native flat storage contract.
        with (raw/'packets.jsonl').open() as rows:
            for line in rows:
                if not line.endswith('\n'):break
                row=json.loads(line);ns=row.get('receive_ns',0)
                if start_ns is not None and ns<start_ns:continue
                if end_ns is not None and ns>=end_ns:continue
                yield row,None


def capture_raw(capture):
    capture=Path(capture);reference=capture/'raw_reference.json'
    if reference.exists():
        ref=json.loads(reference.read_text())
        return (capture/ref['raw']).resolve(),ref['start_receive_ns'],ref.get('end_receive_ns')
    return capture/'raw_state/raw',None,None


def snapshot_chunks(raw, destination):
    """Copy sealed bytes; re-seal the complete-frame prefix of the live chunk."""
    raw=Path(raw);destination=Path(destination);(destination/'chunks').mkdir(parents=True,exist_ok=True)
    cuts={}
    with chunk_streams(raw) as chunks:
        for path,stream,size in chunks:
            relative=Path('chunks')/path.name.replace('.open.gz','.bin.gz');target=destination/relative
            if path.name.endswith('.bin.gz'):
                with target.open('xb') as out:
                    left=size
                    while left:
                        data=stream.read(min(left,1024**2))
                        if not data:raise ValueError('封存块在保存时被截短')
                        out.write(data);left-=len(data)
                count=None
            else:
                count=0
                with target.open('xb') as out,gzip.GzipFile(fileobj=out,mode='wb',compresslevel=1,mtime=0) as compressed:
                    compressed.write(MAGIC)
                    for encoded,cdr in encoded_frames(stream,size,sealed=False):
                        compressed.write(struct.pack('<II',len(encoded),len(cdr)));compressed.write(encoded);compressed.write(cdr);count+=1
            cuts[str(relative)]=dict(bytes=target.stat().st_size,source_chunk=path.name,source_bytes_at_open=size,complete_frames_resealed=count)
    return cuts
