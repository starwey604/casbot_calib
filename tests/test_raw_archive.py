"""Lossless framing, active-prefix recovery, rotation and snapshot dependencies."""
import gzip
import json
from pathlib import Path
import struct
import zlib

import pytest

from casbot_calib.raw_archive import MAGIC,SCHEMA,encoded_frames,iter_records,snapshot_chunks,chunk_streams,capture_raw


def make_raw(root):
    (root/'chunks').mkdir(parents=True)
    (root/'format.json').write_text(json.dumps(dict(schema_version=SCHEMA)))
    return root


def frame(i=0):
    # Include non-target fields, nan payload bits and NUL in opaque CDR.
    cdr=b'\x00\xff\x7f\x80'+struct.pack('<Q',0x7ff8000000000001)
    row=dict(receive_ns=10**18+i,receive_sequence=i,cdr_size=len(cdr),cdr_offset=i*len(cdr),
        channel='joint',type='sensor_msgs/msg/JointState',message=dict(position=[.2]*14,extra_axis='waist'))
    encoded=json.dumps(row,separators=(',',':')).encode()
    return row,cdr,struct.pack('<II',len(encoded),len(cdr))+encoded+cdr


def test_sealed_roundtrip_preserves_every_frame_and_cdr_bit(tmp_path):
    raw=make_raw(tmp_path/'raw');values=[frame(i) for i in range(20)]
    (raw/'chunks/00000000.bin.gz').write_bytes(gzip.compress(MAGIC+b''.join(v[2] for v in values)))
    assert list(iter_records(raw))==[(r,c) for r,c,_ in values]
    assert list(iter_records(raw,start_ns=10**18+2,end_ns=10**18+4))==[(r,c) for r,c,_ in values[2:4]]


def test_active_sync_flush_and_incomplete_frame_reseal(tmp_path):
    raw=make_raw(tmp_path/'raw');a,b=frame(0),frame(1)
    compressor=zlib.compressobj(1,zlib.DEFLATED,31)
    active=compressor.compress(MAGIC+a[2]+b[2][:15])+compressor.flush(zlib.Z_SYNC_FLUSH)
    (raw/'chunks/00000000.open.gz').write_bytes(active)
    assert list(iter_records(raw))==[(a[0],a[1])]
    target=tmp_path/'saved';cuts=snapshot_chunks(raw,target)
    (target/'format.json').write_text((raw/'format.json').read_text())
    assert cuts['chunks/00000000.bin.gz']['complete_frames_resealed']==1
    assert list(iter_records(target))==[(a[0],a[1])]
    assert gzip.decompress((target/'chunks/00000000.bin.gz').read_bytes())==MAGIC+a[2]


def test_truncated_sealed_or_corrupt_chunk_is_not_accepted(tmp_path):
    raw=make_raw(tmp_path/'raw');data=gzip.compress(MAGIC+frame()[2])
    path=raw/'chunks/00000000.bin.gz';path.write_bytes(data[:-4])
    with pytest.raises(ValueError,match='gzip'):list(iter_records(raw))
    bad=bytearray(data);bad[-8]^=0xff;path.write_bytes(bad)
    with pytest.raises(ValueError,match='损坏'):list(iter_records(raw))


def test_rotation_between_inventory_and_open_uses_same_chunk(tmp_path):
    raw=make_raw(tmp_path/'raw');(raw/'.archive.lock').touch()
    path=raw/'chunks/00000000.open.gz';path.write_bytes(gzip.compress(MAGIC+frame()[2]))
    with chunk_streams(raw) as sources:
        path.rename(path.with_name('00000000.bin.gz'))
        entries=list((p.name,size) for p,f,size in sources)
    assert entries[0][0]=='00000000.bin.gz'


def test_snapshot_shared_reference_is_relocatable_and_not_duplicated(tmp_path):
    from casbot_calib.fieldwork_storage import save_snapshot,last_observation
    root=tmp_path/'workspace';raw=make_raw(root/'observations/one/capture/raw')
    row,cdr,data=frame();row['message'].update(name=[str(i) for i in range(14)],velocity=[],effort=[])
    encoded=json.dumps(row).encode();data=struct.pack('<II',len(encoded),len(cdr))+encoded+cdr
    (raw/'chunks/00000000.bin.gz').write_bytes(gzip.compress(MAGIC+data))
    capture=root/'controls/c1/runs/r1/capture';capture.mkdir(parents=True)
    import os
    (capture/'raw_reference.json').write_text(json.dumps(dict(raw=os.path.relpath(raw,capture),start_receive_ns=10**18,end_receive_ns=None)))
    (root/'events').mkdir();(root/'events/one.json').write_text('{"event":"control_created","control":"controls/c1"}')
    saved=tmp_path/'saved';save_snapshot(root,saved)
    assert len(list(saved.rglob('*.bin.gz')))==1
    actual,start,end=capture_raw(saved/capture.relative_to(root))
    assert list(iter_records(actual,start_ns=start,end_ns=end))==[(row,cdr)]
    assert last_observation(saved,[str(i) for i in range(14)])['q']==[.2]*14
    with pytest.raises(ValueError,match='完整工作区'):save_snapshot(root/'controls/c1',tmp_path/'bad')


def test_many_chunks_are_read_with_one_data_descriptor(tmp_path):
    raw=make_raw(tmp_path/'raw')
    for i in range(3):(raw/'chunks'/f'{i:08d}.bin.gz').write_bytes(gzip.compress(MAGIC+frame(i)[2]))
    with chunk_streams(raw) as sources:
        _,first,_=next(sources);assert not first.closed
        _,second,_=next(sources);assert first.closed and not second.closed
    assert second.closed


def test_missing_chunk_is_reported_not_silently_joined(tmp_path):
    raw=make_raw(tmp_path/'raw')
    for i in (0,2):(raw/'chunks'/f'{i:08d}.bin.gz').write_bytes(gzip.compress(MAGIC+frame(i)[2]))
    with pytest.raises(ValueError,match='不连续'):list(iter_records(raw))
