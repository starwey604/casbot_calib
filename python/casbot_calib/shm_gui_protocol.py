"""Version 1 remote GUI framing; integers remain integers on both architectures."""
import json
import struct

MAX_FRAME = 32768
HEADER = struct.Struct('<4sIQqqqiiQQqQqIIQQqQQ')
HEADER_NAMES = ('magic', 'version', 'seq', 'mono_ns', 'receive_ns', 'observation_ns', 'clock_segment',
                'phase', 'control', 'revision', 'applied_ns', 'challenge', 'expires_ns',
                'disable_ack', 'recording_stopped', 'writes', 'skips', 'max_gap_ns', 'master_frame', 'source_timestamp_ms')
VALUES = struct.Struct('<112d')
JOINT = struct.Struct('<HiihhH')
HAND = struct.Struct('<HH'+'hhhH'*6)
SIZE_V1 = HEADER.size+VALUES.size+35*JOINT.size+2*HAND.size
SIZE = SIZE_V1+16
FIELDS = ('position', 'velocity', 'effort', 'kp', 'kd')


def frame(kind, body):
    if not 1 <= len(body)+1 <= MAX_FRAME:
        raise ValueError('frame_length')
    return struct.pack('<I', len(body)+1)+bytes((kind,))+body


def json_frame(value):
    return frame(1, json.dumps(value, allow_nan=False, ensure_ascii=False, separators=(',', ':')).encode())


def read_exact(stream, size):
    result = bytearray()
    while len(result) < size:
        block = stream.read(size-len(result))
        if not block:
            raise EOFError('通信流已关闭/帧未完整')
        result.extend(block)
    return bytes(result)


def read_frame(stream):
    size, = struct.unpack('<I', read_exact(stream, 4))
    if not 1 <= size <= MAX_FRAME:
        raise ValueError('frame_length')
    data = read_exact(stream, size)
    if data[0] not in (1, 2):
        raise ValueError('frame_type')
    return data[0], data[1:]


def telemetry(body):
    if len(body) not in (SIZE,SIZE_V1):
        raise ValueError(f'telemetry_length: {len(body)} != {SIZE}')
    result = dict(zip(HEADER_NAMES, HEADER.unpack_from(body)))
    magic=result.pop('magic')
    if (magic,result['version'],len(body)) not in ((b'SGT1',1,SIZE_V1),(b'SGT2',2,SIZE)):
        raise ValueError('telemetry_version')
    offset = HEADER.size
    numbers = VALUES.unpack_from(body, offset);offset += VALUES.size
    result.update(q=numbers[:14], dq=numbers[14:28], effort=numbers[28:42])
    result['command'] = {name: numbers[42+j*14:56+j*14] for j, name in enumerate(FIELDS)}
    result['joints'] = [JOINT.unpack_from(body, offset+j*JOINT.size) for j in range(35)]
    offset += 35*JOINT.size
    result['hands'] = [HAND.unpack_from(body, offset+j*HAND.size) for j in range(2)]
    result['recording_bytes'],result['budget_bytes']=struct.unpack_from('<QQ',body,SIZE_V1) if result['version']==2 else (None,None)
    return result
