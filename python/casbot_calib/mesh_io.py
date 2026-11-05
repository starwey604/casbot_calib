"""Metric binary STL interchange for build-time CAD assets (no runtime CAD API)."""
from pathlib import Path
import struct
import numpy as np

STL_RECORD=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])


def write_stl(path, vertices, triangles):
    vertices=np.asarray(vertices,dtype=float);triangles=np.asarray(triangles,dtype=int)
    points=vertices[triangles];normal=np.cross(points[:,1]-points[:,0],points[:,2]-points[:,0])
    norm=np.linalg.norm(normal,axis=1);normal/=np.maximum(norm[:,None],1e-30)
    rows=np.zeros(len(points),dtype=STL_RECORD);rows['vertices']=points;rows['normal']=normal
    Path(path).write_bytes(b'casbot_calib metric STL'.ljust(80,b' ')+struct.pack('<I',len(rows))+rows.tobytes())


def read_stl(path):
    payload=Path(path).read_bytes()
    if len(payload)<84:raise ValueError('Binary metric STL required: '+str(path))
    count=struct.unpack('<I',payload[80:84])[0]
    if len(payload)!=84+count*50 or count==0:raise ValueError('Invalid binary STL: '+str(path))
    points=np.frombuffer(payload,STL_RECORD,offset=84,count=count)['vertices'].astype(float)
    vertices,indices=np.unique(points.reshape(-1,3),axis=0,return_inverse=True)
    if not np.isfinite(vertices).all():raise ValueError('Nonfinite STL vertices')
    return vertices,indices.reshape(-1,3).astype(np.int32)
