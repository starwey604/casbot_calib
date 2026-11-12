"""Owned Linux shared-memory RGB double buffer; never a recording or control channel."""
import fcntl
import mmap
import os
import struct

import numpy as np

WIDTH, HEIGHT = 480, 360
_HEADER=struct.Struct('<QQ') # last complete sequence, active slot
_OFFSET=64
_SIZE=WIDTH*HEIGHT*3


class FrameBuffer:
    def __init__(self, fd=None):
        if fd is None:
            self.fd=os.memfd_create('casbot-fieldwork-rgb',os.MFD_CLOEXEC)
            os.ftruncate(self.fd,_OFFSET+2*_SIZE)
        else:
            # Reopen: inherited descriptors share one open-file description,
            # which would make flock ineffective between parent and child.
            self.fd=os.open('/proc/self/fd/'+str(fd),os.O_RDWR)
        self.memory=mmap.mmap(self.fd,_OFFSET+2*_SIZE)

    def write(self,pixels,sequence):
        if pixels.shape!=(HEIGHT,WIDTH,3) or pixels.dtype!=np.uint8:raise ValueError('Invalid live RGB frame')
        fcntl.flock(self.fd,fcntl.LOCK_EX)
        try:
            _,active=_HEADER.unpack_from(self.memory);slot=1-active
            offset=_OFFSET+slot*_SIZE
            self.memory[offset:offset+_SIZE]=pixels.tobytes()
            # Publish only after the entire inactive buffer is complete.
            _HEADER.pack_into(self.memory,0,sequence,slot)
        finally:fcntl.flock(self.fd,fcntl.LOCK_UN)

    def read(self,after=0):
        try:fcntl.flock(self.fd,fcntl.LOCK_SH|fcntl.LOCK_NB)
        except BlockingIOError:return None
        try:
            sequence,active=_HEADER.unpack_from(self.memory)
            if sequence<=after:return None
            pixels=np.frombuffer(self.memory,dtype=np.uint8,count=_SIZE,offset=_OFFSET+active*_SIZE).reshape(HEIGHT,WIDTH,3).copy()
            return sequence,pixels
        finally:fcntl.flock(self.fd,fcntl.LOCK_UN)

    def close(self):
        if self.memory is not None:
            self.memory.close();os.close(self.fd);self.memory=None
