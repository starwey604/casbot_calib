"""Shared RGB transport is coherent, latest-only, bounded, and not a stop channel."""
import os
import subprocess
import sys
import time

import numpy as np

from casbot_calib.fieldwork_frames import FrameBuffer,HEIGHT,WIDTH
from casbot_calib.fieldwork_window import DisplaySchedule


def test_frame_header_published_after_complete_copy():
    owner=FrameBuffer();reader=FrameBuffer(owner.fd)
    try:
        assert reader.read() is None
        owner.write(np.full((HEIGHT,WIDTH,3),17,dtype=np.uint8),1)
        seq,pixels=reader.read();assert seq==1 and np.all(pixels==17)
        assert reader.read(after=1) is None
        owner.write(np.full((HEIGHT,WIDTH,3),23,dtype=np.uint8),2)
        assert np.all(pixels==17) # Reader owns its copy, not an overwritten view.
        assert reader.read(after=1)[0]==2
    finally:reader.close();owner.close();owner.close()


def test_shared_frames_never_tear_between_processes():
    owner=FrameBuffer()
    script='''import sys,time,numpy as np
from casbot_calib.fieldwork_frames import FrameBuffer,HEIGHT,WIDTH
f=FrameBuffer(int(sys.argv[1]))
for i in range(1,100):
 f.write(np.full((HEIGHT,WIDTH,3),i,dtype=np.uint8),i)
 time.sleep(.001)
f.close()
'''
    child=subprocess.Popen([sys.executable,'-c',script,str(owner.fd)],pass_fds=(owner.fd,))
    last=0;seen=0
    try:
        while child.poll() is None:
            item=owner.read(last)
            if item:
                sequence,pixels=item;assert np.all(pixels==sequence)
                assert sequence>last;last=sequence;seen+=1
            time.sleep(.001)
        child.wait(timeout=2);assert child.returncode==0 and seen>3
        assert owner.read()[0]==99
    finally:
        if child.poll() is None:child.kill();child.wait()
        owner.close()


def test_hidden_drawing_does_not_schedule_and_input_wakes(monkeypatch):
    from casbot_calib import fieldwork_window as window
    now=[10.];monkeypatch.setattr(window.time,'monotonic',lambda:now[0])
    schedule=DisplaySchedule();schedule.drawn(10.)
    assert not schedule.due(10.2) and schedule.due(10.5)
    schedule.interact();assert schedule.due(10.2)
    assert not schedule.due(11.,visible=False)
    schedule.drawn(10.,short=True);assert schedule.next_draw==10.+1/30
