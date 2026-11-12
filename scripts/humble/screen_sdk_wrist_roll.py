#!/usr/bin/env python3
"""c00 right-roll signal screening: exactly two probes, local SDK Mock only."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['run','analyse'])
    p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--domain-id',type=int,default=227);a=p.parse_args()
    if a.action=='run':
        from casbot_calib.hardware_probe_capture import design_roll_screen,collect
        design_roll_screen(a.source,a.output);collect(a.output,a.domain_id)
        subprocess.run([sys.executable,__file__,'analyse','--source',str(a.output),'--output',str(a.output/'analysis')],check=True)
    else:
        recipe=Path(json.loads((a.source/'protocol/experiment.json').read_text())['source_recipe']).resolve()
        def guard(event,args):
            if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
                path=Path(os.fsdecode(args[0])).resolve()
                if 'evaluation' in path.parts or path.is_relative_to(recipe):raise RuntimeError('Only new public roll observations may be inspected')
        sys.addaudithook(guard)
        import casbot_calib.rigid_identification as rigid
        def no_fit(*args,**kwargs):raise RuntimeError('Signal screening does not fit a rigid candidate')
        rigid.fit_rigid=no_fit
        from casbot_calib.hardware_probe_capture import analyse
        analyse(a.source,a.output)


if __name__=='__main__':main()
