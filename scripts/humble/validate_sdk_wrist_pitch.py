#!/usr/bin/env python3
"""Frozen candidate, independent third-pose pair; local SDK Mock only."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['run','analyse'])
    parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--domain-id',type=int,default=227);a=parser.parse_args()
    if a.action=='run':
        from casbot_calib.hardware_probe_validation import design,collect
        design(a.source,a.output);collect(a.output,a.domain_id)
        subprocess.run([sys.executable,__file__,'analyse','--source',str(a.output),'--output',str(a.output/'analysis')],check=True)
    else:
        protocol=json.loads((a.source/'protocol/experiment.json').read_text());train=Path(protocol['source_campaign']).resolve()
        def guard(event,args):
            if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
                path=Path(os.fsdecode(args[0])).resolve()
                if 'evaluation' in path.parts or path.is_relative_to(train):
                    raise RuntimeError('Training and private evaluation forbidden during independent validation')
        sys.addaudithook(guard)
        import casbot_calib.rigid_identification as rigid
        def no_fit(*args,**kwargs):raise RuntimeError('Frozen validation forbids training fit')
        rigid.fit_rigid=no_fit
        from casbot_calib.hardware_probe_validation import analyse
        analyse(a.source,a.output)


if __name__=='__main__':main()
