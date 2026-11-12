#!/usr/bin/env python3
"""Collect exactly six supplementary local SDK probes; reuse the original pair."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
from casbot_calib.hardware_probe_campaign import design,collect,analyse


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['run','analyse'])
    p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--operator-probes',type=Path);p.add_argument('--domain-id',type=int,default=227)
    a=p.parse_args()
    if a.action=='run':
        if a.operator_probes is None:p.error('run requires original public --operator-probes')
        design(a.source,a.output,a.operator_probes);collect(a.output,a.domain_id)
        subprocess.run([sys.executable,__file__,'analyse','--source',str(a.output),'--output',str(a.output/'analysis')],check=True)
    else:
        def guard(event,args):
            if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)) and 'evaluation' in Path(os.fsdecode(args[0])).parts:
                raise RuntimeError('Private evaluation forbidden during SDK campaign analysis')
        sys.addaudithook(guard);analyse(a.source,a.output)


if __name__=='__main__':main()
