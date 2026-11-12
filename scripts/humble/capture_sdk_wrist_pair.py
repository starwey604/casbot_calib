#!/usr/bin/env python3
"""Capture exactly two c00 right-pitch probes over isolated local SDK Mock."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
from casbot_calib.hardware_probe_capture import design,collect,analyse


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['run','analyse'])
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--source',type=Path)
    parser.add_argument('--domain-id',type=int,default=227)
    a=parser.parse_args()
    if a.action=='run':
        design(a.output);collect(a.output,a.domain_id)
        subprocess.run([sys.executable,__file__,'analyse','--source',str(a.output),'--output',str(a.output/'analysis')],check=True)
    else:
        if a.source is None:parser.error('analyse requires --source')
        def guard(event,args):
            if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)) and 'evaluation' in Path(os.fsdecode(args[0])).parts:
                raise RuntimeError('Private evaluation data forbidden in public probe inspection')
        sys.addaudithook(guard)
        analyse(a.source,a.output)


if __name__=='__main__':main()
