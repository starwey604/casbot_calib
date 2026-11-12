#!/usr/bin/env python3
"""Local SDK Mock wrist H4 exploration; never connects physical endpoints."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
from casbot_calib.hardware_wrist_exploration import design, collect, analyse


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['run','analyse'])
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--source', type=Path)
    p.add_argument('--operator-probes', type=Path)
    p.add_argument('--domain-id', type=int, default=227, help='Explicit local-only Mock domain, default227')
    a=p.parse_args()
    if a.action=='analyse':
        if a.source is None: p.error('analyse requires --source')
        # Block every hidden evaluation directory, including fresh-fit verification.
        def guard(event,args):
            if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)) and 'evaluation' in Path(os.fsdecode(args[0])).parts:
                raise RuntimeError('Hidden evaluation access forbidden during exploratory analysis')
        sys.addaudithook(guard)
        analyse(a.source,a.output)
    else:
        if a.operator_probes is None: p.error('run requires the original public --operator-probes')
        if not 0<=a.domain_id<=232:p.error('local domain must be0..232')
        design(a.output,a.operator_probes)
        collect(a.output,a.domain_id)
        subprocess.run([sys.executable,__file__,'analyse','--source',str(a.output),
                        '--output',str(a.output/'analysis')],check=True)


if __name__=='__main__': main()
