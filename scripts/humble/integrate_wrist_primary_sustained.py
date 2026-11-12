#!/usr/bin/env python3
"""Bind uniform sustained primary-wrist support to a new development protocol."""
import argparse
from pathlib import Path
from casbot_calib.staged_protocol import _read
from casbot_calib.staged_wrist_periodic import plan_integration,fit_integration

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ('source','support','output'):p.add_argument(arg,type=Path)
    a=p.parse_args()
    capture=Path(_read(a.source/'wrist_periodic_input.json')['capture'])
    root=plan_integration(a.source,capture,a.output,wrist_primary_support=a.support)
    result=fit_integration(root)
    print('Uniform primary-wrist sustained R1',result['passed'],flush=True)
    raise SystemExit(0 if result['passed'] else 1)
