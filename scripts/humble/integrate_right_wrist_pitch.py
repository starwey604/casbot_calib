#!/usr/bin/env python3
"""Bind approved positive-roll data and refit R1; no device I/O."""
import argparse
from pathlib import Path

from casbot_calib.staged_wrist_periodic import fit_integration, plan_integration


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path)
    for name in ('left-capture','right-capture','left-support','output'):
        p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    root=plan_integration(a.source,a.left_capture,a.output,right_pitch=a.right_capture,left_support=a.left_support)
    print('Bound eight positive-roll right pitch probes; refitting R1',flush=True)
    result=fit_integration(root)
    print('R1 refit:', 'PASS' if result['passed'] else 'REJECT',flush=True)
    raise SystemExit(0 if result['passed'] else 1)
