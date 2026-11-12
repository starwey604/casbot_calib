#!/usr/bin/env python3
"""Create and refit a separate sealed development protocol for sustained pitch."""
import argparse
from pathlib import Path

from casbot_calib.staged_protocol import _read
from casbot_calib.staged_wrist_periodic import fit_integration,plan_integration

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('source','support','output'):p.add_argument(name,type=Path)
    a=p.parse_args()
    left=Path(_read(a.source/'wrist_periodic_input.json')['capture'])
    root=plan_integration(a.source,left,a.output,sustained_pitch_support=a.support)
    r=fit_integration(root)
    print('Sustained pitch R1',r['passed'],flush=True)
    raise SystemExit(0 if r['passed'] else 1)
