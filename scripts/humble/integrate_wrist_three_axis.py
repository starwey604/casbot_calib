#!/usr/bin/env python3
"""Bind the four existing wrist records and refit R1; never acquire data."""
import argparse
from pathlib import Path

from casbot_calib.staged_wrist_periodic import fit_integration, plan_integration


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('--capture', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    root = plan_integration(args.source, args.capture, args.output)
    print('Bound four train-only wrist-periodic records; refitting R1', flush=True)
    result = fit_integration(root)
    print('R1 refit:', 'PASS' if result['passed'] else 'REJECT', flush=True)
    raise SystemExit(0 if result['passed'] else 1)
