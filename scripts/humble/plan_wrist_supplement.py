#!/usr/bin/env python3
"""Seal the 004 wrist supplement without collecting any data."""
import argparse
from pathlib import Path

import yaml

from casbot_calib.staged_wrist_supplement import plan_supplement


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--design', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config, report = plan_supplement(args.source, args.design, args.output)
    # Write only the public config here. The capture driver reads the synthetic
    # plant separately from the session, as in the existing P0 split.
    public = {k:v for k,v in config.items() if k != 'mock'}
    (args.output/'supplement_config.yaml').write_text(yaml.safe_dump(public), encoding='utf-8')
    print(f"Reference gate passed: {report['new_development_trials']} new probes; "
          f"{report['reused_development_trials']} existing development records")
