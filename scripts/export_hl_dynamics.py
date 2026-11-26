"""Export runtime_dynamics.json to hl_robot_control sidecar YAML."""
import argparse
from pathlib import Path

from casbot_calib.dynamic_export import export_hl_dynamics_ff


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runtime_json', type=Path, help='Path to runtime_dynamics.json')
    parser.add_argument(
        '-o',
        '--output',
        type=Path,
        default=None,
        help='Output dynamics_ff.yaml path (defaults to runtime_json sibling dynamics_ff.yaml)',
    )
    args = parser.parse_args()
    output = args.output or (args.runtime_json.parent / 'dynamics_ff.yaml')
    written = export_hl_dynamics_ff(args.runtime_json, output)
    print(f'Wrote {written}')


if __name__ == '__main__':
    main()
