"""SDK source identity; no ROS imports and no robot access."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def sdk_identity(root):
    root = Path(root).resolve()
    baseline = json.loads((root/'examples/config/runtime-contract.json').read_text())
    exported = set(re.findall(r'"((?:msg|srv|action)/[^"\n]+)"', (root/'crb_ros_msg/CMakeLists.txt').read_text()))
    expected = set(baseline['interfaces']) | set(baseline['sdk_extensions'])
    if exported != expected:
        raise ValueError('SDK interface exports differ from its audited manifest')
    digests = {name: sha256(root/'crb_ros_msg'/name) for name in sorted(exported)}
    if any(digests[k] != v for k, v in baseline['interfaces'].items()):
        raise ValueError('SDK interface bytes differ from its audited manifest')
    revision = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    dirty = subprocess.check_output(['git', '-C', str(root), 'status', '--porcelain', '--untracked-files=no'], text=True)
    return dict(schema_version='casbot_sdk_identity_v1', sdk_root=str(root), sdk_commit=revision,
                tracked_worktree_dirty=bool(dirty), interfaces=digests,
                manifest_sha256=sha256(root/'examples/config/runtime-contract.json'),
                cmake_sha256=sha256(root/'crb_ros_msg/CMakeLists.txt'),
                audited_motion_commit=baseline['motion_commit'], audited_messages_commit=baseline['messages_commit'],
                runtime_version_verified=False, interface_hashes_verified=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sdk', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    identity = sdk_identity(args.sdk)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(identity, indent=2)+'\n')


if __name__ == '__main__':
    main()
