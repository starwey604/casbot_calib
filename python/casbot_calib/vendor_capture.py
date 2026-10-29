"""Launch the optional subscription-only C++ recorder. No ROS Python dependency."""
import json
import math
import os
from pathlib import Path
import subprocess
from .vendor_data import load_config, raw_fingerprints
from .vendor_identity import sha256


def capture(config_path, output, *, duration_s, domain_id, scope):
    config = load_config(config_path)
    if not math.isfinite(duration_s) or duration_s <= 0 or not 0 <= domain_id <= 232:
        raise ValueError('A positive duration and explicit ROS domain 0..232 are required')
    if scope not in ('localhost_contract_mock', 'hardware_read_only'):
        raise ValueError('Explicit connection scope is required')
    if scope == 'localhost_contract_mock' and any(
            config[key] and not config[key].startswith(('/casbot_calib_s1_', '/casbot_calib_s2_'))
            for key in ('joint_topic', 'debug_topic', 'mode_topic', 'robot_state_topic')):
        raise ValueError('Local contract tests require private calibration S1/S2 topics')
    prefix = Path(os.environ.get('CASBOT_VENDOR_INSTALL', 'install/vendor')).resolve()
    binary = prefix/'casbot_calib_vendor/lib/casbot_calib_vendor/state_recorder'
    source = json.loads((prefix/'vendor-source.json').read_text())
    for name, digest in source['interfaces'].items():
        if sha256(prefix/'crb_ros_msg/share/crb_ros_msg'/name) != digest:
            raise ValueError('Installed vendor interfaces differ from the build identity')
    libraries = sorted((prefix/'crb_ros_msg/lib').glob('*.so'))
    if not libraries or not binary.is_file():
        raise ValueError('Build the optional vendor overlay first')
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
    session = dict(schema_version='casbot_vendor_capture_v1', config=config, sdk_identity=source,
                   binary_sha256=sha256(binary), type_library_sha256={p.name: sha256(p) for p in libraries},
                   config_sha256=sha256(config_path), connection_scope=scope,
                   domain_id=domain_id, duration_s=duration_s, read_only=True,
                   environment={key: os.environ.get(key) for key in ('RMW_IMPLEMENTATION', 'ROS_DISTRO', 'CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE')})
    (output/'session.json').write_text(json.dumps(session, ensure_ascii=False, indent=2)+'\n')
    args = [str(binary), '--ros-args', '-p', 'output:='+str(output/'raw'), '-p', 'duration_s:='+str(float(duration_s))]
    for key in ('joint_topic', 'joint_type', 'debug_topic', 'mode_topic', 'robot_state_topic', 'reliable', 'depth'):
        value = config[key]
        # ROS CLI needs an explicit YAML empty string rather than a missing value.
        value = "''" if value == '' else str(value).lower() if isinstance(value, bool) else str(value)
        args += ['-p', key+':='+value]
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain_id), ROS_LOCALHOST_ONLY='1' if scope == 'localhost_contract_mock' else '0')
    with (output/'process.log').open('x') as log:
        process = subprocess.Popen(args, env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            code = process.wait()
        except KeyboardInterrupt:
            import signal
            process.send_signal(signal.SIGINT)
            try: code = process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate(); code = process.wait(timeout=5)
    result = dict(returncode=code, completed=False, read_only=True)
    if (output/'raw/summary.json').exists():
        result['completed'] = code == 0 and json.loads((output/'raw/summary.json').read_text())['reason'] == 'duration_complete'
        result['source_sha256'] = raw_fingerprints(output)
    (output/'capture.json').write_text(json.dumps(result, indent=2)+'\n')
    return result
