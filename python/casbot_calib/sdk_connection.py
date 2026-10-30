"""Frozen transport selection shared by H1/H2; no ROS I/O or plant truth."""
SDK_ENDPOINTS = ('/joint_states', '/motion/joint_state', '/motion/debug/joint_state',
                 '/motion/status', '/upper_body_debug/joint_cmd', '/motion/upper_body_debug')


def mock_connection(endpoint_root, domain_id, protocol_sha256):
    result = dict(schema_version='casbot_sdk_connection_v1', scope='localhost_contract_mock',
                  endpoint_root=endpoint_root, domain_id=domain_id, mock_protocol_sha256=protocol_sha256)
    validate_connection(result)
    return result


def validate_connection(value):
    if value.get('schema_version') != 'casbot_sdk_connection_v1' or value.get('scope') != 'localhost_contract_mock':
        raise ValueError('Explicit local SDK Mock connection required; hardware uses the default SDK endpoints')
    root = value.get('endpoint_root', '')
    if (not root.startswith('/casbot_calib_s2_') or root.endswith('/') or
        any(not (ch.isalnum() or ch == '_') for ch in root[1:])):
        raise ValueError('Single private SDK Mock namespace required')
    if type(value.get('domain_id')) is not int or not 0 <= value['domain_id'] <= 232:
        raise ValueError('Explicit local Mock ROS domain required')
    checksum = value.get('mock_protocol_sha256', '')
    if len(checksum) != 64 or any(ch not in '0123456789abcdef' for ch in checksum):
        raise ValueError('Frozen Mock protocol digest required')
    return value


def connection_remaps(config, domain_id, explicit=None):
    value = config.get('connection')
    if value is None:
        return explicit
    validate_connection(value)
    if domain_id != value['domain_id']:
        raise ValueError('Use the ROS domain frozen in the SDK Mock connection')
    remaps = {name: value['endpoint_root'] + name for name in SDK_ENDPOINTS}
    if explicit is not None and explicit != remaps:
        raise ValueError('Explicit remaps differ from the frozen SDK connection')
    return remaps
