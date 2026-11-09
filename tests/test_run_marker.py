from datetime import datetime

from casbot_calib.run_marker import run_marker


def test_run_marker_is_a_utc_timestamp_not_a_device_identity():
    marker = run_marker()
    assert set(marker) == {'run_id', 'created_utc'}
    assert marker['run_id'].endswith('Z') and len(marker['run_id']) == 22
    assert marker['created_utc'].endswith('+00:00')
    assert datetime.fromisoformat(marker['created_utc']).utcoffset().total_seconds() == 0
    assert not any(k in marker for k in ('robot_serial', 'firmware_version', 'device_identity'))
