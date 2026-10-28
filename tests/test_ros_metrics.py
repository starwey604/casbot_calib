"""Transport statistics must not invent cross-clock delays or count sampling skips as DDS loss."""
import numpy as np
import json
import pytest
from casbot_calib.ros_benchmark import rate,latency,distribution,driver_data


def test_unknown_clock_has_no_one_way_latency_even_when_numbers_look_aligned(tmp_path):
    a=np.array([100,200,300],dtype=np.int64)
    assert latency(a+10,a,False)==dict(observable=False,reason='clock_domains_not_aligned',milliseconds=None)
    result=latency(a+10,a,True)
    assert result['observable'] and abs(result['milliseconds']['p50']-1e-5)<1e-15
    (tmp_path/'clocks.json').write_text(json.dumps(dict(client='client',host='host',sample='unmapped_device',sample_time_known=True)))
    with pytest.raises(ValueError,match='aligned'):driver_data(tmp_path)


def test_rate_and_jitter_use_actual_arrivals_including_missed_deadlines():
    result=rate(np.array([0,1_000_000,2_000_000,5_000_000],dtype=np.int64))
    assert result['hz']==600. and result['interval_ms']['maximum']==3.
    assert result['nonpositive_intervals']==0
    assert rate(np.array([1]))['hz'] is None
    assert rate(np.array([1,1]))['hz'] is None
    assert distribution([]) is None
