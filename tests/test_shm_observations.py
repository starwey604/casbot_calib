"""Independent time-axis fixtures: coarse stamps, large integer epochs and jumps."""
import csv
import json
import numpy as np
import pytest
from casbot_calib.shm_observations import ObservationReader
from casbot_calib.hardware_probe import probe_problem,field_probe_config,settings,inertia_balance
from test_hardware_probe import rotor


def recording(root,*,receive_period=1_000_000,jump=False):
    root.mkdir();origin=10**18+234567;wall_ms=10**12;offset=wall_ms*1_000_000-origin
    (root/'result.txt').write_text(f'first_command_mono_ns={origin}\nfirst_shm_write_mono_ns={origin-100_000_000}\n')
    with (root/'commands.csv').open('w') as f:f.write(f'send_mono_ns,phase\n{origin-100_000_000},0\n{origin},3\n')
    with (root/'clock_pairs.csv').open('w') as f:
        f.write('mono_before_ns,wall_ns,mono_after_ns,offset_ns,clock_segment,jump_detected\n')
        f.write(f'{origin-1_000_000_000},{wall_ms*1_000_000-1_000_000_000},{origin-1_000_000_000},{offset},0,0\n')
        if jump:f.write(f'{origin+2_000_000_000},{wall_ms*1_000_000+3_000_000_000},{origin+2_000_000_000},{offset+1_000_000_000},1,1\n')
    with (root/'feedback.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['receive_mono_ns','source_timestamp_ms','observation_time_ns','clock_segment','frame_counter','passive',*[p+str(j) for p in ('q','v','effort') for j in range(14)]])
        for k in range(4001):
            t=k*.001;segment=1 if jump and k>=2000 else 0;stamp=wall_ms+k+1000*segment
            q=np.zeros(14);v=q.copy();tau=q.copy();omega=10*np.pi
            q[4]=.18*t+.004*np.sin(omega*t);v[4]=.18+.004*omega*np.cos(omega*t)
            tau[4]=5e-5*(-.004*omega**2*np.sin(omega*t))+.035*v[4]+.04
            writer.writerow([origin+k*receive_period,stamp,origin+k*1_000_000,segment,k,0,*q,*v,*tau])
    return origin


def problem(obs,elapsed):
    meta=dict(conditions=dict(joint=4,direction=1),labels=['probe_measure'],knots_s=[1.,3.])
    return probe_problem(obs,elapsed,meta,rotor(),field_probe_config(settings()),dict(effort_gain=1.,effort_zero_nm=0.),observation_time_ns=obs['observation_time_ns'])


def test_a_uses_observation_ns_not_first_shm_write_or_receive_clock(tmp_path):
    origin=recording(tmp_path/'normal');recording(tmp_path/'delayed',receive_period=3_000_000)
    obs,t,scope=ObservationReader(tmp_path/'normal').window((1.,3.))
    other,other_t,_=ObservationReader(tmp_path/'delayed').window((1.,3.))
    assert obs['observation_time_ns'].dtype==np.int64 and obs['observation_time_ns'][0]==origin
    assert t[0]==0. and scope['clock_segment']==0
    np.testing.assert_array_equal(t,other_t)
    first=problem(obs,t);second=problem(other,other_t)
    np.testing.assert_allclose(first['target'],second['target'],atol=0)
    assert inertia_balance(first)['inertia_kg_m2']==pytest.approx(5e-5,rel=.001)
    assert 'cache_standard_ns' not in first['preprocessing']


def test_duplicate_rows_share_timestamp_group_exposure_without_deduplication(tmp_path):
    recording(tmp_path/'normal');obs,t,_=ObservationReader(tmp_path/'normal').window((1.,3.))
    base=problem(obs,t)
    counts=1+np.arange(len(t))%4;indices=np.repeat(np.arange(len(t)),counts)
    repeated={k:v[indices] for k,v in obs.items()}
    # Receive events remain distinct. Their cadence is intentionally irrelevant.
    repeated['receive_ns']=10**18+np.arange(len(indices))*1000
    duplicate=problem(repeated,t[indices])
    np.testing.assert_allclose(base['target'],duplicate['target'],rtol=1e-9,atol=1e-10)
    np.testing.assert_allclose(base['regression'],duplicate['regression'],rtol=1e-8,atol=1e-9)
    assert duplicate['preprocessing']['observation_rows']>duplicate['preprocessing']['observation_time_groups']


def test_clock_jump_does_not_get_sorted_or_glued_into_a_window(tmp_path):
    recording(tmp_path/'jump',jump=True)
    reader=ObservationReader(tmp_path/'jump')
    with pytest.raises(ValueError,match='跳变|分段'):reader.window((1.,3.))
    obs,t,scope=reader.window((2.1,3.),segment=1)
    assert scope['clock_segment']==1 and np.all(np.diff(obs['observation_time_ns'])>0)


def test_source_formula_corruption_and_partial_tail_are_errors(tmp_path):
    recording(tmp_path/'bad');path=tmp_path/'bad/feedback.csv'
    lines=path.read_text().splitlines();values=lines[2].split(',');values[2]=str(int(values[2])+1);lines[2]=','.join(values)
    path.write_text('\n'.join(lines)+'\n')
    with pytest.raises(ValueError,match='不一致'):list(ObservationReader(tmp_path/'bad').rows())
    recording(tmp_path/'tail');path=tmp_path/'tail/feedback.csv'
    with path.open('a') as f:f.write('123,partial\n')
    with pytest.raises(ValueError,match='不完整'):list(ObservationReader(tmp_path/'tail').rows())
