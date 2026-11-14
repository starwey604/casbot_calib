"""Preparation exception is bounded to its prefix; hardware is never accessed."""
import csv
import json
import os
from pathlib import Path
import struct
import subprocess
import numpy as np
import pytest
from casbot_calib.shm_data import ROOT
from casbot_calib.shm_h1 import load_profile,DEFAULT_PROFILE,snapshot,prepare,execute,analyse,replan
from casbot_calib.shm_ready import preparation,ready_position,handover_fraction
from casbot_calib.model_runtime import model_for,urdf_for
from casbot_calib.field_artifacts import seal,FieldContractError

BIN=Path(os.environ.get('CASBOT_SHM_TEST_BIN',ROOT/'build/humble/cpp/src/shm'))
# The measured hanging posture which failed H1 before any motion, 2026-11-14.
HANGING=np.array([-.037817,.10047065,-.132868,-.016,.023154,.045726,-.12043,
                  -.014166,-.11496665,-.12538533,-.054911,.18399067,.03161545,-.05161767])


@pytest.fixture
def native():
    if not (BIN/'shm_executor').is_file():pytest.skip('Build native SHM first')
    return BIN/'shm_executor'


@pytest.fixture
def hanging(tmp_path,native):
    c=load_profile(DEFAULT_PROFILE);engine,_=model_for(c)
    cfg=json.loads((ROOT/'configs/shm/h1_mock.json').read_text());cfg['urdf']=str(urdf_for(c))
    cfg['initial_q']=HANGING.tolist();cfg['initial_fields']['position']=HANGING.tolist()
    cfg['initial_fields']['effort']=engine.gravity(HANGING).tolist()
    fixture=tmp_path/'hanging.json';fixture.write_text(json.dumps(cfg))
    snap=tmp_path/'snapshot';snapshot(DEFAULT_PROFILE,snap,executor=native,mock=True,mock_config=fixture)
    return snap


def test_preparation_uses_urdf_and_velocity_not_collision():
    c=load_profile(DEFAULT_PROFILE);engine,_=model_for(c)
    motion,proof=preparation(HANGING,engine)
    assert proof['passed'] and proof['collision_check']=='skipped_user_authorized_preparation_only'
    assert proof['reference_velocity']['maximum_rad_s']<=.15*(1+1e-9)
    np.testing.assert_array_equal(motion.states[-1][0],ready_position())
    np.testing.assert_array_equal(handover_fraction([0,proof['move_end_ns']],proof),0.)
    assert handover_fraction(proof['handover_end_ns'],proof)==1.
    bad=HANGING.copy();bad[0]=engine.model.joints[0].upper+.1
    with pytest.raises(ValueError,match='URDF'):preparation(bad,engine)


def test_ready_prefix_allows_known_hanging_start_but_old_path_still_refuses(hanging,tmp_path,native):
    with pytest.raises(FieldContractError,match='碰撞'):
        prepare(DEFAULT_PROFILE,hanging,tmp_path/'old',executor=native)
    root=tmp_path/'ready';m=prepare(DEFAULT_PROFILE,hanging,root,executor=native,with_ready=True)
    p=m['preparation'];bundle=json.loads((root/'reference.json').read_text())
    formal=json.loads((root/'calibration_motion.json').read_text());motion=json.loads((root/'motion.json').read_text())
    proof=json.loads((root/'collision_reference.json').read_text())
    assert proof['passed'] and proof['scope']=='formal_calibration_reference_only'
    assert m['recipe']['preparation']=='arm_ready_v1'
    assert bundle['purpose']=='shm_h1_ready_then_collision_checked'
    k=bundle['knots_ns'].index(p['calibration_begin_ns'])
    np.testing.assert_array_equal(bundle['states'][k],formal['states'][0])
    np.testing.assert_array_equal(bundle['states'][k+1:],formal['states'][1:])
    assert np.all(np.asarray(bundle['states'][k])[1:]==0)
    assert motion['conditions']['comparison_window_s'][0]>p['calibration_begin_ns']*1e-9
    assert all(not any(row['value']) for row in bundle['effort'] if row['t_ns']<=p['move_end_ns'])
    assert any(any(row['value']) for row in bundle['effort'] if row['t_ns']>=p['handover_end_ns'])
    rebuilt=tmp_path/'rebuilt';assert replan(root,hanging,rebuilt,executor=native)['recipe']==m['recipe']


def test_native_rejects_bad_schedule_before_opening_any_channel(tmp_path):
    from casbot_calib.shm_reference import jog_bundle
    b=jog_bundle([0.]*14,12,.01,1)
    b['pd_schedule']=[dict(t_ns=0,kp=[501.]*14,kd=[2.]*14)]
    path=tmp_path/'bad.json';path.write_text(json.dumps(b))
    r=subprocess.run([str(BIN/'shm_reference_check'),str(path)],capture_output=True,text=True)
    assert r.returncode!=0 and 'Scheduled PD' in r.stderr
    b['pd_schedule'][0].update(t_ns=1,kp=[20.]*14);path.write_text(json.dumps(b))
    r=subprocess.run([str(BIN/'shm_reference_check'),str(path)],capture_output=True,text=True)
    assert r.returncode!=0 and 'schedule times' in r.stderr


def test_one_enable_cycle_ready_then_h1_physical_capture_and_analysis(hanging,tmp_path,native):
    config=load_profile(DEFAULT_PROFILE);config.update(kp=[30.]*14,kd=[3.]*14)
    plan=tmp_path/'plan';m=prepare(config,hanging,plan,executor=native,with_ready=True,amplitude_deg=1.)
    capture=tmp_path/'capture';assert execute(plan,capture,executor=native,mock=True,do_execute=True)==0
    child=capture/json.loads((capture/'result.json').read_text())['effective_capture']
    with (child/'native/events.csv').open() as f:events=list(csv.DictReader(f))
    names=[r['event'] for r in events]
    assert names.count('enabling_at_start')==1
    assert names.index('ready_begin')<names.index('ready_field_handover_begin')<names.index('h1_calibration_begin')<names.index('exiting_disable')
    with (child/'native/commands.csv').open() as f:commands=list(csv.DictReader(f))
    stream=[r for r in commands if r['phase']=='3'];origin=int(stream[0]['send_mono_ns'])
    assert [int(r['phase']) for r in commands]==sorted(int(r['phase']) for r in commands)
    early=[r for r in stream if int(r['send_mono_ns'])-origin<m['preparation']['move_end_ns']]
    assert early and all(float(r['kp0'])==20 and float(r['kd0'])==2 and all(float(r['effort'+str(j)])==0 for j in range(14)) for r in early)
    late=[r for r in stream if int(r['send_mono_ns'])-origin>=m['preparation']['handover_end_ns']]
    assert late and all(float(r['kp0'])==30 and float(r['kd0'])==3 for r in late)
    middle_ns=(m['preparation']['move_end_ns']+m['preparation']['handover_end_ns'])//2
    middle=min(stream,key=lambda r:abs(int(r['send_mono_ns'])-origin-middle_ns))
    assert abs(float(middle['kp0'])-25)<.2 and abs(float(middle['kd0'])-2.5)<.11
    report=analyse(plan,capture,tmp_path/'analysis')
    assert report['execution_completed'] and report['analysis_available']
    collision=json.loads((tmp_path/'analysis/collision_received_interpolant.json').read_text())
    assert collision['passed'] and collision['excluded_rows']>0 and collision['collision_checked_rows']>1000
    assert report['calibration_begin_s']==m['preparation']['calibration_begin_ns']*1e-9


def test_formal_collision_is_not_covered_by_preparation_exception(hanging,tmp_path,native):
    # Explicit old comparison anchor at all-zero remains forbidden even with prep.
    from casbot_calib.shm_data import NAMES
    recipe=dict(kind='jog',joint=NAMES[0],amplitude_deg=1.,anchor_rad=[0.]*14,preparation='arm_ready_v1')
    with pytest.raises(FieldContractError,match='碰撞'):
        prepare(DEFAULT_PROFILE,hanging,tmp_path/'bad_formal',executor=native,recipe=recipe)


def test_static_windows_shift_but_preparation_is_not_a_measurement(hanging,tmp_path,native):
    from casbot_calib.staged_motion import Motion
    home=ready_position();trial=Motion('single_static','gravity',home,dict(batch='train'))
    trial.add(.1,home,label='static_settle').add(.2,home,label='static_measure')
    recipe=dict(kind='static',home_rad=home.tolist(),preparation='arm_ready_v1',
        planning_envelope=dict(speed_rad_s=.05,acceleration_rad_s2=.1,jerk_rad_s3=1.),
        motions=[dict(trial.metadata(),trial_name='single_static')])
    plan=tmp_path/'static';m=prepare(DEFAULT_PROFILE,hanging,plan,executor=native,recipe=recipe)
    offset=m['preparation']['calibration_begin_ns']*1e-9
    assert len(m['trials'])==1 and m['trials'][0]['measure_begin_s']>offset
    assert m['trials'][0]['measure_end_s']-m['trials'][0]['measure_begin_s']==pytest.approx(.2)
    assert json.loads((plan/'collision_reference.json').read_text())['passed']


def test_ten_degree_jog_retains_collision_and_slow_reference(hanging,tmp_path,native):
    from casbot_calib.staged_motion import derivative_bounds
    from scipy.interpolate import BPoly
    plan=tmp_path/'ten_deg';m=prepare(DEFAULT_PROFILE,hanging,plan,executor=native,with_ready=True,amplitude_deg=10.)
    motion=json.loads((plan/'calibration_motion.json').read_text())
    assert m['recipe']['amplitude_deg']==10 and m['duration_ns']>40_000_000_000
    assert json.loads((plan/'collision_reference.json').read_text())['passed']
    assert derivative_bounds(BPoly.from_derivatives(motion['knots_s'],np.asarray(motion['states']))).max()<=.05*(1+1e-9)
    q=np.asarray(motion['states'])[:,0,0]
    assert q.min()==pytest.approx(-np.deg2rad(10)) and q.max()==pytest.approx(np.deg2rad(10))
    with pytest.raises(ValueError,match='10'):
        prepare(DEFAULT_PROFILE,hanging,tmp_path/'too_big',executor=native,with_ready=True,amplitude_deg=10.1)


def test_preparation_fault_disables_without_entering_formal_reference(hanging,tmp_path,native):
    fixture=json.loads((hanging/'mock/config.json').read_text());fixture['fault_after_commands']=180
    (hanging/'mock/config.json').write_text(json.dumps(fixture));seal(hanging)
    plan=tmp_path/'fault_plan';prepare(DEFAULT_PROFILE,hanging,plan,executor=native,with_ready=True)
    capture=tmp_path/'fault';assert execute(plan,capture,executor=native,mock=True,do_execute=True)!=0
    result=json.loads((capture/'result.json').read_text())
    assert len(result['attempts'])==1 and not result['resume_permitted'] and not result['execution_completed']
    child=capture/result['effective_capture'];native_result=json.loads((child/'result.json').read_text())['native']
    assert 'drive_error' in native_result['reason'] and native_result['disable_ack']=='0'
    # A faulted drive does not return ready-to-switch-on without fault clear.
    # Verify the disable request, not a fabricated successful ready response.
    from casbot_calib.shm_fieldwork import records
    raw=list(records(child/'native/commands.bin',752))
    assert raw and all(struct.unpack_from('<H',raw[-1],8+12+j*18)[0]==6 for j in range(12,26))
    assert all(struct.unpack_from('<H',row,8+12+12*18)[0]!=0x80 for row in raw)
    with (child/'native/events.csv').open() as f:assert 'h1_calibration_begin' not in [r['event'] for r in csv.DictReader(f)]
