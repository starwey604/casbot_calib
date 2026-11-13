"""Real independent processes and the physical plant; all endpoints are private."""
import csv
import json
import os
from pathlib import Path
import struct
import subprocess
import sys

import numpy as np
import pytest
from casbot_calib import shm_fieldwork as fw
from casbot_calib.shm_reference import jog_bundle,export_motion,fields_bundle,save_bundle
from casbot_calib.shm_data import NAMES,ROOT,URDF
from casbot_calib.staged_motion import Motion

pytestmark=pytest.mark.skipif(sys.platform!='linux',reason='Linux POSIX SHM only')
BIN=Path(os.environ.get('CASBOT_SHM_TEST_BIN',ROOT/'build/humble/cpp/src/shm'))


@pytest.fixture
def native():
    for name in ('shm_executor','shm_mock_server','shm_reference_check'):
        if not (BIN/name).exists():pytest.skip('Build --with-mock for native physics tests')
    return BIN


def cli(*args):
    return subprocess.run([sys.executable,'-m','casbot_calib.shm_fieldwork',*map(str,args)],text=True,capture_output=True,timeout=25)


def evaluate(native,path,times):
    result=subprocess.run([str(native/'shm_reference_check'),str(path)],input=''.join(f'{int(t)}\n' for t in times),capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    return np.array([[float(v) for v in line.split(',')] for line in result.stdout.splitlines()])


def test_septic_evaluator_matches_scipy_with_nonzero_internal_derivatives(native,tmp_path):
    motion=Motion('independent','test',np.zeros(14))
    motion.add(.7,np.full(14,.1),velocity=np.full(14,.02)).add(.9,np.full(14,-.05))
    motion.states[1][2]=.03;motion.states[1][3]=-.04
    bundle=export_motion(motion,kp=[20.]*14,kd=[2.]*14,
                         effort=[dict(t_ns=0,value=[0.]*14),dict(t_ns=700_000_000,value=[.12]*14)])
    path=save_bundle(tmp_path/'reference',bundle)
    times=np.r_[0,np.arange(1234567,1_600_000_000,17000000),700_000_000,1_600_000_000]
    actual=evaluate(native,path,times);curve=motion.polynomial()
    for d in range(4):np.testing.assert_allclose(actual[:,1+d*14:1+(d+1)*14],curve(times*1e-9,nu=d),rtol=1e-9,atol=1e-8)
    np.testing.assert_allclose(actual[:,57:],np.where((times>=700_000_000)[:,None],.12,0)*np.ones((1,14)),atol=0)


def test_direct_fields_do_not_force_velocity_to_position_derivative(native,tmp_path):
    bundle=fields_bundle([0.]*14,12,velocity=.05,effort=.12,kp=0,kd=0,duration=.1)
    path=save_bundle(tmp_path/'fields',bundle);actual=evaluate(native,path,[0,50_000_000,100_000_000])
    np.testing.assert_array_equal(actual[:,1:15],0)
    np.testing.assert_allclose(actual[:,15],.05,atol=0)
    np.testing.assert_allclose(actual[:,57],.12,atol=0)


def test_relative_reference_physical_loop_is_not_target_echo(native,tmp_path):
    path=save_bundle(tmp_path/'reference',jog_bundle([0.]*14,12,.01,1,relative=True))
    output=tmp_path/'run'
    completed=cli('execute',path,'--mock','--execute','--executor',native/'shm_executor','--output',output)
    assert completed.returncode==0,completed.stderr+completed.stdout
    result=json.loads((output/'result.json').read_text())
    assert result['execution_completed'] and result['mock_exit']['returncode']==0
    assert result['native']['reference_kind']=='septic_hermite'
    assert result['feedback']['joints'][12]['errors']==[]
    assert (output/'native/reference_origin.csv').exists()
    # Raw commands and actual feedback have different shapes/extrema: a rigid
    # body with gravity/local PD is not an ideal position follower.
    q=result['feedback']['joints'][12]['q_rad'];u=result['commands']['arms'][0]['q_rad']
    assert max(abs(q[k]-u[k]) for k in ('min','max','last'))>1e-4
    assert result['feedback']['joints'][12]['raw_torque']['max']!=0
    name=result['case']['endpoint']
    assert not Path('/dev/shm'+name).exists()
    assert not Path('/dev/shm/sem.'+name[1:]+'_front').exists()


def test_physics_observer_has_no_command_sender(native,tmp_path):
    output=tmp_path/'observe'
    done=cli('observe','--duration','.1','--mock','--executor',native/'shm_executor','--output',output)
    assert done.returncode==0,done.stdout+done.stderr
    r=json.loads((output/'result.json').read_text())
    assert r['commands']['count']==0 and r['feedback']['count']>2
    assert r['feedback']['joints'][13]['raw_position']['max']!=r['feedback']['joints'][13]['raw_position']['min']


def test_direct_effort_is_quantized_and_stops_with_disable(native,tmp_path):
    # This is a deliberate zero-gravity fixture, still with the full inertia/PD
    # plant. Default physics profile retains gravity and is tested separately.
    cfg=json.loads((ROOT/'configs/shm/mock.json').read_text());cfg['urdf']=str(URDF);cfg['gravity_world']=[0,0,0]
    profile=tmp_path/'zero_g.json';profile.write_text(json.dumps(cfg))
    b=fields_bundle(cfg['initial_q'],12,velocity=.05,effort=.129,kp=0,kd=0,duration=.08)
    path=save_bundle(tmp_path/'fields',b);out=tmp_path/'run'
    done=cli('execute',path,'--mock','--mock-config',profile,'--execute','--executor',native/'shm_executor','--output',out)
    assert done.returncode==0,done.stdout+done.stderr
    r=json.loads((out/'result.json').read_text());assert r['native']['disable_ack']=='1'
    rows=list(csv.DictReader((out/'native/commands.csv').open()))
    stream=[row for row in rows if row['phase']=='3']
    assert stream and all(abs(float(row['effort0'])-.12)<1e-12 for row in stream)
    assert all(abs(float(row['v0'])-.05)<1.1e-6 for row in stream)
    assert r['feedback']['joints'][12]['raw_torque']['max']>0


def test_physics_drive_fault_ends_no_reset_or_reenable(native,tmp_path):
    cfg=json.loads((ROOT/'configs/shm/mock.json').read_text());cfg['urdf']=str(URDF);cfg['fault_after_commands']=80
    profile=tmp_path/'fault.json';profile.write_text(json.dumps(cfg));out=tmp_path/'run'
    done=cli('run','C01','--duration','.2','--mock','--mock-config',profile,'--execute',
             '--executor',native/'shm_executor','--output',out)
    assert done.returncode!=0,done.stdout
    r=json.loads((out/'result.json').read_text())
    assert not r['execution_completed'] and r['native']['reason']=='drive_error_slot_12_42'
    records=list(fw.records(out/'native/commands.bin',752))
    assert struct.unpack_from('<H',records[-1],8+12+12*18)[0]==6
    assert all(struct.unpack_from('<H',d,8+12+12*18)[0]!=0x80 for d in records)


def test_mock_name_guard_refuses_real_shm_before_creation(native,tmp_path):
    out=tmp_path/'never'
    r=subprocess.run([str(native/'shm_mock_server'),'--shm','/shm_mc','--config',str(ROOT/'configs/shm/mock.json'),
                      '--output',str(out)],capture_output=True,text=True)
    assert r.returncode!=0 and not out.exists()


def test_bad_reference_refuses_before_any_motion(native,tmp_path):
    cfg=jog_bundle([0.]*14,12,.01,1,relative=True);cfg['knots_ns'][2]=cfg['knots_ns'][1]
    file=save_bundle(tmp_path/'bad',cfg);out=tmp_path/'run'
    done=cli('execute',file,'--mock','--execute','--executor',native/'shm_executor','--output',out)
    assert done.returncode!=0
    r=json.loads((out/'result.json').read_text());assert r['commands']['count']==0
    assert r['native']['reason']=='Reference knots must increase strictly'


def test_owner_cleans_hard_killed_mock_without_touching_another(native,tmp_path):
    from casbot_calib.shm_mock import SHMMock
    with SHMMock(tmp_path/'second',native/'shm_executor') as second:
        with SHMMock(tmp_path/'first',native/'shm_executor') as first:
            endpoint=first.endpoint
            first.process.kill();first.process.wait(timeout=5)
        assert not Path('/dev/shm'+endpoint).exists()
        assert not Path('/dev/shm/sem.'+endpoint[1:]+'_front').exists()
        assert second.process.poll() is None and Path('/dev/shm'+second.endpoint).exists()


def test_bad_mock_startup_is_recorded_without_hardware_fallback(native,tmp_path):
    cfg=json.loads((ROOT/'configs/shm/mock.json').read_text());cfg['urdf']=str(URDF);cfg['integration_ns']=100000
    file=tmp_path/'bad.json';file.write_text(json.dumps(cfg));out=tmp_path/'run'
    done=cli('observe','--mock','--mock-config',file,'--duration','.1','--executor',native/'shm_executor','--output',out)
    assert done.returncode!=0
    r=json.loads((out/'result.json').read_text())
    assert not r['execution_completed'] and r['commands']['count']==0
    assert 'mock_startup_error' in r['process']


def test_mock_accepts_shared_inertia_viscous_settings_and_rejects_implicit_dry_friction(native,tmp_path):
    cfg=json.loads((ROOT/'configs/shm/mock.json').read_text());cfg['urdf']=str(URDF)
    cfg['inertia_scales']=[1.5]*14;cfg['friction_viscous']=[.02]*14
    profile=tmp_path/'plant.json';profile.write_text(json.dumps(cfg));out=tmp_path/'run'
    done=cli('observe','--duration','.05','--mock','--mock-config',profile,
             '--executor',native/'shm_executor','--output',out)
    assert done.returncode==0,done.stdout+done.stderr
    r=json.loads((out/'result.json').read_text());assert r['execution_completed']
    assert r['commands']['count']==0 and r['feedback']['count']>2
    recorded=json.loads((out/'mock/server/config.json').read_text())
    assert recorded['inertia_scales']==[1.5]*14 and recorded['friction_viscous']==[.02]*14
    cfg['friction_coulomb']=[.01]*14;cfg['friction_static']=[.02]*14
    profile.write_text(json.dumps(cfg));out=tmp_path/'rejected'
    done=cli('observe','--duration','.05','--mock','--mock-config',profile,
             '--executor',native/'shm_executor','--output',out)
    assert done.returncode!=0
    r=json.loads((out/'result.json').read_text())
    assert r['commands']['count']==0 and not r['execution_completed']
    assert 'stiction_enabled=true' in r['process']['mock_startup_error']


def test_mock_cleanup_failure_is_visible_and_returns_failure(tmp_path,monkeypatch):
    from types import SimpleNamespace
    (tmp_path/'native').mkdir();(tmp_path/'mock').mkdir()
    (tmp_path/'native/result.txt').write_text('execution_completed=1\nreason=observe_completed\n')
    meta=dict(returncode=0)
    (tmp_path/'process.json').write_text(json.dumps(meta))
    monkeypatch.setattr(fw,'supervise',lambda *args,**kwargs:meta)
    def close():
        (tmp_path/'mock/exit.json').write_text(json.dumps(dict(returncode=0,cleanup_errors=['injected permission error'])))
    args=fw.parser().parse_args(['observe','--duration','.1'])
    assert fw._native_action(args,tmp_path,peer=SimpleNamespace(returncode=0),peer_close=close)==1
    result=json.loads((tmp_path/'result.json').read_text())
    assert not result['execution_completed']
    assert 'injected permission error' in (tmp_path/'report.md').read_text()
