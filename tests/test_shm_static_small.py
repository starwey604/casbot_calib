"""Near-ready static commissioning uses the existing H1 executor and PSD fit."""
import csv
import json
import os
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
import yaml
from casbot_calib.shm_data import ROOT,NAMES
from casbot_calib.shm_h1 import DEFAULT_PROFILE,load_profile,snapshot,prepare,execute,analyse
from casbot_calib.shm_static_small import make_preset,DEFAULT_SETTINGS
from casbot_calib.field_gravity import write_static_diagnostics
from casbot_calib.field_artifacts import check_seal

BIN=Path(os.environ.get('CASBOT_SHM_TEST_BIN',ROOT/'build/humble/cpp/src/shm'))


def test_default_preset_stays_in_user_twenty_degree_shoulder_range(tmp_path):
    root=tmp_path/'preset';report=make_preset(DEFAULT_PROFILE,root);check_seal(root)
    assert report['target_count']==4 and report['measurement_windows']==8
    assert report['including_approach_shoulder_pitch_range_deg']==[-20,20]
    rows=json.loads((root/'candidates.json').read_text())
    assert {(r['arm'],r['split']) for r in rows}=={(a,b) for a in ('left','right') for b in ('train','validation')}
    for name in report['planned_execution_order']:
        folder=root/'plans'/name;m=json.loads((folder/'motion.json').read_text())
        assert json.loads((folder/'collision_reference.json').read_text())['passed']
        assert m['labels'].count('static_measure')==1
        states=np.array(m['states']);np.testing.assert_allclose(states[:,0,1],np.deg2rad(20),atol=1e-15)
        np.testing.assert_allclose(states[:,0,8],-np.deg2rad(20),atol=1e-15)
        assert np.max(abs(states[:,0,[0,7]]))<=np.deg2rad(20)*(1+1e-12)
        assert len(np.flatnonzero(np.ptp(states[:,0,:],axis=0)>1e-10))==1


def test_static_diagnostics_use_actual_q_and_keep_pair_pose_difference(tmp_path):
    rows=[]
    for sign,q,tau in ((-1,.09,.2),(1,.1,.3)):
        rows.append(dict(name=str(sign),conditions=dict(batch='train',pose='p',approach=sign,target_q_rad=[.5]*14),
            q=np.full(14,q),torque=np.full(14,tau)))
    summary=write_static_diagnostics(rows,SimpleNamespace(gravity=lambda q:2*q),tmp_path,NAMES)
    value=json.loads((tmp_path/'static_diagnostics.json').read_text())
    assert summary['available_windows']==2 and summary['paired_targets']==1
    np.testing.assert_allclose(value['windows'][1]['nominal_gravity_nm'],.2)
    np.testing.assert_allclose(value['pairs'][0]['q_plus_minus_rad'],.01)
    np.testing.assert_allclose(value['pairs'][0]['residual_half_difference_nm'],.04)
    with (tmp_path/'static_comparison.csv').open() as f:assert len(list(csv.DictReader(f)))==28


def test_small_static_rejects_expanded_scope_without_clipping(tmp_path):
    c=yaml.safe_load(DEFAULT_SETTINGS.read_text());c['target_offset_deg']=20
    with pytest.raises(ValueError,match='20°'):make_preset(DEFAULT_PROFILE,tmp_path/'bad',settings=c)
    assert not (tmp_path/'bad').exists()


@pytest.mark.skipif(not (BIN/'shm_executor').exists(),reason='build native physics')
def test_eight_windows_physics_fit_and_solver_unavailable_report(tmp_path,monkeypatch):
    native=BIN/'shm_executor';c=load_profile(DEFAULT_PROFILE)
    c.update(initial_hold_s=.05,final_hold_s=.05,anchor_transition_s=.1)
    settings=yaml.safe_load(DEFAULT_SETTINGS.read_text());settings.update(target_offset_deg=.1,approach_offset_deg=.03)
    settings['timing']=dict(initial_hold_s=.05,minimum_transition_s=.1,minimum_paired_approach_s=.1,settle_s=.05,measure_s=.1,final_hold_s=.05)
    preset=tmp_path/'preset';make_preset(c,preset,settings=settings)
    snap=tmp_path/'start';snapshot(c,snap,executor=native,mock=True)
    plan=tmp_path/'plan';m=prepare(c,snap,plan,preset=preset,with_ready=True,executor=native)
    assert len(m['trials'])==8
    bundle=json.loads((plan/'reference.json').read_text());markers=bundle['markers']
    assert len(markers)==19 and all(b['t_ns']>a['t_ns'] for a,b in zip(markers,markers[1:]))
    capture=tmp_path/'capture';assert execute(plan,capture,executor=native,mock=True,do_execute=True)==0
    out=tmp_path/'analysis';report=analyse(plan,capture,out)
    assert report['execution_completed'] and report['analysis_available'] and report['fit_succeeded']
    assert report['static_diagnostics']['available_windows']==8 and report['static_diagnostics']['paired_targets']==4
    assert report['fit']['rank']<report['fit']['columns'] and not report['hardware_calibration_accepted']
    with (out/'static_comparison.csv').open() as f:assert len(list(csv.DictReader(f)))==112
    # Numerical failure is an explicit unavailable analysis, not a motion fault.
    from casbot_calib import rigid_identification
    def failed(*args,**kwargs):raise RuntimeError('test solver failure')
    monkeypatch.setattr(rigid_identification,'fit_rigid',failed)
    unavailable=analyse(plan,capture,tmp_path/'unavailable')
    assert unavailable['execution_completed'] and unavailable['collection_complete'] and not unavailable['analysis_available']
    assert not unavailable['fit_succeeded'] and 'test solver failure' in unavailable['analysis_unavailable_reason']
    assert unavailable['static_diagnostics']['available_windows']==8
