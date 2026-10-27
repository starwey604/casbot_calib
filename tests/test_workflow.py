"""Negative gates and independent physical/identification checks for handoff."""
from dataclasses import replace
import numpy as np
import pytest
from casbot_calib.workflow import DriveSession, repeated_batch_summary
from casbot_calib.demo import DemoRecorder
from casbot_calib.device import simulate, default_plant
from casbot_calib.excitation import multisine
from casbot_calib.identification import (Prior, identify, features, scaled_fit,
    identify_current_response, current_response_residual)


def test_session_rejects_changed_drive_and_undeclared_fixture(tmp_path):
    recorder=DemoRecorder(tmp_path/"session",{"plant":{"current_tau":.002,"command_delay_ns":1300000}})
    with pytest.raises(ValueError,match="Drive configuration changed"):
        recorder.collect("changed_driver",{"current_tau":.001})
    recorder.plant["current_limit"]=3.
    with pytest.raises(ValueError,match="Drive configuration changed"):
        recorder.collect("mutated_driver")
    recorder.plant["current_limit"]=10.
    with pytest.raises(ValueError,match="Declare changed mechanical"):
        recorder.collect("hidden_payload",{"inertia":.12})
    assert not (recorder.folder/"changed_driver").exists()
    assert recorder.session.records==[]


def test_repeatability_does_not_certify_unstable_or_biased_batches():
    reference=dict(inertia=.08,damping=.12)
    rows=[dict(seed=k,parameters={**reference,"inertia":.08*(1+.02*k)},
               frozen_torque_rmse=.002 if k<8 else .02) for k in range(10)]
    result=repeated_batch_summary(rows,reference)
    assert result["passed_count"]<9
    assert result["empirical_parameter_quantiles"]["inertia"]["p95"]>.09
    # Stable repeated refits can still all fail frozen-model prediction.
    result=repeated_batch_summary([dict(parameters=reference.copy(),frozen_torque_rmse=.03)
                                   for _ in range(10)],reference)
    assert result["passed_count"]==0


def test_signed_gravity_retrieval_from_analytic_data_without_simulator():
    t=np.arange(0.,12.001,.001)
    q,v,a=multisine(t,amplitudes=(.65,.22,.08))
    i=(.08*a+.12*v+.06*np.tanh(v/.03)+.35*np.sin(q)-.12*np.cos(q))/2
    ns=np.rint(t*1e9).astype(np.int64)
    obs=dict(q=q,dq=v,current=i,sample_ns=ns,current_sample_ns=ns,receive_ns=ns,
             sequence=np.arange(len(t),dtype=np.int64),status=np.zeros(len(t),dtype=np.uint32))
    fit=identify(obs,Prior(discard_s=4.,include_spring=False,include_gravity=True))
    assert fit["identifiable"] and fit["model_version"]=="output_joint_gravity_v2"
    for name,truth in dict(inertia=.08,damping=.12,coulomb=.06,gravity_sin=.35,gravity_cos=-.12).items():
        assert abs(fit["parameters"][name]/truth-1)<.01
    # Near-zero motion cannot separate sin(q) from a physical linear spring.
    q,v,a=multisine(np.linspace(0,2,2000,endpoint=False),amplitudes=(.001,.0003,.0001))
    x,names=features(q,v,a,Prior(include_gravity=True))
    assert not scaled_fit(x,x@np.ones(len(names)),names)["identifiable"]


def test_gravity_potential_matches_equation_and_conserves_energy():
    p={**default_plant(),"spring":0.,"damping":0.,"coulomb":0.,
       "gravity_sin":.35,"gravity_cos":-.12,"initial_q":.7,"initial_v":.4}
    _,truth,_=simulate(p,np.empty(0,dtype=np.int64),np.empty((0,6)),2_000_000_000)
    expected=.5*p["inertia"]*truth["dq"]**2-.35*np.cos(truth["q"])-.12*np.sin(truth["q"])
    np.testing.assert_allclose(truth["energy"],expected,atol=1e-13)
    assert np.ptp(truth["energy"])<1e-10
    with pytest.raises(ValueError,match="Gravity coefficients"):
        simulate({**p,"gravity_cos":float("nan")},np.empty(0,dtype=np.int64),np.empty((0,6)),1000000)


def test_preliminary_gain_is_nuisance_and_anchored_timing_predicts_new_pulses():
    ts=np.arange(0,600000000,1000000,dtype=np.int64)
    cmd=np.zeros((len(ts),6)); cmd[:,3]=.4*np.where((ts//20000000)%2,1.,-1.)
    p={**default_plant(),"current_tau":.002,"command_delay_ns":1300000}
    obs,_,_=simulate(p,ts,cmd,600000000)
    preliminary=identify_current_response(obs,ts,cmd,1.7,fit_gain=True)
    assert preliminary["gain_status"].startswith("preliminary")
    assert abs(preliminary["effective_gain"]-2.)<1e-5
    fit=identify_current_response(obs,ts,cmd,2.)
    assert abs(fit["command_delay_s"]-.0013)<1e-6
    cmd[:,3]=.32*np.where((ts//29000000)%3==0,1.,-.6)
    new,_,_=simulate(p,ts,cmd,600000000)
    prediction=current_response_residual(new,ts,cmd,2.,fit["current_tau_s"],fit["command_delay_s"])
    assert prediction["parameters_frozen"] and prediction["rmse_a"]<1e-6
    wrong=current_response_residual(new,ts,cmd,2.,.001,0.)
    assert wrong["rmse_a"]>.01
    saturated={**obs,"status":obs["status"].copy()};saturated["status"][40]=1
    with pytest.raises(ValueError,match="Saturated"):
        identify_current_response(saturated,ts,cmd,2.)


def test_workflow_config_records_defaults_and_rejects_unknown_or_invalid_gates():
    from casbot_calib.workflow_config import workflow_config
    a=workflow_config({"identification":{"gain":1.6},"repeat_noise":{"current_noise":.0005}})
    assert a["identification"]["gain"]==1.6 and a["plant"]["current_tau"]==.002
    assert workflow_config()["repeat_noise"]["current_noise"]==.0002
    with pytest.raises(ValueError,match="Unknown workflow"):
        workflow_config({"acceptence":{}})
    with pytest.raises(ValueError,match="Unknown acceptance"):
        workflow_config({"acceptance":{"torque_rmse":.001}})
    with pytest.raises(ValueError,match="Invalid acceptance"):
        workflow_config({"acceptance":{"current_rmse_a":float("nan")}})
    with pytest.raises(ValueError,match="integer"):
        workflow_config({"acceptance":{"minimum_repeat_passes":9.5}})
    with pytest.raises(ValueError,match="Known added inertia"):
        workflow_config({"added_inertia":-.04})


def test_failed_acceptance_stops_and_preserves_raw_records(tmp_path):
    import json
    from casbot_calib.demo import run_demo
    folder=tmp_path/"failed_workflow"
    with pytest.raises(RuntimeError,match="driver_response"):
        run_demo(folder,config={"acceptance":{"current_rmse_a":1e-9}},progress=lambda *_:None)
    failure=json.loads((folder/"workflow_failure.json").read_text(encoding="utf-8"))
    assert failure["failed_gate"]=="driver_response"
    assert (folder/"records/driver_response/observations.npz").exists()
    assert (folder/"workflow_config.json").exists()
    assert not (folder/"identified_model.json").exists()
