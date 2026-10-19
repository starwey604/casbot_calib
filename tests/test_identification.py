from dataclasses import replace
import json
import numpy as np
import pytest
from casbot_calib.device import simulate,default_plant
from casbot_calib.excitation import torque_commands
from casbot_calib.identification import Prior,Fourier,identify,identify_gain,scaled_fit,prepare
from casbot_calib.data import save_run,load_observations
from casbot_calib.cli import main

@pytest.fixture(scope="module")
def periodic_data():
    times,commands=torque_commands()
    return simulate(default_plant(),times,commands,24_000_000_000)[:2]

def test_fourier_derivatives_with_missing_observations():
    t=np.linspace(0,8,4001)
    t=t[~((t>2.1)&(t<2.2))]
    y=.3*np.sin(np.pi*t+.2)+.04*np.cos(3*np.pi*t)
    f=Fourier(t,y,2,8)
    grid=np.linspace(0,2,1700)
    np.testing.assert_allclose(f.evaluate(grid,1),.3*np.pi*np.cos(np.pi*grid+.2)-.12*np.pi*np.sin(3*np.pi*grid),atol=1e-12)
    np.testing.assert_allclose(f.evaluate(grid,2),-.3*np.pi**2*np.sin(np.pi*grid+.2)-.36*np.pi**2*np.cos(3*np.pi*grid),atol=1e-11)

def test_svd_rejects_single_frequency():
    q=np.sin(np.linspace(0,30,1000));dq=np.cos(np.linspace(0,30,1000))
    fit=scaled_fit(np.column_stack((-4*q,dq,q)),q,["inertia","damping","spring"])
    assert not fit["identifiable"] and fit["rank"]==2 and fit["parameters"]=={}

def test_truth_isolation_and_raw_immutability(periodic_data,tmp_path):
    obs,truth=periodic_data
    original={k:v.copy() for k,v in obs.items()}
    run=tmp_path/"recording"
    save_run(run,obs,truth,{"known_conditions":{"gain":2.}})
    (run/"evaluation_truth.npz").unlink()
    # Metadata is unnecessary for estimator access; truth cannot be read back.
    (run/"metadata.json").unlink()
    estimate=identify(load_observations(run))
    assert estimate["identifiable"]
    for name,value in estimate["parameters"].items():
        assert abs(value/default_plant()[name]-1)<.01
    for k,v in original.items(): np.testing.assert_array_equal(obs[k],v)
    config=tmp_path/"inference.yaml";config.write_text("identification: {}\n")
    assert main(["identify",str(run),"--config",str(config),"--output",str(tmp_path/"fit.json")])==0
    assert json.loads((tmp_path/"fit.json").read_text())["identifiable"]

def test_gain_scale_anchor_is_required(periodic_data):
    o,_=periodic_data
    assert identify_gain(o,o,None)["reason"]=="missing_known_inertia_scale_anchor"

def test_timing_guards_and_packet_quality(periodic_data):
    obs,_=periodic_data
    corrupt={k:v.copy() for k,v in obs.items()}
    corrupt["status"][13000:13100]=2
    corrupt["q"][14000]=np.nan
    *_,diagnostics=prepare(corrupt,Prior())
    assert diagnostics["sequence_gaps"]>=2
    assert diagnostics["used_samples"]<12001
    with pytest.raises(ValueError,match="correct twice"):
        identify(obs,offset_bounds_s=(-.004,.004))
    with pytest.raises(ValueError,match="Missing"):
        identify({"q":np.zeros(10)})

def test_unit_schema_rejects_degrees(periodic_data,tmp_path):
    obs,_=periodic_data
    run=tmp_path/"units"
    save_run(run,obs,None,{})
    schema=run/"observation_schema.json"
    data=json.loads(schema.read_text());data["units"]["q"]="degree"
    schema.write_text(json.dumps(data))
    with pytest.raises(ValueError,match="SI units"):
        load_observations(run)
