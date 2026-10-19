"""Recorded calibration workflow. All motion comes from the existing C++ Mock."""
from __future__ import annotations
from dataclasses import replace
import csv
from datetime import datetime, timezone
from importlib.resources import files
import hashlib
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from .data import write_json
from .device import simulate
from .excitation import tracking_commands, multisine
from .experiments import Experiment
from .identification import identify, identify_gain, identify_current_response, predict_residual
from .telemetry import TelemetryAdapter, ReplaySource


def mock_encoder_channels(observations):
    """Fixed synthetic producer convention, independent of the viewer mapping.

    A wrong viewer sign/zero must visibly change the decoded state, rather than
    cancel against a producer configured by the same erroneous mapping.
    """
    return {"raw_position":8192.-observations["q"]*16384./(2*np.pi),
            "raw_velocity":-observations["dq"]*16384./(2*np.pi),
            "raw_current":observations["current"].copy()}


class DemoRecorder(Experiment):
    def collect(self,name,*args,**kwargs):
        observations,truth=super().collect(name,*args,**kwargs)
        np.savez_compressed(self.folder/name/"raw_telemetry.npz",**mock_encoder_channels(observations))
        return observations,truth


def write_tables(folder,report):
    folder=Path(folder)
    for name,rows in (("control_metrics.csv",list(report["controls"].values())),
                      ("pd_candidates.csv",report["pd_candidates"])):
        with (folder/name).open("w",newline="",encoding="utf-8") as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    with (folder/"identified_parameters.csv").open("w",newline="",encoding="utf-8") as f:
        writer=csv.writer(f);writer.writerow(["parameter","estimate","unit"])
        units={"inertia":"kg m^2","damping":"Nm s/rad","spring":"Nm/rad","coulomb":"Nm"}
        for name,value in report["identified"]["parameters"].items():writer.writerow([name,value,units[name]])
        writer.writerow(["gain",report["drive_scale"]["gain"],"Nm/A output side"])


def score_control(observations,args,gain):
    """Same scoring path for a future real recording; evaluation truth is absent."""
    t=observations["sample_ns"]*1e-9
    if np.any(t<0):raise ValueError("Controller comparison needs aligned acquisition time")
    reference=multisine(t,**args)[0]
    valid=(t>=1)&((observations["status"]&2)==0)
    return dict(rmse_rad=float(np.sqrt(np.mean((observations["q"][valid]-reference[valid])**2))),
        peak_velocity_rad_s=float(np.max(np.abs(observations["dq"]))),
        peak_estimated_torque_nm=float(np.max(np.abs(observations["current"]*gain))),
        saturation_fraction=float(np.mean((observations["status"]&1)!=0)))


def run_demo(folder,*,progress=print):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    start=perf_counter()
    adapter=TelemetryAdapter.load(files("casbot_calib").joinpath("assets/bench_mapping.json"))
    write_json(folder/"mapping.json",{"robot_model_id":adapter.robot_model_id,
        "joints":[m.__dict__ for m in adapter.mappings],
        "raw_units_note":"synthetic floating encoder ticks derived from mock observations, not ENCOS data"})
    experiment=DemoRecorder(folder/"records",{"sources":["casbot_calib C++ Mock; synthetic output-side joint"],
        "plant":{"position_noise":1e-6,"current_noise":.0002,"seed":20261020}})
    events=[];checks={};clips=[]
    def event(stage,state,**detail):
        events.append({"stage":stage,"state":state,"utc":datetime.now(timezone.utc).isoformat(),**detail})
        with (folder/"workflow_events.jsonl").open("a",encoding="utf-8") as f:
            f.write(json.dumps(events[-1],ensure_ascii=False)+"\n")
        progress(f"{stage}: {state}")
    def gate(name,condition,**detail):
        checks[name]=bool(condition)
        event(name,"passed" if condition else "failed",**detail)
        if not condition:
            write_json(folder/"workflow_failure.json",{"checks":checks,"failed_gate":name,"detail":detail})
            raise RuntimeError(f"Calibration workflow stopped at {name}; raw records retained")
    def clip(key,title,run,seconds,note,before=None):
        clips.append(dict(key=key,title=title,run=run,seconds=seconds,note=note,before=before))

    event("prepare","started")
    gate("configuration",adapter.mappings[0].model_joint=="observed_joint")
    event("direction","started")
    ts=np.arange(0,3_000_000_000,1_000_000,dtype=np.int64);t=ts*1e-9
    q=.12*.5*(1-np.cos(2*np.pi*np.clip((t-.3)/2.,0,1)))
    v=np.where((t>.3)&(t<2.3),.12*np.pi/2.*np.sin(np.pi*(t-.3)),0.)
    commands=np.column_stack((np.ones(len(ts)),q,v,np.zeros(len(ts)),np.full(len(ts),12.),np.full(len(ts),1.2)))
    jog,_=experiment.collect("direction_jog",{"torque_limit":2.,"current_limit":1.},duration=3.,commands=(ts,commands))
    mapping=adapter.mappings[0];raw=mock_encoder_channels(jog)["raw_position"]
    roundtrip=mapping.position(raw)
    gate("direction_software",np.max(np.abs(roundtrip-jog["q"]))<1e-12 and np.max(jog["q"])>.04)
    direction={"software_mapping":"passed","physical_direction":"unconfirmed_without_independent_physical_observation",
        "positive_model_direction":"counterclockwise about local +Z",
        "mapping":mapping.__dict__,"raw_ticks_at_peak":float(raw[np.argmax(jog["q"])])}
    # A real SDK can publish this exact JSONL format through LatestStateSource.
    with (folder/"telemetry_example.jsonl").open("w",encoding="utf-8") as f:
        for i in range(0,len(jog["q"]),20):
            j=min(i,len(ts)-1)
            packet={"robot_model_id":adapter.robot_model_id,"source":"cpp_mock",
                "joint_names":[mapping.name],"sequence":int(jog["sequence"][i]),
                "sample_ns":int(jog["sample_ns"][i]),"receive_ns":int(jog["receive_ns"][i]),
                "raw_position":[float(raw[i])],"raw_velocity":[float(-jog["dq"][i]*16384./(2*np.pi))],
                "raw_current":[float(jog["current"][i])],"target_q":[float(q[j])],"clock_domain":"virtual_device_clock"}
            f.write(json.dumps(packet)+"\n")
    clip("direction","Direction & zero check","direction_jog",4.,"Raw encoder / normalized angle / model axis. Physical direction still needs independent confirmation.")

    event("timing","started")
    ts=np.arange(0,2_000_000_000,1_000_000,dtype=np.int64);values=np.zeros((len(ts),6))
    values[:,3]=.4*np.where((ts//20_000_000)%2==0,1.,-1.)
    driver,_=experiment.collect("driver_response",{"current_tau":.002,"command_delay_ns":1_300_000},duration=2.,commands=(ts,values))
    response=identify_current_response(driver,ts,values,experiment.prior.gain)
    gate("driver_response",response["success"] and response["rmse_a"]<.005)
    clip("timing","Measure driver response","driver_response",4.,"Measured current and command log identify response separately from mechanics.")
    clips[-1]["play_end_s"]=.12

    event("scale","started")
    unloaded,_=experiment.collect("unloaded")
    loaded,_=experiment.collect("known_inertia",{"inertia":experiment.plant["inertia"]+.04})
    anchored=identify_gain(unloaded,loaded,.04,experiment.prior)
    gate("known_load_scale",anchored["identifiable"] and .5<anchored["gain"]<5.)
    experiment.prior=replace(experiment.prior,gain=anchored["gain"])
    clip("scale","Anchor current-to-torque scale","known_inertia",4.,"Paired recordings: unloaded / known added output inertia. No force sensor.")

    event("identification","started")
    observed,_=experiment.collect("mechanical_training")
    identified=identify(observed,experiment.prior)
    gate("mechanical_identifiability",identified["identifiable"] and identified.get("optimizer_success",False))
    clip("identify","Identify mechanical parameters","mechanical_training",6.,"Periodic multisine, analytic Fourier derivatives, SVD and bounded robust fit.")

    event("validation","started")
    validation,_=experiment.collect("heldout_validation",excitation={"frequencies":[.5,1.,2.],
        "amplitudes":[.25,.18,.1],"phases":[-.4,.3,1.4]})
    heldout=predict_residual(validation,identified["parameters"],experiment.prior)
    gate("independent_validation",heldout["torque_rmse"]<.01)
    clip("validate","Validate on an independent trajectory","heldout_validation",5.,"Parameters are frozen. Validation uses a separate complete experiment.")

    event("control_selection","started")
    original=dict(inertia=.045,damping=.06,spring=1.5,coulomb=.025)
    candidates=[];chosen={};controls={}
    for label,model in (("original",original),("identified",identified["parameters"])):
        feasible=[]
        for p in (.5,.75,1.,1.25,1.5):
            for d in (.5,.75,1.,1.25,1.5):
                ts,values,args=tracking_commands(model,12*p,1.2*d)
                oo,_,_=simulate(experiment.plant,ts,values,6_000_000_000)
                metrics=score_control(oo,args,experiment.prior.gain)
                row={"model":label,"kp":12*p,"kd":1.2*d,**metrics}
                row["feasible"]=bool(metrics["peak_velocity_rad_s"]<4. and
                    metrics["peak_estimated_torque_nm"]<=experiment.plant["torque_limit"] and metrics["saturation_fraction"]==0.)
                candidates.append(row)
                if row["feasible"]:feasible.append(row)
        gate(f"{label}_pd_candidates",bool(feasible))
        chosen[label]=min(feasible,key=lambda r:r["rmse_rad"])
        for gains in ("baseline","selected"):
            kp,kd=(12.,1.2) if gains=="baseline" else (chosen[label]["kp"],chosen[label]["kd"])
            ts,values,args=tracking_commands(model,kp,kd,validation=True)
            name=f"{label}_{gains}"
            oo,_=experiment.collect(name,duration=6.,commands=(ts,values))
            metrics=score_control(oo,args,experiment.prior.gain)
            controls[name]={"model":label,"gains":gains,"kp":kp,"kd":kd,**metrics}
    improvement=1-controls["identified_baseline"]["rmse_rad"]/controls["original_baseline"]["rmse_rad"]
    gate("fixed_pd_feedforward",improvement>=.2,rmse_reduction=improvement)
    clip("feedforward","Before / after: same PD","identified_baseline",7.,"Same plant, reference and limits. Only the feedforward model changes.","original_baseline")
    clips[-1]["play_start_s"]=1.
    clip("pd","Add PD selection","identified_selected",7.,"PD is selected on a different trajectory; all four comparisons remain in the report.","original_selected")
    clips[-1]["play_start_s"]=1.
    clip("summary","Calibration results & handoff","identified_selected",4.,"Synthetic demonstration. The viewer reads telemetry; it never controls a motor.","original_selected")
    clips[-1]["play_start_s"]=1.

    # Candidate model contains only observational inference and declared priors.
    model={**identified,"training_runs":["mechanical_training"],"validation_runs":["heldout_validation"],
        "gain_calibration_runs":["unloaded","known_inertia"],"robot_model_id":adapter.robot_model_id,
        "control_period_ns":1_000_000,"status":"validated_synthetic_candidate_not_hardware_authorized"}
    write_json(folder/"identified_model.json",model)
    write_json(folder/"storyboard.json",{"schema_version":1,"clips":clips,
        "note":"Each clip replays an independent recorded experiment; accelerated replay is explicitly labeled."})
    event("complete","passed")
    report={"schema_version":1,"synthetic_only":True,"passed":all(checks.values()),"checks":checks,
        "direction":direction,"driver_response":response,"drive_scale":anchored,"identified":identified,
        "heldout":heldout,"controls":controls,"pd_candidates":candidates,"fixed_pd_improvement":improvement,
        "inference_inputs":"observations and command logs only; evaluation_truth.npz is never read",
        "viewer_policy":"geometry only: no dynamics, no integration, no actuation, no device writes",
        "wall_s":perf_counter()-start,"runtime":experiment.runtime}
    write_json(folder/"demo_report.json",report)
    write_tables(folder,report)
    # Hash raw evidence once; render export will verify that none of it changed.
    evidence={str(p.relative_to(folder)):hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted((folder/"records").rglob("*.npz")) if p.name!="evaluation_truth.npz"}
    write_json(folder/"evidence_sha256.json",evidence)
    (folder/"README.md").write_text("# Calibration workflow demonstration\n\n"
        "C++ Mock motion; Python observation-only inference; MuJoCo geometry-only viewer.\n\n"
        "- [Numerical report](demo_report.json)\n- [Validated candidate](identified_model.json)\n"
        "- [Workflow events](workflow_events.jsonl)\n- [Joint mapping](mapping.json)\n"
        "- [Control metrics](control_metrics.csv)\n- [Viewer guide](../../docs/viewer_demo.md)\n\n"
        "Physical motor direction remains unconfirmed. Generated raw encoder ticks are synthetic.\n",encoding="utf-8")
    return report
