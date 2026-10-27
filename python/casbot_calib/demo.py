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
from .presentation_text import STORIES
from .workflow_config import workflow_config
from .workflow import DriveSession, repeated_batch_summary
from .identification import current_response_residual, identify, identify_gain, identify_current_response, predict_residual
from .telemetry import TelemetryAdapter


def mock_encoder_channels(observations):
    """Fixed synthetic producer convention, independent of the viewer mapping.

    A wrong viewer sign/zero must visibly change the decoded state, rather than
    cancel against a producer configured by the same erroneous mapping.
    """
    return {"raw_position":8192.-observations["q"]*16384./(2*np.pi),
            "raw_velocity":-observations["dq"]*16384./(2*np.pi),
            "raw_current":observations["current"].copy()}


class DemoRecorder(Experiment):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.session=DriveSession(self.plant)

    def collect(self,name,plant=None,*,condition="same test article",**kwargs):
        merged={**self.plant,**(plant or {})}
        self.session.verify(merged)
        mechanical=set(plant or {})-{"seed","position_noise","current_noise", "velocity_noise",
            "position_quantum","current_quantum","drop_probability","repeat_probability"}
        if mechanical and condition=="same test article":
            raise ValueError(f"Declare changed mechanical conditions explicitly: {sorted(mechanical)}")
        observations,truth=super().collect(name,plant,**kwargs)
        self.session.record(name,merged,kwargs.get("period_ns",1_000_000),condition)
        write_json(self.folder/name/"session.json",self.session.records[-1])
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
        units={"inertia":"kg m^2","damping":"Nm s/rad","spring":"Nm/rad","coulomb":"Nm","gravity_sin":"Nm","gravity_cos":"Nm"}
        for name,value in report["identified"]["parameters"].items():writer.writerow([name,value,units[name]])
        writer.writerow(["gain",report["drive_scale"]["gain"],"Nm/A output side"])


def score_control(observations,args,gain):
    """Same scoring path for a future real recording; evaluation truth is absent."""
    t=observations["sample_ns"]*1e-9
    if np.any(t<0):raise ValueError("Controller comparison needs aligned acquisition time")
    reference=multisine(t,**args)[0]
    if len(t)<100 or np.any(np.diff(t)<=0):
        raise ValueError("Tracking comparison needs monotone acquisition samples")
    valid=(t>=1)&((observations["status"]&2)==0)
    valid &= np.isfinite(observations["q"]) & np.isfinite(observations["dq"]) & np.isfinite(observations["current"])
    if np.count_nonzero(valid)<100:
        raise ValueError("Insufficient valid tracking samples after startup")
    return dict(rmse_rad=float(np.sqrt(np.mean((observations["q"][valid]-reference[valid])**2))),
        peak_velocity_rad_s=float(np.max(np.abs(observations["dq"]))),
        peak_estimated_torque_nm=float(np.max(np.abs(observations["current"]*gain))),
        saturation_fraction=float(np.mean((observations["status"]&1)!=0)))


def gravity_validation(experiment,gate,torque_limit=.01):
    """A separate fixture condition, not a hidden whole-arm parameter fit.

    Other joints are fixed. The signed sin/cos coefficients absorb the effective
    link/tool first moment at that posture. Inter-joint inertial coupling is not
    observable in this experiment and no full-arm feedforward is authorized.
    """
    condition="fixture spring removed; other joints fixed; effective link/tool first moment"
    plant=dict(spring=0.,gravity_sin=.35,gravity_cos=-.12)
    prior=replace(experiment.prior,include_spring=False,include_gravity=True)
    def commands(phases):
        ts=np.arange(0,24_000_000_000,1_000_000,dtype=np.int64)
        q,v,_=multisine(ts*1e-9,frequencies=(.5,1.,2.),amplitudes=(.65,.22,.08),phases=phases)
        # PD creates safe wide position coverage; no true gravity in the request.
        return ts,np.column_stack((np.ones(len(ts)),q,v,np.zeros(len(ts)),
                                  np.full(len(ts),16.),np.full(len(ts),1.6)))
    training,_=experiment.collect("gravity_training",plant,condition=condition,
        commands=commands((.2,1.1,-.6)))
    fit=identify(training,prior)
    gate("gravity_identifiability",fit["identifiable"] and fit.get("optimizer_success",False))
    holdout,_=experiment.collect("gravity_heldout",plant,condition=condition,
        commands=commands((-.4,.3,1.4)))
    frozen=predict_residual(holdout,fit["parameters"],prior)
    omitted=identify(training,replace(prior,include_gravity=False))
    missing=predict_residual(holdout,omitted["parameters"],replace(prior,include_gravity=False))
    gate("gravity_heldout",frozen["torque_rmse"]<torque_limit and
         frozen["torque_rmse"]<.2*missing["torque_rmse"])
    result=dict(identified=fit,heldout=frozen,omitted_gravity_heldout=missing,
        applicability="single joint; other joints and mounting posture fixed; no moving-joint inertial coupling",
        closed_loop_bias="PD excitation: possible errors-in-variables bias remains; robust fitting is not an IV estimator",
        authorization="synthetic candidate only; do not reuse spring-bench parameters or authorize whole-arm control")
    write_json(experiment.folder.parent/"gravity_model.json",result)
    return result


def run_demo(folder,*,config=None,progress=print):
    config=workflow_config(config)
    limits=config["acceptance"]
    added_inertia=config["added_inertia"]
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    write_json(folder/"workflow_config.json",config)
    start=perf_counter()
    adapter=TelemetryAdapter.load(files("casbot_calib").joinpath("assets/bench_mapping.json"))
    write_json(folder/"mapping.json",{"robot_model_id":adapter.robot_model_id,
        "joints":[m.__dict__ for m in adapter.mappings],
        "raw_units_note":"synthetic floating encoder ticks derived from mock observations, not ENCOS data"})
    experiment=DemoRecorder(folder/"records",{"sources":["casbot_calib C++ Mock; synthetic output-side joint"],
        "plant":config["plant"],"identification":config["identification"]})
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
    def clip(key,run,seconds,before=None):
        title,note=STORIES[key]
        clips.append(dict(key=key,title=title,run=run,seconds=seconds,note=note,before=before))

    event("prepare","started")
    gate("configuration",adapter.mappings[0].model_joint=="observed_joint")
    event("direction","started")
    ts=np.arange(0,3_000_000_000,1_000_000,dtype=np.int64);t=ts*1e-9
    q=.12*.5*(1-np.cos(2*np.pi*np.clip((t-.3)/2.,0,1)))
    v=np.where((t>.3)&(t<2.3),.12*np.pi/2.*np.sin(np.pi*(t-.3)),0.)
    commands=np.column_stack((np.ones(len(ts)),q,v,np.zeros(len(ts)),np.full(len(ts),12.),np.full(len(ts),1.2)))
    jog,_=experiment.collect("direction_jog",duration=3.,commands=(ts,commands))
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
    clip("direction","direction_jog",4.)

    event("timing","started")
    ts=np.arange(0,2_000_000_000,1_000_000,dtype=np.int64);values=np.zeros((len(ts),6))
    values[:,3]=.4*np.where((ts//20_000_000)%2==0,1.,-1.)
    driver,_=experiment.collect("driver_response",duration=2.,commands=(ts,values))
    preliminary_response=identify_current_response(driver,ts,values,experiment.prior.gain,fit_gain=True)
    response=preliminary_response
    gate("driver_response",response["success"] and response["rmse_a"]<limits["current_rmse_a"])
    clip("timing","driver_response",4.)
    clips[-1]["play_end_s"]=.12

    event("scale","started")
    unloaded,_=experiment.collect("unloaded")
    loaded,_=experiment.collect("known_inertia",{"inertia":experiment.plant["inertia"]+added_inertia},
        condition=f"balanced known flywheel: delta J={added_inertia} kg m2, unchanged gravity and friction")
    anchored=identify_gain(unloaded,loaded,added_inertia,experiment.prior)
    gate("known_load_scale",anchored["identifiable"] and .5<anchored["gain"]<5.)
    experiment.prior=replace(experiment.prior,gain=anchored["gain"])
    clip("scale","known_inertia",4.)

    event("timing_recheck","started")
    response=identify_current_response(driver,ts,values,experiment.prior.gain)
    # Independent pulse pattern: reuse frozen delay/tau, never fit the holdout.
    validation_values=values.copy()
    validation_values[:,3]=.32*np.where((ts//29_000_000)%3==0,1.,-.6)
    driver_validation,_=experiment.collect("driver_heldout",duration=2.,commands=(ts,validation_values))
    response_validation=current_response_residual(driver_validation,ts,validation_values,
        experiment.prior.gain,response["current_tau_s"],response["command_delay_s"])
    gate("anchored_driver_recheck",response["success"] and response["rmse_a"]<limits["current_rmse_a"]
         and response_validation["rmse_a"]<limits["current_rmse_a"])

    event("identification","started")
    observed,_=experiment.collect("mechanical_training")
    identified=identify(observed,experiment.prior)
    gate("mechanical_identifiability",identified["identifiable"] and identified.get("optimizer_success",False))
    clip("identify","mechanical_training",6.)

    event("validation","started")
    validation,_=experiment.collect("heldout_validation",excitation={"frequencies":[.5,1.,2.],
        "amplitudes":[.25,.18,.1],"phases":[-.4,.3,1.4]})
    heldout=predict_residual(validation,identified["parameters"],experiment.prior)
    gate("independent_validation",heldout["torque_rmse"]<limits["torque_rmse_nm"])
    clip("validate","heldout_validation",5.)

    event("repeatability","started")
    noise_profile=config["repeat_noise"]
    repetitions=[]
    for seed in range(10):
        oo,_=experiment.collect(f"repeat_{seed:02}",{**noise_profile,"seed":seed},
            excitation={"frequencies":[.5,1.,2.],"amplitudes":[.25,.18,.1],
                        "phases":[-.4+.11*seed,.3-.07*seed,1.4+.05*seed]})
        fitted=identify(oo,experiment.prior)
        gate(f"repeat_{seed:02}_rank",fitted["identifiable"] and fitted.get("optimizer_success",False))
        repetitions.append(dict(seed=seed,parameters=fitted["parameters"],
            frozen_torque_rmse=predict_residual(oo,identified["parameters"],experiment.prior)["torque_rmse"]))
    repeatability=repeated_batch_summary(repetitions,identified["parameters"],
        relative_limit=limits["repeat_relative_limit"],residual_limit=limits["torque_rmse_nm"])
    gate("batch_repeatability",repeatability["passed_count"]>=limits["minimum_repeat_passes"])
    frequency_validation=[]
    for period in (1_000_000,2_000_000,4_000_000):
        oo,_=experiment.collect(f"period_{period}",period_ns=period,excitation={
            "frequencies":[.5,1.,2.],"amplitudes":[.25,.18,.1],"phases":[-.4,.3,1.4]})
        metrics=predict_residual(oo,identified["parameters"],experiment.prior)
        frequency_validation.append(dict(control_period_ns=period,**metrics))
    gate("frozen_model_across_control_periods",all(r["torque_rmse"]<limits["torque_rmse_nm"] for r in frequency_validation))

    event("gravity","started")
    gravity=gravity_validation(experiment,gate,limits["torque_rmse_nm"])

    event("control_selection","started")
    original=dict(inertia=.045,damping=.06,spring=1.5,coulomb=.025)
    candidates=[];chosen={};controls={}
    for label,model in (("original",original),("identified",identified["parameters"])):
        feasible=[]
        for p in (.5,.75,1.,1.25,1.5):
            for d in (.5,.75,1.,1.25,1.5):
                ts,values,args=tracking_commands(model,12*p,1.2*d)
                experiment.session.verify(experiment.plant)
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
    gate("heldout_control_constraints",all(
        row["peak_velocity_rad_s"]<4. and row["saturation_fraction"]==0. and
        row["peak_estimated_torque_nm"]<=experiment.plant["torque_limit"] for row in controls.values()))
    improvement=1-controls["identified_baseline"]["rmse_rad"]/controls["original_baseline"]["rmse_rad"]
    gate("fixed_pd_feedforward",improvement>=.2,rmse_reduction=improvement)
    clip("feedforward","identified_baseline",7.,"original_baseline")
    clips[-1]["play_start_s"]=1.
    clip("pd","identified_selected",7.,"original_selected")
    clips[-1]["play_start_s"]=1.
    clip("summary","identified_selected",4.,"original_selected")
    clips[-1]["play_start_s"]=1.

    # Candidate model contains only observational inference and declared priors.
    model={**identified,"absolute_scale":"known_added_output_inertia","training_runs":["mechanical_training"],"validation_runs":["heldout_validation"],
        "gain_calibration_runs":["unloaded","known_inertia"],"robot_model_id":adapter.robot_model_id,
        "drive_session_sha256":experiment.session.fingerprint,
        "validity":{"joint":"single output-side joint","fixture":"spring bench, no gravity",
            "tested_control_period_ns":[1000000,2000000,4000000],
            "tested_excitation_hz":[.5,1.,1.5,2.,2.5],
            "coupling":"other joints fixed; varying-arm posture requires a different model",
            "noise_profile_source":"declared synthetic; real stationary recordings still required"},
        "uncertainty":repeatability["empirical_parameter_quantiles"],
        "control_period_ns":1_000_000,"status":"validated_synthetic_candidate_not_hardware_authorized"}
    write_json(folder/"identified_model.json",model)
    write_json(folder/"storyboard.json",{"schema_version":1,"clips":clips,
        "note":"Each clip replays an independent recorded experiment; accelerated replay is explicitly labeled."})
    event("complete","passed")
    report={"schema_version":2,"workflow_config":config,"synthetic_only":True,"passed":all(checks.values()),"checks":checks,
        "direction":direction,"driver_response":response,
        "preliminary_driver_response":preliminary_response,"driver_heldout":response_validation,
        "repeatability":repeatability,"noise_profile":noise_profile,"gravity":gravity,
        "frequency_validation":frequency_validation,
        "session":{"drive_sha256":experiment.session.fingerprint,"drive_profile":experiment.session.profile,
                   "records":experiment.session.records},"drive_scale":anchored,"identified":identified,
        "heldout":heldout,"controls":controls,"pd_candidates":candidates,"fixed_pd_improvement":improvement,
        "inference_inputs":"observations and command logs only; evaluation_truth.npz is never read",
        "viewer_policy":"geometry only: no dynamics, no integration, no actuation, no device writes",
        "wall_s":perf_counter()-start,"runtime":experiment.runtime}
    write_json(folder/"demo_report.json",report)
    write_tables(folder,report)
    # Hash raw evidence once; render export will verify that none of it changed.
    evidence_files=[p for p in sorted((folder/"records").rglob("*"))
                    if p.is_file() and p.name!="evaluation_truth.npz"]
    evidence_files += [folder/name for name in ("workflow_config.json","mapping.json",
        "identified_model.json","gravity_model.json","demo_report.json","storyboard.json")]
    evidence={str(p.relative_to(folder)):hashlib.sha256(p.read_bytes()).hexdigest() for p in evidence_files}
    write_json(folder/"evidence_sha256.json",evidence)
    (folder/"README.md").write_text("# Calibration workflow demonstration\n\n"
        "C++ Mock motion; Python observation-only inference; MuJoCo geometry-only viewer.\n\n"
        "- [Numerical report](demo_report.json)\n- [Validated candidate](identified_model.json)\n"
        "- [Workflow events](workflow_events.jsonl)\n- [Joint mapping](mapping.json)\n"
        "- [Control metrics](control_metrics.csv)\n- [Viewer guide](../../docs/viewer_demo.md)\n\n"
        "Physical motor direction remains unconfirmed. Generated raw encoder ticks are synthetic.\n",encoding="utf-8")
    return report
