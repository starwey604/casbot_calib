"""Reproducible A-F protocols; evaluation uses truth only after inference."""
from __future__ import annotations
from dataclasses import replace
from pathlib import Path
from time import perf_counter
import numpy as np
from scipy.ndimage import binary_closing
from .device import default_plant, simulate
from .excitation import torque_commands, tracking_commands, multisine, feedforward, guitar_reference
from .identification import Prior, identify, identify_gain, predict_residual, identify_current_response
from .data import save_run, write_json
from .report import publish, trace_plot, parameter_plot, comparison_plot

def relative_errors(estimated, truth):
    return {n: abs(v-truth[n])/abs(truth[n]) for n,v in estimated.items() if truth[n]!=0}

class Experiment:
    def __init__(self, folder, config):
        self.folder=Path(folder); self.folder.mkdir(parents=True, exist_ok=False)
        self.config=config
        self.plant={**default_plant(), **config.get("plant", {})}
        self.duration=float(config.get("duration_s", 24.))
        self.prior=Prior(**config.get("identification", {}))
        self.runtime=[]

    def collect(self, name, plant=None, *, period_ns=1_000_000, excitation=None, duration=None, commands=None):
        plant={**self.plant, **(plant or {})}
        duration=self.duration if duration is None else duration
        if commands is None:
            times,values=torque_commands(duration,period_ns,**(excitation or {}))
        else:
            times,values=commands
        obs,truth,runtime=simulate(plant,times,values,round(duration*1e9))
        self.runtime.append({"run":name,**runtime})
        save_run(self.folder/name,obs,truth,{"plant_synthetic_truth":plant,
            "known_conditions":{"control_period_ns":period_ns,"excitation":excitation or "custom"},
            "timing":{"sample_time": "device_virtual_clock" if plant.get("expose_sample_time",True) else "unknown",
                      "receive_time": "host_virtual_clock"},
            "source":self.config.get("sources", []),"runtime":runtime},times,values)
        return obs,truth

    def finish(self, stage, metrics, checks, rows=None):
        return publish(self.folder,stage,{**metrics,"runtime":self.runtime},checks,rows)

    def stage_a(self):
        ideal,hi=self.collect("ideal", {"coulomb":0.})
        fit_ideal=identify(ideal,replace(self.prior,include_friction=False))
        obs,truth=self.collect("friction")
        fit=identify(obs,self.prior)
        errors=relative_errors(fit["parameters"],self.plant)
        single,_=self.collect("single_frequency",{"coulomb":0.},duration=36.,
                            excitation={"frequencies":[.5],"amplitudes":[.3],"phases":[.2]})
        negative=identify(single,replace(self.prior,discard_s=24.,include_friction=False))
        validation,_=self.collect("validation",excitation={"frequencies":[.5,1.,2.],
                               "amplitudes":[.25,.18,.1],"phases":[-.4,.3,1.4]})
        prediction=predict_residual(validation,fit["parameters"],self.prior)
        noise_config=self.config.get("noise",dict(position_noise=1e-6,current_noise=.0002,
                                                  position_quantum=1e-6,current_quantum=1e-4))
        noisy=[]
        for seed in range(10):
            oo,_=self.collect(f"noise_{seed:02}",{**noise_config,"seed":seed})
            ff=identify(oo,self.prior); ee=relative_errors(ff["parameters"],self.plant)
            noisy.append({"seed":seed,"max_relative_error":max(ee.values()),"passed":max(ee.values())<=.05})
        free,_=self.collect("free_joint",{"spring":0.})
        free_fit=identify(free,replace(self.prior,include_spring=False))
        cold,_=self.collect("cold_friction",{"temperature":20.,"coulomb":.075})
        warm,_=self.collect("warm_friction",{"temperature":45.,"coulomb":.045})
        thermal={"cold":identify(cold,self.prior),"warm":identify(warm,self.prior)}
        trace_plot(self.folder/"trace.png",obs,truth)
        parameter_plot(self.folder/"parameters.png",self.plant,fit["parameters"])
        comparison_plot(self.folder/"singular_values.png",["s1","s2","s3","s4"],
                        {"scaled regressor":fit["singular_values"]},"singular value")
        write_json(self.folder/"identified_model.json",{**fit,"training_runs":["friction"],
                   "validation_runs":["validation"],"control_period_ns":1_000_000,
                   "validity":"synthetic output-side joint, declared Fourier band and excitation amplitudes"})
        ideal_err=relative_errors(fit_ideal["parameters"],self.plant)
        return self.finish("A",{"ideal":fit_ideal,"friction":fit,"errors":errors,
               "single_frequency":negative,"validation":prediction,"noise":noisy,
               "free_joint":free_fit,"thermal_profiles":thermal},
               {"ideal_parameter_error_le_1pct":max(ideal_err.values())<=.01,
                "friction_parameter_error_le_1pct":max(errors.values())<=.01,
                "single_frequency_rank_rejected":not negative["identifiable"],
                "noise_at_least_9_of_10":sum(r["passed"] for r in noisy)>=9,
                "heldout_torque_rmse_le_0p01Nm":prediction["torque_rmse"]<.01,
                "free_joint_error_le_1pct":max(relative_errors(free_fit["parameters"],self.plant).values())<=.01},noisy)

    def stage_b(self):
        added=float(self.config.get("added_inertia",.04))
        unloaded,truth=self.collect("unloaded")
        loaded,_=self.collect("known_inertia",{"inertia":self.plant["inertia"]+added})
        fit=identify_gain(unloaded,loaded,added,self.prior)
        negative=identify_gain(unloaded,loaded,None,self.prior)
        trace_plot(self.folder/"trace.png",unloaded,truth)
        return self.finish("B",{"known_payload":fit,"no_scale_anchor":negative},
                           {"gain_error_le_1pct":abs(fit["gain"]/self.plant["gain"]-1)<=.01,
                            "missing_anchor_rejected":not negative["identifiable"]})

    def stage_c(self):
        rows=[]; details=[]
        baseline=None
        # Adequately sampled first-order response for the pure-lag accuracy gate.
        # A separate fast-drive stress test below exposes sampling-phase aliasing.
        timing_tau=float(self.config.get("pure_lag_current_tau_s",.02))
        for period in (1_000_000,2_000_000,4_000_000):
            for follows in (False,True):
                for command_delay in (0,1_500_000):
                    tag=f"p{period}_sample{'follow' if follows else 'fixed'}_cmd{command_delay}"
                    oo,_=self.collect(tag,{"current_tau":timing_tau,"command_delay_ns":command_delay,"feedback_delay_ns":2_000_000,
                        "current_lag_ns":1_700_000,"sample_period_ns":period if follows else 1_000_000},period_ns=period)
                    known=identify(oo,self.prior)
                    ignored=identify(oo,self.prior,use_timestamps=False)
                    estimated=identify(oo,self.prior,use_timestamps=False,offset_bounds_s=(-.004,.004))
                    err=max(relative_errors(known["parameters"],self.plant).values())
                    estimate_err=max(relative_errors(estimated["parameters"],self.plant).values())
                    row={"control_period_ms":period/1e6,"sampling_follows_control":follows,
                         "command_delay_ms":command_delay/1e6,"estimated_offset_ms":estimated["offset_s"]*1000,
                         "offset_error_ms":abs(estimated["offset_s"]-.0017)*1000,
                         "timestamp_parameter_max_error":err,"estimated_parameter_max_error":estimate_err,
                         "unaligned_parameter_max_error":max(relative_errors(ignored["parameters"],self.plant).values())}
                    rows.append(row);details.append({"case":tag,"known":known,"unaligned":ignored,"estimated":estimated})
                    if baseline is None: baseline=known["parameters"]
        # Independent validation uses a new complete trajectory and the first fit,
        # never a per-rate refit to hide differences.
        cross=[]
        for period in (1_000_000,2_000_000,4_000_000):
            oo,_=self.collect(f"heldout_p{period}",period_ns=period,
                excitation={"frequencies":[.5,1.,2.],"amplitudes":[.25,.18,.1],"phases":[-.4,.3,1.4]})
            cross.append({"control_period_ms":period/1e6,**predict_residual(oo,baseline,self.prior)})
        aliasing=[]
        for period in (1_000_000,2_000_000,4_000_000):
            oo,_=self.collect(f"fast_drive_sampling_stress_p{period}",
                {"current_tau":.001,"sample_period_ns":period,"command_delay_ns":1_500_000,
                 "feedback_delay_ns":2_000_000,"current_lag_ns":1_700_000},period_ns=period)
            ff=identify(oo,self.prior,use_timestamps=False,offset_bounds_s=(-.004,.004))
            aliasing.append({"sample_period_ms":period/1e6,"estimated_offset_ms":1000*ff["offset_s"],
                "offset_error_ms":abs(ff["offset_s"]-.0017)*1000,
                "interpretation":"sparse synchronous samples alias fast ZOH/current response; apparent offset is biased"})
        oo,truth=self.collect("filtered_current",{"current_filter_tau":.003,"feedback_delay_ns":2_000_000,
                                                   "expose_sample_time":False})
        filtered=identify(oo,self.prior,use_timestamps=False,offset_bounds_s=(-.006,.006))
        # Driver response is an independent short broadband step protocol.
        duration=2.; times=np.arange(0,round(duration*1e9),1_000_000,dtype=np.int64)
        values=np.zeros((len(times),6)); values[:,3]=.4*np.where((times//20_000_000)%2==0,1.,-1.)
        driver,_=self.collect("driver_response",{"current_tau":.002,"command_delay_ns":1_300_000},
                              duration=duration,commands=(times,values))
        response=identify_current_response(driver,times,values,self.prior.gain)
        bad,_=self.collect("loss_jitter",{"drop_probability":.005,"repeat_probability":.005,"jitter_ns":100_000,
                                          "feedback_delay_ns":1_000_000})
        packet_fit=identify(bad,self.prior)
        trace_plot(self.folder/"trace.png",oo,truth)
        comparison_plot(self.folder/"offset_recovery.png",
            [f"{r['control_period_ms']:g}ms/{'follow' if r['sampling_follows_control'] else '1kHz'}/cmd{r['command_delay_ms']:g}" for r in rows],
            {"absolute offset error":[r["offset_error_ms"] for r in rows]},"offset error (ms)")
        return self.finish("C",{"rates":details,"fixed_model_heldout":cross,"filter_mismatch":filtered,
               "driver_response":response,"packet_impairment":packet_fit,
               "pure_lag_current_tau_s":timing_tau,"fast_drive_sampling_stress":aliasing,
               "filter_interpretation":"apparent phase offset is not a pure transport delay; residual remains"},
               {"offset_error_le_0p1ms":max(r["offset_error_ms"] for r in rows)<=.1,
                "timestamp_parameters_le_5pct":max(r["timestamp_parameter_max_error"] for r in rows)<=.05,
                "estimated_parameters_le_5pct":max(r["estimated_parameter_max_error"] for r in rows)<=.05,
                "fixed_model_cross_rate_rmse_le_0p01Nm":max(r["torque_rmse"] for r in cross)<.01,
                "driver_tau_error_le_5pct":abs(response["current_tau_s"]/.002-1)<=.05,
                "driver_command_delay_error_le_0p1ms":abs(response["command_delay_s"]-.0013)<=.0001,
                "packet_fit_le_5pct":max(relative_errors(packet_fit["parameters"],self.plant).values())<=.05},rows)

    def stage_d(self):
        plant={"double_inertia":True,**self.config.get("mismatch",dict(motor_inertia=.02,coupling_spring=90.,coupling_damping=.25))}
        low,truth=self.collect("low_band",plant)
        fit=identify(low,self.prior)
        validation,_=self.collect("low_band_validation",plant,excitation={"frequencies":[.5,1.,2.],
            "amplitudes":[.25,.18,.1],"phases":[-.4,.3,1.4]})
        high,_=self.collect("high_band_validation",plant,excitation={"frequencies":[5.,8.,11.],
            "amplitudes":[.2,.15,.1],"phases":[-.4,.3,1.4]})
        lo=predict_residual(validation,fit["parameters"],self.prior)
        hi=predict_residual(high,fit["parameters"],self.prior)
        trace_plot(self.folder/"trace.png",low,truth)
        parameter_plot(self.folder/"parameters.png",{**self.plant,"inertia":self.plant["inertia"]+plant["motor_inertia"]},fit["parameters"])
        comparison_plot(self.folder/"heldout_residual.png",["low frequency","high frequency"],
            {"frozen single-inertia model":[lo["torque_rmse"],hi["torque_rmse"]]},"torque residual RMSE (Nm)")
        return self.finish("D",{"effective_model":fit,"low_band_validation":lo,"high_band_validation":hi,
            "training_excitation_hz":[.5,1.5,2.5],"heldout_excitation_hz":[5,8,11],
            "validity":"only reported low-frequency effective fit; coupling parameters remain fixed, not estimated"},
            {"effective_model_identifiable":fit["identifiable"],
             "high_band_residual_exceeds_low_band":hi["torque_rmse"]>lo["torque_rmse"]})

    def evaluate_controller(self,name,model,kp,kd,validation=False,save=True):
        ts,values,args=tracking_commands(model,kp,kd,validation=validation,
            spring_zero=self.prior.spring_zero,friction_velocity=self.prior.friction_velocity)
        if save: obs,truth=self.collect(name,duration=6.,commands=(ts,values))
        else: obs,truth,_=simulate(self.plant,ts,values,6_000_000_000)
        q,v,_=multisine(truth["time_ns"]*1e-9,**args)
        mask=truth["time_ns"]>=1_000_000_000
        metrics={"rmse_rad":float(np.sqrt(np.mean((truth["q"][mask]-q[mask])**2))),
            "peak_velocity_rad_s":float(np.max(np.abs(truth["dq"]))),
            "peak_torque_nm":float(np.max(np.abs(truth["current"]*self.plant["gain"]))),
            "saturation_fraction":float(np.mean((obs["status"]&1)!=0)),"kp":kp,"kd":kd}
        metrics["feasible"]=bool(metrics["peak_velocity_rad_s"]<4 and
                                  metrics["peak_torque_nm"]<=self.plant["torque_limit"] and
                                  metrics["saturation_fraction"]==0)
        return metrics,obs,truth,(truth["time_ns"]*1e-9,q)

    def stage_e(self):
        obs,_=self.collect("calibration")
        fit=identify(obs,self.prior)
        original=self.config.get("original_model",dict(inertia=.045,damping=.06,spring=1.5,coulomb=.025))
        kp,kd=self.config.get("baseline_pd",[12.,1.2])
        factors=self.config.get("pd_factors",[.5,.75,1.,1.25,1.5])
        selected={};candidates=[]
        for label,model in (("original",original),("identified",fit["parameters"])):
            options=[]
            for p in factors:
                for d in factors:
                    metrics,*_=self.evaluate_controller("selection",model,kp*p,kd*d,save=False)
                    candidates.append({"model":label,**metrics})
                    if metrics["feasible"]: options.append(metrics)
            if not options: raise RuntimeError("No PD candidate satisfies constraints")
            selected[label]=min(options,key=lambda row:row["rmse_rad"])
        rows=[];traces={}
        for label,model in (("original",original),("identified",fit["parameters"])):
            for gains in ("baseline","selected"):
                p,d=(kp,kd) if gains=="baseline" else (selected[label]["kp"],selected[label]["kd"])
                name=f"{label}_{gains}"
                metrics,oo,tt,ref=self.evaluate_controller(name,model,p,d,validation=True)
                rows.append({"model":label,"gains":gains,**metrics})
                traces[name]=metrics
                trace_plot(self.folder/f"{name}.png",oo,tt,ref)
        fixed_gain_improvement=1-traces["identified_baseline"]["rmse_rad"]/traces["original_baseline"]["rmse_rad"]
        comparison_plot(self.folder/"four_way.png",[f"{r['model']} / {r['gains']}" for r in rows],
            {"held-out trajectory":[r["rmse_rad"] for r in rows]},"tracking RMSE (rad)")
        return self.finish("E",{"identification":fit,"four_way":rows,"selection_candidates":candidates,
                "fixed_pd_feedforward_rmse_reduction":fixed_gain_improvement,
                "selection_trajectory_hz":[.5,1.,2.],"validation_trajectory_hz":[.75,1.25,2.75],
                "constraints":{"velocity_rad_s":4.,"torque_nm":self.plant["torque_limit"],"saturation_fraction":0}},
                {"identified_feedforward_improvement_ge_20pct":fixed_gain_improvement>=.2,
                 "all_four_groups_feasible":all(r["feasible"] for r in rows)},rows)

    def stage_f(self):
        air,_=self.collect("air_calibration")
        fit=identify(air,self.prior)
        rows=[]
        geometry=self.config.get("trajectory",dict(lever=.2,speed=1.,stroke=.1,reversal=.05))
        contact=self.config.get("contact",dict(string_mass=.01,string_spring=400.,string_damping=.03,
            contact_spring=700.,contact_damping=.2,pick_offset=0.,attack_angle=0.))
        air_cache={}
        def mechanical_residual(samples):
            if np.any(np.diff(samples["sequence"])!=1):
                raise ValueError("Contact residual needs gap-free segments")
            time=samples["sample_ns"]*1e-9
            if np.any(time<0) or np.any(np.diff(time)<=0):
                raise ValueError("Contact residual requires known monotone sample timestamps")
            acceleration=np.gradient(samples["dq"],time)
            modeled=feedforward(samples["q"],samples["dq"],acceleration,fit["parameters"],
                                self.prior.spring_zero,self.prior.friction_velocity)
            return self.prior.gain*samples["current"]-modeled
        # Same frozen air model and gains in all contact comparisons.
        for name,depth,direction,duration in (("air_sweep",0.,1.,.12),("single_pluck",.002,1.,.12),
            ("reverse_pluck",.002,-1.,.12),("shallow",.001,1.,.12),("deep",.003,1.,.12),
            ("repeated",.002,1.,.9),("vibrating_repluck",.002,1.,.9)):
            ts=np.arange(0,round(duration*1e9),1_000_000,dtype=np.int64)
            q,v,a=guitar_reference(ts*1e-9,direction=direction,**geometry)
            values=np.column_stack((np.ones(len(ts)),q,v,feedforward(q,v,a,fit["parameters"],
                self.prior.spring_zero,self.prior.friction_velocity),
                                   np.full(len(ts),80.),np.full(len(ts),4.)))
            plant={**contact,"lever":geometry["lever"],"contact_enabled":depth>0,"depth":max(depth,.001),"integration_ns":10_000,
                   "initial_q":float(q[0]),"initial_v":float(v[0]),
                   "initial_string_x":.0005 if name=="vibrating_repluck" else 0.}
            oo,tt=self.collect(name,plant,duration=duration,commands=(ts,values))
            time=oo["sample_ns"]*1e-9
            raw_residual=mechanical_residual(oo)
            key=(direction,duration)
            if key not in air_cache:
                air_cache[key],_=self.collect(f"{name}_matched_air",{**plant,"contact_enabled":False,
                    "seed":int(self.plant.get("seed",0))+1009},
                                               duration=duration,commands=(ts,values))
            matched_air=air_cache[key]
            if not np.array_equal(matched_air["sample_ns"],oo["sample_ns"]):
                raise ValueError("Matched-air residual comparison needs identical acquisition grids")
            # Fast ZOH/PD/current ripple aliases into velocity differences. A
            # separately observed, identical free-space task is the baseline;
            # neither force truth nor simulated acceleration enters this proxy.
            baseline_residual=mechanical_residual(matched_air)
            residual=raw_residual-baseline_residual
            qr,_,_=guitar_reference(tt["time_ns"]*1e-9,direction=direction,**geometry)
            # Event inference uses current residual, separate from truth labels.
            inferred=binary_closing(np.abs(residual)>.015,structure=np.ones(3,dtype=bool))
            entries=int(np.sum(np.diff(tt["contact"].astype(int))==1))
            inferred_entries=int(np.sum(np.diff(inferred.astype(int))==1))
            truth_labels=tt["contact"]
            tp=int(np.sum(inferred&truth_labels))
            row={"case":name,"depth_m":depth,"direction":direction,
                "rmse_rad":float(np.sqrt(np.mean((tt["q"]-qr)**2))),
                "raw_current_residual_rms_a":float(np.sqrt(np.mean(raw_residual**2)))/self.prior.gain,
                "additional_current_residual_rms_a":float(np.sqrt(np.mean(residual**2)))/self.prior.gain,
                "inferred_contact_segments":inferred_entries,"truth_contact_entries":entries,
                "peak_contact_force_n_truth":float(np.max(np.abs(tt["contact_force"]))),
                "peak_string_displacement_m_truth":float(np.max(np.abs(tt["string_x"]))),
                "repeat_interval_s":geometry["stroke"]/geometry["speed"]+geometry["reversal"],
                "event_precision_truth_eval":tp/max(int(np.sum(inferred)),1),
                "event_recall_truth_eval":tp/max(int(np.sum(truth_labels)),1),
                "material_id":"synthetic_combined_pick_mount"}
            rows.append(row)
            np.savez_compressed(self.folder/name/"processed_residual.npz",time_s=time,
                raw_current_residual_a=raw_residual/self.prior.gain,air_current_residual_a=baseline_residual/self.prior.gain,
                additional_current_residual_a=residual/self.prior.gain,inferred_contact=inferred)
            trace_plot(self.folder/f"{name}.png",oo,tt,(tt["time_ns"]*1e-9,qr))
        # Convergence compares the same contact trajectory with half the step.
        half_plant={**plant,"integration_ns":5_000}
        _,half,_=simulate({**self.plant,**half_plant},ts,values,round(duration*1e9))
        convergence={"q_max_change_rad":float(np.max(np.abs(half["q"]-tt["q"]))),
            "q_relative_change":float(np.max(np.abs(half["q"]-tt["q"]))/max(np.max(np.abs(tt["q"])),1e-12)),
            "peak_force_relative_change":float(abs(np.max(np.abs(half["contact_force"]))/max(np.max(np.abs(tt["contact_force"])),1e-12)-1))}
        comparison_plot(self.folder/"contact_residuals.png",[r["case"] for r in rows],
            {"additional residual versus matched air":[r["additional_current_residual_rms_a"] for r in rows]},"current residual RMS (A)")
        return self.finish("F",{"frozen_air_model":fit,"tasks":rows,"step_convergence":convergence,
            "observability":"event proxy subtracts separately observed matched-air residual; raw residual includes sampling/PD ripple; force is evaluation truth only; no general force observer or contact parameter recovery claimed",
            "contact_law":"finite-width smooth potential plus dissipative relative-velocity term; equivalent synthetic catch/release",
            "single_joint_mapping":geometry,
            "legacy_string_comparison":{"mass_kg":.01,"spring_n_m":400.,"damping_ns_m":2.},
            "active_string_damping_ns_m":contact["string_damping"]},
            {"air_sweep_has_no_truth_contact":rows[0]["truth_contact_entries"]==0,
             "air_sweep_has_no_inferred_contact":rows[0]["inferred_contact_segments"]==0,
             "forward_reverse_and_depth_cases_contact":all(r["truth_contact_entries"]>0 for r in rows[1:]),
             "contact_cases_detected_without_force":all(r["inferred_contact_segments"]>0 for r in rows[1:]),
             "repeated_has_multiple_contacts":rows[-1]["truth_contact_entries"]>=2,
             "step_halving_change_le_1pct":max(convergence["q_relative_change"],convergence["peak_force_relative_change"])<.01},rows)

def run_stage(config, folder):
    stage=config["stage"].lower()
    if stage not in "abcdef" or len(stage)!=1: raise ValueError("Stage must be A-F")
    experiment=Experiment(folder,config)
    write_json(experiment.folder/"experiment_config.json",config)
    start=perf_counter()
    report=getattr(experiment,f"stage_{stage}")()
    write_json(experiment.folder/"timing.json",{"total_wall_s":perf_counter()-start})
    return report
