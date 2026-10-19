"""Observation-only inference. No dependency on the simulator or evaluation truth."""
from __future__ import annotations
from dataclasses import dataclass
from time import perf_counter
import numpy as np
from scipy.linalg import lstsq
from scipy.optimize import least_squares, minimize_scalar

NAMES = ("inertia", "damping", "spring", "coulomb")

@dataclass(frozen=True)
class Prior:
    gain: float = 2.
    period_s: float = 2.
    harmonics: int = 40
    discard_s: float = 12.
    spring_zero: float = 0.
    friction_velocity: float = .03
    include_spring: bool = True
    include_friction: bool = True
    receive_to_sample_s: float = 0.

def prepare(observations, prior: Prior, use_timestamps=True):
    """Global periodic fit uses only valid measurements, never differences over gaps."""
    required = ("q", "dq", "current", "sequence", "status", "sample_ns", "current_sample_ns", "receive_ns")
    if any(k not in observations for k in required):
        raise ValueError("Missing observation columns")
    n = len(observations["q"])
    if any(len(observations[k]) != n for k in required):
        raise ValueError("Unequal observation lengths")
    for name in ("sequence","sample_ns","current_sample_ns","receive_ns"):
        if observations[name].dtype!=np.dtype("int64"):
            raise ValueError(f"{name} must be int64 (nanoseconds or acquisition sequence)")
    known = bool(use_timestamps and np.all(observations["sample_ns"] >= 0))
    tq = (observations["sample_ns"] if known else observations["receive_ns"])*1e-9
    if not known:
        tq = tq-prior.receive_to_sample_s
    ti = observations["current_sample_ns"]*1e-9 if known else tq.copy()
    mask = ((observations["status"] & 3) == 0)
    mask &= np.isfinite(observations["q"]) & np.isfinite(observations["dq"]) & np.isfinite(observations["current"])
    mask &= (tq >= prior.discard_s) & (ti >= prior.discard_s)
    idx = np.flatnonzero(mask)
    idx = idx[np.argsort(tq[idx], kind="stable")]
    # Deduplicate retransmitted data using acquisition sequence, then require monotone time.
    _, first = np.unique(observations["sequence"][idx], return_index=True)
    idx = idx[np.sort(first)]
    if len(idx) < max(100, 4*(2*prior.harmonics+1)):
        raise ValueError("Too few valid periodic samples")
    if np.any(np.diff(tq[idx]) <= 0) or np.any(np.diff(ti[idx]) <= 0):
        raise ValueError("Non-monotone acquisition timestamps")
    if np.any(np.diff(observations["sequence"][idx])<=0):
        raise ValueError("Acquisition sequence conflicts with selected timestamps")
    if tq[idx[-1]]-tq[idx[0]] < prior.period_s:
        raise ValueError("At least one complete period is required")
    diagnostics = {"input_samples": n, "used_samples": len(idx), "timestamps_used": known,
                   "sequence_gaps": int(np.sum(np.diff(observations["sequence"][idx])>1)),
                   "derivative_method": "periodic Fourier analytic derivative; no finite differences across gaps"}
    return tq[idx], ti[idx], observations["q"][idx], observations["current"][idx], diagnostics

class Fourier:
    def __init__(self, t, y, period_s, harmonics):
        self.w = 2*np.pi/period_s*np.arange(1, harmonics+1)
        phase = np.asarray(t)[:, None]*self.w
        design = np.column_stack((np.ones(len(t)), np.cos(phase), np.sin(phase)))
        self.coef, _, rank, _ = lstsq(design, y, lapack_driver="gelsy")
        if rank != design.shape[1]:
            raise ValueError("Insufficient Fourier phase coverage")
    def evaluate(self, t, derivative=0):
        phase = np.asarray(t)[:, None]*self.w
        h = len(self.w)
        cosine, sine = self.coef[1:1+h], self.coef[1+h:]
        if derivative == 0:
            return self.coef[0]+np.cos(phase)@cosine+np.sin(phase)@sine
        if derivative == 1:
            return -np.sin(phase)@(self.w*cosine)+np.cos(phase)@(self.w*sine)
        return -np.cos(phase)@(self.w**2*cosine)-np.sin(phase)@(self.w**2*sine)

def features(q, dq, ddq, prior):
    columns = [ddq, dq]; names = ["inertia", "damping"]
    if prior.include_spring:
        columns.append(q-prior.spring_zero); names.append("spring")
    if prior.include_friction:
        friction=np.tanh(dq/prior.friction_velocity)
        # The nonlinear regressor must pass through the same low-pass projection
        # as current and acceleration. Otherwise high friction harmonics that
        # were removed from the observations bias a robust time-domain fit.
        spectrum=np.fft.rfft(friction)
        spectrum[prior.harmonics+1:]=0
        friction=np.fft.irfft(spectrum,n=len(friction))
        columns.append(friction); names.append("coulomb")
    return np.column_stack(columns), names

def scaled_fit(x, y, names, robust=True):
    scale = np.linalg.norm(x, axis=0)
    if np.any(scale < 1e-14):
        return {"identifiable": False, "reason": "zero_regressor", "parameters": {}, "rank": 0}
    xs = x/scale
    _, singular, _ = np.linalg.svd(xs, full_matrices=False)
    rank = int(np.sum(singular > singular[0]*1e-4))
    result = {"rank": rank, "columns": len(names), "singular_values": singular.tolist(),
              "condition": float(singular[0]/max(singular[-1], 1e-30)), "identifiable": rank==len(names)}
    if rank < len(names):
        return {**result, "reason": "rank_deficient", "parameters": {}}
    linear = np.linalg.lstsq(xs, y, rcond=1e-4)[0]
    params = linear/scale
    if robust:
        # Explicit bounds enforce positive inertia and dissipative coefficients.
        lower = np.array([1e-8 if n=="inertia" else 0. for n in names])*scale
        opt = least_squares(lambda p: xs@p-y, np.maximum(linear, lower+1e-10),
                            bounds=(lower, np.full(len(names), np.inf)), loss="soft_l1",
                            f_scale=.01, ftol=1e-10, xtol=1e-10, gtol=1e-10)
        params = opt.x/scale
        result["optimizer_success"] = bool(opt.success)
    result.update(parameters=dict(zip(names, params, strict=True)),
                  linear_parameters=dict(zip(names, linear/scale, strict=True)),
                  torque_rmse=float(np.sqrt(np.mean((x@params-y)**2))))
    return result

def identify(observations, prior=Prior(), *, use_timestamps=True, offset_bounds_s=None):
    """Estimate mechanical parameters; positive offset means current lags position.

    Only observations and declared priors are accepted. Absolute gain must be
    independently anchored before calling this routine.
    """
    start = perf_counter()
    tq, ti, q, current, diagnostics = prepare(observations, prior, use_timestamps)
    qfit = Fourier(tq, q, prior.period_s, prior.harmonics)
    ifit = Fourier(ti, current, prior.period_s, prior.harmonics)
    # Uniform phase grid avoids weighting repeated cycles unevenly after packet loss.
    grid = np.linspace(0, prior.period_s, max(2000, prior.harmonics*40), endpoint=False)
    qq, dq, ddq = (qfit.evaluate(grid, d) for d in (0, 1, 2))
    x, names = features(qq, dq, ddq, prior)
    offset = 0.
    if offset_bounds_s is not None:
        if diagnostics["timestamps_used"]:
            raise ValueError("Offset search requires unknown current time; don't correct twice")
        lo, hi = offset_bounds_s
        if not lo < hi:
            raise ValueError("Empty offset search bounds")
        # Fit current once. Search scalar phase shift with an inner linear fit.
        # A coarse grid precedes bounded refinement to avoid relying on unimodality.
        def loss(shift):
            target = prior.gain*ifit.evaluate(grid+shift)
            p = np.linalg.lstsq(x, target, rcond=1e-8)[0]
            return float(np.mean((x@p-target)**2))
        candidates = np.linspace(lo, hi, 81)
        costs = [loss(v) for v in candidates]
        j = int(np.argmin(costs))
        bounds = (candidates[max(0, j-1)], candidates[min(80, j+1)])
        opt = minimize_scalar(loss, bounds=bounds, method="bounded", options={"xatol": 1e-9})
        offset = float(opt.x)
        diagnostics["offset_at_boundary"] = bool(min(offset-lo, hi-offset)<1e-5)
    y = prior.gain*ifit.evaluate(grid+offset)
    result = scaled_fit(x, y, names)
    result.update(offset_s=offset, diagnostics=diagnostics, prior=prior.__dict__,
                  fit_band_hz=[1/prior.period_s, prior.harmonics/prior.period_s],
                  solve_wall_s=perf_counter()-start, model_version="output_joint_v1",
                  absolute_scale="declared_gain_prior", scope="periodic training trajectory")
    return result

def predict_residual(observations, parameters, prior=Prior()):
    tq, ti, q, current, _ = prepare(observations, prior)
    qfit = Fourier(tq, q, prior.period_s, prior.harmonics)
    ifit = Fourier(ti, current, prior.period_s, prior.harmonics)
    grid = np.linspace(0, prior.period_s, 2000, endpoint=False)
    x, names = features(*(qfit.evaluate(grid,d) for d in (0,1,2)), prior)
    residual = x@np.array([parameters[n] for n in names])-prior.gain*ifit.evaluate(grid)
    return {"torque_rmse": float(np.sqrt(np.mean(residual**2))),
            "peak_torque_residual": float(np.max(np.abs(residual)))}

def identify_gain(unloaded, loaded, added_inertia, prior=Prior()):
    if added_inertia is None or added_inertia <= 0:
        return {"identifiable": False, "reason": "missing_known_inertia_scale_anchor", "parameters": {}}
    # Joint fit: i = (J/g)*ddq+(b/g)*dq+...+(1/g)*DeltaJ*ddq.
    # Both experiments share the same base mechanical parameters and gain.
    blocks=[];targets=[]
    for observations,delta in ((unloaded,0.),(loaded,added_inertia)):
        tq,ti,q,current,_=prepare(observations,prior)
        qfit=Fourier(tq,q,prior.period_s,prior.harmonics)
        ifit=Fourier(ti,current,prior.period_s,prior.harmonics)
        grid=np.linspace(0,prior.period_s,max(2000,prior.harmonics*40),endpoint=False)
        qq,dq,ddq=(qfit.evaluate(grid,d) for d in (0,1,2))
        x,names=features(qq,dq,ddq,prior)
        blocks.append(np.column_stack((x,delta*ddq)))
        targets.append(ifit.evaluate(grid))
    fit=scaled_fit(np.vstack(blocks),np.concatenate(targets),names+["inverse_gain"])
    if not fit["identifiable"]:
        return {**fit,"reason":"rank_deficient_load_experiment"}
    inverse_gain=fit["parameters"].pop("inverse_gain")
    if inverse_gain<=1e-8:
        return {"identifiable":False,"reason":"inconsistent_load_scale","parameters":{}}
    gain=1/inverse_gain
    return {**fit,"gain":gain,
            "parameters":{n:v*gain for n,v in fit["parameters"].items()},
            "added_inertia":added_inertia,"scale_anchor":"known_added_output_inertia",
            "assumption":"only known added inertia changes between the two runs"}

def identify_current_response(observations, send_ns, commands, gain, delay_bounds_s=(0., .005), tau_bounds_s=(.0001, .01)):
    """Separate open-loop actuator experiment, using measured current and ZOH requests.

    A one-step exact first-order recurrence is fit on acquisition-time samples.
    Commands must be pure torque and unsaturated. Mechanical model is not fitted.
    """
    if np.any(commands[:, 0] != 0):
        raise ValueError("Current response fit requires torque-only excitation")
    keep = (observations["status"] == 0) & (observations["current_sample_ns"] >= 0)
    t = observations["current_sample_ns"][keep]*1e-9
    measured = observations["current"][keep]
    order = np.argsort(t); t, measured = t[order], measured[order]
    sends = np.asarray(send_ns)*1e-9
    targets = commands[:, 3]/gain
    def evaluate(tau, delay):
        arrival = sends+delay
        state = 0.; at = 0.; command_index = 0; held = 0.; output=[]
        for sample_t in t:
            while command_index<len(arrival) and arrival[command_index]<=sample_t:
                change = arrival[command_index]
                state = held+(state-held)*np.exp(-(change-at)/tau)
                at=change; held=targets[command_index]; command_index+=1
            output.append(held+(state-held)*np.exp(-(sample_t-at)/tau))
        return np.asarray(output)
    lo,hi=delay_bounds_s
    # Joint robust nonlinear fit; delay bounds must be within one excitation feature.
    fit=least_squares(lambda p:evaluate(p[0],p[1])-measured,
                      [(tau_bounds_s[0]+tau_bounds_s[1])/2, (lo+hi)/2],
                      bounds=([tau_bounds_s[0],lo],[tau_bounds_s[1],hi]),
                      diff_step=1e-3,loss="soft_l1",f_scale=.01,xtol=1e-10,gtol=1e-10,ftol=1e-10)
    return {"current_tau_s": float(fit.x[0]), "command_delay_s": float(fit.x[1]),
            "rmse_a": float(np.sqrt(np.mean(fit.fun**2))), "success": bool(fit.success),
            "model": "ZOH plus first-order current response; separate from mechanical parameters"}
