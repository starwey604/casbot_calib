"""Session invariants and observation-only acceptance, independent of any viewer."""
from __future__ import annotations
from hashlib import sha256
import json
import numpy as np

# These change the actuator/measurement transfer function, not the test article.
DRIVE_DEFAULTS=dict(gain=2., current_tau=.001, current_filter_tau=0.,
    torque_limit=20., current_limit=10., sample_period_ns=1_000_000,
    command_delay_ns=0, feedback_delay_ns=0, current_lag_ns=0, jitter_ns=0,
    expose_sample_time=True)


class DriveSession:
    def __init__(self, plant):
        self.profile={k:plant.get(k,v) for k,v in DRIVE_DEFAULTS.items()}
        self.fingerprint=sha256(json.dumps(self.profile,sort_keys=True).encode()).hexdigest()
        self.records=[]

    def verify(self, plant):
        changed=[k for k,v in self.profile.items() if plant.get(k,DRIVE_DEFAULTS[k])!=v]
        if changed:
            raise ValueError(f"Drive configuration changed within calibration session: {changed}")

    def record(self, name, plant, period_ns, condition):
        self.verify(plant)
        self.records.append(dict(run=name,drive_sha256=self.fingerprint,
                                 control_period_ns=period_ns,condition=condition))


def repeated_batch_summary(rows, reference, relative_limit=.05, residual_limit=.01):
    """Empirical spread over whole runs, not an IID time-sample confidence interval.

    No simulator truth or true parameter is accepted here. This checks repeatability
    around a frozen candidate; synthetic accuracy is checked separately by A-F.
    """
    names=list(reference)
    for row in rows:
        row["max_relative_deviation"]=max(
            abs(row["parameters"][n]-reference[n])/max(abs(reference[n]),1e-8) for n in names)
        row["passed"]=bool(row["max_relative_deviation"]<=relative_limit
                           and row["frozen_torque_rmse"]<=residual_limit)
    return dict(rows=rows,passed_count=sum(r["passed"] for r in rows),total=len(rows),
        empirical_parameter_quantiles={n:dict(zip(("p05","p50","p95"),
            np.quantile([r["parameters"][n] for r in rows],[.05,.5,.95]).tolist(),strict=True)) for n in names},
        interpretation="whole-run empirical repeatability under declared synthetic noise; not hardware confidence intervals",
        relative_limit=relative_limit,residual_limit_nm=residual_limit)
