from __future__ import annotations
import csv
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from .data import write_json

def trace_plot(path, observations, truth=None, reference=None):
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True, layout="constrained")
    known = np.all(observations["sample_ns"] >= 0)
    t = observations["sample_ns" if known else "receive_ns"]*1e-9
    axes[0].plot(t, observations["q"], lw=.7, label="observed output position")
    if reference is not None:
        axes[0].plot(reference[0], reference[1], "--", lw=.8, label="reference")
    axes[1].plot(t, observations["current"], lw=.7, label="measured current")
    if truth is not None:
        tt = truth["time_ns"]*1e-9
        axes[2].plot(tt, truth["contact_force"], lw=.7, label="contact force (evaluation truth)")
        axes[2].plot(tt, truth["string_x"]*1000, lw=.7, label="string displacement (mm, truth)")
    for ax in axes:
        ax.legend(loc="upper right", fontsize=8); ax.grid(alpha=.2)
    axes[0].set_ylabel("position (rad)");axes[1].set_ylabel("current (A)")
    axes[2].set_ylabel("contact (N / mm)");axes[2].set_xlabel("time (s)")
    fig.savefig(path, dpi=140);plt.close(fig)

def parameter_plot(path, truth, estimate):
    names = list(estimate)
    fig, ax = plt.subplots(figsize=(8, 4), layout="constrained")
    x = np.arange(len(names))
    ax.bar(x-.18, [truth[n] for n in names], .36, label="synthetic truth")
    ax.bar(x+.18, [estimate[n] for n in names], .36, label="identified")
    ax.set_xticks(x, names);ax.set_ylabel("SI value (different units)");ax.legend()
    fig.savefig(path, dpi=140);plt.close(fig)

def comparison_plot(path, labels, series, ylabel):
    fig,ax=plt.subplots(figsize=(10,4),layout="constrained")
    x=np.arange(len(labels));width=.8/len(series)
    for i,(name,values) in enumerate(series.items()):
        ax.bar(x+(i-(len(series)-1)/2)*width,values,width,label=name)
    ax.set_xticks(x,labels,rotation=25,ha="right");ax.set_ylabel(ylabel)
    ax.legend(fontsize=8);ax.grid(axis="y",alpha=.2)
    fig.savefig(path,dpi=140);plt.close(fig)

def publish(folder, stage, metrics, checks, rows=None):
    folder = Path(folder)
    report = {"stage": stage, "synthetic_only": True, "checks": checks,
              "passed": all(checks.values()), "metrics": metrics}
    write_json(folder/"report.json", report)
    rows = rows or [{"check": k, "passed": v} for k, v in checks.items()]
    keys = list(dict.fromkeys(k for row in rows for k in row))
    with (folder/"metrics.csv").open("w", newline="", encoding="utf-8") as f:
        writer=csv.DictWriter(f, fieldnames=keys);writer.writeheader();writer.writerows(rows)
    lines = [f"# Experiment {stage}", "", "Synthetic experiment; no hardware performance claim.", ""]
    lines += [f"- {'PASS' if v else 'FAIL'}: {k}" for k, v in checks.items()]
    lines += ["", "See report.json for numerical estimates, assumptions and diagnostics.", ""]
    (folder/"report.md").write_text("\n".join(lines), encoding="utf-8")
    return report
