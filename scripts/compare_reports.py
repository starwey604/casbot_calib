"""Compare two platform reports, excluding wall-clock times. Standard library only."""
import argparse
import json
import math
from pathlib import Path

def metrics(root, stage):
    report=json.loads((root/stage/"report.json").read_text(encoding="utf-8"))
    if not report["passed"]:
        raise ValueError(f"{root}/{stage} did not pass acceptance")
    return report["metrics"]

def compare(left, right):
    a,b=(metrics(left,"A"),metrics(right,"A"))
    values=[]
    for group in ("ideal","friction","free_joint"):
        for name,value in a[group]["parameters"].items():
            values.append((f"A.{group}.{name}",value,b[group]["parameters"][name],1e-5))
    for stage,key in (("B","known_payload"),("D","effective_model")):
        a,b=metrics(left,stage)[key],metrics(right,stage)[key]
        for name,value in a["parameters"].items():
            values.append((f"{stage}.{name}",value,b["parameters"][name],1e-5))
    a,b=metrics(left,"C"),metrics(right,"C")
    for key in ("current_tau_s","command_delay_s"):
        values.append((f"C.{key}",a["driver_response"][key],b["driver_response"][key],2e-5))
    for i,(aa,bb) in enumerate(zip(a["rates"],b["rates"],strict=True)):
        values.append((f"C.case{i}.offset_s",aa["estimated"]["offset_s"],bb["estimated"]["offset_s"],2e-5))
    for aa,bb in zip(metrics(left,"E")["four_way"],metrics(right,"E")["four_way"],strict=True):
        values.append((f"E.{aa['model']}.{aa['gains']}",aa["rmse_rad"],bb["rmse_rad"],1e-5))
    for aa,bb in zip(metrics(left,"F")["tasks"],metrics(right,"F")["tasks"],strict=True):
        for key in ("rmse_rad","peak_contact_force_n_truth","peak_string_displacement_m_truth"):
            values.append((f"F.{aa['case']}.{key}",aa[key],bb[key],1e-5))
    failed=[]
    for name,a,b,atol in values:
        if not math.isclose(a,b,rel_tol=1e-3,abs_tol=atol):
            failed.append(f"{name}: {a} vs {b}")
    if failed:
        raise ValueError("\n".join(failed))
    print(f"PASS: {len(values)} numerical metrics agree within explicit tolerances")

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("left",type=Path);parser.add_argument("right",type=Path)
    args=parser.parse_args()
    compare(args.left,args.right)
