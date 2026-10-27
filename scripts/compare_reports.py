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
    r1_paths=[root/"R1/report.json" for root in (left,right)]
    if any(path.exists() for path in r1_paths):
        if not all(path.exists() for path in r1_paths):raise ValueError("R1 report missing on one platform")
        r1_a,r1_b=[json.loads(path.read_text(encoding="utf-8")) for path in r1_paths]
        if not r1_a["passed"] or not r1_b["passed"]:raise ValueError("R1 did not pass on both platforms")
        for key in ("minimum_mass_eigenvalue","maximum_mass_condition","maximum_off_diagonal_mass","maximum_velocity_bias_nm",
                    "maximum_regressor_torque_error_nm","maximum_tcp_jacobian_error","maximum_workspace_component_error_nm",
                    "probe_regressor_rank","rigid_parameter_count"):
            values.append((f"R1.{key}",r1_a["metrics"][key],r1_b["metrics"][key],1e-8))
    dual_paths=[root/"D14/workflow.json" for root in (left,right)]
    if any(path.exists() for path in dual_paths):
        if not all(path.exists() for path in dual_paths):raise ValueError("Dual-arm report missing on one platform")
        workflows=[json.loads(path.read_text(encoding="utf-8")) for path in dual_paths]
        reports=[json.loads((root/"D14/identification/report.json").read_text(encoding="utf-8")) for root in (left,right)]
        if not all(r['passed'] for r in workflows+reports):raise ValueError('Dual-arm workflow failed')
        for i,(a,b) in enumerate(zip(reports[0]['stages']['dynamic']['parameters'],reports[1]['stages']['dynamic']['parameters'],strict=True)):
            values.append((f'D14.parameter{i}',a,b,1e-5))
        values.append(('D14.rigid_rank',reports[0]['full_rigid_regressor']['rank'],reports[1]['full_rigid_regressor']['rank'],0.))
        for name,value in reports[0]['validation'].items():
            values.append((f'D14.{name}',value['maximum_normalized_rmse'],reports[1]['validation'][name]['maximum_normalized_rmse'],1e-5))
        for key in ('aggregate_nominal_rmse_rad','aggregate_identified_rmse_rad'):
            values.append((f'D14.{key}',workflows[0]['control'][key],workflows[1]['control'][key],1e-6))
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
