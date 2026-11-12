#!/usr/bin/env python3
"""Blind raw-feedback refit, independent operators, then selected-axis Mock scoring."""
import argparse
from pathlib import Path
import subprocess
import sys
import numpy as np
import yaml
from casbot_calib.hardware_wrist_exploration import read,FLAGS
from casbot_calib.data import write_json
from casbot_calib.rigid_identification import model_from_parameters
from casbot_calib.model_runtime import model_for


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();root=a.source.resolve();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    # The fresh process installs the evaluation-data guard before reading captures.
    with (out/'blind_refit.log').open('x') as log:
        subprocess.run([sys.executable,'scripts/humble/supplement_sdk_wrist_pitch.py','analyse',
            '--source',str(root),'--output',str(out/'blind_analysis')],stdout=log,stderr=subprocess.STDOUT,check=True)
    protocol=read(root/'protocol/experiment.json');original=read(root/'analysis/report.json');fresh=read(out/'blind_analysis/report.json')
    observations=read(root/'analysis/observations.json');largest_parameter=0.;largest_feature=0.;largest_operator=0.
    for old,new in zip(original['variants'],fresh['variants']):
        if old['label']!=new['label']:raise ValueError('Variant inventory changed')
        largest_parameter=max(largest_parameter,float(np.max(abs(np.asarray(old['fit']['parameters'])-new['fit']['parameters']))))
    for row in observations:
        name=row['job']['id']+'_problem.npz'
        with np.load(root/'analysis'/name,allow_pickle=False) as old,np.load(out/'blind_analysis'/name,allow_pickle=False) as new:
            for key in old.files:largest_feature=max(largest_feature,float(np.max(abs(old[key]-new[key]))))
    # Independent native batch interface; caller does not reuse operator_difference.
    with np.load(root/'protocol/operator_probes.npz',allow_pickle=False) as d:probe={k:d[k] for k in d.files}
    nominal,_=model_for(protocol['model_profile']);model=nominal.model;model.gravity_world=np.asarray(protocol['model_profile']['gravity_world']);nominal=type(nominal)(model)
    prior=protocol['fixture_friction_prior'];operators=[];engines={};local_probe=[]
    def candidate(theta):return model_from_parameters(nominal,theta,prior['viscous'],prior['coulomb'],prior['static_friction'],stribeck_velocity=.005)
    for row in original['variants']:
        e=candidate(row['fit']['parameters']);engines[row['label']]=e
        terms=e.evaluate_batch(probe['q'],probe['dq'],probe['ddq']);operators.append(terms)
    baseline=operators[0]
    for row,terms in zip(original['variants'],operators):
        diff=terms['mass_matrix']-baseline['mass_matrix'];denom=np.maximum(np.diagonal(baseline['mass_matrix'],axis1=1,axis2=2),protocol['operator_policy']['mass_diagonal_floor_kg_m2'])
        diag=float(np.max(abs(np.diagonal(diff,axis1=1,axis2=2))/denom))
        frob=float(np.max(np.linalg.norm(diff,axis=(1,2))/np.linalg.norm(baseline['mass_matrix'],axis=(1,2))))
        tau=np.sqrt(np.mean((terms['total']-baseline['total'])**2,axis=0))/np.maximum(np.sqrt(np.mean(baseline['total']**2,axis=0)),.2)
        independent=[diag,frob,float(np.max(tau))]
        recorded=[row['operators'][k] for k in ('maximum_mass_diagonal_relative','maximum_mass_relative_frobenius','maximum_inverse_axis_normalized_rms')]
        largest_operator=max(largest_operator,float(np.max(abs(np.asarray(independent)-recorded))))
    full=engines['full'];theta=np.asarray(original['variants'][0]['fit']['parameters'])
    for obs in observations:
        name=obs['job']['id']
        with np.load(root/'analysis'/(name+'_problem.npz'),allow_pickle=False) as d:
            error=d['regression']@theta-d['rigid_target'];q=d['q']
        rigid_mass=float(np.mean(full.evaluate_batch(q,np.zeros_like(q),np.zeros_like(q))['mass_matrix'][:,12,12]))-full.model.joints[12].rotor_inertia
        scalar=obs['inertia_diagnostic']['inertia_kg_m2'];disagreement=abs(rigid_mass-scalar)/max(abs(scalar),1e-8)
        rmse=float(np.sqrt(np.mean(error**2)))
        local_probe.append(dict(id=name,independent_public_probe_inertia_kg_m2=scalar,candidate_mean_rigid_joint_inertia_kg_m2=rigid_mass,
            relative_disagreement=disagreement,rmse_nm=rmse,original_probe_numerics_passed=bool(scalar>0 and disagreement<=.1 and rmse<=.002)))
    # Post-fit scoring from the declared synthetic fixture, not acquisition truth
    # delivered to the observation builder or used for model selection.
    mock=yaml.safe_load((root/'protocol/mock.yaml').read_text());model=nominal.model;joints=model.joints
    for j,joint in enumerate(joints):
        body=joint.body;body.inertia_com=body.inertia_com*mock['plant']['inertia_scales'][j];joint.body=body
    model.joints=joints;synthetic=type(nominal)(model)
    truth=synthetic.evaluate_batch(probe['q'],probe['dq'],probe['ddq'])['mass_matrix'][:,12,12]
    fit=baseline['mass_matrix'][:,12,12]
    base=nominal.evaluate_batch(probe['q'],probe['dq'],probe['ddq'])['mass_matrix'][:,12,12]
    error=abs(fit-truth)/truth
    write_json(out/'selected_axis_synthetic_evaluation.json',dict(joint=12,scope='right_wrist_pitch only, original128 public operator poses',
        maximum_relative_error=float(error.max()),maximum_nominal_prior_relative_error=float(np.max(abs(base-truth)/truth)),
        worst_operator_point=int(np.argmax(error)),candidate_inertia_kg_m2=fit.tolist(),synthetic_inertia_kg_m2=truth.tolist(),
        truth_used_for_fitting=False,truth_used_for_selection=False,physical_measurement=False))
    logical={r['label']:r['selected_records'] for r in original['variants']}
    checks=dict(eight_records_six_new_two_reused=len(observations)==8 and original['new_records']==6 and original['reused_records']==2,
        evaluation_denied_fresh_fit_identical=largest_parameter<1e-8 and largest_feature<1e-10,
        original128_operator_recalculation_identical=largest_operator<1e-10,
        fixed_scales_and_complete_metric=all(r['fit']['coordinate_scale_source']=='explicit_frozen' and r['fit']['regularization_metric_source']=='explicit_complete_quadratic' for r in original['variants']),
        duplicate_actual_session_subset_reused=logical['delete_batch_b01']==logical['delete_session_b01'] and original['unique_fits']==7,
        original_operator_deletions_passed=original['all_operator_deletions_passed'],
        original_local_probe_numerics_passed=all(r['original_probe_numerics_passed'] for r in local_probe),
        selected_axis_synthetic_error_within10_percent=float(error.max())<=.1,
        qualification_flags_remain_false=all(not original[k] for k in FLAGS))
    write_json(out/'verification.json',dict(passed=all(checks.values()),checks=checks,
        maximum_parameter_difference=largest_parameter,maximum_feature_difference=largest_feature,
        maximum_independent_operator_difference=largest_operator,local_probe_checks=local_probe,
        full_p0_r1_r2_completed=False,device_writes=0,**FLAGS))
    print(checks)


if __name__=='__main__':main()
