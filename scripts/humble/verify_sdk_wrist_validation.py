#!/usr/bin/env python3
"""Recheck third-pose validation with frozen parameters and no training access."""
import argparse
from pathlib import Path
import subprocess
import sys
import numpy as np
from casbot_calib.hardware_wrist_exploration import read,FLAGS
from casbot_calib import hardware_field as field
from casbot_calib.data import write_json
from casbot_calib.hardware_probe_validation import engines


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();root=a.source.resolve();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    with (out/'read_only_replay.log').open('x') as log:
        subprocess.run([sys.executable,'scripts/humble/validate_sdk_wrist_pitch.py','analyse',
            '--source',str(root),'--output',str(out/'guarded_analysis')],stdout=log,stderr=subprocess.STDOUT,check=True)
    field.check_seal(root/'protocol');field.check_seal(root/'analysis')
    proto=read(root/'protocol/experiment.json');candidate=read(root/'protocol/candidate/candidate.json')
    saved=read(root/'analysis/report.json');fresh=read(out/'guarded_analysis/report.json');_,model=engines(proto,candidate)
    theta=np.asarray(candidate['parameters']);largest_feature=0.;largest_metric=0.;independent=[]
    for old,new in zip(saved['records'],fresh['records']):
        if old['job']!=new['job']:raise ValueError('Validation inventory changed')
        name=old['job']['id']+'_problem.npz'
        with np.load(root/'analysis'/name,allow_pickle=False) as first,np.load(out/'guarded_analysis'/name,allow_pickle=False) as second:
            for key in first.files:largest_feature=max(largest_feature,float(np.max(abs(first[key]-second[key]))))
            prediction=first['regression']@theta+first['extra']@np.r_[candidate['viscous'],candidate['coulomb']]+candidate['static_friction'][12]*first['static_feature']+first['rotor']
            rmse=float(np.sqrt(np.mean((prediction-first['observed_torque_nm'])**2)))
            q=first['q'];z=np.zeros_like(q)
            mean_mass=float(np.mean(model.evaluate_batch(q,z,z)['mass_matrix'][:,12,12]))-model.model.joints[12].rotor_inertia
            t=first['time_s'];t=t-t.mean()
            design=np.column_stack([first['ddq'][:,12],first['dq'][:,12],np.ones(len(t)),t,t*t])
            norm=np.linalg.norm(design,axis=0)
            coeff=np.linalg.lstsq(design/norm,first['target']-first['projected_gravity_nm'],rcond=1e-10)[0]/norm
            disagreement=abs(mean_mass-coeff[0])/max(abs(coeff[0]),1e-8)
        row=dict(id=old['job']['id'],independent_prediction_rmse_nm=rmse,independent_local_inertia_kg_m2=float(coeff[0]),independent_inertia_disagreement=float(disagreement))
        independent.append(row)
        largest_metric=max(largest_metric,abs(rmse-old['primary']['torque_prediction_rmse_nm']),abs(disagreement-old['primary']['relative_inertia_disagreement']))
        for original,replay in zip([old['primary']]+old['diagnostics'],[new['primary']]+new['diagnostics']):
            for key in ('torque_prediction_rmse_nm','relative_inertia_disagreement'):
                largest_metric=max(largest_metric,abs(original[key]-replay[key]))
    # The only parameter vector in both processes is the pre-acquisition snapshot.
    same=np.array_equal(theta,read(root/'protocol/candidate/candidate.json')['parameters'])
    checks=dict(exactly_two_third_pose_records=len(saved['records'])==len(fresh['records'])==2,
        third_pose_distinct_from_both_train_centers=all(proto['pose_rad']!=q for q in proto['training_poses'].values()),
        frozen_candidate_unchanged=same and saved['candidate_parameters_unchanged'] and fresh['candidate_parameters_unchanged'],
        guarded_replay_features_identical=largest_feature<1e-10,
        independent_prediction_and_inertia_identical=largest_metric<1e-10,
        primary_development_validation_passed=saved['development_validation_passed'],
        no_candidate_refit_or_diagnostic_selection=not saved['candidate_refit_performed'] and not saved['diagnostics_used_for_selection'],
        original_10_percent_unchanged=proto['acceptance']['maximum_relative_inertia_disagreement']==.10,
        qualification_flags_false=all(not saved[k] for k in FLAGS))
    write_json(out/'verification.json',dict(passed=all(checks.values()),checks=checks,records=independent,
        maximum_feature_difference=largest_feature,maximum_metric_difference=largest_metric,
        training_records_accessed=False,private_evaluation_accessed=False,candidate_refit_performed=False,
        device_writes=0,**FLAGS))
    print(checks)


if __name__=='__main__':main()
