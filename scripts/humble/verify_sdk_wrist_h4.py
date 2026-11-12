#!/usr/bin/env python3
"""Verify completed local wrist exploration, then evaluate declared Mock truth.

The fresh fit runs in the existing evaluation-denied analysis entry point.
Truth comparison is read-only, subsequent and cannot change any fitted model.
"""
import argparse
from pathlib import Path
import subprocess
import sys
import numpy as np
import yaml
from casbot_calib.hardware_wrist_exploration import read, FLAGS, WRISTS, mass_diagonal, mass_difference
from casbot_calib.rigid_identification import model_from_parameters
from casbot_calib.model_runtime import model_for
from casbot_calib.data import write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--analysis',type=Path,help='Current-policy analysis to verify; defaults to source/analysis')
    a=parser.parse_args();root=a.source.resolve();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    analysis=(a.analysis if a.analysis is not None else root/'analysis').resolve()
    report=read(analysis/'report.json');protocol=read(root/'protocol/experiment.json')
    command=[sys.executable,'scripts/humble/explore_sdk_wrist_h4.py','analyse',
             '--source',str(root),'--output',str(out/'fresh_blind_analysis')]
    with (out/'blind_analysis.log').open('x') as stream:
        subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,check=True)
    fresh=read(out/'fresh_blind_analysis/report.json');largest=0.;all_fit=True
    for old,new in zip(report['cases'],fresh['cases']):
        for key,fit in old['fits'].items():
            other=new['fits'][key];all_fit &= fit['completed'] and other['completed']
            if fit['completed'] and other['completed']:
                largest=max(largest,float(np.max(abs(np.asarray(fit['fit']['parameters'])-other['fit']['parameters']))))
    assert len(fresh['cases'])==len(report['cases'])
    checks=dict(same_analysis_contract=report['schema_version']==fresh['schema_version'],all_preregistered_variants_and_fits_completed=bool(all_fit and len(report['cases'])==18),
                fresh_evaluation_denied_refit_identical=bool(all_fit and largest<1e-8),
                all_formal_flags_remain_false=all(not report[k] for k in FLAGS),
                captures_complete=read(root/'collection.json')['records']==12)
    # Private synthetic model settings are first consumed here, after blind fit.
    with np.load(root/'protocol/operator_probes.npz',allow_pickle=False) as data:q=data['q']
    nominal,_=model_for(protocol['h4']);nominal_mass=mass_diagonal(nominal,q)
    truth=[]
    for case in protocol['cases']:
        config=yaml.safe_load((root/'protocol'/(case['id']+'.yaml')).read_text())
        engine,_=model_for(config);model=engine.model;joints=model.joints
        for j,joint in enumerate(joints):
            body=joint.body;body.inertia_com=body.inertia_com*config['plant']['inertia_scales'][j]
            joint.body=body
        model.joints=joints;plant=type(engine)(model);truth_mass=mass_diagonal(plant,q)
        for row in report['cases']:
            if row['case']!=case['id'] or not row['fits']['full']['completed']:continue
            engine,_=model_for(protocol['h4'])
            candidate=model_from_parameters(engine,row['fits']['full']['fit']['parameters'])
            error=mass_difference(truth_mass,mass_diagonal(candidate,q),1e-6)
            truth.append(dict(case=case['id'],variation=row['variation']['id'],candidate_vs_synthetic_truth=error,
                              nominal_vs_synthetic_truth=mass_difference(truth_mass,nominal_mass,1e-6),
                              only_excited_wrist_axes_assessed=True))
    write_json(out/'synthetic_truth_comparison.json',dict(records=truth,truth_used_for_fitting=False,
               truth_used_for_selection=False,physical_truth_measured=False,**FLAGS))
    signal_budget(root,protocol,out)
    audit=read(analysis/'public_capture_audit.json')
    checks['actual_received_paths_passed']=all(r['received_collision_passed'] for r in audit)
    verification=dict(checks=checks,passed=all(checks.values()),maximum_refit_parameter_difference=largest,
                      fits_compared=sum(len(r['fits']) for r in report['cases']),
                      original_threshold_unchanged=.10,operator_probe_count=128,**FLAGS)
    write_json(out/'verification.json',verification)
    plot(report,truth,audit,out)
    print(verification)


def signal_budget(root,protocol,out):
    """Post-fit model-based signal estimate on public reconstructed trajectories."""
    import csv
    from casbot_calib import hardware_field as field
    from casbot_calib.hardware_periodic import periodic_problem
    rows=[];c=protocol['h4']
    for case in protocol['cases']:
        config=yaml.safe_load((root/'protocol'/(case['id']+'.yaml')).read_text())
        nominal,_=model_for(c);model=nominal.model;joints=model.joints
        for j,joint in enumerate(joints):
            body=joint.body;body.inertia_com=body.inertia_com*config['plant']['inertia_scales'][j];joint.body=body
        model.joints=joints;synthetic=type(nominal)(model)
        for job in protocol['jobs']:
            folder=root/case['id']/job['id'];plan,captured=field.resolve_field_capture(folder/'plan',folder/'capture');meta=read(plan/'plan.json')
            obs=field.read_feedback(captured/'execution/feedback.csv')
            with (captured/'execution/commands.csv').open() as stream:first=next(csv.DictReader(stream))
            origin=int(first['planned_ns'])-int(first['planned_index'])*meta['command_period_ns']
            elapsed=(obs['receive_ns']-np.int64(origin))*1e-9
            problem=periodic_problem(obs,elapsed,read(plan/'motion.json'),nominal,c,
                                     meta['config'],protocol['friction_fixture_prior'])
            terms=synthetic.evaluate_batch(problem['q'],problem['dq'],problem['ddq'])
            rows.append(dict(case=case['id'],job=job['id'],
                acceleration_torque_rms_nm=np.sqrt(np.mean(terms['inertia'][:,WRISTS]**2,axis=0)).tolist(),
                velocity_coupling_torque_rms_nm=np.sqrt(np.mean(terms['velocity'][:,WRISTS]**2,axis=0)).tolist()))
    write_json(out/'synthetic_signal_budget.json',dict(rows=rows,
        basis='synthetic model evaluated on public callback-time reconstructed q/dq/ddq; not hidden acquisition trajectory',
        used_for_fitting=False,used_for_selection=False,physical_measurement=False))


def plot(report,truth,audit,out):
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from casbot_calib.presentation import _font
    font_path=_font(12).path
    font_manager.fontManager.addfont(font_path)
    plt.rcParams['font.family']=font_manager.FontProperties(fname=font_path).get_name()
    plt.rcParams['axes.unicode_minus']=False
    fig,ax=plt.subplots(2,2,figsize=(14,9),layout='constrained')
    labels=['左 yaw','左 pitch','左 roll','右 yaw','右 pitch','右 roll']
    for case,color in zip(dict.fromkeys(r['case'] for r in report['cases']), ['tab:blue','tab:orange']):
        base=next(r for r in report['cases'] if r['case']==case and r['variation']['id']=='baseline')
        diff=np.max([f['operator_difference_from_full']['axis_maximum_relative'] for k,f in base['fits'].items() if k!='full'],axis=0)
        ax[0,0].plot(labels,np.asarray(diff)[WRISTS]*100,'o-',label=case.replace('cache_','缓存 '),color=color)
        error=next(t for t in truth if t['case']==case and t['variation']=='baseline')['candidate_vs_synthetic_truth']
        ax[0,1].plot(labels,np.asarray(error['axis_maximum_relative'])[WRISTS]*100,'o-',label=case.replace('cache_','缓存 '),color=color)
        variants=[r for r in report['cases'] if r['case']==case]
        shifts=[r for r in variants if 'shift_s' in r['variation'] or r['variation']['id']=='baseline']
        shifts.sort(key=lambda r:r['variation'].get('shift_s',0))
        ax[1,0].plot([r['variation'].get('shift_s',0)*1000 for r in shifts],
                     [r['perturbation_difference_from_baseline']['wrist_maximum_relative']*100 for r in shifts],'o-',label=case.replace('cache_','缓存 '),color=color)
        subset=[r for r in variants if r['variation']['id'] in ['baseline','effort_lowpass_5ms','effort_lowpass_20ms','band_8','band_12']]
        ax[1,1].plot([r['variation']['id'].replace('effort_lowpass_','LP ') for r in subset],
                     [r['perturbation_difference_from_baseline']['wrist_maximum_relative']*100 for r in subset],'o-',label=case.replace('cache_','缓存 '),color=color)
    for panel,title in zip(ax.flat,['删构型／删批次最差差异：原 128 点','对独立合成惯量的误差',
                                    '施加 effort 相位偏移：不是实测 SDK 延迟','施加 effort 低通／改变共同拟合频带']):
        panel.set_title(title);panel.set_ylabel('腕部对角惯量差异／误差（%）');panel.grid(alpha=.25);panel.legend()
        panel.axhline(10,color='red',linestyle='--',linewidth=1)
    ax[1,0].set_xlabel('施加 effort 相位提前量（ms）')
    ax[1,1].tick_params(axis='x',rotation=15)
    fig.suptitle('SDK 回调时间腕部探索｜仅本机 Mock，不是实机验收',fontsize=16)
    fig.savefig(out/'wrist_sensitivity.png',dpi=150);plt.close(fig)


if __name__=='__main__':main()
