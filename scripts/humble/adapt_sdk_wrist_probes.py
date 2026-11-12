#!/usr/bin/env python3
"""Offline SDK adaptation of existing non-reversing wrist references; no ROS/plant."""
import argparse
from pathlib import Path
import json
import shutil
import numpy as np
import yaml
from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for,collision_for,context_for
from casbot_calib.hardware_probe import settings,make_probe,probe_problem,inertia_balance
from casbot_calib.hardware_wrist_exploration import FLAGS,WRISTS,make_motion
from casbot_calib.hardware_inertia import settings as h4_settings
from casbot_calib.staged_motion import gate_motion
from casbot_calib.staged_multiconfiguration import motion_coverage
from casbot_calib.collision import certify_polyline
from casbot_calib.rigid_identification import model_from_parameters


def readout_fixture(motion,engine,config,phase_s,offset_s=0.):
    """Declared ideal-reference readout, not a capture or simulated device run."""
    begin=motion.windows('probe_measure')[0][0];end=motion.windows('probe_measure')[-1][1]
    t=np.arange(begin-.02,end+.02,1/config['publication_hz'])
    cache=config['cache_period_ns']*1e-9
    cached=np.floor((t-phase_s)/cache)*cache+phase_s
    q,v,_,_=motion.at(cached)
    tq,tv,ta,_=motion.at(cached+offset_s)
    torque=engine.evaluate_batch(tq,tv,ta)['total']
    return dict(receive_ns=10**15+np.rint(t*1e9).astype(np.int64),q=q,dq=v,effort_raw=torque),t


def run(output, reference_checks=None):
    root=Path(output).resolve();root.mkdir(parents=True,exist_ok=False)
    config=settings();h4raw=yaml.safe_load(Path('configs/hardware_inertia.yaml').read_text())
    nominal,_=model_for(h4raw);names=[j.name for j in nominal.model.joints]
    h4=h4_settings('configs/hardware_inertia.yaml',names)
    # Identical declared friction coefficients to the previous local SDK fixture.
    engine=model_from_parameters(nominal,nominal.rigid_parameters(),[.01]*14,[.005]*14,[.01]*14,stribeck_velocity=.005)
    home=np.asarray(h4['home_rad']);poses={}
    for name,pitch,roll in [('c00',.2,.3),('c01',-.2,.15)]:
        pose=home.copy();pose[[5,12]]=pitch;pose[[6,13]]=roll;poses[name]=pose
    protocol=dict(schema_version='casbot_sdk_wrist_probe_offline_v1',config=config,
        poses={k:v.tolist() for k,v in poses.items()},joint_names=names,
        source_waveform='staged_motion.wrist_probe_motion, unchanged5Hz/.004rad/.18rad_s/2s',
        cache_phases_s=[0.,.0013,.0027,.0049],relative_effort_diagnostics_s=[-.005,0.,.005],
        observation_source='synthetic ideal-reference readout fixture; not a recorded robot/Mock device run',
        command_preview_only=True,collection_performed=False,training_dataset_refitted=False,
        analytic_fixture_inertia_regression_performed=True,
        original_inertia_relative_threshold=.1,**FLAGS)
    write_json(root/'protocol.json',protocol)
    context=dict(model_context=context_for(h4raw).identity,collision=collision_for(h4raw).metadata)
    write_json(root/'reference_context.json',context)
    if reference_checks is not None:
        reference_checks=Path(reference_checks).resolve()
        if json.loads((reference_checks/'reference_context.json').read_text())!=context:
            raise ValueError('Reference model/collision context changed; regenerate reference checks')
        previous=json.loads((reference_checks/'report.json').read_text())
        if previous['protocol']['config']!=config or previous['protocol']['poses']!=protocol['poses']:
            raise ValueError('Reference configuration changed')
        previous={r['name']:r for r in previous['references']}
    reports=[];signals={j:[] for j in WRISTS};profile=dict(effort_gain=1.,effort_zero_nm=0.)
    for center,pose in poses.items():
        for joint in WRISTS:
            for direction in (-1,1):
                name=f'{center}_j{joint:02}_{direction:+d}'
                motion=make_probe(name,home,pose,joint,direction,config=config)
                if reference_checks is not None:
                    source=reference_checks/'references'/name
                    source_motion=json.loads((source/'motion.json').read_text())
                    source_motion['conditions'].pop('sample_period_ns',None)
                    if source_motion!=motion.metadata():
                        raise ValueError('Reference motion changed; cannot reuse checks')
                    folder=root/'references'/name;folder.mkdir(parents=True)
                    write_json(folder/'motion.json',motion.metadata())
                    for file in ('reference.npz','collision_reference.json',
                                 'collision_command_preview.json','position_plan_preview.csv'):
                        shutil.copyfile(source/file,folder/file)
                    gate=json.loads((source/'collision_reference.json').read_text())
                    preview_gate=json.loads((source/'collision_command_preview.json').read_text())
                    coverage=previous[name]['coverage']
                else:
                    coverage=motion_coverage(motion,{})
                    scene=collision_for(h4raw);gate=gate_motion(scene,motion,scene.metadata['scene']['clearance_m'])
                    folder=root/'references'/name;folder.mkdir(parents=True)
                    write_json(folder/'motion.json',motion.metadata());write_json(folder/'collision_reference.json',gate)
                    curve=motion.polynomial();np.savez_compressed(folder/'reference.npz',coefficients=curve.c,breakpoints=curve.x)
                    stamps=np.arange(0,int(round(motion.duration*1e9))+1,1000000000//config['command_hz'],dtype=np.int64)
                    q=curve(stamps*1e-9)
                    preview_gate=certify_polyline(scene,q,stamps*1e-9,clearance_m=scene.metadata['scene']['clearance_m'],block_samples=1000)
                    write_json(folder/'collision_command_preview.json',preview_gate)
                    np.savetxt(folder/'position_plan_preview.csv',np.column_stack([stamps,q,curve(stamps*1e-9,nu=1)]),delimiter=',',
                        header='relative_ns'+''.join(','+key+str(j) for key in ('q','v') for j in range(14)),comments='',fmt=['%d']+['%.17g']*28)
                configured_limits=[config['reference_limits'][key] for key in (
                    'maximum_velocity_rad_s','maximum_acceleration_rad_s2','maximum_jerk_rad_s3')]
                coverage['passed'] &= all(peak<=limit for peak,limit in zip(coverage['peak_derivative_bounds'],configured_limits))
                if not (coverage['passed'] and gate['passed'] and preview_gate['passed']):
                    raise ValueError('Existing dedicated probe envelope/collision gate rejected '+name)
                window=motion.windows('probe_measure');t=np.arange(window[0][0]+.2,window[-1][1]-.2,.002)
                refq,refv,refa,_=motion.at(t);terms=engine.evaluate_batch(refq,refv,refa)
                inertia_rms=float(np.sqrt(np.mean(terms['inertia'][:,joint]**2)));signals[joint].append(inertia_rms)
                true_mass=float(engine.mass_matrix(refq[len(refq)//2])[joint,joint])
                cases=[]
                for phase in protocol['cache_phases_s']:
                    offsets=protocol['relative_effort_diagnostics_s'] if phase==.0013 else [0.]
                    for offset in offsets:
                        obs,elapsed=readout_fixture(motion,engine,config,phase,offset)
                        problem=probe_problem(obs,elapsed,motion.metadata(),engine,config,profile)
                        estimate=inertia_balance(problem)
                        predicted=problem['regression']@engine.rigid_parameters()+problem['extra']@np.r_[np.full(14,.01),np.full(14,.005)]+.01*problem['static_feature']
                        row=dict(cache_phase_s=phase,relative_effort_offset_s=offset,**estimate,
                            reference_joint_inertia_kg_m2=true_mass,
                            relative_inertia_error=abs(estimate['inertia_kg_m2']-true_mass)/true_mass,
                            matched_balance_rms_nm=float(np.sqrt(np.mean((predicted-problem['target'])**2))),
                            preprocessing=problem['preprocessing'])
                        cases.append(row)
                        if phase==.0013 and offset==0:
                            np.savez_compressed(folder/'public_fixture_problem.npz',**{k:v for k,v in problem.items() if isinstance(v,np.ndarray)})
                reports.append(dict(name=name,joint=joint,center=center,direction=direction,coverage=coverage,
                    reference_collision_passed=gate['passed'],command_preview_collision_passed=preview_gate['passed'],
                    reference_acceleration_torque_rms_nm=inertia_rms,
                    maximum_coherent_torque_error_for_10_percent_nm=.1*true_mass*np.sqrt(np.mean(refa[:,joint]**2)),cases=cases))
                print('ADAPTED',name,flush=True)
    # Compare reference signal budgets using the SAME nominal model, not the
    # previous evaluation plant scaled1.8 or any formal observations.
    low={j:[] for j in WRISTS}
    for center,pose in poses.items():
        for seed in (401,402):
            job=dict(id=f'{center}_{seed}',split='train',seed=seed)
            motion=make_motion(home,pose,h4,job);start=motion.windows('periodic')[0][0]
            q,v,a,_=motion.at(start+np.arange(1024)*h4['period_s']/1024)
            terms=engine.evaluate_batch(q,v,a)
            for j in WRISTS:low[j].append(float(np.sqrt(np.mean(terms['inertia'][:,j]**2))))
    budget=[dict(joint=j,name=names[j],low_frequency_acceleration_torque_rms_nm=float(np.mean(low[j])),
                 probe_acceleration_torque_rms_nm=float(np.mean(signals[j])),
                 gain=float(np.mean(signals[j])/np.mean(low[j]))) for j in WRISTS]
    baseline=[case for r in reports for case in r['cases'] if case['relative_effort_offset_s']==0]
    shifted=[case for r in reports for case in r['cases'] if case['relative_effort_offset_s']!=0]
    summary=dict(protocol=protocol,references=reports,signal_budget=budget,
        checks=dict(all_reference_and_command_preview_gates_passed=True,
            all_common_cache_fixture_inertia_errors_within_original_10_percent=max(c['relative_inertia_error'] for c in baseline)<=.1),
        maximum_common_cache_fixture_inertia_error=max(c['relative_inertia_error'] for c in baseline),
        maximum_relative_offset_fixture_inertia_error=max(c['relative_inertia_error'] for c in shifted),
        reference_readout_fixture_only=True,parameters_exported_for_deployment=False,**FLAGS)
    write_json(root/'report.json',summary)
    plot(summary,root)
    return summary


def plot(report,root):
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from casbot_calib.presentation import _font
    path=_font(12).path;font_manager.fontManager.addfont(path)
    plt.rcParams['font.family']=font_manager.FontProperties(fname=path).get_name();plt.rcParams['axes.unicode_minus']=False
    fig,ax=plt.subplots(1,2,figsize=(12,4.5),layout='constrained');labels=['左yaw','左pitch','左roll','右yaw','右pitch','右roll']
    x=np.arange(6);budget=report['signal_budget']
    ax[0].bar(x-.18,[r['low_frequency_acceleration_torque_rms_nm']*1000 for r in budget],.36,label='原低频参考')
    ax[0].bar(x+.18,[r['probe_acceleration_torque_rms_nm']*1000 for r in budget],.36,label='5Hz 非换向探针')
    ax[0].set_ylabel('加速力矩 RMS（mNm）');ax[0].set_title('同一 nominal 模型的参考信号预算');ax[0].set_xticks(x,labels);ax[0].legend()
    for offset,label in [(0.,'共用 5ms 缓存'),(-.005,'effort 相对 −5ms'),(.005,'effort 相对 +5ms')]:
        values=[max(c['relative_inertia_error'] for r in report['references'] if r['joint']==j for c in r['cases'] if c['relative_effort_offset_s']==offset)*100 for j in WRISTS]
        ax[1].plot(x,values,'o-',label=label)
    ax[1].axhline(10,color='red',ls='--',lw=1);ax[1].set_xticks(x,labels);ax[1].set_ylabel('对参考惯量的误差（%）');ax[1].set_title('理想参考读出夹具：缓存与相对错位分开');ax[1].legend()
    for a in ax:a.grid(axis='y',alpha=.2)
    fig.savefig(root/'offline_adaptation.png',dpi=160);plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--reuse-reference-checks',type=Path,help='Reuse identical model/scene/reference checks; only observation math is rerun')
    a=parser.parse_args();run(a.output,a.reuse_reference_checks)
