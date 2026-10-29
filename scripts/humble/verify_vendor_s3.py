#!/usr/bin/env python3
"""Offline S3 qualification from existing native Mock train data and S1 feedback."""
import argparse
import copy
import json
from pathlib import Path
import csv
import subprocess
import sys
import numpy as np
import yaml
from casbot_calib.dynamics_data import import_dynamics,read_json
from casbot_calib.dynamics_qualification import qualify_dynamics
from casbot_calib.dynamics_problem import build_periodic_problem
from casbot_calib.vendor_identity import sha256
from casbot_calib.dual_model import nominal_dual_arm


def write_json(path,value):Path(path).write_text(json.dumps(value,indent=2)+'\n')


def bundle(folder,payload,fields,names,model_sha,*,experiment=None,firmware='native_mock_fixture'):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    identity=dict(dataset_id=folder.name,robot_serial='synthetic_native_mock',firmware_version=firmware,
                  motion_software_version='legacy_schema1_train',model_sha256=model_sha,
                  provenance_reference='existing digest-verified observable capture; never evaluation truth')
    contract=dict(schema_version='casbot_observation_contract_v1',scope='synthetic_observations',
                  source_sha256=sha256(payload),joint_names=names,model_id='casbot_20251015_nominal_fixed_waist_dual',
                  identity=identity,fields=fields,position_unit='rad',velocity_unit='rad/s',
                  position_time=dict(field='sample_ns',role='acquisition',clock_id='native_mock_acquisition'),
                  signal_time=dict(field='sample_ns',role='acquisition',clock_id='native_mock_acquisition'),
                  signal=dict(origin='motor_current',unit='A',side='output_equivalent',includes='total_actuator_output',
                              gain_nm_per_raw_unit=[2.]*14,zero_raw=[0.]*14,sensor_identity='synthetic_output_current',
                              raw_acquisition=True,filter=dict(kind='none')),experiment=experiment or {})
    # Public synthetic unit/scale reference, separate from robot model/plant state.
    levels=np.array([-1.,-.5,-.2,.2,.5,1.])[:,None]*np.ones((1,14))
    scale=dict(schema_version='casbot_scale_validation_v1',scope=contract['scope'],joint_names=names,model_id=contract['model_id'],
               device_identity={k:identity[k] for k in ('robot_serial','firmware_version')},sensor_identity='synthetic_output_current',
               reference_basis='synthetic_reference',raw_delta=levels.tolist(),reference_delta_nm=(2*levels).tolist(),
               reference_zero_raw=[0.]*14,zero_uncertainty_raw=[1e-6]*14,
               note='Analytic synthetic scale fixture, not an ENCOS measurement or validation of hardware gain')
    write_json(folder/'contract.json',contract);write_json(folder/'scale.json',scale)
    profile=dict(schema_version='casbot_vendor_s3_profile_v1',scope=contract['scope'],
                 contract=dict(path='contract.json',sha256=sha256(folder/'contract.json')),
                 scale_validation=dict(path='scale.json',sha256=sha256(folder/'scale.json')),clock_mapping=None,
                 requested_band_hz=40/6)
    (folder/'profile.yaml').write_text(yaml.safe_dump(profile))
    return folder/'profile.yaml',contract,profile


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--staged-source',type=Path,required=True)
    parser.add_argument('--vendor-source',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    legacy=args.staged_source.resolve();session=read_json(legacy/'session.json')
    if session['config']['schema_version']!=1:raise ValueError('Use explicit legacy Mock train data; no P0 formal test access')
    engine,model=nominal_dual_arm();names=[j.name for j in engine.model.joints];checks={};cases={}
    def check_case(name,payload,profile,source_type='host',*,inertia=None,gravity=None):
        target=root/name
        import_dynamics(payload,profile,target/'canonical',source_type=source_type)
        report=qualify_dynamics(target/'canonical',target/'qualification');cases[name]=report
        if inertia is not None:checks[name+'_inertia']=report['stage_data_eligibility']['inertia']['observations_eligible']==inertia
        if gravity is not None:checks[name+'_gravity']=report['stage_data_eligibility']['gravity']['observations_eligible']==gravity
        return report
    observed={}
    for trial in ('inertia_train','gravity_train_00_-1'):
        folder=legacy/'runs'/trial;meta=read_json(folder/'metadata.json');payload=folder/'observations.npz'
        if not meta['passed'] or meta['conditions']['batch']!='train' or sha256(payload)!=meta['raw_sha256']:raise ValueError('Train record identity/integrity rejected')
        with np.load(payload,allow_pickle=False) as data:observed[trial]={key:data[key].copy() for key in data.files}
    train=legacy/'runs/inertia_train/observations.npz';motion=read_json(legacy/'plans/inertia_train/motion.json')
    intervals=[i for i,label in enumerate(motion['labels']) if label=='periodic']
    experiment=dict(kind='periodic',begin_ns=round(motion['knots_s'][intervals[0]]*1e9),end_ns=round(motion['knots_s'][intervals[-1]+1]*1e9),
                    period_s=6.,discard_periods=2,harmonics=40,phase_points=1024)
    fields=dict(position='q',velocity='dq',signal='current',receive='receive_ns',device_sequence='sequence',status='status')
    profile,contract,settings=bundle(root/'evidence_4k',train,fields,names,model['sha256'],experiment=experiment)
    report=check_case('native_4k',train,profile,inertia=True)
    problem=build_periodic_problem(root/'native_4k/qualification',root/'native_4k/problem')
    from casbot_calib.staged_inertia import periodic_problem
    old=periodic_problem(engine,observed['inertia_train'],motion,2.,np.zeros(14))
    with np.load(root/'native_4k/problem/problem.npz') as new:
        regressor_error=float(np.max(abs(new['regression']-old['regression'])))
        torque_error=float(np.max(abs(new['torque_nm']-old['current_torque'])))
    checks['existing_pinocchio_problem_equivalence']=regressor_error<1e-8 and torque_error<1e-8
    # Same observable trace in explicit Nm: no fabricated current-shaped input.
    torque_raw={k:v.copy() for k,v in observed['inertia_train'].items()}
    torque_raw['output_torque']=2*torque_raw.pop('current')
    torque_path=root/'output_torque.npz';np.savez_compressed(torque_path,**torque_raw)
    torque_fields=dict(fields,signal='output_torque')
    tp,tc,ts=bundle(root/'evidence_torque',torque_path,torque_fields,names,model['sha256'],experiment=experiment)
    tc['signal'].update(origin='output_torque_estimate',unit='Nm',gain_nm_per_raw_unit=[1.]*14)
    write_json(tp.parent/'contract.json',tc);ts['contract']['sha256']=sha256(tp.parent/'contract.json')
    scale=read_json(tp.parent/'scale.json');scale['reference_delta_nm']=scale['raw_delta']
    write_json(tp.parent/'scale.json',scale);ts['scale_validation']['sha256']=sha256(tp.parent/'scale.json');tp.write_text(yaml.safe_dump(ts))
    check_case('output_torque_nm',torque_path,tp,inertia=True)
    build_periodic_problem(root/'output_torque_nm/qualification',root/'output_torque_nm/problem')
    with np.load(root/'output_torque_nm/problem/problem.npz') as direct, np.load(root/'native_4k/problem/problem.npz') as current:
        checks['torque_nm_without_current_alias']=np.array_equal(direct['target'],current['target'])
    # Fresh-process replay forbids opening any evaluation path, rather than
    # relying on the caller having ignored truth during one interactive run.
    replay_code='''import sys,os
from pathlib import Path
def guard(event,args):
    if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
        if 'evaluation' in Path(os.fsdecode(args[0])).parts:
            raise RuntimeError('evaluation access forbidden')
sys.addaudithook(guard)
try:open('/__casbot_guard_probe__/evaluation/truth.npz','rb')
except RuntimeError:pass
else:raise RuntimeError('audit hook did not enforce truth isolation')
from casbot_calib.dynamics_problem import build_periodic_problem
build_periodic_problem(sys.argv[1],sys.argv[2])
'''
    replay=subprocess.run([sys.executable,'-c',replay_code,str(root/'native_4k/qualification'),str(root/'truth_isolated_problem')],capture_output=True,text=True)
    (root/'truth_isolation.log').write_text(replay.stdout+replay.stderr)
    checks['fresh_process_evaluation_access_forbidden']=replay.returncode==0
    # CSV keeps original acquisition times / device sequence / observations.
    gravity=observed['gravity_train_00_-1'];csv_path=root/'gravity_host.csv'
    with csv_path.open('w') as f:
        writer=csv.writer(f);writer.writerow(['sample_ns','receive_ns','sequence']+[k+str(j) for k in ('q','v','i','s') for j in range(14)])
        for row in range(len(gravity['sample_ns'])):
            writer.writerow([int(gravity[k][row]) for k in ('sample_ns','receive_ns','sequence')]+
                            [float(gravity[k][row,j]) for k in ('q','dq','current','status') for j in range(14)])
    csv_fields=dict(position=['q'+str(j) for j in range(14)],velocity=['v'+str(j) for j in range(14)],
                    signal=['i'+str(j) for j in range(14)],status=['s'+str(j) for j in range(14)],
                    receive='receive_ns',device_sequence='sequence')
    low_profile,_,_=bundle(root/'evidence_static_csv',csv_path,csv_fields,names,model['sha256'])
    check_case('static_1k_csv',csv_path,low_profile,inertia=False,gravity=True)
    # The existing SDK/S2 feedback keeps its unknown torque / sampling semantics.
    unknown=root/'unknown.yaml';unknown.write_bytes(Path('configs/vendor_s3_unknown.yaml').read_bytes())
    sdk=check_case('sdk_unknown',args.vendor_source,unknown,'vendor',inertia=False,gravity=False)
    checks['sdk_no_fabricated_current_or_time']=sdk['valid_samples']==0 and not sdk['passed']
    # Provenance / gain / filter / clock failures use new immutable inputs.
    for name in ('no_scale','command_echo','double_gain','unknown_filter','unmapped_clock'):
        folder=root/('evidence_'+name);folder.mkdir();c=copy.deepcopy(contract);p=copy.deepcopy(settings)
        p['scale_validation']=dict(path='../evidence_4k/scale.json',sha256=sha256(root/'evidence_4k/scale.json'))
        if name=='no_scale':p['scale_validation']=None
        if name=='command_echo':c['signal']['origin']='command_echo'
        if name=='double_gain':c['signal']['gain_nm_per_raw_unit']=[4.]*14
        if name=='unknown_filter':c['signal']['filter']['kind']='unknown'
        if name=='unmapped_clock':c['position_time']['clock_id']='unmapped_encoder_clock'
        write_json(folder/'contract.json',c);p['contract']['sha256']=sha256(folder/'contract.json');(folder/'profile.yaml').write_text(yaml.safe_dump(p))
        check_case(name,train,folder/'profile.yaml',inertia=False)
    # Clock encoding is changed synthetically; acquired mechanics are untouched.
    clock_raw={k:v.copy() for k,v in observed['inertia_train'].items()};original=clock_raw['sample_ns'];offset=1_700_000_000_000_000_000
    clock_raw['encoder_time_ns']=offset+original+original//10000
    clock_path=root/'encoded_clock.npz';np.savez_compressed(clock_path,**clock_raw)
    p,c,s=bundle(root/'evidence_clock',clock_path,fields,names,model['sha256'],experiment=experiment)
    c['position_time']=dict(field='encoder_time_ns',role='acquisition',clock_id='synthetic_encoder_clock')
    write_json(p.parent/'contract.json',c);s['contract']['sha256']=sha256(p.parent/'contract.json')
    pick=[0,len(original)//2,len(original)-1]
    mapping=dict(schema_version='casbot_clock_mapping_v1',scope=c['scope'],joint_names=names,model_id=c['model_id'],
                 source_clock='synthetic_encoder_clock',target_clock='native_mock_acquisition',source_origin_ns=offset,target_origin_ns=0,
                 rate_ratio=1/1.0001,uncertainty_ns=100.,valid_source_range_ns=[int(clock_raw['encoder_time_ns'][0]),int(clock_raw['encoder_time_ns'][-1])],
                 validation_source_ns=clock_raw['encoder_time_ns'][pick].tolist(),validation_target_ns=original[pick].tolist())
    write_json(p.parent/'clock.json',mapping);s['clock_mapping']=dict(path='clock.json',sha256=sha256(p.parent/'clock.json'));p.write_text(yaml.safe_dump(s))
    clock=check_case('mapped_clock',clock_path,p,inertia=True);checks['clock_precision']=clock['time']['maximum_alignment_error_ns']<=101
    # A real hole inside the steady window is retained, never Fourier-stitched.
    gap={k:np.delete(v,np.searchsorted(original,20_000_000_000),axis=0) for k,v in observed['inertia_train'].items()}
    gap_path=root/'gapped.npz';np.savez_compressed(gap_path,**gap)
    gp,_,_=bundle(root/'evidence_gap',gap_path,fields,names,model['sha256'],experiment=experiment)
    check_case('missing_sample',gap_path,gp,inertia=False)
    try:build_periodic_problem(root/'missing_sample/qualification',root/'missing_sample/forbidden_problem')
    except ValueError:checks['gap_blocks_problem']=True
    else:checks['gap_blocks_problem']=False
    with (root/'qualification_matrix.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.writer(f);writer.writerow(['case','gravity','friction','breakaway','inertia','hardware_executable'])
        for name,case in cases.items():
            writer.writerow([name]+[case['stage_data_eligibility'][stage]['observations_eligible'] for stage in ('gravity','friction','breakaway','inertia')]+[False])
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.font_manager import FontProperties
    from casbot_calib.presentation import _font
    font=FontProperties(fname=_font(12).path);figure=Figure(figsize=(10,7));FigureCanvasAgg(figure);axis=figure.subplots()
    matrix=np.array([[int(case['stage_data_eligibility'][stage]['observations_eligible']) for stage in ('gravity','friction','breakaway','inertia')] for case in cases.values()])
    axis.imshow(matrix,cmap='RdYlGn',vmin=0,vmax=1,aspect='auto')
    axis.set_yticks(range(len(cases)),list(cases));axis.set_xticks(range(4),['重力','运动摩擦','起动摩擦','惯量'],fontproperties=font)
    for i in range(len(cases)):
        for j in range(4):axis.text(j,i,'具备数据资格' if matrix[i,j] else '阻断',ha='center',va='center',fontproperties=font,fontsize=9)
    axis.set_title('S3 离线观测资格；全部实验执行权限仍关闭',fontproperties=font)
    figure.tight_layout();figure.savefig(root/'qualification_matrix.png',dpi=140)
    result=dict(passed=all(checks.values()),checks=checks,hardware_executable=False,physical_semantics_verified_by_software=False,
                scope='S3 offline observation contracts and solver-boundary regression; existing native train observations only',
                old_regressor_max_error=regressor_error,old_torque_max_error=torque_error,mechanical_parameters_fitted=False,
                qualification_paths={name:str(root/name/'qualification/report.json') for name in cases},
                native_source_sha256={str(legacy/'runs'/name/'observations.npz'):sha256(legacy/'runs'/name/'observations.npz') for name in observed})
    write_json(root/'verification.json',result);print(json.dumps(result,indent=2));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
