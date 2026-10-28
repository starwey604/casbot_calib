"""Reviewable K2 delivery: measurement, faults, collision and truth-free replay."""
from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
from .kinematics import workflow_config,nominal_kinematics
from .trical_workflow import run_mock
from .trical_analysis import analyse
from .data import write_json


def run_suite(folder, config=None):
    root=Path(folder);root.mkdir(parents=True,exist_ok=False);config=workflow_config() if config is None else deepcopy(config)
    baseline=run_mock(root/'baseline',config);checks={'baseline':baseline['passed']}
    if not baseline['passed']:
        result=dict(passed=False,checks=checks,baseline=baseline);write_json(root/'suite.json',result);return result
    # The complete evaluation directory is physically removed from the session.
    hidden=root/'baseline/evaluation';quarantine=root/'withheld_evaluation';hidden.rename(quarantine)
    try:analysis=analyse(root/'baseline',root/'baseline/analysis')
    finally:quarantine.rename(hidden)
    checks['truth_isolated_analysis']=analysis['passed'] and not analysis['truth_read']
    for name,expected in json.loads((root/'baseline/manifest.json').read_text())['files'].items():
        if sha256((root/'baseline'/name).read_bytes()).hexdigest()!=expected:raise ValueError('Reanalysis modified raw input')
    from .trical_evaluation import evaluate,plot_observations
    evaluation=evaluate(root/'baseline',root/'synthetic_evaluation');checks['synthetic_position_evaluation']=evaluation['passed']
    plot_observations(root/'baseline',root/'readings_zh.png')
    cases={}
    for fault in ('disconnected','frozen','out_of_range','no_contact'):
        fault_config=deepcopy(config);fault_config['workflow']['measurement_timeout_s']=.4
        result=run_mock(root/fault,fault_config,fault=fault,first_only=True)
        expected='stable_window_timeout' if fault=='no_contact' else 'indicator_fault_'
        cases[fault]=dict(passed=not result['passed'] and result['measurements']==0 and result['recovered_to_home'] is True and result['failure'].startswith(expected),report=result)
    dropped=deepcopy(config);dropped['sensor']['drop_probability']=1.;dropped['workflow']['measurement_timeout_s']=.4
    result=run_mock(root/'dropped',dropped,first_only=True)
    cases['dropped']=dict(passed=not result['passed'] and result['measurements']==0 and result['recovered_to_home'] and result['failure']=='stable_window_timeout',report=result)
    unknown=deepcopy(config);unknown['sensor']['expose_sample_time']=False
    result=run_mock(root/'unknown_timestamps',unknown,first_only=True)
    verified=analyse(root/'unknown_timestamps',root/'unknown_timestamps/analysis') if result['passed'] else {'passed':False}
    cases['unknown_timestamps']=dict(passed=result['passed'] and verified['passed'],report=result)
    blocked=deepcopy(config);engine,_=nominal_kinematics(blocked);visit=blocked['workflow']['visits'][0]
    center=engine.frame_pose(visit['nominal_q_rad'],visit['frame'])[:3,3].tolist()
    blocked['workflow']['collision_scene']['obstacles'].append(dict(name='blocked_target',center=center,half_extents=[.07]*3,attached_to_base=False))
    result=run_mock(root/'collision_rejection',blocked,first_only=True)
    cases['collision_rejection']=dict(passed=not result['passed'] and result['measurements']==0 and result['failure'].startswith('complete_visit_preflight_rejected'),report=result)
    checks.update({name:bool(case['passed']) for name,case in cases.items()})
    report=dict(schema_version=1,stage='K2',passed=all(checks.values()),checks=checks,cases=cases,
        synthetic_only=True,hardware_executable=False,calibration_performed=False,
        scope='K2 low-speed measurement state machine; no parameter identification, real robot or physical Ethernet validation',
        native_sha256=baseline['native_sha256'],geometry_sha256=baseline['geometry_sha256'],
        maximum_accepted_reading_um=max(r['final_indicator_max_um'] for r in evaluation['measurements']),
        maximum_synthetic_position_error_um=max(r['evaluated_position_max_um'] for r in evaluation['measurements']))
    write_json(root/'suite.json',report)
    (root/'README.md').write_text('# K2双臂千分计测量闭环\n\n结果：'+('通过' if report['passed'] else '失败')+
        '。命令500Hz、三表各50Hz、四球左右各两次测量；均为虚拟时间合成条件。\n\n'
        '[综合结果](suite.json) · [无真值复核](baseline/analysis/report.json) · [读数图](readings_zh.png) · '
        '[独立合成评估](synthetic_evaluation/report.json) · [CSV](synthetic_evaluation/measurements.csv)\n\n'
        'K2验证对中/采样和故障退出；没有几何参数求解，不代表实物位置精度。MuJoCo视频在K4接入。\n',encoding='utf-8')
    return report
