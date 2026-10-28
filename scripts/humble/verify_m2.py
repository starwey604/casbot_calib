"""Verify captured M2 evidence and repeat estimation with evaluation reads forbidden."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import yaml
from casbot_calib.ros_benchmark import analyse_suite, fingerprint


def verify(folder,regression,truth_archive=None):
    def read(path):return json.loads(path.read_text(encoding='utf-8'))
    report=read(folder/'analysis/report.json');session=read(folder/'session.json')
    assert report['passed'] and not report['hardware_tested'] and not report['physical_network_tested']
    assert session['control_hz']==500
    training=next(c for c in session['cases'] if c.get('train'))
    assert training['command_hz']==training['feedback_hz']==500
    archive=truth_archive or folder/'evaluation_archive'
    original_truth=folder/training['name']/'evaluation/plant.json'
    truth=read(original_truth if original_truth.exists() else archive/training['name']/'plant.json')
    tau_error=max(abs(f['current_tau_s']/truth['current_tau_s']-1) for f in report['driver_parameters'])
    delay_error=max(abs(f['command_delay_s']-truth['drive_delay_ns']*1e-9) for f in report['driver_parameters'])
    assert tau_error<.05 and delay_error<.0001
    recording={}
    for case in session['cases']:
        path=folder/case['name'];capture=read(path/'capture.json')
        assert capture['client_returncode']==capture['host_returncode']==0
        metrics=next(r for r in report['cases'] if r['name']==case['name'])
        assert fingerprint(path)==metrics['input_sha256'] and not metrics['parameters_refitted']
        if case.get('record'):
            metadata=yaml.safe_load((path/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
            counts={row['topic_metadata']['name'].split('/')[-1]:row['message_count'] for row in metadata['topics_with_message_count']}
            assert all(counts.get(name,0)>0 for name in ('commands','observations','events'))
            recording[case['name']]=counts
    previous=read(regression/'verification.json');assert previous['passed'] and previous['baseline_metrics_compared']==114
    if not archive.exists():archive.mkdir()
    for case in session['cases']:
        original_folder=folder/case['name']/'evaluation'
        if original_folder.exists():shutil.move(str(original_folder),str(archive/case['name']))
    original=Path.open
    def guarded(self,*args,**kwargs):
        assert '/evaluation' not in str(self),'Estimator attempted to read evaluation truth'
        return original(self,*args,**kwargs)
    Path.open=guarded
    try:again=analyse_suite(folder,folder/'reanalysis_verified')
    finally:Path.open=original
    assert again['passed'] and again['driver_parameters']==report['driver_parameters']
    for old,new in zip(report['cases'],again['cases'],strict=True):
        assert old['frozen_driver_current_rmse_a']==new['frozen_driver_current_rmse_a']
    sources=[p for directory in ('cpp','python','configs','ros','scripts/humble','tests') for p in Path(directory).rglob('*')
             if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ('.so','.pyc','.pyd')]
    result=dict(passed=True,milestone='M2 local ROS Mock and stream validation',physical_network_tested=False,ros2_control_hardware_adapter_implemented=False,
        default_command_control_feedback_hz=500,device_sampling_hz=1000,cases=len(report['cases']),tests=previous['tests'],baseline_metrics_compared=114,
        maximum_current_rmse_a=max(c['maximum_current_rmse_a'] for c in report['cases']),
        maximum_tau_relative_error=tau_error,maximum_delay_error_ms=delay_error*1000,
        truth_isolated_reanalysis_identical=True,rosbag_message_counts=recording,
        qualified_single_batches=report['qualified_local_profiles'],operating_frequency_recommendation=None,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    (folder/'verification.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('M2 local ROS / frozen model / fault rejection / rosbag / truth isolation: PASS')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--folder',type=Path,required=True);parser.add_argument('--regression',type=Path,required=True)
    parser.add_argument('--truth-archive',type=Path)
    args=parser.parse_args();verify(args.folder,args.regression,args.truth_archive)
