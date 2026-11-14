"""A diagnostics using mapped SHM observation time and unchanged balance kernels."""
import argparse
from pathlib import Path
import numpy as np
from .data import write_json
from .shm_observations import ObservationReader
from .shm_h1 import load_profile,DEFAULT_PROFILE
from .field_artifacts import output_outside
from .probe_diagnostics import window_a


def analyse(capture,output,*,profile=DEFAULT_PROFILE,window,joint_id=12,band=30.,clock_segment=None):
    if joint_id not in range(12,26):raise ValueError('使用双臂真实ID12–25')
    output=output_outside(output,capture);output.mkdir(parents=True,exist_ok=False)
    config=load_profile(profile)
    report=dict(schema_version='casbot_shm_a_evidence_v1',source=str(Path(capture).resolve()),joint_id=joint_id,
        window_s=list(window),band_hz=band,source_timestamp_quantum_ns=1000000,
        sample_time_known=False,automatic_calibration_acceptance=False,time_basis='mapped master conversion time',
        effort_resolution_nm=.01,relative_channel_alignment='same frame development default; no inferred filter compensation')
    try:
        reader=ObservationReader(capture);obs,elapsed,scope=reader.window(window,clock_segment)
        report.update(scope,first_command_mono_ns=reader.origin)
        result=window_a(obs,elapsed,window,joint_id-12,config,band,observation_time_ns=obs['observation_time_ns'])
        # SHM has no IMU stream in this adapter; never turn an omitted sensor into zero base motion.
        result['body_motion']=dict(available=False,next_action='当前SHM记录不含IMU；身体运动仍是未建模项')
        report.update(available=True,A=result)
    except (ValueError,KeyError,np.linalg.LinAlgError) as error:report.update(available=False,unavailable_reason=str(error))
    write_json(output/'report.json',report);return report


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('capture',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--profile',type=Path,default=DEFAULT_PROFILE);p.add_argument('--window',nargs=2,type=float,required=True)
    p.add_argument('--joint-id',type=int,choices=range(12,26),default=12);p.add_argument('--band',type=float,default=30.)
    p.add_argument('--clock-segment',type=int);a=p.parse_args(argv)
    try:
        report=analyse(a.capture,a.output,profile=a.profile,window=a.window,joint_id=a.joint_id,band=a.band,clock_segment=a.clock_segment)
        print('A分析：'+('可计算' if report['available'] else '不可计算：'+report['unavailable_reason'])+'；'+str(a.output/'report.json'));return 0
    except (ValueError,OSError) as error:print('A错误：'+str(error),file=__import__('sys').stderr);return 2
