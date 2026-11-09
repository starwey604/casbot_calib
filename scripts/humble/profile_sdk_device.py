#!/usr/bin/env python3
"""Separate native device cost from ROS/DDS/logging using recorded SDK writes.

This is evaluation only: it reads the declared synthetic plant and private
adopted commands, never contributes observations or truth to identification.
"""
import argparse,csv,json,subprocess
from pathlib import Path
import yaml
from casbot_calib.model_context import load_context
from casbot_calib.data import write_json


def profile(source,output,tool,duration=None):
    source=Path(source).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    config=yaml.safe_load((source/'mock.yaml').read_text());context=load_context()
    if config['model_context']!=context.identity:raise ValueError('Profiling source model differs')
    raw=source/'mock/evaluation/plant';settings=dict(config['plant'],initial_q=config['initial_q'])
    settings['zero_gravity']=int(settings['zero_gravity']);settings['stiction_enabled']=int(settings.get('stiction_enabled',False))
    with (raw/'host_samples.csv').open() as stream:
        rows=csv.DictReader(stream);epoch=int(next(rows)['sample_ns'])
        last=epoch
        for row in rows:last=int(row['sample_ns'])
    with (raw/'device_commands.csv').open() as stream:
        rows=csv.DictReader(stream)
        for row in rows:last=max(last,int(row['write_ns'])+3_000_000)
    end=last-epoch if duration is None else min(round(duration*1e9),last-epoch)
    with (output/'settings.csv').open('x') as stream:
        for key,value in settings.items():
            if key=='device_pd':continue
            stream.write(','.join([key,*[str(x) for x in (value if isinstance(value,list) else [value])]])+'\n')
    result=subprocess.run([str(Path(tool).resolve()),str(context.urdf),str(output/'settings.csv'),str(raw/'device_commands.csv'),str(epoch),str(end),str(output/'native')],text=True,capture_output=True,check=True)
    (output/'process.txt').write_text(result.stdout+'\n'+result.stderr)
    report=json.loads(result.stdout)
    cost_by_second={}
    with (output/'native_advances.csv').open() as stream:
        for row in csv.DictReader(stream):
            second=(int(row['virtual_ns'])-1)//1_000_000_000
            cost_by_second[second]=cost_by_second.get(second,0.)+float(row['compute_ns'])*1e-9
    report.update(maximum_one_second_compute_s=max(cost_by_second.values()),
        virtual_second_bins_exceeding_one_second_compute=sum(v>1. for v in cost_by_second.values()),
        source=str(source),model_context=context.identity,
        integration_ns=settings['integration_ns'],source_unchanged=True,identification_input=False,
        scope='native-only1ms chunk replay through shared SimulatedArmDevice; no ROS/logging inside timed region; no real-time guarantee',
        recorded_commands='SDK device-adopted five fields, including the original position interpolation; no new controller or model')
    write_json(output/'report.json',report);print(json.dumps(report,ensure_ascii=False));return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--tool',type=Path,default=Path('build/humble/profile_device'));p.add_argument('--duration',type=float)
    a=p.parse_args();profile(a.source,a.output,a.tool,a.duration)
