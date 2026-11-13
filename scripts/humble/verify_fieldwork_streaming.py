#!/usr/bin/env python3
"""Nonvisual numeric/RSS checks on >1GB synthetic CSV; no robot or renderer."""
import argparse
import bisect
import json
import os
from pathlib import Path
import resource
import shutil
import subprocess
import sys
import time

ORIGIN=10**18


def stamp(i):return ORIGIN+i*1000000+i*(i-1)//2


def worker(args):
    start=time.perf_counter()
    if args.mode=='analysis':
        from casbot_calib.fieldwork_a_evidence import run
        report=run(args.source,args.output)
        payload=dict(record=report['records'][0],scratch_clean=not list(args.output.glob('fieldwork-analysis-*')))
    else:
        from casbot_calib.fieldwork_streaming import VideoFeedback
        source=VideoFeedback(args.source/'capture/execution/feedback.csv')
        count=total=0
        for t,ns,q in source.frames():count+=1;total+=ns-ORIGIN
        payload=dict(frames=count,selected_ns_sum=total,rows=source.rows,duration_s=source.duration_s)
    print(json.dumps(dict(mode=args.mode,elapsed_s=time.perf_counter()-start,
        peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,**payload)))
    return 0


def generate(root,n):
    execution=root/'capture/execution';execution.mkdir(parents=True)
    payload=','.join(['0.12345678901234567']*42)+'\n'
    with (execution/'feedback.csv').open('w') as feedback,(execution/'commands.csv').open('w') as commands:
        feedback.write('receive_ns,'+','.join(p+str(j) for p in ('q','v','effort_raw') for j in range(14))+'\n')
        commands.write('publish_ns,planned_ns\n')
        for i in range(n):
            ns=stamp(i);feedback.write(str(ns)+','+payload)
            if i%2==0:commands.write(f'{ns},{ns-(i//2)%97}\n')
    return sum(p.stat().st_size for p in execution.iterdir())


class Stamps:
    def __init__(self,n):self.n=n
    def __len__(self):return self.n
    def __getitem__(self,index):return stamp(index)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=('verify','analysis','video'),default='verify')
    parser.add_argument('--source',type=Path);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--large-rows',type=int,default=1200000)
    args=parser.parse_args()
    if args.mode!='verify':return worker(args)
    root=args.output.resolve();root.mkdir(parents=True,exist_ok=False);results={};checks={}
    for label,n in [('small',20000),('large',args.large_rows)]:
        source=root/label;size=generate(source,n);results[label]=dict(csv_bytes=size,rows=n)
        for mode in ('analysis','video'):
            cmd=[sys.executable,__file__,'--mode',mode,'--source',str(source),'--output',str(root/(label+'_'+mode))]
            process=subprocess.run(cmd,capture_output=True,text=True,check=True)
            result=json.loads(process.stdout);results[label][mode]=result
        row=results[label]['analysis']['record']
        checks[label+'_full_counts_and_exact_hz']=row['receiver']['rows']==n and row['sender']['rows']==(n+1)//2 and row['receiver']['hz']==(n-1)*1e9/(stamp(n-1)-ORIGIN)
        checks[label+'_full_missing_counts']=row['missing_values']==dict(q=0,dq=0,effort_raw=0)
        checks[label+'_scratch_removed']=results[label]['analysis']['scratch_clean']
        last=stamp(n-1)-ORIGIN;count=(last+25000000+49999999)//50000000
        series=Stamps(n)
        total=sum(stamp(max(0,bisect.bisect_right(series,ORIGIN+i*50000000)-1))-ORIGIN for i in range(count))
        video=results[label]['video']
        checks[label+'_video_matches_independent_bisection']=video['frames']==count and video['selected_ns_sum']==total and video['rows']==n
    for mode in ('analysis','video'):
        small=results['small'][mode]['peak_rss_kib'];large=results['large'][mode]['peak_rss_kib']
        checks[mode+'_rss_growth_below_32_mib']=large-small<32*1024
    checks['over_one_gb_csv_exercised']=results['large']['csv_bytes']>=10**9
    # Fixtures belong only to this invocation; keep reports, not another GB spool.
    for label in ('small','large'):shutil.rmtree(root/label)
    report=dict(passed=all(checks.values()),checks=checks,results=results,
        scope='synthetic CSV statistics and video feedback selection only; no rendering, encoding or physical robot',
        generated_csv_fixtures_removed=True)
    (root/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
