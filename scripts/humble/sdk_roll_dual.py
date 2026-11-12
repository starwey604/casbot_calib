#!/usr/bin/env python3
"""Freeze, explicitly collect local SDK Mock, or analyse the roll dual-tone pair."""
import argparse
import os
from pathlib import Path
import subprocess
import sys


def install_analysis_guard(action):
    # The launcher owns private Mock runtime logs; only observation analysis
    # must be unable to consume them or the evaluation truth.
    if action!='analyse':return
    def guard(event,args):
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)) and 'evaluation' in Path(os.fsdecode(args[0])).parts:
            raise RuntimeError('No private evaluation data in dual-tone exploration')
    sys.addaudithook(guard)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['freeze','collect','analyse'])
    parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output',type=Path)
    parser.add_argument('--domain-id',type=int,default=227);a=parser.parse_args()
    if a.action!='collect' and a.output is None:parser.error('freeze/analyse requires a new --output directory')
    if a.action=='collect' and a.output is not None:parser.error('collect uses the existing frozen --source session')
    install_analysis_guard(a.action)
    if a.action=='analyse':
        import casbot_calib.rigid_identification as rigid
        def no_fit(*args,**kwargs):raise RuntimeError('Dual-tone observation analysis does not fit a rigid candidate')
        rigid.fit_rigid=no_fit
    from casbot_calib import hardware_roll_dual as dual
    if a.action=='freeze':dual.freeze(a.source,a.output)
    elif a.action=='collect':
        dual.collect(a.source,a.domain_id)
        subprocess.run([sys.executable,__file__,'analyse','--source',str(a.source),'--output',str(a.source/'analysis')],check=True)
        from casbot_calib.data import write_json
        from casbot_calib.hardware_wrist_exploration import FLAGS
        write_json(a.source/'status.json',dict(stage='collected_and_analysed',records_collected=2,analysis='analysis/report.json',**FLAGS))
    else:dual.analyse(a.source,a.output)


if __name__=='__main__':main()
