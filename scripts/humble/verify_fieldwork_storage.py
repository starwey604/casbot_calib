#!/usr/bin/env python3
"""Nonvisual private ROS: canonical records, relocatable saves and GUI quotas."""
import argparse
import csv
import json
import os
from pathlib import Path
import threading
import time


def wait(predicate, timeout=25):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.025)
    raise RuntimeError('等待私有记录验证条件超时')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--domain-id',type=int,required=True);args=p.parse_args()
    root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),ROS_LOCALHOST_ONLY='1',CASBOT_FIELDWORK_CACHE=str(root/'cache'))
    from casbot_calib.fieldwork_cli import parser
    from casbot_calib.fieldwork_launch import prepare_gui
    from casbot_calib.fieldwork_storage import cache_for,attach_budget,CacheMonitor,recording_pause_file,save_snapshot
    from casbot_calib.fieldwork_control import Workspace,Session,create_session,initial_commands,export_readback
    from casbot_calib.fieldwork_settings import defaults
    from casbot_calib.fieldwork_observer import observe
    from casbot_calib.fieldwork_a_evidence import run as evidence
    checks={};results={}
    ui=parser().parse_args(['gui','--mock','--mock-diagnostics','--domain-id',str(args.domain_id),
        '--output',str(root/'external'),'--cache-limit-mib','32'])
    with prepare_gui(ui) as (workspace,monitor,mock):
        cache=cache_for(workspace)
        checks['owned_mock_external_output_in_same_budget']=set(monitor.roots)=={cache,workspace}
        wait(lambda:any(p.stat().st_size>10000 for p in (cache/'mock').glob('evaluation/**/*.csv')))
        monitor.limit=1;monitor.check();time.sleep(.5)
        logs=list((cache/'mock').glob('evaluation/**/*.csv'));sizes={p:p.stat().st_size for p in logs}
        time.sleep(.5)
        checks['owned_diagnostics_stop_on_shared_marker']=bool(logs) and all(p.stat().st_size==n for p,n in sizes.items())
        checks['external_pause_marker_is_shared']=recording_pause_file(workspace)==cache/'recording_paused.json'
    checks['owned_mock_shutdown']=json.loads((cache/'mock/shutdown.json').read_text())['returncodes']==[0]
    from verify_fieldwork_round1 import ProtocolFixture
    fixture=ProtocolFixture(root/'fixture',args.domain_id);fixture.velocity=[0.]*14;fixture.enable_service()
    workspace=root/'explicit';create_session(fixture.client_profile,workspace,args.domain_id,'localhost_contract_mock',workspace=True)
    attach_budget(workspace);monitor=CacheMonitor(workspace);pause=recording_pause_file(workspace)
    stop=threading.Event();observation_stop=threading.Event();latest={};failures=[]
    def publish(kind,**values):
        if kind=='feedback':latest.update(values)
    def observing():
        try:observe(workspace,'observations/one',lambda:'operator_cancel' if observation_stop.is_set() else None,publish)
        except Exception as exc:failures.append(str(exc))
    observer=threading.Thread(target=observing);observer.start();sender=None
    try:
        wait(lambda:latest.get('q') is not None)
        control=workspace/Workspace(workspace).new_control();session=Session(control)
        run=control/'runs/one';run.mkdir(parents=True)
        settings=defaults(session.config,session.names,session.metadata['model_context'])
        values,_=initial_commands(latest['q'],None,None,session.metadata['manual_tuning'])
        def sending():
            try:results['control']=session.stream('runs/one',run/'latest.csv',initial_values=values,settings=settings,
                recording_pause_file=pause,cancel_check=lambda:'operator_cancel' if stop.is_set() else None)
            except Exception as exc:failures.append(str(exc))
        sender=threading.Thread(target=sending);sender.start()
        wait(lambda:fixture.commands>100)
        saved=root/'saved';save_snapshot(workspace,saved)
        saved_run=saved/run.relative_to(workspace)
        manifest=json.loads((saved_run/'recording/recording.json').read_text())
        checks['save_generates_no_duplicate_or_derived_csv']=not list(saved.glob('**/recording/*.csv'))
        checks['manifest_references_native_files']=all((saved_run/'recording'/v).is_file() for v in manifest['files'].values())
        results['avoided_duplicate_csv_bytes']=sum((saved_run/'recording'/v).stat().st_size for v in manifest['files'].values())
        moved=root/'relocated';saved.rename(moved);moved_run=moved/run.relative_to(workspace)
        checks['relocated_manifest_resolves']=all((moved_run/'recording'/v).is_file() for v in manifest['files'].values())
        out=export_readback(moved_run,root/'selected_readback.csv',start_s=0.,end_s=1.,joints=[0])
        checks['readback_is_explicit']=out['rows']>0
        report=evidence(moved,root/'analysis')
        checks['analysis_consumes_single_native_record']=len(report['records'])==1
        monitor.limit=monitor.used+1;monitor.check();wait(lambda:pause.exists());time.sleep(.5)
        files=list(workspace.glob('**/chunks/*.gz'))+list(workspace.glob('**/packets.jsonl'))+list(workspace.glob('**/cdr.bin'))+[run/'capture/execution/commands.csv',run/'capture/execution/feedback.csv']
        sizes={p:p.stat().st_size for p in files};before=fixture.commands;received=latest['receive_ns'];time.sleep(1.)
        checks['quota_stops_all_manual_data_appends']=all(p.stat().st_size==n for p,n in sizes.items())
        checks['quota_preserves_sending_and_live_feedback']=fixture.commands>before and latest['receive_ns']>received
    finally:
        stop.set()
        if sender:sender.join(timeout=20)
        observation_stop.set();observer.join(timeout=20);monitor.close();fixture.close()
    checks['workers_closed']=not observer.is_alive() and sender is not None and not sender.is_alive() and not failures
    checks['stop_after_quota_is_controlled']=results.get('control',{}).get('controlled_stop') is True
    report=dict(passed=all(checks.values()),checks=checks,results=results,errors=failures,
        scope='private localhost SDK Mock + fixed-state protocol fixture; no rendering or physical robot')
    (root/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
