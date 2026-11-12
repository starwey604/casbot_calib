#!/usr/bin/env python3
"""Nonvisual private ROS protocol checks: start refresh and 2s/3s receive gates."""
import argparse
from contextlib import contextmanager
import csv
import json
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace


def read(path):
    return json.loads(Path(path).read_text())


def rows(path):
    if not path.exists(): return []
    with path.open() as stream:
        return list(csv.DictReader(line for line in stream if line.endswith('\n')))


def wait_for(predicate, timeout=15.):
    deadline = time.monotonic()+timeout
    while not predicate():
        if time.monotonic()>deadline: raise TimeoutError('Private protocol check timed out')
        time.sleep(.02)


@contextmanager
def executing(config, output, domain, plan=None, **kwargs):
    from casbot_calib.hardware_field import run_executor
    cancel = threading.Event(); result = {}
    def run():
        try:
            result['value'] = run_executor(config, output, domain, plan=plan, timeout_s=35,
                cancel_check=lambda: 'operator_cancel' if cancel.is_set() else None, **kwargs)
        except BaseException as exc: result['error'] = exc
    thread = threading.Thread(target=run); thread.start()
    try: yield thread, result
    finally:
        cancel.set(); thread.join(timeout=20)
        if thread.is_alive(): raise RuntimeError('Owned executor did not finish cleanup')
        if 'error' in result: raise result['error']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain-id', type=int, required=True)
    args = parser.parse_args(); root = args.output.resolve(); root.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1')
    from verify_fieldwork_round1 import ProtocolFixture
    from casbot_calib import hardware_field as field
    from casbot_calib.staged_motion import Motion
    fixture = ProtocolFixture(root/'fixture', args.domain_id)
    fixture.velocity = [0.]*14; fixture.enable_service()
    checks = {}; gates = {}
    try:
        config = field.load_profile(fixture.client_profile)
        checks['defaults_2s_3s'] = (config['max_feedback_age_ns'],config['max_mode_age_ns']) == (2_000_000_000,3_000_000_000)
        field.run_executor(config, root/'snapshot', args.domain_id, snapshot=True)
        original = root/'jog_plan'
        field.prepare_jog(fixture.client_profile, root/'snapshot', original, fixture.names[0], .05)
        fixture.q[0] += .012  # Test-only fixed-state fixture; no robot/plant movement.
        output = root/'refreshed_capture'; result = field.execute_field(original, output, args.domain_id)
        rejected = read(output/'start_replans/attempt_000/capture/execution/result.json')
        effective_plan, effective_capture = field.resolve_field_capture(original, output)
        actual = read(effective_plan/'plan.json'); native = read(effective_capture/'execution/result.json')
        checks.update(start_drift_rejected_before_control=field.start_replan_required(rejected),
            no_old_start_commands=rejected['published_commands']==0 and not rejected['mode_requested'],
            reread_and_replan_success=result['passed'] and actual['observed_start_rad'][0]==fixture.q[0],
            only_successful_attempt_requested_mode=fixture.calls==2,
            first_command_matches_fresh_start=abs(float(rows(effective_capture/'execution/commands.csv')[0]['q0'])-fixture.q[0])<1e-8,
            new_reference_certified=read(effective_plan/'collision_reference.json')['passed'] and read(effective_plan/'collision_command_interpolant.json')['passed'],
            successful_attempt_published=native['published_commands']>0)
        # A constant reference isolates freshness from tracking/physical dynamics.
        import numpy as np
        motion = Motion('receive_gate_hold','position',np.array(fixture.q)).add(12.,fixture.q,label='hold')
        hold = root/'hold_plan'; hold.mkdir()
        field.finish_plan(hold,config,motion,[],{})
        for channel, key, expected_ms, code in [('joint','max_feedback_age_ns',2000.,'stale_feedback'),
                                              ('mode','max_mode_age_ns',3000.,'stale_mode')]:
            output = root/(channel+'_silence')
            publisher = fixture.pub if channel=='joint' else fixture.mode
            def silence(value):
                if channel=='joint': fixture.pub=value
                else: fixture.mode=value
            with executing(config,output,args.domain_id,hold/'position_plan.csv') as (thread, future):
                try:
                    wait_for(lambda: any(r.get('state')=='STREAMING' for r in rows(output/'execution/events.csv')))
                    silence(SimpleNamespace(publish=lambda _: None)); count = fixture.commands
                    time.sleep(1.05)
                    checks[channel+'_1s_gap_does_not_stop'] = thread.is_alive() and fixture.commands>count
                    silence(publisher); time.sleep(.3)
                    silence(SimpleNamespace(publish=lambda _: None))
                    thread.join(timeout=10)
                    if thread.is_alive(): raise TimeoutError(channel+' gate did not terminate')
                finally: silence(publisher)
            result = future['value']; native = read(output/'execution/result.json')
            stop = next(int(r['receive_ns']) for r in rows(output/'execution/events.csv') if r['state']=='STOPPING')
            last = native['last_feedback_receive_ns' if channel=='joint' else 'last_mode_receive_ns']
            interval_ms = (stop-last)*1e-6
            checks[channel+'_real_silence_ends_control'] = result['fault']==code and not result['passed']
            checks[channel+'_actual_threshold'] = expected_ms<=interval_ms<expected_ms+250
            checks[channel+'_no_publish_after_stop'] = all(int(r['publish_ns'])<=stop for r in rows(output/'execution/commands.csv'))
            gates[channel] = dict(configured_ns=config[key],observed_gap_ms=interval_ms,result=result)
            time.sleep(.2)
    finally: fixture.close()
    report = dict(passed=all(checks.values()),checks=checks,gates=gates,
        scope='private localhost fixed-state ROS protocol fixture; no physical robot or dynamics qualification',visual_inspection=False)
    (root/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
