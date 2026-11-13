"""H2 event durability, ungated collection, terminal faults and reference isolation."""
import csv
import json
from pathlib import Path
import shutil
import numpy as np
import pytest
import yaml
from casbot_calib import hardware_batch as batch
from casbot_calib import hardware_field as field
from casbot_calib.data import write_json
from casbot_calib.artifact_io import sha256
from test_hardware_field import make_inputs, make_capture

ROOT = Path(__file__).resolve().parents[1]


class FakeBackend(batch.SDKBatchBackend):
    """Synthetic records only, exercising the real H1 planner and inspectors."""
    def __init__(self, seed_snapshot, *, interrupt=False, fault=False, unstable=False, slow_hz=None):
        super().__init__({'/joint_states':'/casbot_calib_s2_unit/joint_states'})
        self.seed_snapshot = seed_snapshot; self.interrupt = interrupt; self.fault = fault
        self.unstable = unstable; self.slow_hz = slow_hz; self.snapshots = 0; self.executions = 0; self.clears = 0

    def snapshot(self, profile, output, domain_id):
        self.snapshots += 1; shutil.copytree(self.seed_snapshot, output)
        write_json(Path(output)/'request.json',dict(config=field.load_profile(profile),domain_id=domain_id))
        field.seal(output)
        return dict(passed=True)

    def execute(self, plan, output, domain_id):
        self.executions += 1; plan=Path(plan); output=Path(output); metadata=batch.read(plan/'plan.json')
        origin = make_capture(plan, metadata, output)
        delta = self.executions * 50_000_000_000
        path=output/'execution/feedback.csv'
        with path.open() as stream: rows=list(csv.DictReader(stream))
        for row in rows: row['receive_ns']=str(int(row['receive_ns'])+delta)
        if self.unstable and metadata['stage']=='gravity':
            for row in rows:
                at=(int(row['receive_ns'])-origin-delta)*1e-9
                if metadata['trials'][0]['measure_begin_s'] <= at < metadata['trials'][0]['measure_end_s']:
                    row['v0']='0.02'
            self.unstable=False
        with path.open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        period=metadata['command_period_ns']; count=round(metadata['duration_s']*1e9/period)+1
        ratio=.5 if self.slow_hz==metadata['config']['command_hz'] else 1.
        with (output/'execution/commands.csv').open('w',newline='') as stream:
            writer=csv.writer(stream);writer.writerow(['planned_index','planned_ns','publish_ns','phase'])
            writer.writerows([i,origin+delta+i*period,origin+delta+round(i*period/ratio),'stream'] for i in range(count))
        write_json(output/'request.json',dict(config=metadata['config'],executor_sha256=metadata['executor_sha256'],
            source_plan_csv_sha256=sha256(plan/'position_plan.csv')))
        result=dict(passed=True)
        if self.interrupt:
            self.interrupt=False;result=dict(passed=False,supervisor_interruption='user_interrupt',fault='interrupted')
        elif self.fault:
            result=dict(passed=False,fault='stale_feedback',reason='stop_unconfirmed')
        write_json(output/'result.json',result);field.seal(output)
        return result

    def clear_fault(self, reason):
        self.clears += 1
        return super().clear_fault(reason)


@pytest.fixture
def fixture(tmp_path, monkeypatch):
    preset,snapshot=make_inputs(tmp_path)
    profile=yaml.safe_load((ROOT/'configs/hardware_field.yaml').read_text())
    profile.update(collision_scene=str(ROOT/'configs/collision_scene.yaml'),anchor_transition_s=.1,initial_hold_s=.05,final_hold_s=.05)
    profile_path=tmp_path/'field.yaml';profile_path.write_text(yaml.safe_dump(profile))
    config=yaml.safe_load((ROOT/'configs/hardware_batch.yaml').read_text())
    config['direction'].update(joints=['left_shoulder_pitch_joint'],amplitude_deg=.2)
    config['frequency'].update(joints=['left_shoulder_pitch_joint'],repeats=1,amplitude_deg=.2)
    config['inter_job_pause_s']=0
    path=tmp_path/'batch.yaml';path.write_text(yaml.safe_dump(config))
    binary=tmp_path/'executor';binary.write_text('unit-test-only')
    monkeypatch.setattr(field,'executor_binary',lambda:(binary,dict(test=True)))
    return dict(root=tmp_path,config=config,path=path,profile=profile_path,preset=preset,snapshot=snapshot)


def create(fixture, *, change=None, name='session'):
    c=fixture['config'].copy()
    if change:
        import copy
        c=copy.deepcopy(c);change(c)
    fixture['path'].write_text(yaml.safe_dump(c))
    root=fixture['root']/name
    batch.plan_batch(fixture['path'],fixture['profile'],root,preset=fixture['preset'],domain_id=21,connection_scope='localhost_contract_mock')
    return root


def test_frozen_randomized_tiers_and_pose_roles(fixture):
    root=create(fixture);_,p=batch.load_batch(root)
    f=[j for j in p['jobs'] if j['kind']=='frequency']
    assert {r['hz'] for r in f}=={100,250,500}
    assert [r['hz'] for r in f] != [100,250,500]
    static=[j for j in p['jobs'] if j['kind']=='static']
    assert {r['split'] for r in static}=={'train','validation'}
    assert len({r['candidate_id'] for r in static})==2
    assert batch.read(root/'latest.json')['report'].startswith('reports/')


def test_batch_runs_all_jobs_without_review_and_keeps_shared_reference(fixture):
    root=create(fixture);backend=FakeBackend(fixture['snapshot'])
    result=batch.run_batch(root,backend=backend)
    assert result['passed'] and result['collection_complete'] and result['gravity']['available']
    assert result['schema_version']=='casbot_hardware_batch_report_v2'
    assert 'direction_review_complete' not in result
    assert not result['formal_hardware_calibration_accepted']
    assert len({r['comparison_reference_sha256'] for r in result['frequency']})==1
    assert backend.executions==6 and backend.snapshots==6 and backend.clears==0


def test_unstable_static_data_is_recorded_once_without_retry(fixture):
    root=create(fixture);backend=FakeBackend(fixture['snapshot'],unstable=True)
    result=batch.run_batch(root,backend=backend)
    assert result['passed'] and backend.executions==6
    first=root/'attempts/static_000/attempt_000'
    assert batch.read(first/'outcome.json')['status']=='recorded'
    assert not (first.parent/'attempt_001').exists()
    assert any(w['maximum_received_speed_rad_s']>=.02 for w in batch.read(first/'inspection/static_windows.json'))
    assert backend.clears==0


def test_controlled_interruption_resumes_new_attempt_not_original_files(fixture):
    root=create(fixture)
    backend=FakeBackend(fixture['snapshot'],interrupt=True)
    result=batch.run_batch(root,backend=backend)
    assert result['state']=='interrupted' and backend.executions==1
    original=sha256(root/'attempts/direction_00/attempt_000/manifest.json')
    result=batch.run_batch(root,resume=True,backend=backend)
    assert result['passed'] and backend.clears==0
    assert original==sha256(root/'attempts/direction_00/attempt_000/manifest.json')
    assert (root/'attempts/direction_00/attempt_001/manifest.json').is_file()


def test_fault_stub_logs_and_terminates_whole_session_without_resume(fixture,caplog):
    root=create(fixture);backend=FakeBackend(fixture['snapshot'],fault=True)
    result=batch.run_batch(root,backend=backend)
    assert result['state']=='faulted' and not result['passed']
    assert backend.executions==backend.snapshots==backend.clears==1
    assert '终止整批采集' in caplog.text
    records=list((root/'faults').glob('*/clear_fault.json'))
    assert len(records)==1 and not batch.read(records[0])['performed']
    with pytest.raises(RuntimeError,match='cannot resume'):batch.run_batch(root,resume=True,backend=backend)
    assert backend.executions==backend.clears==1


def test_start_change_rebuilds_only_current_job_without_spending_quality_budget(fixture):
    from test_hardware_start_replan import rejection
    root=create(fixture)
    class DriftingBackend(FakeBackend):
        def snapshot(self,profile,output,domain_id):
            result=super().snapshot(profile,output,domain_id)
            path=Path(output)/'execution/snapshot.csv'
            with path.open() as stream:rows=list(csv.DictReader(stream))
            rows[0]['q0']=str(float(rows[0]['q0'])+.006*self.snapshots)
            with path.open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
            field.seal(output);return result
        def execute(self,plan,output,domain_id):
            result=super().execute(plan,output,domain_id)
            if self.executions<=2:
                result=rejection();write_json(Path(output)/'result.json',result)
                (Path(output)/'execution/commands.csv').write_text('planned_index,planned_ns,publish_ns,phase\n')
                field.seal(output)
            return result
    backend=DriftingBackend(fixture['snapshot'])
    result=batch.run_batch(root,backend=backend,max_jobs=1)
    assert result['state']=='checkpoint' and backend.executions==backend.snapshots==3 and backend.clears==0
    rows,_=batch.outcomes(root,batch.load_batch(root)[1])
    assert [r['status'] for r in rows['direction_00']]==['start_replan','start_replan','recorded']
    parents=[root/r['attempt'] for r in rows['direction_00']]
    starts=[batch.read(p/'plan/plan.json')['observed_start_rad'][0] for p in parents]
    np.testing.assert_allclose(np.diff(starts),.006)
    assert not any(e['event']=='terminal_fault' for e in batch.events(root))
    for p in parents:field.check_seal(p)


def test_start_change_after_control_is_not_automatically_retried(fixture):
    from test_hardware_start_replan import rejection
    root=create(fixture)
    class UnsafeRetryBackend(FakeBackend):
        def execute(self,plan,output,domain_id):
            super().execute(plan,output,domain_id)
            result=rejection(mode_requested=True)
            write_json(Path(output)/'result.json',result);field.seal(output);return result
    backend=UnsafeRetryBackend(fixture['snapshot'])
    result=batch.run_batch(root,backend=backend)
    assert result['state']=='faulted' and backend.executions==backend.clears==1


def test_missing_incomplete_outcome_is_terminal_and_retained(fixture):
    root=create(fixture);backend=FakeBackend(fixture['snapshot'])
    target=root/'attempts/direction_00/attempt_000';target.mkdir(parents=True)
    (target/'partial.csv').write_text('retained')
    batch.event(root,'attempt_started',job_id='direction_00',attempt=str(target.relative_to(root)))
    report=batch.run_batch(root,resume=True,backend=backend)
    assert report['state']=='faulted' and backend.snapshots==0 and backend.clears==1
    assert (target/'partial.csv').read_text()=='retained'


def test_low_default_rate_is_measured_without_blocking_static(fixture):
    root=create(fixture);backend=FakeBackend(fixture['snapshot'],slow_hz=500)
    result=batch.run_batch(root,backend=backend)
    assert result['state']=='completed' and result['passed']
    assert all(r['status']=='recorded' for r in result['jobs'])
    assert any(r['measured_hz']<r['hz']*.9 for r in result['frequency'])
    assert backend.executions==6 and backend.clears==0


def test_resealed_protocol_and_changed_event_are_rejected_before_io(fixture):
    root=create(fixture);p=root/'protocol/protocol.json';record=batch.read(p);record['domain_id']=22
    write_json(p,record);field.seal(root/'protocol')
    with pytest.raises(ValueError,match='planning event'):batch.load_batch(root)


def test_concurrent_collection_lock_rejects_second_owner(fixture):
    root=create(fixture)
    with batch.batch_lock(root):
        with pytest.raises(RuntimeError,match='active'):
            with batch.batch_lock(root):pass


def test_review_api_and_config_requirements_are_removed(fixture):
    assert not hasattr(batch,'write_direction_template') and not hasattr(batch,'import_direction_review')
    c=batch.settings(fixture['path'],['left_shoulder_pitch_joint'])
    assert 'require_review' not in c['direction'] and 'quality_retries' not in c['static']
    assert 'minimum_publish_ratio' not in c['frequency']


def test_checkpoint_skips_completed_capture_and_read_only_reanalysis(fixture):
    root=create(fixture)
    backend=FakeBackend(fixture['snapshot'])
    result=batch.run_batch(root,backend=backend,max_jobs=2);assert result['state']=='checkpoint'
    completed=backend.executions
    with pytest.raises(FileExistsError):batch.run_batch(root,backend=backend)
    report=batch.analyse_batch(root,fixture['root']/'readonly')
    assert not report['collection_complete'] and backend.executions==completed
    assert batch.run_batch(root,resume=True,backend=backend)['passed']


def test_analysis_unavailable_does_not_retry_lock_or_stop_batch(fixture):
    root=create(fixture)
    class MissingAnalysis(FakeBackend):
        def inspect(self,plan,capture,output,*,static=False):
            if static:raise ValueError('No numerical observations available')
            result=super().inspect(plan,capture,output,static=static)
            result['passed']=False
            return result
    backend=MissingAnalysis(fixture['snapshot']);report=batch.run_batch(root,backend=backend)
    assert report['state']=='completed' and report['passed'] and report['collection_complete']
    assert not report['gravity']['available'] and backend.clears==0
    assert backend.executions==6
    history,_=batch.outcomes(root,batch.load_batch(root)[1])
    assert all(len(rows)==1 and rows[0]['status']=='recorded' for rows in history.values())
    assert not any(e['event']=='terminal_fault' for e in batch.events(root))
    assert batch.run_batch(root,resume=True,backend=backend)['passed']
    assert backend.executions==6


def test_replay_uses_delivered_pose_and_holds_between_attempts(fixture):
    from casbot_calib.hardware_batch_viewer import BatchReplaySource
    root=create(fixture)
    backend=FakeBackend(fixture['snapshot'])
    batch.run_batch(root,backend=backend,max_jobs=2)
    before={str(p.relative_to(root)):sha256(p) for p in (root/'attempts').rglob('manifest.json')}
    source=BatchReplaySource(root,max_seconds_per_attempt=.3)
    segment=source.segments[0]
    data=field.read_feedback(root/segment['outcome']['attempt']/'capture/execution/feedback.csv')
    at=123456789
    target=segment['first_ns']+round(at*segment['time_scale'])
    index=np.searchsorted(data['receive_ns'],target,side='right')-1
    frame=source.read(at)
    np.testing.assert_array_equal(frame.q,data['q'][index])
    assert frame.receive_ns==int(data['receive_ns'][index])
    gap_frame=source.read(segment['end']+100000000)
    np.testing.assert_array_equal(gap_frame.q,data['q'][-1])
    assert source.is_stale() and '不重建' in source.error
    following=source.read(source.segments[1]['start'])
    next_data=field.read_feedback(root/source.segments[1]['outcome']['attempt']/'capture/execution/feedback.csv')
    np.testing.assert_array_equal(following.q,next_data['q'][0])
    assert following.sample_ns==-1
    assert before=={str(p.relative_to(root)):sha256(p) for p in (root/'attempts').rglob('manifest.json')}


def test_execution_or_geometry_contract_error_still_ends_batch(fixture):
    root=create(fixture)
    class BrokenContract(FakeBackend):
        def inspect(self,*args,**kwargs):raise field.FieldContractError('Executed a different frozen command')
    backend=BrokenContract(fixture['snapshot']);report=batch.run_batch(root,backend=backend)
    assert report['state']=='faulted' and backend.executions==backend.clears==1
    with pytest.raises(RuntimeError,match='cannot resume'):batch.run_batch(root,resume=True,backend=backend)


def test_processed_report_does_not_claim_skipped_model_dependent_job_was_collected(fixture,monkeypatch):
    from casbot_calib import hardware_inertia as inertia
    root=create(fixture);_,protocol=batch.load_batch(root)
    protocol['jobs']=[dict(protocol['jobs'][0],kind='control_validation',feedforward='identified',gain_strategy='original')]
    protocol['config']['static']['enabled']=False
    monkeypatch.setattr(batch,'load_batch',lambda *a,**k:(root,protocol))
    monkeypatch.setattr(inertia,'freeze_inertia',lambda *a:False)
    backend=FakeBackend(fixture['snapshot']);report=batch.run_batch(root,backend=backend)
    assert report['passed'] and report['workflow_processed'] and report['state']=='completed'
    assert not report['collection_complete'] and report['jobs_not_run'][0]['status']=='not_run_missing_input'
    assert backend.snapshots==backend.executions==backend.clears==0
