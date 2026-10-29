"""S4 frozen qualified-data protocol and sequential analysis, without device I/O."""
import copy
import json
from pathlib import Path
import numpy as np
import yaml
from .dynamics_data import read_json
from .dynamics_qualification import qualified_observations
from .dynamics_problem import periodic_observation_problem
from .vendor_identity import sha256
from .data import write_json

STAGES=('gravity','friction','breakaway','inertia')
WRIST_PROBE_ROLES={(j,d,b) for j in (4,11) for d in (-1,1) for b in ('train','validation')}


def require_wrist_probes(conditions):
    roles={(c['joint'],c['direction'],c['batch']) for c in conditions}
    if not WRIST_PROBE_ROLES<=roles:raise ValueError('Both wrists require signed training and independent validation probes')


def reference(path):
    path=Path(path).resolve()
    return dict(path=str(path),sha256=sha256(path))


def checked(ref):
    path=Path(ref['path'])
    if sha256(path)!=ref['sha256']:raise ValueError('S4 input changed: '+str(path))
    return path


def freeze_staged(spec_path,output):
    """Seal the declared roles before any fitting; no motion permissions."""
    spec_path=Path(spec_path).resolve();spec=yaml.safe_load(spec_path.read_text());base=spec_path.parent
    if spec.get('schema_version')!='casbot_vendor_s4_spec_v1':raise ValueError('Explicit S4 protocol schema required')
    if spec.get('scope') not in ('synthetic_observations','hardware_observations'):raise ValueError('Explicit observation scope required')
    settings=spec['settings']
    if 'mock' in settings or settings.get('mode')!='qualified_observations_only':raise ValueError('S4 analysis must not contain a plant or an execution mode')
    protocol=copy.deepcopy(spec);protocol['records']=[];seen=set();roles={}
    for key,ref in spec.get('preflight',{}).items():
        path=(base/ref['path']).resolve()
        if sha256(path)!=ref['sha256']:raise ValueError('Preflight evidence changed')
        protocol['preflight'][key]=reference(path)
    for entry in spec['records']:
        row=copy.deepcopy(entry);name=row['name'];stage=row['stage'];split=row['split']
        if name in ('.','..') or Path(name).name!=name or name in seen or stage not in (*STAGES,'inertia_probe') or split not in ('train','validation','sampling'):
            raise ValueError('Duplicate record or unknown stage/split')
        if not isinstance(row.get('time_origin_ns'),int) or row['time_origin_ns']<0:raise ValueError('Explicit nonnegative integer acquisition origin required')
        seen.add(name)
        for key in ('qualification','motion','guard','reference_collision','actual_collision'):
            if row.get(key) is None:continue
            old=row[key];path=(base/old['path']).resolve()
            if sha256(path)!=old['sha256']:raise ValueError('S4 evidence digest mismatch: '+key)
            row[key]=reference(path)
        motion=read_json(checked(row['motion']))
        if motion['name']!=name or motion['stage']!=stage:raise ValueError('S4 motion identity mismatch')
        conditions=motion['conditions']
        actual_split=('train' if conditions['repeat']==0 else 'validation') if stage=='breakaway' else conditions['batch']
        if stage=='inertia' and actual_split in ('left_validation','right_validation'):actual_split='validation'
        if actual_split!=split:raise ValueError('Role differs from independently recorded motion conditions')
        qroot=checked(row['qualification']).parent
        # Binding alone is allowed for rejected data, so inspection can explain
        # why a stage is blocked without fabricating missing capabilities.
        qreport=read_json(qroot/'report.json');source=Path(qreport['source'])
        metadata=read_json(source/'metadata.json')
        if metadata['scope']!=spec['scope']:raise ValueError('Synthetic and hardware records must not share a protocol')
        raw_contract=metadata.get('contract')
        if raw_contract:
            raw_id=raw_contract['source_sha256']
            if raw_id in roles:raise ValueError('Same raw capture reused across records/roles: '+roles[raw_id])
            roles[raw_id]=name
        row['source_kind']=row.get('source_kind','supplied_observations')
        protocol['records'].append(row)
    protocol['hardware_executable']=False
    protocol['protocol_note']='Data roles frozen before analysis. No hardware execution, new final test or physical semantics qualification implied.'
    protocol['spec_source']=reference(spec_path)
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    write_json(output/'protocol.json',protocol)
    (output/'protocol.sha256').write_text(sha256(output/'protocol.json')+'\n')
    return inspect_staged(output,output/'inspection')


class StagedObservations:
    def __init__(self,folder):
        self.root=Path(folder);path=self.root/'protocol.json'
        if sha256(path)!=(self.root/'protocol.sha256').read_text().strip():raise ValueError('Frozen S4 protocol changed')
        self.protocol=read_json(path);self.entries={r['name']:r for r in self.protocol['records']}
        self.config=copy.deepcopy(self.protocol['settings'])
        self.config['drive']['gain_status']='synthetic_declared' if self.protocol['scope']=='synthetic_observations' else 'independently_validated'
        self.config['drive']['gain_nm_a']=None  # total Nm observations, never a fake current gain
        self.identity=None
        for row in self.entries.values():
            for key in ('qualification','motion','guard','reference_collision','actual_collision'):
                if row.get(key):checked(row[key])
        self.preflight={k:read_json(checked(ref)) for k,ref in self.protocol.get('preflight',{}).items()}

    def load(self,name):
        row=self.entries[name];stage=row['stage'];qroot=checked(row['qualification']).parent
        metadata,data,report=qualified_observations(qroot,stage)
        if stage=='inertia_probe' and report['requested_band_hz']<30.:
            raise ValueError('S4 wrist-probe preprocessing requires a qualified 30Hz band')
        from .dual_model import nominal_dual_arm
        nominal,model=nominal_dual_arm()
        if metadata['contract']['identity']['model_sha256']!=model['sha256'] or metadata['joint_names']!=[j.name for j in nominal.model.joints]:
            raise ValueError('S4 requires matching nominal URDF and joint order')
        identity=metadata['contract']['identity']
        binding=(model['sha256'],identity['robot_serial'],identity['firmware_version'],identity['motion_software_version'])
        if self.identity is not None and self.identity!=binding:raise ValueError('Device/firmware/model changed between S4 records')
        self.identity=binding
        motion=read_json(checked(row['motion']))
        if row['stage']=='inertia':
            motion['conditions']['source_batch']=motion['conditions']['batch']
            motion['conditions']['batch']=row['split']
        for key in ('reference_collision','actual_collision'):
            if key not in row or not read_json(checked(row[key])).get('passed'):
                raise ValueError('S4 reference/observed path evidence missing or rejected: '+name)
        return row,motion,metadata,data,report

    def __call__(self,stage):
        for name,row in self.entries.items():
            if row['stage']!=stage:continue
            row,motion,metadata,data,report=self.load(name)
            time=data['aligned_position_time_ns']
            # These legacy non-periodic numerical kernels require an entire
            # uniform, synchronously acquired trial. Do not silently interpolate
            # clocks, drop invalid rows or synthesize device sequence numbers.
            if stage!='inertia':
                if len(report['segments'])!=1 or report['segments'][0]['samples']!=len(time):
                    raise ValueError('S4 whole-trial processing refuses invalid samples or gaps: '+name)
                if not np.array_equal(time,data['signal_time_ns']):raise ValueError('S4 nonperiodic kernels require synchronized acquisition: '+name)
                if np.any(np.diff(time)!=np.diff(time)[0]):raise ValueError('S4 nonperiodic kernels require uniform acquisition: '+name)
            if stage=='gravity':
                windows=[i for i,label in enumerate(motion['labels']) if label=='static_measure']
                if len(windows)!=1:raise ValueError('One explicit static window required')
                end=motion['knots_s'][windows[0]+1]
                active=(time-row['time_origin_ns']>(end-.5)*1e9)&(time-row['time_origin_ns']<(end-.05)*1e9)
                if not np.any(active) or np.max(abs(data['dq'][active]))>=.002 or np.max(np.ptp(data['q'][active],axis=0))>.0005:
                    raise ValueError('Declared gravity measurement window is not static: '+name)
            yield name,motion,dict(q=data['q'],dq=data['dq'],torque_nm=data['torque_nm'],
                sample_ns=time-row['time_origin_ns'],receive_ns=data['receive_ns'],
                sequence=data['device_sequence'],status=data['status_raw'])

    def guard(self,name):
        row=self.entries[name];guard=read_json(checked(row['guard']));origin=row['time_origin_ns']
        for event in guard['events']:
            for key in ('confirmed_sample_ns','decision_ns'):
                if key in event:event[key]-=origin
        return guard

    def periodic(self,name,static):
        row,motion,meta,data,report=self.load(name)
        windows=[i for i,label in enumerate(motion['labels']) if label=='periodic']
        exp=meta['contract'].get('experiment',{});origin=row['time_origin_ns']
        if (exp.get('begin_ns')!=origin+round(motion['knots_s'][windows[0]]*1e9) or
            exp.get('end_ns')!=origin+round(motion['knots_s'][windows[-1]+1]*1e9) or exp.get('harmonics')!=40 or
            exp.get('period_s')!=motion['conditions']['period_s']):raise ValueError('S4 periodic window/band differs from frozen motion')
        arrays,details=periodic_observation_problem(meta,data,report)
        basis=arrays['friction_basis'];fixed=basis[:,:,28:]@np.asarray(static)
        return dict(regression=arrays['regression'],extra=basis[:,:,:28],fixed_friction=fixed,rotor=arrays['rotor'],
                    target=arrays['target']-fixed,observed_torque_nm=arrays['torque_nm'],q=arrays['q'],dq=arrays['dq'],ddq=arrays['ddq'],
                    moving=abs(arrays['dq'])>.015,preprocessing=dict(frequency_band_hz=[0,details['fit_band_hz']],
                        source='S3 qualified output torque and Pinocchio; no current alias',harmonics=details['harmonics']))


def inspect_staged(folder,output):
    reader=StagedObservations(folder);results={};records={}
    for name,row in reader.entries.items():
        try:
            # Exercise the actual reader, not a supplied passed flag.
            reader.load(name)
            records[name]=dict(passed=True,stage=row['stage'],split=row['split'])
        except (ValueError,KeyError,FileNotFoundError) as exc:
            records[name]=dict(passed=False,stage=row['stage'],split=row['split'],reason=str(exc))
    for stage in STAGES:
        items=[r for r in records.values() if r['stage']==stage]
        splits={r['split'] for r in items if r['passed']}
        reasons=[]
        if not items:reasons.append('missing_stage_records')
        if any(not r['passed'] for r in items):reasons.append('unqualified_observations')
        if not {'train','validation'}<=splits:reasons.append('separate_training_and_validation_required')
        if stage=='friction' and 'sampling' not in splits:reasons.append('independent_denser_sampling_required')
        if stage=='inertia' and not any(r['stage']=='inertia_probe' and r['passed'] for r in records.values()):reasons.append('dedicated_wrist_probes_required')
        results[stage]=dict(data_ready=not reasons,reasons=reasons,execution_enabled=False)
    result=dict(schema_version='casbot_vendor_s4_inspection_v1',passed=all(r['data_ready'] for r in results.values()),stages=results,records=records,
        hardware_executable=False,protocol_sha256=sha256(Path(folder)/'protocol.json'),
        execution_blockers=['No real SDK torque/compensation capability or physical stop certification',
                            'Actual geometry, fixture/gravity transform and device constraints are unqualified',
                            'S2 localhost-only restriction remains; offline data readiness never authorizes motion'])
    output=Path(output);output.mkdir(parents=True,exist_ok=False);write_json(output/'report.json',result)
    return result


def analyse_staged(folder,output,through='inertia'):
    from .staged_identification import identify_gravity,identify_friction
    from .staged_breakaway import identify_breakaway
    from .staged_inertia import identify_inertia
    if through not in STAGES:raise ValueError('Unknown S4 stage')
    reader=StagedObservations(folder);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    from .dual_model import nominal_dual_arm
    _,model_source=nominal_dual_arm()
    completed={};report=dict(schema_version='casbot_vendor_s4_analysis_v1',passed=False,completed_through=None,
        stages=completed,scope=reader.protocol['scope'],hardware_executable=False,physical_semantics_verified_by_software=False,
        independent_final_test_passed=False,control_validated=False,protocol_sha256=sha256(Path(folder)/'protocol.json'),
        source_model_sha256=model_source['sha256'])
    try:
        if not all(reader.preflight.get(k,{}).get('passed') for k in ('mapping','timing')):
            raise ValueError('Qualified mapping and timing preflight evidence required before staged analysis')
        for stage in STAGES[:STAGES.index(through)+1]:
            # Materialize the stage once to validate every record BEFORE a fit;
            # avoid holding the raw matrices of all records in memory.
            count=0
            for _ in reader(stage):count+=1
            if not count:raise ValueError('Missing '+stage+' observations')
            rows=[(name,meta,obs) for name,meta,obs in reader(stage)] if stage=='gravity' else None
            if rows is not None:
                for split in ('train','validation'):
                    group=[r for r in rows if r[1]['conditions']['batch']==split]
                    if len(group)<4 or len(group)%2:raise ValueError('At least two opposite-approach gravity pose pairs per split required')
                    for a,b in zip(group[::2],group[1::2]):
                        ca,cb=a[1]['conditions'],b[1]['conditions']
                        if ca['pose']!=cb['pose'] or {ca['approach'],cb['approach']}!={-1,1}:raise ValueError('Gravity approach pair identity/order mismatch')
            if stage=='friction':
                primary={};dense={}
                for _,motion,obs in reader(stage):
                    c=motion['conditions'];dt=int(np.diff(obs['sample_ns'])[0]);j=c['joint']
                    if c['batch'] in ('train','validation'):primary[j]=max(dt,primary.get(j,0))
                    elif c['batch']=='sampling':dense[j]=max(dt,dense.get(j,0))
                if set(primary)!=set(range(14)) or set(dense)!=set(range(14)) or any(dense[j]>=primary[j] for j in primary):
                    raise ValueError('Independent denser raw acquisition required on every axis')
            if stage=='inertia':
                require_wrist_probes([motion['conditions'] for _,motion,_ in reader('inertia_probe')])
            if stage=='gravity':
                gravity,model=identify_gravity(folder,output/stage,records=reader,config=reader.config)
                value=gravity
            elif stage=='friction':
                value=identify_friction(folder,output/stage,model,records=reader,config=reader.config,sampling_policy='s3_raw_with_independent_rate_convergence');friction=value
            elif stage=='breakaway':
                value=identify_breakaway(folder,output/stage,model,records=reader,config=reader.config,guard_reader=reader.guard);onset=value
                write_json(output/'pre_inertia_acceptance.json',dict(passed=value['passed'],gravity=gravity['passed'],friction=friction['passed'],
                    gravity_report_sha256=sha256(output/'gravity/report.json'),friction_report_sha256=sha256(output/'friction/report.json'),
                    breakaway_report_sha256=sha256(output/'breakaway/report.json'),hardware_executable=False))
            else:
                value,candidate=identify_inertia(folder,output/stage,gravity,friction,onset,output/'gravity/gravity_constraints.npz',
                    records=reader,config=reader.config,periodic_builder=reader.periodic)
            completed[stage]=dict(passed=value['passed'],report=stage+'/report.json')
            report['completed_through']=stage
            if not value['passed']:raise ValueError(stage+' independent acceptance failed; following stage blocked')
        report['passed']=True
    except (ValueError,KeyError,RuntimeError,IndexError) as exc:
        report['blocked_reason']=str(exc)
    finally:
        write_json(output/'analysis.json',report)
        (output/'manifest.json').write_text(json.dumps({str(p.relative_to(output)):sha256(p) for p in sorted(output.rglob('*')) if p.is_file() and p.name!='manifest.json'},indent=2)+'\n')
    return report
