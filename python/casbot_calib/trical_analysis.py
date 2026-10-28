"""K2 independent observation reanalysis, without evaluation truth or Mock settings."""
from hashlib import sha256
import json
from pathlib import Path
import numpy as np
from .data import write_json
from .kinematics import nominal_kinematics,calibration
from . import _core
from .trical_data import load_observations
from .trical_workflow import stable_window,observed_profile
from .trical_fixture import fixture_profile
from .trical_motion import segment
from .collision import certify_polyline


def analyse(folder, output):
    folder=Path(folder);output=Path(output)
    if output.exists():raise FileExistsError(output)
    manifest=json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
    for name,expected in manifest['files'].items():
        parts=Path(name).parts
        if Path(name).is_absolute() or '..' in parts or 'evaluation' in parts:raise ValueError('Invalid raw manifest entry')
        if sha256((folder/name).read_bytes()).hexdigest()!=expected:raise ValueError('K2 immutable input digest mismatch')
    config=json.loads((folder/'profile.json').read_text(encoding='utf-8'))
    if 'mock' in config:raise ValueError('Public profile must not contain hidden Mock parameters')
    engine,_=nominal_kinematics(config);measurements=json.loads((folder/'measurements.json').read_text());motions=json.loads((folder/'motions.json').read_text())
    events=json.loads((folder/'events.json').read_text());failed=any(e['state']=='rejected' for e in events)
    with np.load(folder/'raw/commands.npz',allow_pickle=False) as raw:command_times=raw['send_ns'];commands=raw['commands']
    if len(command_times)>1 and not np.all(np.diff(command_times)==config['control_period_ns']):raise ValueError('Commands are not on declared common frequency grid')
    if commands.shape!=(len(command_times),14,5) or not np.isfinite(commands).all():raise ValueError('Invalid executed commands')
    windows=[]
    if measurements:
        arrays,_=load_observations(folder/'observations')
        with np.load(folder/'raw/row_decision_times.npz',allow_pickle=False) as raw:times=raw['time_ns']
        for measurement in measurements:
            found=stable_window(arrays,times,measurement['accepted_ns'],config,measurement['stable_after_ns'],measurement['frame'],measurement['ball_id'])
            if found is None or found['indices']!=measurement['window']['indices']:raise ValueError('Accepted measurement has no independently valid stable window')
            offset=_core.trical_offset(np.array(found['readings_m']),calibration(config))
            if np.max(abs(offset))>config['workflow']['centering_tolerance_m']:raise ValueError('Accepted measurement outside centering tolerance')
            if not np.allclose(found['q_rad'],measurement['window']['q_rad'],atol=1e-14,rtol=0):raise ValueError('Measurement q differs from raw observations')
            windows.append(dict(visit=measurement['visit'],max_offset_m=float(np.max(abs(offset))),samples=len(found['indices'])))
    if (folder/'raw/joint_packets.npz').exists():
        with np.load(folder/'raw/joint_packets.npz',allow_pickle=False) as raw:joints={k:raw[k] for k in raw.files}
    else:joints=dict(q=np.empty((0,14)),sample_ns=np.empty(0,dtype=np.int64),receive_ns=np.empty(0,dtype=np.int64))
    free=fixture_profile(engine,config);profiles={};gates=[]
    for motion in motions:
        if not motion['reference']['passed']:continue
        target=motion['target'];key=(target['frame'],target['ball_id'])
        contact=bool(motion['collision']['exact_contact_pairs'])
        if key not in profiles:profiles[key]=fixture_profile(engine,config,key,True)
        profile=profiles[key] if contact else free
        if profile.metadata['pair_clearances_sha256']!=motion['collision']['pair_clearances_sha256']:raise ValueError('Collision pair policy changed')
        if profile.metadata['geometry_sha256']!=motion['collision']['geometry_sha256']:raise ValueError('Collision scene identity changed')
        metadata=motion['motion'];start=np.array(metadata['states'][0][0]);end=np.array(metadata['states'][-1][0])
        _,reference=segment(profile,start,end,config,motion['label'],metadata['duration_s'])
        if 'end_ns' not in motion:
            gates.append(dict(label=motion['label'],reference=reference['passed'],observed=False));continue
        chosen=(joints['sample_ns']>=motion['start_ns'])&(joints['receive_ns']<=motion['end_ns'])
        actual=certify_polyline(observed_profile(profile,config['workflow']['geometry_allowance_rad']),joints['q'][chosen],joints['sample_ns'][chosen]*1e-9,
            clearance_m=config['workflow']['clearance_m'],max_gap_s=config['drive']['sample_period_ns']*1.5e-9)
        gates.append(dict(label=motion['label'],reference=reference['passed'],observed=actual['passed']))
    passed=not failed and bool(windows) and all(g['reference'] and g['observed'] for g in gates)
    result=dict(schema_version=1,passed=passed,rejected_session=failed,measurements=windows,gates=gates,truth_read=False,
        source_manifest_sha256=sha256((folder/'manifest.json').read_bytes()).hexdigest(),
        original_native_sha256=manifest['native_sha256'],analysis_native_sha256=sha256(Path(_core.__file__).read_bytes()).hexdigest(),
        configured_command_hz=1e9/config['control_period_ns'],measured_command_hz=float(1e9/np.median(np.diff(command_times))) if len(command_times)>1 else None,
        scope='K2 stable observations and collision interpolants; not geometry identification or absolute robot accuracy')
    output.mkdir(parents=True,exist_ok=False);write_json(output/'report.json',result);return result
