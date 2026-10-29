import copy
from importlib.resources import files
import json
from pathlib import Path
import numpy as np
import pytest
import yaml
from casbot_calib.dynamics_data import import_dynamics,load_dynamics
from casbot_calib.dynamics_qualification import qualify_dynamics,qualified_observations
from casbot_calib.dynamics_problem import fourier_projection,build_periodic_problem
from casbot_calib.vendor_identity import sha256


def write(path,obj):Path(path).write_text(json.dumps(obj)+'\n')


@pytest.fixture
def fixture(tmp_path):
    """Analytic sensor/time contract fixture, not a new robot mechanics model."""
    n=56001;t=np.arange(n,dtype=np.int64)*250000;seconds=t*1e-9
    source=json.loads(files('casbot_calib').joinpath('assets/casbot_dual_arm_source.json').read_text())
    names=[side+'_'+axis+'_joint' for side in ('left','right') for axis in
           ('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow_pitch','wrist_yaw','wrist_pitch','wrist_roll')]
    home=np.array([0,.45,0,-.9,0,0,0,0,-.45,0,-.9,0,0,0])
    raw=dict(q=home+.02*np.sin(2*np.pi*seconds[:,None]/6)*np.ones((1,14)),
             dq=.02*2*np.pi/6*np.cos(2*np.pi*seconds[:,None]/6)*np.ones((1,14)),
             current=.3*np.sin(2*np.pi*seconds[:,None]/6)*np.ones((1,14)),sample_ns=t,receive_ns=t+1234,
             sequence=np.arange(n,dtype=np.int64),status=np.zeros((n,14),dtype=np.uint32))
    contract=dict(schema_version='casbot_observation_contract_v1',scope='synthetic_observations',joint_names=names,
                  model_id='nominal_fixture',identity=dict(dataset_id='test',model_sha256=source['sha256'],
                      robot_serial='synthetic',firmware_version='fixture_v1',motion_software_version='fixture_v1',provenance_reference='analytic sensor fixture'),
                  fields=dict(position='q',velocity='dq',signal='current',receive='receive_ns',device_sequence='sequence',status='status'),
                  position_unit='rad',velocity_unit='rad/s',
                  position_time=dict(field='sample_ns',role='acquisition',clock_id='acq'),
                  signal_time=dict(field='sample_ns',role='acquisition',clock_id='acq'),
                  signal=dict(origin='motor_current',unit='A',side='output_equivalent',includes='total_actuator_output',
                              gain_nm_per_raw_unit=[2.]*14,zero_raw=[0.]*14,filter=dict(kind='none'),raw_acquisition=True,sensor_identity='test_adc'),
                  experiment=dict(kind='periodic',begin_ns=0,end_ns=int(t[-1]),period_s=6.,discard_periods=0,harmonics=6,phase_points=128))
    delta=np.array([-1.,-.5,.5,1.])[:,None]*np.ones((1,14))
    scale=dict(schema_version='casbot_scale_validation_v1',scope='synthetic_observations',joint_names=names,model_id='nominal_fixture',
               device_identity=dict(robot_serial='synthetic',firmware_version='fixture_v1'),sensor_identity='test_adc',
               reference_basis='synthetic_reference',raw_delta=delta.tolist(),reference_delta_nm=(2*delta).tolist(),
               reference_zero_raw=[0.]*14,zero_uncertainty_raw=[0.]*14)
    return raw,contract,scale


def imported(tmp_path,fixture,*,change=None,raw_change=None,scale_change=None,mapping=None):
    raw,c,s=copy.deepcopy(fixture)
    if change:change(c)
    if raw_change:raw_change(raw)
    if scale_change:scale_change(s)
    path=tmp_path/'raw.npz';np.savez_compressed(path,**raw);c['source_sha256']=sha256(path)
    write(tmp_path/'contract.json',c);write(tmp_path/'scale.json',s)
    p=dict(schema_version='casbot_vendor_s3_profile_v1',scope=c['scope'],requested_band_hz=1.,
           contract=dict(path='contract.json',sha256=sha256(tmp_path/'contract.json')),
           scale_validation=dict(path='scale.json',sha256=sha256(tmp_path/'scale.json')),clock_mapping=None)
    if mapping is not None:
        write(tmp_path/'mapping.json',mapping);p['clock_mapping']=dict(path='mapping.json',sha256=sha256(tmp_path/'mapping.json'))
    (tmp_path/'profile.yaml').write_text(yaml.safe_dump(p))
    import_dynamics(path,tmp_path/'profile.yaml',tmp_path/'canonical',source_type='host')
    return qualify_dynamics(tmp_path/'canonical',tmp_path/'qualification')


def test_4k_current_is_one_torque_channel_with_no_extra_gear_factor(tmp_path,fixture):
    report=imported(tmp_path,fixture)
    assert report['stage_data_eligibility']['inertia']['observations_eligible']
    assert report['torque_observation_channel_count']==1 and report['independent_torque_sensor_count']==0
    meta,data=load_dynamics(tmp_path/'canonical')
    np.testing.assert_allclose(data['torque_nm'],fixture[0]['current']*2)
    assert meta['conversion']['reducer_applied'] is False and not report['hardware_executable']


@pytest.mark.parametrize('case', ['echo','unknown_units','motor_side','compensated','double_gain','unknown_filter','publish_time','unmapped_clock'])
def test_unknown_or_misleading_provenance_blocks_dynamic_use(tmp_path,fixture,case):
    def change(c):
        if case=='echo':c['signal']['origin']='command_echo'
        if case=='unknown_units':c['signal']['unit']='unknown'
        if case=='motor_side':c['signal']['side']='motor'
        if case=='compensated':c['signal']['includes']='command_without_compensation'
        if case=='double_gain':c['signal']['gain_nm_per_raw_unit']=[4.]*14
        if case=='unknown_filter':c['signal']['filter']['kind']='unknown'
        if case=='publish_time':c['signal_time']['role']='publish'
        if case=='unmapped_clock':c['position_time']['clock_id']='other_clock'
    report=imported(tmp_path,fixture,change=change)
    assert not report['stage_data_eligibility']['inertia']['observations_eligible']
    with pytest.raises(ValueError,match='gate blocks'):qualified_observations(tmp_path/'qualification','inertia')


def test_explicit_second_reducer_factor_is_rejected_before_import(tmp_path,fixture):
    with pytest.raises(ValueError,match='second reducer'):
        imported(tmp_path,fixture,change=lambda c:c['signal'].update(gear_ratio=100))


@pytest.mark.parametrize('case',['no_reference','same_current_reference','bad_zero','sensor_mismatch','firmware_mismatch'])
def test_scale_requires_separate_bidirectional_identity_bound_reference(tmp_path,fixture,case):
    def change(s):
        if case=='no_reference':s['reference_delta_nm']=[[0.]*14]*4
        if case=='same_current_reference':s['reference_basis']='derived_from_current'
        if case=='bad_zero':s['reference_zero_raw']=[.1]*14
        if case=='sensor_mismatch':s['sensor_identity']='another_motor'
        if case=='firmware_mismatch':s['device_identity']['firmware_version']='changed'
    report=imported(tmp_path,fixture,scale_change=change)
    assert not report['scale']['passed'] and not report['passed']


def test_synthetic_reference_cannot_become_hardware_qualification(tmp_path,fixture):
    report=imported(tmp_path,fixture,change=lambda c:c.update(scope='hardware_observations'),scale_change=lambda s:s.update(scope='hardware_observations'))
    assert 'synthetic_reference_cannot_qualify_hardware' in report['scale']['reasons'] and not report['passed']


def test_acquisition_rate_not_claimed_metadata_controls_dynamic_gate(tmp_path,fixture):
    def raw_change(raw):
        # An independently generated 500Hz timestamped sensor fixture.
        for key in list(raw):raw[key]=raw[key][::8].copy()
        raw['sequence']=np.arange(len(raw['sample_ns']),dtype=np.int64)
    report=imported(tmp_path,fixture,raw_change=raw_change,change=lambda c:c['signal'].update(declared_hz=4000))
    assert max(r['observed_acquisition_hz'] for r in report['segments'])==pytest.approx(500)
    assert not report['stage_data_eligibility']['inertia']['observations_eligible']


@pytest.mark.parametrize('bad',['missing','saturation','duplicate_time','sequence_skip'])
def test_bad_window_is_split_without_sorting_or_cross_gap_derivatives(tmp_path,fixture,bad):
    def mutate(raw):
        i=28000
        if bad=='missing':
            for key in list(raw):raw[key]=np.delete(raw[key],i,axis=0)
        if bad=='saturation':raw['status'][i]=1
        if bad=='duplicate_time':raw['sample_ns'][i]=raw['sample_ns'][i-1]
        if bad=='sequence_skip':raw['sequence'][i:]+=1
    report=imported(tmp_path,fixture,raw_change=mutate)
    assert len(report['segments'])>=2 and not report['stage_data_eligibility']['inertia']['observations_eligible']
    with pytest.raises(ValueError):build_periodic_problem(tmp_path/'qualification',tmp_path/'problem')


def test_no_device_sequence_is_invented_if_only_acquisition_times_exist(tmp_path,fixture):
    report=imported(tmp_path,fixture,change=lambda c:c['fields'].update(device_sequence=None))
    _,data=load_dynamics(tmp_path/'canonical')
    assert np.all(data['device_sequence']==-1) and not report['device_sequence_known']
    assert report['stage_data_eligibility']['inertia']['observations_eligible']


def test_effort_in_nm_does_not_require_fake_current_or_independent_force_sensor(tmp_path,fixture):
    def raw(raw):raw['output_torque']=2*raw.pop('current')
    def contract(c):
        c['fields']['signal']='output_torque';c['signal'].update(origin='output_torque_estimate',unit='Nm',gain_nm_per_raw_unit=[1.]*14)
    def scale(s):s['reference_delta_nm']=s['raw_delta']
    report=imported(tmp_path,fixture,raw_change=raw,change=contract,scale_change=scale)
    assert report['stage_data_eligibility']['inertia']['observations_eligible'] and report['independent_torque_sensor_count']==0
    _,data=load_dynamics(tmp_path/'canonical');assert 'current' not in data


def test_clock_mapping_subtracts_large_integer_origins_first(tmp_path,fixture):
    original=fixture[0]['sample_ns'];epoch=1_700_000_000_000_000_000
    encoded=epoch+original+original//10000
    mapping=dict(schema_version='casbot_clock_mapping_v1',scope='synthetic_observations',joint_names=fixture[1]['joint_names'],model_id='nominal_fixture',
                 source_clock='encoder',target_clock='acq',source_origin_ns=epoch,target_origin_ns=0,rate_ratio=1/1.0001,
                 uncertainty_ns=100,valid_source_range_ns=[int(encoded[0]),int(encoded[-1])],
                 validation_source_ns=encoded[[0,len(encoded)//2,-1]].tolist(),validation_target_ns=original[[0,len(encoded)//2,-1]].tolist())
    report=imported(tmp_path,fixture,raw_change=lambda r:r.update(encoder_ns=encoded),
                    change=lambda c:c.update(position_time=dict(field='encoder_ns',role='acquisition',clock_id='encoder')),mapping=mapping)
    assert report['time']['passed'] and report['time']['maximum_alignment_error_ns']<=101
    assert report['stage_data_eligibility']['inertia']['observations_eligible']


def test_fourier_analytic_derivatives_are_independent_oracle_at_large_epoch():
    epoch=1_700_000_000_000_000_000;time=epoch+np.arange(12000,dtype=np.int64)*1_000_000
    seconds=(time-epoch)*1e-9;omega=2*np.pi/6
    values=.03*np.sin(omega*seconds)[:,None]
    out,meta=fourier_projection(time,values,epoch,6.,3,128,True)
    for k in range(3):np.testing.assert_allclose(out[k][:,0],.03*omega**k*np.sin(omega*meta['phase_s']+k*np.pi/2),atol=1e-12)


def test_canonical_tampering_is_rejected_before_qualification(tmp_path,fixture):
    imported(tmp_path,fixture)
    with (tmp_path/'canonical/observations.npz').open('ab') as f:f.write(b'changed')
    with pytest.raises(ValueError,match='Canonical'):qualify_dynamics(tmp_path/'canonical',tmp_path/'another')
