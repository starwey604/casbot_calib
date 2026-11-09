"""DVT2 P0 consumers share one source without reusing legacy final observations."""
import copy
import json
from pathlib import Path
import numpy as np
import pytest
import yaml
from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for,context_for,collision_for
from casbot_calib.staged_protocol import prepare_protocol,load_protocol,build_split_manifest
from casbot_calib.staged_multifit import candidate_bundle
from casbot_calib.staged_control import load_candidate

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def config():return yaml.safe_load((ROOT/'configs/dvt2/staged_dual_arm_p0.yaml').read_text())


def test_p0_keeps_role_coverage_and_thresholds_under_new_experiment(config):
    legacy=yaml.safe_load((ROOT/'tests/fixtures/legacy_nominal/staged_dual_arm_p0.yaml').read_text())
    assert config['model_context']==context_for(config).identity
    assert config['p0']['experiment_version']!=legacy['p0']['experiment_version']
    for key in ('weighting','preprocessing','selection','acceptance','release'):
        assert config['p0'][key]==legacy['p0'][key]
    for key in ('dynamic_period_s','dynamic_periods','pose_amplitude_rad','control_period_ns','inertia_sample_period_ns'):
        assert config[key]==legacy[key]
    rows=build_split_manifest(config)['trials'];old=build_split_manifest(legacy)['trials']
    assert len(rows)==88 and sum(r['stage']=='inertia' for r in rows)==32
    assert not {r['acquisition_seed'] for r in rows}&{r['acquisition_seed'] for r in old}


def test_r0_seals_actual_model_and_never_calls_legacy_collision(config,tmp_path,monkeypatch):
    import casbot_calib.collision as collision
    def forbidden(*args,**kwargs):raise AssertionError('DVT2 consumer called legacy nominal collision')
    monkeypatch.setattr(collision,'nominal_profile',forbidden)
    root=tmp_path/'r0';r=prepare_protocol(root,config)
    protocol,public,_=load_protocol(root)
    assert r['contract_passed'] and not r['passed']
    assert protocol['model']['model_context']==public['model_context']==config['model_context']
    assert protocol['model']['sha256']==model_for(config)[1]['sha256']
    assert json.loads((root/'session.json').read_text())['collision']['model_context']==config['model_context']
    assert not (root/'runs').exists()


def test_p0_candidate_and_predecessors_reload_same_model(config,tmp_path):
    from casbot_calib.staged_development import predecessor_model
    e,source=model_for(config);theta=e.rigid_parameters();zero=np.zeros(14)
    bundle=candidate_bundle(theta,np.r_[zero,zero],zero,e,source,config,dict(identifiability='unit_fixture'),[])
    path=tmp_path/'candidate.json';write_json(path,bundle);restored=load_candidate(path)
    assert restored.model.model_id==e.model.model_id
    np.testing.assert_allclose(restored.mass_matrix(context_for(config).home),e.mass_matrix(context_for(config).home),atol=1e-13)
    bundle.pop('model_context');write_json(tmp_path/'untagged.json',bundle)
    assert load_candidate(tmp_path/'untagged.json').model.model_id==e.model.model_id
    bundle['source_sha256']=json.loads((ROOT/'tests/fixtures/legacy_nominal/casbot_dual_arm_source.json').read_text())['sha256'];write_json(tmp_path/'old.json',bundle)
    with pytest.raises(ValueError,match='source/order mismatch'):load_candidate(tmp_path/'old.json')
    for stage,payload in [('gravity',dict(fit=dict(parameters=theta.tolist()))),('friction',dict(viscous=zero.tolist(),coulomb=zero.tolist())),('breakaway',dict(static_friction=zero.tolist()))]:
        (tmp_path/stage).mkdir();write_json(tmp_path/stage/'report.json',payload)
    assert predecessor_model(tmp_path,config).model.model_id==e.model.model_id


def test_r4_control_plans_use_dvt2_and_preserve_all_nine_references(config,tmp_path):
    from casbot_calib.staged_final import prepare_control
    from casbot_calib.staged_collection import load_trial
    scene=yaml.safe_load((ROOT/config['collision_scene']).read_text());profile=collision_for(config,scene=scene)
    root=tmp_path/'control';prepare_control(root,config,profile)
    session=json.loads((root/'session.json').read_text());plan=json.loads((root/'plan_report.json').read_text())
    assert session['config']['schema_version']==1 and 'p0' not in session['config']
    assert session['config']['model_context']==config['model_context']
    assert len(plan['selected_trials'])==9
    for name in plan['selected_trials']:
        meta,curve=load_trial(root,name)
        np.testing.assert_allclose(curve(0),config['home_rad'],atol=1e-14)
        assert meta['stage'] in ('control_tuning','control_validation')


def test_development_reference_checks_model_before_reading_observations(config,tmp_path):
    from casbot_calib.staged_multiconfiguration import bind_development_reference
    old=yaml.safe_load((ROOT/'tests/fixtures/legacy_nominal/staged_dual_arm.yaml').read_text());source=tmp_path/'old';source.mkdir()
    write_json(source/'session.json',dict(config=old,collision={}))
    with pytest.raises(ValueError,match='contexts differ'):
        bind_development_reference(tmp_path/'new',source,config,None)


def test_dvt2_p0_does_not_fall_back_when_context_is_missing(config,tmp_path):
    from casbot_calib.staged_plan import plan_session
    changed=copy.deepcopy(config);changed.pop('model_context')
    for entry in (prepare_protocol,plan_session):
        with pytest.raises(ValueError,match='explicit model context'):entry(tmp_path/'missing',changed)
    assert not (tmp_path/'missing').exists()


def test_recovery_damping_does_not_change_ramp_or_guard_thresholds(config):
    from casbot_calib.staged_multiconfiguration import final_prior_recipes
    from casbot_calib.staged_breakaway import ramp_policy
    e,_=model_for(config);m=next(m for m in final_prior_recipes(config) if m.stage=='breakaway')
    base=copy.deepcopy(config);base.pop('breakaway_restore_kd_multiplier')
    old,ga=ramp_policy(m.metadata(),m.polynomial(),e,base)
    new,gb=ramp_policy(m.metadata(),m.polynomial(),e,config)
    assert gb.restore_kd==2*ga.restore_kd and gb.restore_kp==ga.restore_kp
    for key in ('sustain_ns','displacement_threshold','speed_threshold','maximum_displacement','maximum_speed','max_sample_age_ns'):
        assert getattr(ga,key)==getattr(gb,key)
    t=np.linspace(0,m.duration,60);commands=np.zeros((len(t),14,5))
    np.testing.assert_array_equal(old(t,commands.copy(),m.metadata()),new(t,commands.copy(),m.metadata()))
    bad=copy.deepcopy(config);bad['breakaway_restore_kd_multiplier']=0
    with pytest.raises(ValueError,match='Positive breakaway recovery'):ramp_policy(m.metadata(),m.polynomial(),e,bad)


def test_damped_probe_is_zero_stiffness_and_does_not_add_to_observed_torque(config):
    from casbot_calib.staged_multiconfiguration import final_prior_recipes
    from casbot_calib.staged_breakaway import ramp_policy
    e,_=model_for(config);m=next(m for m in final_prior_recipes(config) if m.stage=='breakaway')
    c=m.metadata()['conditions'];assert c['probe_kp']==0 and c['probe_kd_nm_s_rad']==.25
    new,guard=ramp_policy(m.metadata(),m.polynomial(),e,config)
    legacy=copy.deepcopy(config);legacy.pop('breakaway_probe_kd_nm_s_rad')
    old,_=ramp_policy(m.metadata(),m.polynomial(),e,legacy)
    begin=m.windows('static_dwell')[0][0];end=m.windows('torque_ramp')[0][1]
    t=np.array([0.,(begin+end)/2,m.duration]);commands=np.zeros((3,14,5));commands[:,:,3:]=1.
    a=old(t,commands.copy(),m.metadata());b=new(t,commands.copy(),m.metadata());j=c['joint']
    np.testing.assert_array_equal(a[:,:,:4],b[:,:,:4]);assert b[1,j,3]==0 and b[1,j,4]==.25 and a[1,j,4]==0
    np.testing.assert_array_equal(a[[0,2]],b[[0,2]])


def test_predecessor_probe_controller_must_match_new_protocol(config,tmp_path):
    from casbot_calib.staged_multiconfiguration import bind_development_reference
    scene=yaml.safe_load((ROOT/config['collision_scene']).read_text());profile=collision_for(config,scene=scene)
    old=copy.deepcopy(config);old.pop('p0');old['schema_version']=1;old['breakaway_probe_kd_nm_s_rad']=0.
    source=tmp_path/'different_controller';source.mkdir();write_json(source/'session.json',dict(config=old,collision=profile.metadata))
    with pytest.raises(ValueError,match='acquisition/model conditions changed'):
        bind_development_reference(tmp_path/'new_protocol',source,config,profile)
