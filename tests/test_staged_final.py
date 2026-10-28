"""R4 active/held-arm semantics and frozen-test/controller authorization."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from casbot_calib.staged_residuals import frequency_budget,aggregate_benefit
from casbot_calib.staged_control import run_frozen_control
from casbot_calib.staged_final import final_prior_records,score_predecessors


def settings():
    return dict(minimum_speed_bin_rms_rad_s=.001,minimum_acceleration_bin_rms_rad_s2=.001,minimum_qualified_bins=1,
        band_non_degradation_factor=1.05,band_non_degradation_additive_nm=.002,minimum_inertia_band_improvement_fraction=.01)


@pytest.mark.parametrize('active',[list(range(7)),list(range(7,14)),list(range(14))])
def test_declared_active_arm_not_silently_deleted_from_frequency_acceptance(active):
    wave=np.sin(2*np.pi*3*np.arange(1024)/1024)[:,None]*np.ones((1,14));motion=np.zeros_like(wave);motion[:,active]=wave[:,active]
    observed=.02*wave;predictions=dict(A=.025*wave,B=observed,C=observed,D=observed)
    def make(signal):return frequency_budget(observed,predictions,signal,signal,signal,signal,[3],8,np.full((8,14),1e-4),settings(),period_s=1.,active_joints=active)
    good=make(motion);result=aggregate_benefit({'trial':good},settings())
    assert result['inertia_benefit_claim_supported'] and not result['missing_axis_evidence']
    held=sorted(set(range(14))-set(active))
    assert all(good['qualified_bins_per_axis'][j]==0 for j in held)
    motion[:,active[0]]=0.;bad=aggregate_benefit({'trial':make(motion)},settings())
    assert not bad['inertia_benefit_claim_supported']
    assert bad['missing_axis_evidence']==[dict(trial='trial',joint=active[0])]


@pytest.mark.parametrize('active',[[],[0,0],[-1],[14]])
def test_invalid_active_group_is_not_a_way_to_skip_validation(active):
    wave=np.ones((1024,14))
    with pytest.raises(ValueError,match='active joint'):
        frequency_budget(wave,{k:wave for k in 'ABCD'},wave,wave,wave,wave,[3],8,np.full((8,14),1e-4),settings(),period_s=1.,active_joints=active)


def test_final_reader_requires_freeze_before_opening_any_raw_data(tmp_path):
    with pytest.raises(FileNotFoundError):next(final_prior_records(tmp_path,'gravity'))
    assert not (tmp_path/'runs').exists()


def test_controller_adapter_requires_final_test_acceptance_before_acquisition(tmp_path,monkeypatch):
    import casbot_calib.staged_final as final
    import casbot_calib.staged_protocol as protocol
    from casbot_calib.data import write_json
    parent=tmp_path/'parent';analysis=tmp_path/'r4';parent.mkdir();analysis.mkdir()
    (parent/'candidate.json').write_text('{}');(analysis/'final_protocol.json').write_text('{}')
    monkeypatch.setattr(final,'load_final_protocol',lambda *a:({},{}))
    monkeypatch.setattr(protocol,'load_frozen_candidate',lambda *a:dict(candidate='candidate.json'))
    report=analysis/'report.json';write_json(report,dict(model_acceptance_passed=False))
    with pytest.raises(ValueError,match='accepted frozen'):run_frozen_control(tmp_path/'control',None,{},None,parent,report)
    assert not (tmp_path/'control').exists()


@pytest.mark.parametrize('fault',['none','gravity','friction','static'])
def test_final_predecessors_score_frozen_parameters_without_fitting(tmp_path,monkeypatch,fault):
    import casbot_calib.staged_final as final
    from casbot_calib.data import write_json
    joints=[SimpleNamespace(friction=SimpleNamespace(viscous=.02,coulomb=.03,static_friction=.08)) for _ in range(14)]
    model=SimpleNamespace(model=SimpleNamespace(joints=joints),gravity=lambda q:np.zeros(14))
    t=np.arange(1000)*.001;obs=dict(sample_ns=np.arange(1000,dtype=np.int64)*1000000,q=np.zeros((1000,14)),dq=np.zeros((1000,14)),current=np.zeros((1000,14)))
    if fault=='gravity':obs['current'][:]=.1
    def records(root,stage):
        if stage=='gravity':
            for sign in (-1,1):yield 'g',dict(knots_s=[0.,1.],labels=['static_measure'],conditions=dict(pose=0,approach=sign)),obs
        else:
            for j in range(14):yield f'{stage}{j}',dict(conditions=dict(joint=j,pose=0)),obs
    monkeypatch.setattr(final,'final_prior_records',records)
    def pairs(obs,meta,model,gain,period):
        x=np.array([[.04,1.],[.08,1.],[.14,1.]])
        return x,x@np.array([.02,.03])+(.1 if fault=='friction' else 0.),dict(joint=meta['conditions']['joint'])
    monkeypatch.setattr(final,'friction_pairs',pairs)
    monkeypatch.setattr(final,'onset_record',lambda name,meta,*args:dict(joint=meta['conditions']['joint'],estimate_nm=.18 if fault=='static' else .08,passed=True))
    original=final._read
    monkeypatch.setattr(final,'_read',lambda path:dict(events=[]) if Path(path).name=='guard_events.json' else original(path))
    (tmp_path/'models').mkdir();write_json(tmp_path/'models/prior_friction.json',dict(sampling_qualification=dict(passed=True)))
    cfg=dict(drive=dict(gain_nm_a=1.),control_period_ns=2000000)
    p=dict(acceptance=dict(torque_floor_nm=.2,gravity_absolute_rmse_nm=.08,gravity_pair_rmse_nm=.01,friction_absolute_rmse_nm=.02),
        static_onset_tolerance_nm=.01,expected_counts=dict(gravity=2,friction=14,breakaway=14))
    result=score_predecessors(tmp_path,tmp_path,model,cfg,p)
    assert result['passed']==(fault=='none')
    assert not result['gravity']['fit_performed'] and not result['friction']['fit_performed'] and not result['breakaway']['fit_performed']


@pytest.mark.parametrize('fault',['none','weak_wrist','friction'])
def test_synthetic_operator_gate_does_not_hide_one_weak_axis_in_frobenius_average(monkeypatch,fault):
    from casbot_calib.staged_final_validation import evaluate_frozen_operators
    import casbot_calib.staged_collection as collection
    monkeypatch.setattr(collection,'load_trial',lambda *a:(dict(knots_s=[0.,24.],labels=['periodic'],conditions=dict(period_s=6.)),lambda t:np.zeros((len(t),14))))
    def engine(changed):
        matrix=np.eye(14)
        if changed and fault=='weak_wrist':matrix[11,11]=1.09
        joints=[SimpleNamespace(friction=SimpleNamespace(viscous=.02,coulomb=.04,static_friction=.08)) for _ in range(14)]
        if changed and fault=='friction':joints[11].friction.coulomb=.05
        return SimpleNamespace(model=SimpleNamespace(joints=joints),evaluate_batch=lambda q,v,a:dict(mass_matrix=np.tile(matrix,(len(q),1,1))))
    policy=dict(query_references=['declared'],points_per_reference=128,maximum_matrix_diagonal_relative_error=.05,maximum_friction_relative_error=.05)
    result=evaluate_frozen_operators(Path('.'),engine(True),engine(False),policy)
    assert result['passed']==(fault=='none')
    if fault=='weak_wrist':assert result['maximum_matrix_relative_frobenius_error']<.05 and result['matrix_diagonal_maximum_relative_errors'][11]>.05
