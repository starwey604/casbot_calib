"""A corrected analysis is explicit and cannot overwrite the rejected R1 fit."""
import json
from pathlib import Path
import pytest
from casbot_calib.data import write_json
from casbot_calib.staged_protocol import _seal,file_hash


def test_rescore_checks_sealed_fit_before_loading_any_observations(tmp_path,monkeypatch):
    import casbot_calib.staged_multifit as module
    monkeypatch.setattr(module,'load_protocol',lambda root:({}, {}, {}))
    root=tmp_path/'root';root.mkdir();write_json(root/'split_manifest.json',dict(trials=[]));write_json(root/'motion_plan.json',dict(final_prior_trials=[]))
    write_json(root/'protocol.json',dict(scope='unit fixture only'))
    p=root/'candidate.json';write_json(p,dict(unit_fixture=1));expected=file_hash(p)
    _seal(root,'development_fit_freeze.json',dict(files={'candidate.json':expected}));write_json(p,dict(unit_fixture=2))
    with pytest.raises(ValueError,match='Frozen R1 fit changed'):module.rescore_development(root,tmp_path/'out')
    assert not (tmp_path/'out').exists()


def test_reanalysis_verification_rejects_a_different_parent_without_overwriting_checkpoint(tmp_path,monkeypatch):
    import casbot_calib.staged_development_validation as module
    monkeypatch.setattr(module,'load_protocol',lambda root:({}, {}, {}))
    root=tmp_path/'root';root.mkdir();write_json(root/'protocol.json',dict(scope='unit fixture only'))
    checkpoint=dict(round='R1',passed=False);write_json(root/'workflow.json',checkpoint)
    analysis=tmp_path/'analysis';analysis.mkdir();write_json(analysis/'analysis.json',dict(round='R1',passed=True,protocol_sha256='wrong'))
    _seal(analysis,'manifest.json',dict(files={'analysis.json':file_hash(analysis/'analysis.json')}))
    with pytest.raises(ValueError,match='another protocol'):module.verify_development(root,tmp_path/'out',analysis=analysis)
    assert json.loads((root/'workflow.json').read_text())==checkpoint and not (tmp_path/'out').exists()


def test_reanalysis_option_cannot_be_mixed_with_r2_or_legacy_validation(tmp_path):
    from casbot_calib.staged_validation import verify_staged
    root=tmp_path/'root';root.mkdir();write_json(root/'session.json',dict(config=dict(schema_version=2)))
    with pytest.raises(ValueError,match='exclusive P0 R1'):
        verify_staged(root,tmp_path/'out',stability_analysis='r2',development_analysis='r1')
    write_json(root/'session.json',dict(config=dict(schema_version=1)))
    with pytest.raises(ValueError,match='exclusive P0 R1'):
        verify_staged(root,tmp_path/'out',development_analysis='r1')


def test_rescore_reuses_parameters_and_cannot_call_mechanical_fit(tmp_path,monkeypatch):
    import numpy as np
    import casbot_calib.staged_multifit as module
    import casbot_calib.staged_development as development
    import casbot_calib.staged_multiconfiguration as configuration
    from casbot_calib.model_runtime import model_for
    from casbot_calib.staged_multifit import candidate_bundle
    # Unit fixture for provenance/control flow, not calibration evidence.
    root=tmp_path/'root';root.mkdir();config=dict(drive=dict(gain_status='synthetic_declared',gain_nm_a=2.))
    engine,source=model_for(config);bundle=candidate_bundle(engine.rigid_parameters(),np.zeros(28),np.zeros(14),engine,source,config,dict(identifiability='unit fixture'),['t'])
    write_json(root/'protocol.json',dict(scope='unit fixture'));write_json(root/'split_manifest.json',dict(trials=[]));write_json(root/'motion_plan.json',dict(final_prior_trials=[]))
    (root/'role_inputs').mkdir()
    for name in ('train','selection'):write_json(root/f'role_inputs/{name}.json',dict(scope='unit fixture'))
    folder=root/'identification_development';folder.mkdir()
    original=dict(protocol_sha256=file_hash(root/'protocol.json'),input_manifests={k:file_hash(root/f'role_inputs/{k}.json') for k in ('train','selection')},
        training_trials=['t'],selection_trials=['s'],prior_stage_acceptance=dict(passed=True),passed=False)
    write_json(folder/'report.json',original)
    for name in ('candidate.json','candidate_initial.json'):write_json(folder/name,bundle)
    _seal(root,'development_fit_freeze.json',dict(files={str(p.relative_to(root)):file_hash(p) for p in folder.iterdir()}))
    monkeypatch.setattr(module,'load_protocol',lambda root:({},config,{}))
    monkeypatch.setattr(development,'frozen_predecessors',lambda root:root)
    monkeypatch.setattr(configuration,'load_development_reference',lambda root:root)
    monkeypatch.setattr(module,'load_problems',lambda root,split,static:dict(t={}) if split=='train' else dict(s={}))
    monkeypatch.setattr(module,'score_problems',lambda *args:({},{}))
    monkeypatch.setattr(module,'development_summary',lambda *args:({'unit_qualification':True},{},{}))
    monkeypatch.setattr(module,'write_development_scores',lambda output,report,*args:write_json(output/'report.json',report))
    def forbidden(*args,**kwargs):raise AssertionError('Mechanical fit called during score-only correction')
    monkeypatch.setattr(module,'fit_rigid',forbidden)
    out=tmp_path/'out';report=module.rescore_development(root,out)
    assert report['passed'] and not report['parameter_refit'] and report['rigid_fit_calls']==0
    assert (out/'candidate.json').read_bytes()==(folder/'candidate.json').read_bytes()
    assert json.loads((folder/'report.json').read_text())==original
