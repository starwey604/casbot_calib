"""Same-recipe repeatability keeps signs, denominators and independent histories."""
from copy import deepcopy
import json
import pytest
from casbot_calib.sdk_feedback_repeat import compare_pair, compare_assessments


def pair(source, values):
    rows=[]
    for direction, value in zip((-1,1), values):
        fits={r:dict(available=True,inertia_kg_m2=value,viscous_nm_s_rad=.01,effective_advance_s=.002) for r in ('A','B')}
        rows.append(dict(metadata=dict(source=source,job=dict(direction=direction)),
            results=[dict(case=dict(id='baseline',family='baseline'),routes=fits)]))
    return rows


def test_same_direction_uses_old_not_new_denominator_and_does_not_average():
    result=compare_pair(pair('old',[.001,.0011]),pair('new',[.0012,.00099]))
    for route in ('A','B'):
        r=result['baseline']['routes'][route]
        assert r['directions'][0]['cross_session_inertia_difference_relative']==pytest.approx(.2)
        assert r['directions'][1]['cross_session_inertia_difference_relative']==pytest.approx(.1)
        assert r['maximum_same_direction_cross_session_relative']==pytest.approx(.2)
        assert not r['cross_session_within_10_percent']
        assert r['all_four_pairwise_maximum_relative']==pytest.approx(.0012/.00099-1)


def test_same_direction_stability_does_not_hide_large_direction_bias():
    r=compare_pair(pair('old',[.001,.0013]),pair('new',[.00102,.00132]))['baseline']['routes']['B']
    assert r['cross_session_within_10_percent'] and not r['all_four_within_10_percent']
    assert r['repeat_direction_difference_relative']>.1


def test_missing_route_and_missing_direction_are_not_passes():
    old=pair('old',[.001,.001]);new=pair('new',[.001,.001])
    new[1]['results'][0]['routes']['B']=dict(available=False,reason='rank')
    r=compare_pair(old,new)['baseline']['routes']['B']
    assert not r['available'] and not r['cross_session_within_10_percent']
    assert r['maximum_same_direction_cross_session_relative'] is None
    with pytest.raises(ValueError,match='negative and one positive'):compare_pair(old,new[:1])


def test_grids_must_match_not_silently_zip_different_cases():
    old=pair('old',[.001,.001]);new=pair('new',[.001,.001]);new[1]['results'][0]['case']['id']='different'
    with pytest.raises(ValueError,match='grids differ'):compare_pair(old,new)


def reports(tmp_path,monkeypatch):
    import casbot_calib.sdk_feedback_repeat as module
    protocol=dict(diagnostic_threshold=.1,inertia_denominator_floor_kg_m2=1e-6,cases=['baseline'],route_A='A',route_B='B',common_processing='same')
    old=dict(schema_version='casbot_sdk_feedback_sensitivity_report_v1',completed=True,
        protocol=dict(protocol,sources=['old']),records=pair('old',[.001,.00103]))
    new=dict(protocol=dict(protocol,sources=['new']),records=pair('new',[.00102,.00104]))
    root=tmp_path/'old_report';root.mkdir();(root/'report.json').write_text(json.dumps(old))
    monkeypatch.setattr(module,'recipe',lambda _:dict(tones=[2.5,10]))
    def binding(row):
        isnew=row['metadata']['source']=='new'
        return dict(endpoint='new' if isnew else 'old',scope='localhost_contract_mock',first_receive_ns=2 if isnew else 1,
            executor_sha256='newbinary' if isnew else 'oldbinary',model='same',pd=[30.,.12])
    monkeypatch.setattr(module,'public_binding',binding)
    return module,root,new,binding


def test_reuse_requires_distinct_runtime_evidence_and_reports_binary_change(tmp_path,monkeypatch):
    module,root,new,binding=reports(tmp_path,monkeypatch)
    r=compare_assessments(root,new,tmp_path/'compare')
    assert r['independent_mock_sessions_verified'] and r['reference_fits_reused_without_refitting']
    assert r['groups'][0]['executor_changed']
    monkeypatch.setattr(module,'public_binding',lambda row:dict(binding(row),endpoint='same',first_receive_ns=1))
    r=compare_assessments(root,new,tmp_path/'not_independent')
    assert not r['independent_mock_sessions_verified']


def test_effective_pd_difference_cannot_be_called_same_recipe_repeat(tmp_path,monkeypatch):
    module,root,new,binding=reports(tmp_path,monkeypatch)
    monkeypatch.setattr(module,'public_binding',lambda row:dict(binding(row),pd=[40 if row['metadata']['source']=='new' else 30]))
    with pytest.raises(ValueError,match='different effective'):compare_assessments(root,new,tmp_path/'compare')


def test_changed_reference_processing_is_not_reused(tmp_path,monkeypatch):
    _,root,new,_=reports(tmp_path,monkeypatch);new['protocol']['route_B']='a different fitted model'
    with pytest.raises(ValueError,match='contracts differ'):compare_assessments(root,new,tmp_path/'compare')


def test_cross_direction_comparison_keeps_reference_denominator():
    r=compare_pair(pair('old',[.001,.0011]),pair('new',[.00102,.00095]))['baseline']['routes']['B']
    worst=r['worst_cross_session_including_cross_direction']
    assert worst['reference_direction']==1 and worst['repeat_direction']==1
    assert worst['difference_relative']==pytest.approx((.0011-.00095)/.0011)


def test_cli_passes_reference_assessment(monkeypatch,tmp_path):
    from casbot_calib import fieldwork_cli,sdk_feedback_sensitivity
    seen=[];monkeypatch.setattr(sdk_feedback_sensitivity,'main',lambda args:seen.extend(args) or 0)
    assert fieldwork_cli.main(['sensitivity','--source','new','--output','out','--reference-assessment','old_report'])==0
    assert seen[-2:]==['--reference-assessment','old_report']
