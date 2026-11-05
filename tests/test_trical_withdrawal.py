"""Release is an explicit, stricter phase; contact orientation is bounded."""
import numpy as np
import pytest
from casbot_calib.trical_assembly import profile,released_profile
from casbot_calib.trical_paths import gate_chord,gate_orientation,withdrawal_rules
from casbot_calib.trical_withdrawal import bounded_profile
from casbot_calib.model_sources import model_directory
from casbot_calib.dvt2_bundle import load_bundle

BUNDLE=model_directory()/'casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2'


def test_release_strengthens_only_exact_tip_pairs_and_leaves_contact_unchanged():
    p=profile(BUNDLE,'left','D',True);before=p.pair_clearance_m.copy();r=released_profile(p)
    assert r.native is p.native
    indices=before<0;assert indices.sum()==3
    np.testing.assert_array_equal(p.pair_clearance_m,before)
    np.testing.assert_array_equal(r.pair_clearance_m[~indices],before[~indices])
    np.testing.assert_array_equal(r.pair_clearance_m[indices],np.full(3,.0005))
    with pytest.raises(ValueError):released_profile(p,0.)
    with pytest.raises(ValueError):released_profile(profile(BUNDLE,'left'))


def test_phase_schema_and_rotation_gate_include_uncertainty():
    assert withdrawal_rules({},4) is None
    spec=dict(schema_version='casbot_trical_release_then_turn_v1',release_index=1,minimum_released_tip_clearance_m=.0005,contact_orientation_tolerance_rad=float(np.deg2rad(1)))
    assert withdrawal_rules(dict(withdrawal=spec),4)==spec
    for index in [0,4,-1,True]:
        with pytest.raises(ValueError):withdrawal_rules(dict(withdrawal={**spec,'release_index':index}),4)
    e,m=load_bundle(BUNDLE);q=np.array(m['profile']['home_rad']);R=e.frame_pose(q,'left_trical_tcp')[:3,:3]
    g=gate_orientation(e,'left_trical_tcp',q,q,R,np.deg2rad(1),.00105)
    assert g['passed'] and g['maximum_angle_bound_rad']==pytest.approx(7*.00105)
    r=q.copy();r[6]+=.1
    assert not gate_orientation(e,'left_trical_tcp',q,r,R,np.deg2rad(1),.00105)['passed']


def test_local_pair_pruning_preserves_values_and_rejects_nonfinite_boxes():
    p=profile(BUNDLE,'right','C',True);_,m=load_bundle(BUNDLE);q=np.array(m['profile']['home_rad']);width=np.full(14,.002)
    sub,proof=bounded_profile(p,q,width)
    if sub is not None:
        probes=np.vstack([q,q+width,q-width]);full=p.check(probes)['distances']
        np.testing.assert_allclose(sub.check(probes)['distances'],full[:,proof['pair_indices']],atol=2e-6,rtol=0)
    with pytest.raises(ValueError):bounded_profile(p,q,np.full(14,float('nan')))


def test_gate_cache_distinguishes_release_policy_and_reuses_reverse_chords():
    p=profile(BUNDLE,'left','D',True);_,m=load_bundle(BUNDLE);q=np.array(m['profile']['home_rad']);cache={}
    gate_chord(p,q,q,cache=cache)
    assert gate_chord(p,q,q,cache=cache)['reused_in_same_planning_run']
    gate_chord(released_profile(p),q,q,cache=cache)
    assert len(cache)==2


def test_cli_passes_explicit_settings_without_changing_legacy_defaults(tmp_path,monkeypatch):
    import json,sys
    import casbot_calib.trical_paths as paths
    settings={'control_period_ns':2_000_000,'local_motion_bounds':True};file=tmp_path/'settings.json';file.write_text(json.dumps(settings))
    captured={}
    def fake(*args,**kwargs):captured.update(kwargs);return {'passed':True}
    monkeypatch.setattr(paths,'plan',fake)
    monkeypatch.setattr(sys,'argv',['trical_paths','--bundle','bundle','--seeds','seed.json','--settings',str(file),'--output','out'])
    assert paths.main()==0 and captured['settings']==settings
    monkeypatch.setattr(sys,'argv',['trical_paths','--bundle','bundle','--seeds','seed.json','--output','out'])
    assert paths.main()==0 and captured['settings'] is None


def test_export_verifier_rejects_a_release_label_without_bound_phase_rules(tmp_path):
    import json
    from pathlib import Path
    from casbot_calib.trical_reference_validation import verify
    from casbot_calib.trical_paths import timed_motion,SETTINGS
    from casbot_calib.dvt2_bundle import read
    from casbot_calib.kinematics import digest
    seeds=Path(__file__).parents[1]/'configs/trical_1105_release_turn_reference.json';source=read(seeds)
    row=next(r for r in source['results'] if (r['arm'],r['ball'])==('left','A'));local=np.array([r['q'] for r in row['contact']])
    _,meta=load_bundle(BUNDLE);home=np.array(meta['profile']['home_rad'])
    points=[home,home,local[-1],*local[-2::-1],local[0],*local[1:],home,home]
    labels=['initial_hold','approach',*['insert']*(len(local)-1),'contact_hold',*['withdraw']*(len(local)-1),'return','final_hold']
    motion=timed_motion('left_A',points,labels);curve=motion.polynomial();target=tmp_path/'plan';folder=target/'left_A';folder.mkdir(parents=True)
    protocol=dict(settings=dict(SETTINGS),fixture_layout=None,bundle_sha256=meta['bundle_sha256'],seed_sha256=digest(source))
    (target/'protocol.json').write_text(json.dumps(protocol));record=motion.metadata();record['labels'][1]='withdraw_released';(folder/'motion.json').write_text(json.dumps(record))
    ns=np.arange(round(motion.duration*500)+1,dtype=np.int64)*2_000_000
    np.savez_compressed(folder/'reference.npz',time_ns=ns,q=curve(ns/1e9),dq=curve(ns/1e9,nu=1))
    with pytest.raises(ValueError,match='Unknown or unbound motion phase'):verify(target,BUNDLE,seeds,tmp_path/'verification')


def test_optional_corner_probe_does_not_require_new_phase_fields_from_legacy_seeds(tmp_path):
    import json
    from pathlib import Path
    from casbot_calib.dvt2_bundle import read
    from casbot_calib.kinematics import digest
    from casbot_calib.trical_reference_validation import probe_local_corners
    source=read(Path(__file__).parents[1]/'configs/trical_1105_release_turn_reference.json')
    for row in source['results']:row.pop('withdrawal',None)
    _,meta=load_bundle(BUNDLE);plan=tmp_path/'plan';plan.mkdir();seeds=tmp_path/'seeds.json';seeds.write_text(json.dumps(source))
    (plan/'protocol.json').write_text(json.dumps(dict(bundle_sha256=meta['bundle_sha256'],seed_sha256=digest(source),fixture_layout=None,settings={'position_allowance_rad':.00105})))
    result=probe_local_corners(plan,BUNDLE,seeds,tmp_path/'probe')
    assert result['passed'] and result['samples']==0 and result['minimum_sampled_surplus_m'] is None


def test_frozen_reference_rejects_another_bundle_before_generating_motion(tmp_path):
    import json
    from pathlib import Path
    from casbot_calib.dvt2_bundle import read
    from casbot_calib.trical_paths import plan
    source=read(Path(__file__).parents[1]/'configs/trical_1105_release_turn_reference.json');source['bundle_sha256']='different-bundle'
    seeds=tmp_path/'wrong.json';seeds.write_text(json.dumps(source))
    with pytest.raises(ValueError,match='another model bundle'):plan(tmp_path/'out',BUNDLE,seeds)
    assert not list((tmp_path/'out').glob('*/reference.npz'))


def test_straight_hole_screen_does_not_silently_reinterpret_curved_retract_coordinates(tmp_path):
    from pathlib import Path
    from casbot_calib.trical_holes import search
    seeds=Path(__file__).parents[1]/'configs/trical_1105_release_turn_reference.json'
    with pytest.raises(ValueError,match='requires straight seeds'):search(tmp_path/'screen',BUNDLE,seeds)
    assert not (tmp_path/'screen').exists()


def test_pose_search_rejects_an_unimplemented_layout_instead_of_using_baseline():
    from casbot_calib.trical_withdrawal import require_baseline_inputs
    source={'schema_version':'casbot_trical_withdrawal_seeds_v1','bundle_sha256':'bundle'}
    require_baseline_inputs(source,{'bundle_sha256':'bundle'})
    with pytest.raises(ValueError,match='baseline fixture only'):require_baseline_inputs({**source,'fixture_layout':{'platform_lift_m':.05}},{'bundle_sha256':'bundle'})
