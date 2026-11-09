"""Additional reference information must not change physics or old protocols."""
from copy import deepcopy
from pathlib import Path
import numpy as np
import pytest
import yaml
from casbot_calib.staged_acceleration import add_tone, center_pool
from casbot_calib.staged_protocol import build_split_manifest, validate_config
from casbot_calib.staged_multiconfiguration import motion_for_slot, motion_coverage


@pytest.fixture(scope='module')
def config():
    return yaml.safe_load(Path('configs/dvt2/staged_dual_arm_p0_v4.yaml').read_text())


def test_original_protocol_is_unchanged_and_new_experiment_is_disjoint(config):
    old=yaml.safe_load(Path('configs/dvt2/staged_dual_arm_p0.yaml').read_text())
    a=build_split_manifest(old);b=build_split_manifest(config)
    assert len(a['trials'])==len(b['trials'])==88
    assert not {s['acquisition_seed'] for s in a['trials']}&{s['acquisition_seed'] for s in b['trials']}
    for key in ('mock','drive','pose_amplitude_rad','control_period_ns','integration_ns'):
        assert old[key]==config[key]
    for key in ('weighting','preprocessing','selection','acceptance','stability'):
        assert old['p0'][key]==config['p0'][key]
    slot=a['trials'][0];m=motion_for_slot(slot,old['home_rad'],old)
    assert add_tone(m,old,'train',slot['reference_seed']) is m
    pool=np.array([old['home_rad']]);assert center_pool(pool,pool[0],slot['center_id'],old['p0']['center_design']) is pool


def test_all_periodic_roles_keep_original_limits_and_add_acceleration(config):
    base=deepcopy(config);base['p0'].pop('acceleration_tone')
    slots=build_split_manifest(config)['trials'];home=np.array(config['home_rad'])
    for slot in (s for s in slots if s['stage']=='inertia' and s['repeat_id']==0):
        center=center_pool(home[None,:],home,slot['center_id'],config['p0']['center_design'])[0]
        m=motion_for_slot(slot,center,config);before=motion_for_slot(slot,center,base)
        r=motion_coverage(m,config);rb=motion_coverage(before,base)
        assert r['passed'],(slot['trial_id'],r['peak_derivative_bounds'])
        axes=m.conditions['acceleration_tone']['joints']
        assert np.min(np.array(r['acceleration_rms_rad_s2'])[axes]/np.array(rb['acceleration_rms_rad_s2'])[axes])>1.1
        assert slot['excitation_harmonics'][-1]==config['p0']['acceleration_tone']['harmonic_by_split'][slot['split']]
        p=m.polynomial();b=before.polynomial()
        t=m.windows('periodic')[0][0]+np.arange(1024)*config['dynamic_period_s']/1024
        delta=p(t)-b(t);fourier=np.fft.rfft(delta,axis=0)/len(t)
        h=config['p0']['acceleration_tone']['harmonic_by_split'][slot['split']]
        np.testing.assert_allclose(2*abs(fourier[h,axes]),.010,rtol=2e-5)
        # Original excitation bins remain; the new tone does not replace them.
        np.testing.assert_allclose(fourier[[1,2,3,4,5,6]],0,atol=2e-7)
        inactive=sorted(set(range(14))-set(axes))
        np.testing.assert_allclose(delta[:,inactive],0,atol=1e-13)
        for endpoint in (0,m.duration):
            np.testing.assert_allclose(p(endpoint),home,atol=1e-12)
            for d in (1,2,3):np.testing.assert_allclose(p(endpoint,nu=d),0,atol=1e-10)
        knot=m.windows('periodic')[0][0]-.67
        for d in range(3):
            np.testing.assert_allclose((p(knot+1e-5,nu=d)-p(knot-1e-5,nu=d))/2e-5,p(knot,nu=d+1),rtol=2e-5,atol=1e-6)
        # Every knot uses one shared jet, so entry/braking remains C3.
        for d in range(4):np.testing.assert_allclose(p(m.knots,nu=d),np.array(m.states)[:,d],atol=1e-9)


def test_posture_anchors_are_local_and_do_not_touch_unselected_joints(config):
    pool=np.arange(42,dtype=float).reshape(3,14)/100;copy=pool.copy();home=np.array(config['home_rad'])
    p=config['p0']['center_design'];q=center_pool(pool,home,'train_c00',p)
    np.testing.assert_array_equal(pool,copy)
    for j in (5,6,12,13):np.testing.assert_allclose(q[:,j],home[j]-.3)
    free=[j for j in range(14) if j not in (5,6,12,13)]
    np.testing.assert_array_equal(q[:,free],pool[:,free])
    assert center_pool(pool,home,'selection_c00',p) is pool


@pytest.mark.parametrize('change',[
    lambda c:c.update(smooth_periodic_envelope=False),
    lambda c:c['p0']['acceleration_tone'].update(joints=[11,11]),
    lambda c:c['p0']['acceleration_tone'].update(amplitude_rad=float('nan')),
    lambda c:c['p0']['acceleration_tone']['harmonic_by_split'].update(test=41),
    lambda c:c['p0']['acceleration_tone']['harmonic_by_split'].update(test=13),
    lambda c:c['p0']['center_design']['fixed_joint_offsets_rad'].update(unknown={5:.1}),
])
def test_unbounded_or_ambiguous_design_is_rejected(config,change):
    c=deepcopy(config);change(c)
    with pytest.raises(ValueError):validate_config(c)


def test_new_optional_design_survives_preregistration_json_roundtrip(config,tmp_path):
    from casbot_calib.staged_protocol import prepare_protocol,load_protocol
    root=tmp_path/'contract'
    prepare_protocol(root,config)
    _,public,_=load_protocol(root)
    assert public=={k:v for k,v in config.items() if k!='mock'}


def test_old_contract_only_entry_does_not_require_r1_center_design(tmp_path):
    from casbot_calib.staged_protocol import prepare_protocol,load_protocol
    old=yaml.safe_load(Path('configs/dvt2/staged_dual_arm_p0.yaml').read_text())
    old['p0'].pop('center_design')
    root=tmp_path/'r0_only'
    prepare_protocol(root,old)
    _,public,_=load_protocol(root)
    assert 'center_design' not in public['p0']
