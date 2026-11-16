from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation
from casbot_calib.model_context import load_context
from casbot_calib.trical.routes import design_points
from casbot_calib.trical.solver import JointFixtureFit, KinematicParameters, fixture_points, export_geometry
from casbot_calib.trical.assembly import ball_center


def samples():
    context = load_context()
    rows = design_points('full', context.engine)
    for row in rows:
        row.update(offset_m=[0.]*3, ball_center_m=ball_center(row['ball']).tolist())
    return context, rows


def test_full_design_rank_and_native_analytic_jacobian():
    context, rows = samples()
    fit = JointFixtureFit(context.engine, rows)
    x = np.random.default_rng(4).uniform(-.1, .1, fit.size)
    _, analytic = fit.evaluate(x)
    for k in range(fit.size):
        delta = np.zeros(fit.size); delta[k] = 1e-3
        finite = (fit.evaluate(x+delta)[0]-fit.evaluate(x-delta)[0])/.002
        np.testing.assert_allclose(analytic[:, :, k], finite, atol=2e-10, rtol=2e-5)
    j = fit.evaluate(np.zeros(fit.size))[1][fit.train, :, :19].reshape(-1,19)
    assert np.linalg.matrix_rank(j) == 19
    assert np.linalg.cond(j) < 200
    for side in ('left', 'right'):
        for ball in 'ABCD':
            subset = [r for r in rows if r['arm']==side and r['ball']==ball]
            train = np.array([r['q_rad'] for r in subset if r['split']=='train'])
            holdout = np.array([r['q_rad'] for r in subset if r['split']=='holdout'])
            assert np.min(np.linalg.norm(train[:,None]-holdout, axis=2)) >= .02


def test_two_layer_fit_offsets_and_geometry_export(tmp_path):
    context, rows = samples()
    nominal = context.engine
    parameters = KinematicParameters(nominal)
    truth = np.zeros(parameters.size)
    truth[:14] = np.linspace(-1.5, 1.5, 14)
    fixture = np.array([2., -2., 1., np.deg2rad(.2)/.001, -np.deg2rad(.2)/.001])
    # Recover the 19 effective coordinates before adding the observable geometry layer.
    engine = parameters.build(truth)
    rng = np.random.default_rng(42)
    for row in rows:
        pose = engine.frame_pose(row['q_rad'], row['arm']+'_trical_tcp')
        ball = fixture_points([row['ball_center_m']], fixture)[0][0]
        row['offset_m'] = (pose[:3,:3].T @ (ball-pose[:3,3]) + rng.normal(0, 1e-8, 3)).tolist()
    fit = JointFixtureFit(nominal, rows)
    result = fit.fit()
    first = np.array(result['layer1_scaled'])
    np.testing.assert_allclose(first[:14], truth[:14], atol=.01)
    np.testing.assert_allclose(first[14:19], fixture, atol=.01)
    assert max(result['holdout_rmse_m']['layer1'].values()) < 1e-5
    assert result['geometry_columns']
    geometric_truth = truth.copy()
    for index in result['geometry_columns']:
        geometric_truth[index-5] = .5
    actual = parameters.build(geometric_truth)
    for row in rows:
        pose = actual.frame_pose(row['q_rad'], row['arm']+'_trical_tcp')
        ball = fixture_points([row['ball_center_m']], fixture)[0][0]
        row['offset_m'] = (pose[:3,:3].T @ (ball-pose[:3,3]) + rng.normal(0,1e-8,3)).tolist()
    geometric_result = JointFixtureFit(nominal, rows).fit()
    assert max(geometric_result['holdout_rmse_m']['layer2'].values()) < 1e-5
    recovered = np.array(geometric_result['layer2_scaled'])
    np.testing.assert_allclose(recovered[:14], truth[:14], atol=.02)
    np.testing.assert_allclose(recovered[14:19], fixture, atol=.02)
    geometry = np.zeros(parameters.size); geometry[14:] = rng.normal(0,.05,parameters.size-14)
    geometric = parameters.build(geometry)
    check = export_geometry(nominal, geometric, context.urdf, tmp_path/'calibrated.urdf')
    assert max(check.values()) < 1e-12
    # Zero corrections are separately applied at the encoder input.
    combined = geometry.copy(); combined[:14] = truth[:14]
    coupled = parameters.build(combined)
    for q in rng.uniform(-.5,.5,(3,14)):
        for side in ('left','right'):
            np.testing.assert_allclose(coupled.frame_pose(q, side+'_trical_tcp'),
                geometric.frame_pose(q + truth[:14]*parameters.scales[:14], side+'_trical_tcp'), atol=1e-12)


def test_mock_contact_uses_hidden_geometry_and_finite_tip_faces():
    from types import SimpleNamespace
    from casbot_calib.trical.controller import mock_contact
    from casbot_calib.trical.assembly import indicator_calibration
    from casbot_calib import _core
    context = load_context()
    row = design_points('smoke', context.engine)[0]
    parameters = KinematicParameters(context.engine)
    x = np.zeros(parameters.size); x[14:] = .2
    truth = parameters.build(x)
    hidden = np.linspace(-.001,.001,14)
    client = SimpleNamespace(q=np.array(row['q_rad']))
    ball = ball_center('A'); cal = indicator_calibration('left',[0.,0.,0.])
    readings, status = mock_contact(truth,client,'left',ball,cal,hidden)
    pose = truth.frame_pose(client.q+hidden,'left_trical_tcp')
    np.testing.assert_allclose(readings, cal.axes @ (pose[:3,:3].T@(ball-pose[:3,3])),atol=1e-14)
    assert status == ['VALID']*3
    _, outside = mock_contact(truth,client,'left',ball+np.array([.02,0,0]),cal,hidden)
    assert 'OUT_OF_RANGE' in outside
