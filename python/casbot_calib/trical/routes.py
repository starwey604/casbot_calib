"""C3 routes through the eight CAD contact seeds, checked continuously in Coal."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from ..model_context import load_context
from ..shm_ready import ready_position
from ..model_sources import model_directory
from .assembly import profile, released_profile, ball_center, static_platform_check
from ..collision import certify_polyline
from .solver import KinematicParameters, fixture_points

SETTINGS = dict(position_allowance_rad=0., clearance_m=.01)


def gate_chord(collision, start, end, cache=None):
    """A monotone C3 segment traverses exactly its joint-space chord."""
    start, end = np.asarray(start), np.asarray(end)
    key = (id(collision), *sorted([start.tobytes(), end.tobytes()]))
    if cache is not None and key in cache:
        return cache[key]
    probes = collision.check(np.array([start, (start+end)/2, end]))['distances']
    if np.any(probes <= collision.pair_clearance_m):
        i,j = np.unravel_index(np.argmin(probes-collision.pair_clearance_m), probes.shape)
        return dict(passed=False, reason='nominal_chord_probe_rejected',
                    minimum_surplus_m=float((probes-collision.pair_clearance_m)[i,j]),
                    pair=[collision.names[k] for k in collision.pairs[j]])
    result = certify_polyline(collision, np.vstack([start,end]), clearance_m=.01,
                              block_samples=1, max_depth=14)
    if cache is not None:
        cache[key] = result
    return result


def gate_orientation(engine,frame,start,end,reference,tolerance,allowance,max_depth=12):
    """SO(3) triangle bound over a nominal joint chord and its joint box."""
    start=np.asarray(start);end=np.asarray(end);joint=next(f.parent for f in engine.model.frames if f.name==frame);ancestors=[]
    while joint>=0:ancestors.append(joint);joint=engine.model.parents[joint]
    box=float(np.sum(np.broadcast_to(allowance,start.shape)[ancestors]));travel=float(np.sum(abs(end-start)[ancestors]))
    pending=[(0.,1.,0)];maximum=sampled=0.;intervals=0
    while pending:
        a,b,depth=pending.pop();mid=(a+b)/2
        angle=float(Rotation.from_matrix(reference.T@engine.frame_pose(start+(end-start)*mid,frame)[:3,:3]).magnitude())
        sampled=max(sampled,angle);bound=angle+travel*(b-a)/2+box
        if bound<=tolerance:maximum=max(maximum,bound);intervals+=1;continue
        if depth>=max_depth:return dict(passed=False,reason='contact_orientation_envelope_unresolved',bound_rad=bound,tolerance_rad=tolerance)
        pending.extend([(a,mid,depth+1),(mid,b,depth+1)])
    return dict(passed=True,maximum_angle_bound_rad=maximum,maximum_sampled_angle_rad=sampled,tolerance_rad=tolerance,certified_intervals=intervals,
        method='SO(3) triangle inequality: midpoint angle + sum of ancestor joint travel and uncertainty')



def seed_rows():
    path = model_directory().parent / 'trical_1105_release_turn_reference.json'
    return json.loads(path.read_text())['results']


def inverse_pose(engine, side, initial, position, rotation, *, iterations=60):
    q = np.array(initial, dtype=float)
    ix = slice(0, 7) if side == 'left' else slice(7, 14)
    frame = side + '_trical_tcp'
    for _ in range(iterations):
        actual = engine.frame_pose(q, frame)
        error = np.r_[position - actual[:3, 3], Rotation.from_matrix(rotation @ actual[:3, :3].T).as_rotvec()]
        if np.linalg.norm(error) < 1e-10:
            return q
        j = engine.frame_jacobian(q, frame)[:, ix]
        step = j.T @ np.linalg.solve(j @ j.T + 1e-10*np.eye(6), error)
        q[ix] += step * min(1., .1 / max(np.max(abs(step)), 1e-12))
    raise ValueError('接触邻域 IK 未收敛')


def design_points(mode, engine=None, routes=None):
    engine = load_context().engine if engine is None else engine
    parameters = KinematicParameters(engine)
    rng = np.random.default_rng(110503)
    rows = []
    information = np.eye(19) * 1e-12
    for seed in seed_rows():
        side, ball = seed['arm'], seed['ball']
        if routes is not None and side != routes.side:
            continue
        if mode == 'smoke' and ball != 'A':
            continue
        base = np.array(seed['contact'][0]['q'])
        rotation = engine.frame_pose(base, side+'_trical_tcp')[:3, :3]
        center = ball_center(ball)
        if mode != 'full':
            rows.append(dict(arm=side, ball=ball, q_rad=base.tolist(), split='diagnostic'))
            continue
        ix = slice(0, 7) if side == 'left' else slice(7, 14)
        _, _, vh = np.linalg.svd(engine.frame_jacobian(base, side+'_trical_tcp')[:, ix])
        null = vh[-1]
        candidates = [base]
        for _ in range(64):
            guess = base.copy(); guess[ix] += null * rng.uniform(-.35, .35)
            perturbation = rng.uniform(-1., 1., 3)
            perturbation *= np.deg2rad(.55) / max(np.linalg.norm(perturbation), 1.)
            try:
                q = inverse_pose(engine, side, guess, center,
                                 Rotation.from_rotvec(perturbation).as_matrix() @ rotation)
                if all(j.lower < v < j.upper for v, j in zip(q, engine.model.joints)):
                    candidates.append(q)
            except ValueError:
                continue
        candidates = np.array(candidates)
        frames = [side+'_trical_tcp'] * len(candidates)
        _, jac = parameters.evaluate(np.zeros(parameters.size), candidates, frames, np.zeros((len(candidates), 3)))
        _, fj = fixture_points(np.repeat(center[None], len(candidates), axis=0), np.zeros(5))
        jac = np.concatenate([jac[:, :, :14], -fj], axis=2)
        chosen = []
        rejected = set()
        def choose(scores):
            while True:
                at = int(np.argmax(scores))
                if not np.isfinite(scores[at]):
                    raise ValueError(f'{side} {ball} 缺少通过路线检查的采集姿态')
                if at in rejected:
                    scores[at] = -np.inf
                    continue
                try:
                    if routes is not None:
                        routes.check_contact(ball, base, candidates[at])
                    return at
                except ValueError:
                    rejected.add(at)
                    scores[at] = -np.inf
        for _ in range(4):
            scores = [np.linalg.slogdet(information + j.T @ j)[1] if k not in chosen else -np.inf
                      for k, j in enumerate(jac)]
            at = choose(scores); chosen.append(at)
            information += jac[at].T @ jac[at]
        holdout = []
        for _ in range(2):
            distance = np.min(np.linalg.norm(candidates[:, None] - candidates[chosen+holdout], axis=2), axis=1)
            scores = np.where(distance >= .02, distance, -np.inf).tolist()
            at = choose(scores)
            holdout.append(at)
        rows.extend(dict(arm=side, ball=ball, q_rad=candidates[k].tolist(),
                         split='train' if k in chosen else 'holdout') for k in chosen+holdout)
    return rows


class Routes:
    def __init__(self, side, context=None):
        self.context = context or load_context()
        self.engine = self.context.engine
        self.side = side
        self.free = profile(self.context.bundle, side)
        self.contacts = {b: profile(self.context.bundle, side, b, True) for b in 'ABCD'}
        self.released = {b: released_profile(p) for b, p in self.contacts.items()}
        self.seeds = {r['ball']: r for r in seed_rows() if r['arm'] == side}
        self.cache = {}

    def check(self, start, end, ball=None, *, released=False, orientation=None):
        limits = np.array([[j.lower, j.upper] for j in self.engine.model.joints])
        endpoints = np.array([start, end])
        if np.any(endpoints < limits[:,0]) or np.any(endpoints > limits[:,1]):
            raise ValueError('路线超出 URDF 关节限位')
        collision = self.free if ball is None else (self.released if released else self.contacts)[ball]
        proof = gate_chord(collision, start, end, cache=self.cache)
        if not proof['passed']:
            raise ValueError(f'{self.side} {ball or "自由空间"} 连续碰撞检查失败：{proof}')
        if orientation is not None:
            proof = gate_orientation(self.engine, self.side+'_trical_tcp', start, end,
                                     np.asarray(orientation), np.deg2rad(1.), SETTINGS['position_allowance_rad'])
            if not proof['passed']:
                raise ValueError(f'接触区姿态包络检查失败：{proof}')
        return proof

    def path(self, ball):
        row = self.seeds[ball]
        contact = [np.array(x['q']) for x in row['contact']]
        # Both arms start on A. Its saved withdrawal grids use different steps.
        if ball == 'A':
            pose = self.engine.frame_pose(contact[0], self.side+'_trical_tcp')
            positions = np.array([self.engine.frame_pose(q, self.side+'_trical_tcp')[:3,3] for q in contact])
            distances = np.linalg.norm(positions-positions[0], axis=1)
            direction = (positions[2]-positions[0])/distances[2]
            def at_distance(distance):
                index = int(np.searchsorted(distances, distance))
                a, b = index-1, index
                u = (distance-distances[a])/(distances[b]-distances[a])
                return inverse_pose(self.engine, self.side, contact[a]*(1-u)+contact[b]*u,
                                    positions[0]+direction*distance, pose[:3,:3])
            contact = [contact[0], at_distance(.0025), at_distance(.005)] + [
                q for d, q in zip(distances, contact) if d > .0050001]

        return [np.array(q) for q in row['transfer']], contact

    def contact_path(self, ball, start, end):
        start, end = np.asarray(start), np.asarray(end)
        first = self.engine.frame_pose(start, self.side+'_trical_tcp')
        last = self.engine.frame_pose(end, self.side+'_trical_tcp')
        delta = Rotation.from_matrix(last[:3,:3] @ first[:3,:3].T).as_rotvec()
        count = max(1, int(np.ceil(np.max(abs(end-start))/.02)))
        path = [start]
        for u in np.linspace(0, 1, count+1)[1:]:
            q = end if u == 1 else inverse_pose(self.engine, self.side, start+(end-start)*u,
                first[:3,3]*(1-u)+last[:3,3]*u,
                Rotation.from_rotvec(delta*u).as_matrix() @ first[:3,:3])
            self.check(path[-1], q, ball, orientation=self.seeds[ball]['rotation'])
            path.append(q)
        return path

    def check_contact(self, ball, start, end):
        path = self.contact_path(ball, start, end)
        self.contact_path(ball, end, self.seeds[ball]['contact'][0]['q'])
        return path

    def check_local(self, ball, start, end):
        rotation = np.array(self.seeds[ball]['rotation'])
        angles = [Rotation.from_matrix(rotation.T @ self.engine.frame_pose(q, self.side+'_trical_tcp')[:3, :3]).magnitude()
                  for q in (start, end)]
        if max(angles) < np.deg2rad(.6):
            return self.check(start, end, ball, orientation=rotation)
        return self.check(start, end, ball, released=True)

    def validate(self, points):
        if not static_platform_check(self.free)['passed']:
            raise ValueError('固定台架与机器人基座的间距检查失败')
        self.check(ready_position(), self.context.home)
        for ball in dict.fromkeys(p['ball'] for p in points):
            transfer, contact = self.path(ball)
            for a, b in zip(transfer, transfer[1:]): self.check(a, b)
            for a, b in zip(contact, contact[1:]): self.check_local(ball, a, b)
            for point in [p for p in points if p['ball'] == ball]:
                self.check_contact(ball, contact[0], point['q_rad'])
        return True
