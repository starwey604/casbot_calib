"""Nominal fixed-waist tree and explicit synthetic physical variations."""
from importlib.resources import files,as_file
import hashlib,json
import numpy as np
from . import _core


def nominal_dual_arm():
    asset=files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf')
    with as_file(asset) as path:engine=_core.ArmDynamics.from_urdf(str(path))
    names=[j.name for j in engine.model.joints]
    if len(names)!=14 or sum(n.startswith('left_') for n in names)!=7 or sum(n.startswith('right_') for n in names)!=7:
        raise ValueError('Expected nominal fixed-waist 7+7 model')
    source=json.loads(files('casbot_calib').joinpath('assets/casbot_dual_arm_source.json').read_text(encoding='utf-8'))
    if hashlib.sha256(asset.read_bytes()).hexdigest()!=source['sha256']:raise ValueError('Nominal URDF digest mismatch')
    return engine,source


def physical_model(nominal,scales,viscous,coulomb,smoothing_velocity=.03):
    """Positive body scales retain each body's COM/shape and scale mass and inertia.

    Includes already-aggregated hand once. This is a restricted physical candidate,
    not recovery of every independent COM/inertia component.
    """
    model=nominal.model;n=len(model.joints)
    scales,viscous,coulomb=(np.broadcast_to(np.asarray(x,dtype=float),(n,)) for x in (scales,viscous,coulomb))
    if not all(np.isfinite(x).all() for x in (scales,viscous,coulomb)) or np.any(scales<=0) or np.any(viscous<0) or np.any(coulomb<0):
        raise ValueError('Invalid physical candidate')
    joints=model.joints
    for j,joint in enumerate(joints):
        body=joint.body;body.mass*=float(scales[j]);body.inertia_com=body.inertia_com*scales[j];joint.body=body
        friction=joint.friction;friction.viscous=float(viscous[j]);friction.coulomb=float(coulomb[j]);friction.static_friction=float(coulomb[j])
        friction.smoothing_velocity=smoothing_velocity;joint.friction=friction
    model.joints=joints
    return _core.ArmDynamics(model)


def body_basis(nominal):
    parameters=nominal.rigid_parameters();n=len(nominal.model.joints);basis=np.zeros((10*n,n))
    for j in range(n):basis[j*10:(j+1)*10,j]=parameters[j*10:(j+1)*10]
    return basis
