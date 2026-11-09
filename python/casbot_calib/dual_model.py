"""Nominal fixed-waist tree and explicit synthetic physical variations."""
import numpy as np
from . import _core


def nominal_dual_arm(context=None):
    from .model_context import load_context
    context=load_context() if context is None else context
    return context.engine,context.source


def physical_model(nominal,scales,viscous,coulomb,smoothing_velocity=.03):
    """Positive body scales retain each body's COM/shape and scale mass and inertia.

    Retains the source fixed-body aggregates once. This is a restricted physical candidate,
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
