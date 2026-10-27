"""Versioned serial-arm contract; rigid-body calculations live in C++ Pinocchio.

Geometry and synthetic physical values have independent provenance. The packaged
right-arm fixture requires neither the adjacent task repository nor mesh files.
"""
from __future__ import annotations
from copy import deepcopy
from importlib.resources import files
import json
from pathlib import Path
import numpy as np
from . import _core

UNITS=dict(length="m",mass="kg",inertia="kg m^2",angle="rad",torque="Nm",gravity="m/s^2")


def default_arm_config():
    return json.loads(files("casbot_calib").joinpath("assets/casbot_right_arm.json").read_text(encoding="utf-8"))


def _pose(data):
    pose=_core.ArmPose()
    pose.rotation=np.asarray(data["rotation"],dtype=float)
    pose.translation=np.asarray(data["translation"],dtype=float)
    return pose


def _body(data):
    body=_core.ArmBody();body.mass=data["mass"]
    body.com=np.asarray(data["com"],dtype=float)
    body.inertia_com=np.asarray(data["inertia_com"],dtype=float)
    return body


def load_arm_model(config=None):
    """Build only declared SI/versioned contracts. Engine copies and validates it."""
    if config is None:config=default_arm_config()
    elif isinstance(config,(str,Path)):config=json.loads(Path(config).read_text(encoding="utf-8"))
    if config.get("schema_version")!=1 or config.get("units")!=UNITS:
        raise ValueError("Expected arm schema 1 with explicit SI units")
    if config.get("model_version")!="serial_rigid_v1":
        raise ValueError("Unsupported arm dynamics model version")
    model=_core.ArmModel();model.model_id=config["robot_model_id"];model.model_version=config["model_version"]
    model.base=_pose(config["base"]);model.tool=_pose(config["tool"])
    model.gravity_world=np.asarray(config["gravity_world"],dtype=float)
    joints=[]
    for entry in config["joints"]:
        if entry.get("type")!="revolute":raise ValueError("Arm contract supports serial revolute joints only")
        joint=_core.ArmJoint();joint.name=entry["name"];joint.link_name=entry["link_name"]
        joint.origin=_pose(entry["origin"]);joint.axis=np.asarray(entry["axis"],dtype=float)
        joint.body=_body(entry["body"]);joint.rotor_inertia=entry["rotor_inertia"]
        for key in ("lower","upper","velocity_limit","effort_limit"):setattr(joint,key,entry["limits"][key])
        friction=_core.ArmFriction()
        for key in ("viscous","coulomb","static_friction","stribeck_velocity","smoothing_velocity"):
            setattr(friction,key,entry["friction"][key])
        joint.friction=friction;joints.append(joint)
    if config["joint_names"]!=[j.name for j in joints]:raise ValueError("Arm joint order does not match contract")
    model.joints=joints
    enabled=config["payload"]["enabled"]
    if not isinstance(enabled,bool):raise ValueError("Payload enable must be boolean")
    model.payload_enabled=enabled
    if model.payload_enabled:model.payload=_body(config["payload"]["body"])
    # Validate now, before callers can use FK/dynamics or accidentally repair inputs.
    _core.ArmDynamics(model)
    return model


def planar_two_link_config():
    """Independent analytic fixture convention: XY plane, gravity along -Y."""
    pose=lambda xyz:dict(translation=xyz,rotation=np.eye(3).tolist())
    config=dict(schema_version=1,model_version="serial_rigid_v1",robot_model_id="analytic_planar_2r_v1",
        units=UNITS.copy(),base=pose([0.,0.,0.]),tool=pose([.45,0.,0.]),
        gravity_world=[0.,-9.81,0.],joint_names=["joint_1","joint_2"],joints=[],payload=dict(enabled=False))
    for j,(length,mass,izz) in enumerate(((.7,1.3,.054),(.45,.8,.0135))):
        config["joints"].append(dict(name=f"joint_{j+1}",link_name=f"link_{j+1}",type="revolute",
            origin=pose([0. if j==0 else .7,0.,0.]),axis=[0.,0.,1.],rotor_inertia=0.,
            body=dict(mass=mass,com=[length/2,0.,0.],inertia_com=(np.eye(3)*izz).tolist()),
            limits=dict(lower=-np.pi,upper=np.pi,velocity_limit=10.,effort_limit=20.),
            friction=dict(viscous=0.,coulomb=0.,static_friction=0.,stribeck_velocity=.05,smoothing_velocity=.01)))
    return deepcopy(config)
