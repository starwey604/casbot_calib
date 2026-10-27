"""Export a self-contained synthetic arm fixture from audited URDF geometry.

Only joint transforms/axes/limits come from URDF. Dynamics and friction values
are explicit synthetic boxes; source inertias remain provenance, never truth.
"""
from __future__ import annotations
from hashlib import sha256
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def urdf_pose(joint):
    origin=joint.find("origin")
    xyz=np.fromstring(origin.get("xyz","0 0 0") if origin is not None else "0 0 0",sep=" ")
    rpy=np.fromstring(origin.get("rpy","0 0 0") if origin is not None else "0 0 0",sep=" ")
    if xyz.shape!=(3,) or rpy.shape!=(3,):raise ValueError("Invalid URDF origin")
    result=np.eye(4);result[:3,3]=xyz;result[:3,:3]=Rotation.from_euler("xyz",rpy).as_matrix()
    return result


def _pose(matrix):
    return dict(translation=matrix[:3,3].tolist(),rotation=matrix[:3,:3].tolist())


def import_right_arm(urdf_path):
    urdf_path=Path(urdf_path);raw=urdf_path.read_bytes();root=ET.fromstring(raw)
    joints=root.findall("joint");links={link.get("name"):link for link in root.findall("link")}
    by_child={j.find("child").get("link"):j for j in joints}
    if len(by_child)!=len(joints):raise ValueError("URDF has multiple parents")
    end="right_wrist_roll_link";cursor=end;path=[];seen=set()
    while cursor!="base_link":
        if cursor in seen or cursor not in by_child:raise ValueError("Invalid fixed-base right-arm chain")
        seen.add(cursor);j=by_child[cursor];path.append(j);cursor=j.find("parent").get("link")
    path.reverse();fixed=np.eye(4);base=np.eye(4);entries=[];fixed_base=[]
    masses=(1.1,.85,.65,.8,.45,.3,.22)
    for joint in path:
        kind=joint.get("type");pose=urdf_pose(joint)
        if kind=="fixed":
            fixed=fixed@pose;fixed_base.append(joint.get("name"));continue
        if kind!="revolute":raise ValueError("Only revolute right arm supported")
        if not entries:base=fixed.copy();origin=pose
        else:origin=fixed@pose
        fixed=np.eye(4)
        name=joint.get("name");link=joint.find("child").get("link")
        axis=np.fromstring(joint.find("axis").get("xyz"),sep=" ");limit=joint.find("limit")
        source_inertial=links[link].find("inertial")
        entries.append(dict(name=name,link_name=link,type="revolute",origin=_pose(origin),axis=axis.tolist(),
            rotor_inertia=0.,limits=dict(lower=float(limit.get("lower")),upper=float(limit.get("upper")),
            velocity_limit=float(limit.get("velocity")),effort_limit=float(limit.get("effort"))),
            source_inertial_xml=ET.tostring(source_inertial,encoding="unicode") if source_inertial is not None else None))
    if len(entries)!=7:raise ValueError("Expected exactly seven right-arm joints")
    for n,entry in enumerate(entries):
        offset=np.array(entries[n+1]["origin"]["translation"]) if n<6 else np.array([0.,0.,-.0735])
        length=max(float(np.linalg.norm(offset)),.06)
        dims=np.array([.06,.05,length]);mass=masses[n]
        inertia=mass/12*np.array([dims[1]**2+dims[2]**2,dims[0]**2+dims[2]**2,dims[0]**2+dims[1]**2])
        entry["body"]=dict(mass=mass,com=(.4*offset).tolist(),inertia_com=np.diag(inertia).tolist())
        fc=(.12,.1,.08,.1,.05,.04,.03)[n]
        entry["friction"]=dict(viscous=.03,coulomb=fc,static_friction=1.5*fc,
                               stribeck_velocity=.04,smoothing_velocity=.008)
        entry["provenance"]=dict(geometry="nominal_URDF_unverified_on_hardware",
            body="synthetic_uniform_box_excludes_rotor",friction="synthetic_steady_state",
            rotor="explicit_zero_not_imported_from_MJCF",limits="source_benchmark_not_device_authority")
    tool_joint=next(j for j in joints if j.get("name")=="right_hand_joint")
    if tool_joint.get("type")!="fixed":raise ValueError("Expected fixed hand")
    descendants=[]
    def visit(parent):
        for j in joints:
            if j.find("parent").get("link")!=parent:continue
            if j.get("type")!="fixed":raise ValueError("Hand subtree must be locked for fixture")
            child=j.find("child").get("link");descendants.append(child);visit(child)
    visit(end)
    return dict(schema_version=1,model_version="serial_rigid_v1",
        robot_model_id="casbot_20251015_right_arm_synthetic_v1",
        units=dict(length="m",mass="kg",inertia="kg m^2",angle="rad",torque="Nm",gravity="m/s^2"),
        base=_pose(base),tool=_pose(urdf_pose(tool_joint)),gravity_world=[0.,0.,-9.81],
        joint_names=[j["name"] for j in entries],joints=entries,
        payload=dict(enabled=True,body=dict(mass=.35,com=[0.,0.,.04],
            inertia_com=np.diag([.0003,.00025,.0002]).tolist()),
            provenance="one synthetic rigid payload in palm/TCP frame; replaces entire source fixed hand subtree"),
        provenance=dict(source_urdf="casbot_tasks/scripts/casbot_dual_arm_model/models/"+urdf_path.name,
            source_sha256=sha256(raw).hexdigest(),fixed_base_joints=fixed_base,
            source_fixed_hand_links_replaced=descendants,
            fixed_subtree_policy="No source hand inertia is imported. One explicit synthetic payload is appended once.",
            excluded_benchmark_fields=["MJCF armature=0.2","MJCF damping=0.5"],
            runtime_requires_source=False,hardware_validated=False),
        reference_pose=[-.4,-.4,.2,-.9,.25,-.15,.2])


def export_right_arm(urdf_path,output,reference_output):
    """Reference FK comes from Pinocchio's independent complete-URDF parser."""
    import pinocchio as pin
    config=import_right_arm(urdf_path)
    model=pin.buildModelFromUrdf(str(urdf_path));data=model.createData()
    rng=np.random.default_rng(20261020);frames=[]
    samples=[np.zeros(7),np.array(config["reference_pose"])]
    limits=np.array([[x["limits"]["lower"],x["limits"]["upper"]] for x in config["joints"]])
    samples.extend(rng.uniform(limits[:,0]*.7,limits[:,1]*.7,size=(10,7)))
    for q_arm in samples:
        q=pin.neutral(model)
        for name,value in zip(config["joint_names"],q_arm,strict=True):
            joint=model.joints[model.getJointId(name)];q[joint.idx_q]=value
        pin.framesForwardKinematics(model,data,q)
        names=[x["link_name"] for x in config["joints"]]+["right_palm_base_link"]
        frames.append(dict(q=q_arm.tolist(),world_poses=[data.oMf[model.getFrameId(name)].homogeneous.tolist() for name in names]))
    ref=dict(schema_version=1,source_sha256=config["provenance"]["source_sha256"],
        generator="Pinocchio "+pin.__version__+" complete URDF parser (geometry reference, not a dynamics oracle)",
        joint_names=config["joint_names"],samples=frames)
    for path,value in ((output,config),(reference_output,ref)):
        Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    return config
