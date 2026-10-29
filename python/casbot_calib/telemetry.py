"""Read-only, source-independent joint states. No device writes or physics imports."""
from __future__ import annotations
from dataclasses import dataclass
import json
from pathlib import Path
from threading import Lock
from time import monotonic_ns
from typing import Protocol
import numpy as np


@dataclass(frozen=True)
class JointMapping:
    name: str
    model_joint: str
    scale_rad_per_unit: float = 1.
    direction: int = 1
    zero_raw: float = 0.
    current_scale_a_per_unit: float = 1.
    current_direction: int = 1

    def __post_init__(self):
        if not self.name or not self.model_joint:
            raise ValueError("Joint names cannot be empty")
        if self.direction not in (-1, 1) or self.current_direction not in (-1, 1):
            raise ValueError("Direction must be +1 or -1")
        if not np.all(np.isfinite([self.scale_rad_per_unit,self.zero_raw,self.current_scale_a_per_unit])):
            raise ValueError("Mapping values must be finite")
        if self.scale_rad_per_unit<=0 or self.current_scale_a_per_unit<=0:
            raise ValueError("Unit scales must be positive; use the separate direction field")

    def position(self, raw):
        return self.direction*self.scale_rad_per_unit*(np.asarray(raw)-self.zero_raw)

    def raw_position(self, q):
        return np.asarray(q)/(self.direction*self.scale_rad_per_unit)+self.zero_raw


@dataclass(frozen=True)
class StateFrame:
    robot_model_id: str
    source: str
    sequence: int
    sample_ns: int
    receive_ns: int
    joint_names: tuple[str, ...]
    q: np.ndarray
    dq: np.ndarray | None
    current: np.ndarray | None
    raw_position: np.ndarray | None
    target_q: np.ndarray | None = None
    status: int | None = 0
    clock_domain: str = "recorded_device_clock"

    def __post_init__(self):
        n=len(self.joint_names)
        object.__setattr__(self,"joint_names",tuple(self.joint_names))
        if not n or len(set(self.joint_names))!=n:
            raise ValueError("Frame requires unique joint names")
        if not self.source or not self.robot_model_id or not self.clock_domain:
            raise ValueError("Source, model and clock domain are required")
        if self.sequence<0 or self.sample_ns < -1 or self.receive_ns<0:
            raise ValueError("Invalid sequence/timestamp")
        for name in ("q","dq","current","raw_position","target_q"):
            original=getattr(self,name)
            if original is None and name in ("target_q","raw_position","dq","current"):
                continue
            array=np.array(original,dtype=float,copy=True)
            if array.shape!=(n,) or not np.all(np.isfinite(array)):
                raise ValueError(f"{name} must have one finite value per joint")
            array.setflags(write=False)
            object.__setattr__(self,name,array)


class TelemetryAdapter:
    """One explicit raw-to-model mapping for Mock, recorded SDK data or live data.

    Raw velocity must be in raw position units per second, target_q in model
    radians. No unwrap is guessed: an adapter must supply unwrapped raw position.
    """
    def __init__(self, robot_model_id: str, mappings: list[JointMapping]):
        self.robot_model_id=robot_model_id
        self.mappings=tuple(mappings)
        for field in ("name","model_joint"):
            names=[getattr(m,field) for m in mappings]
            if not names or len(names)!=len(set(names)):
                raise ValueError(f"Mappings require unique {field}")

    @classmethod
    def load(cls,path):
        config=json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(config["robot_model_id"],[JointMapping(**m) for m in config["joints"]])

    def convert(self, packet: dict) -> StateFrame:
        if packet.get("robot_model_id")!=self.robot_model_id:
            raise ValueError("Telemetry robot model does not match mapping")
        for key in ("sequence","sample_ns","receive_ns"):
            value=packet[key]
            if isinstance(value,bool) or not isinstance(value,(int,np.integer)) or not -(2**63)<=value<2**63:
                raise ValueError(f"{key} must be an int64 value")
        raw_names=packet["joint_names"]
        if len(raw_names)!=len(set(raw_names)) or set(raw_names)!={m.name for m in self.mappings}:
            raise ValueError("Telemetry joints must exactly match configured joints")
        order=[raw_names.index(m.name) for m in self.mappings]
        def raw(key):
            value=np.asarray(packet[key],dtype=float)
            if value.shape!=(len(order),):
                raise ValueError(f"Bad telemetry shape for {key}")
            return value[order]
        position,velocity,current=(raw(k) for k in ("raw_position","raw_velocity","raw_current"))
        q=np.array([m.position(v) for m,v in zip(self.mappings,position,strict=True)])
        dq=np.array([m.direction*m.scale_rad_per_unit*v for m,v in zip(self.mappings,velocity,strict=True)])
        amps=np.array([m.current_direction*m.current_scale_a_per_unit*v for m,v in zip(self.mappings,current,strict=True)])
        target=raw("target_q") if packet.get("target_q") is not None else None
        return StateFrame(self.robot_model_id,packet["source"],int(packet["sequence"]),int(packet["sample_ns"]),
            int(packet["receive_ns"]),tuple(m.name for m in self.mappings),q,dq,amps,position,target,
            int(packet.get("status",0)),packet.get("clock_domain","device_clock_unsynchronized"))


class FrameSource(Protocol):
    def read(self, presentation_ns: int) -> StateFrame | None: ...


class ReplaySource:
    """Sample-and-hold playback of acquired samples; never fabricate motion."""
    def __init__(self, observations, adapter: TelemetryAdapter, commands=None, source="cpp_mock"):
        if len(adapter.mappings)!=1:
            raise ValueError("This recording adapter expects a single-joint calibration run")
        self.adapter=adapter;self.observations=observations;self.commands=commands;self.source=source
        valid=np.isfinite(observations["q"]) & ((observations["status"]&2)==0)
        self.indices=np.flatnonzero(valid)
        if not len(self.indices):
            raise ValueError("No playable samples")
        # Receive times provide availability order and remain meaningful when
        # device acquisition time is unknown. Preserve sample time in the frame.
        self.times=np.asarray(observations["receive_ns"])[self.indices]
        order=np.argsort(self.times,kind="stable")
        self.indices=self.indices[order];self.times=self.times[order]

    @classmethod
    def load(cls,folder,adapter,source="recording"):
        from .data import load_observations
        folder=Path(folder)
        commands=None
        if (folder/"commands.npz").exists():
            with np.load(folder/"commands.npz",allow_pickle=False) as data:
                commands={k:data[k] for k in data.files}
        observations=load_observations(folder)
        if (folder/"raw_telemetry.npz").exists():
            with np.load(folder/"raw_telemetry.npz",allow_pickle=False) as data:
                observations.update({k:data[k] for k in data.files})
        return cls(observations,adapter,commands,source)

    def read(self,presentation_ns):
        cursor=np.searchsorted(self.times,int(presentation_ns),side="right")-1
        if cursor<0:
            return None
        i=self.indices[cursor];o=self.observations;m=self.adapter.mappings[0]
        target=None
        if self.commands is not None:
            j=np.searchsorted(self.commands["send_ns"],presentation_ns,side="right")-1
            if j>=0 and self.commands["commands"][j,0]==1:
                target=[float(self.commands["commands"][j,1])]
        raw_keys=("raw_position","raw_velocity","raw_current")
        if not any(k in o for k in raw_keys):
            # Legacy recordings already store canonical SI observations. Never
            # fabricate encoder counts using the very mapping being checked.
            return StateFrame(self.adapter.robot_model_id,self.source,int(o["sequence"][i]),
                int(o["sample_ns"][i]),int(o["receive_ns"][i]),(m.name,),[o["q"][i]],
                [o["dq"][i]],[o["current"][i]],None,target,int(o["status"][i]),"recorded_virtual_clock")
        if not all(k in o for k in raw_keys):raise ValueError("Raw telemetry requires all three raw channels")
        return self.adapter.convert({"robot_model_id":self.adapter.robot_model_id,"source":self.source,
            "sequence":int(o["sequence"][i]),"sample_ns":int(o["sample_ns"][i]),
            "receive_ns":int(o["receive_ns"][i]),"joint_names":[m.name],
            "raw_position":[float(o["raw_position"][i])],
            "raw_velocity":[float(o["raw_velocity"][i])],
            "raw_current":[float(o["raw_current"][i])],
            "target_q":target,"status":int(o["status"][i]),"clock_domain":"recorded_virtual_clock"})


class LatestStateSource:
    """Bounded latest-only handoff. Renderer cannot block acquisition or write motors."""
    def __init__(self,adapter,stale_after_ns=250_000_000):
        self.adapter=adapter;self.stale_after_ns=stale_after_ns
        self._lock=Lock();self._frame=None;self._arrival_ns=0

    def publish(self,packet,*,arrival_ns=None):
        frame=self.adapter.convert(packet)
        with self._lock:
            if self._frame is not None and frame.sequence<=self._frame.sequence:
                return False
            self._frame=frame;self._arrival_ns=monotonic_ns() if arrival_ns is None else arrival_ns
        return True

    def read(self,presentation_ns=0):
        with self._lock:
            return self._frame

    def age_ns(self,now_ns=None):
        with self._lock:
            if self._frame is None:
                return None
            return max(0,(monotonic_ns() if now_ns is None else now_ns)-self._arrival_ns)

    def is_stale(self,now_ns=None):
        age=self.age_ns(now_ns)
        return age is None or age>self.stale_after_ns
