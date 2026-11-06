"""The selected DVT2 source shared by dynamics, geometry, resources and transport.

N3 migration callers pass this context explicitly until the atomic default
switch. No alternate/legacy model selection or download-directory fallback.
"""
from copy import deepcopy
from dataclasses import dataclass
from importlib.resources import files
import json,os
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from . import _core
from .model_sources import model_directory

BUNDLE_RELATIVE='casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2'
MODEL_ID='casbot_hl12p1_dvt2_bare_flange_1105_03_gauge_v2'


@dataclass(frozen=True)
class ModelContext:
    bundle:Path
    _model:object
    _metadata:dict

    @property
    def engine(self):return _core.ArmDynamics(self._model)

    @property
    def model_id(self):return self._metadata['model_id']

    @property
    def urdf(self):return self.bundle/'model.urdf'

    @property
    def joint_names(self):return tuple(self._metadata['joint_names'])

    @property
    def home(self):return np.array(self._metadata['profile']['home_rad'],dtype=float)

    @property
    def identity(self):
        return dict(schema_version='casbot_model_context_v1',model_id=self.model_id,
                    bundle_sha256=self._metadata['bundle_sha256'],urdf_sha256=self._metadata['urdf_sha256'])

    @property
    def metadata(self):return deepcopy(self._metadata)

    @property
    def source(self):
        return dict(source=self.urdf.relative_to(model_directory()).as_posix(),sha256=self._metadata['urdf_sha256'],
            geometry='DVT2 fixed-waist bare-flange CAD, not calibrated',inertias='source CAD prior, not hardware truth',
            hands='ordinary hands removed; fixed source inertias retained once',fixture='1105_03 CAD registration, not field metrology',
            hardware_validated=False,fixture_transform_known=False,waist_fixed=True,tool_load_known=False,model_context=self.identity)

    def transport_parameters(self):
        """Model portion only; does not grant a ROS endpoint or hardware scope."""
        return dict(model_path=str(self.urdf),model_id=self.model_id,joint_names=list(self.joint_names))

    def collision_profile(self,*,active_tool=None,ball_id=None,contact=False,engine=None):
        from .dvt2_bundle import robot_profile
        from .trical_assembly import profile
        from .collision import CollisionProfile
        from .kinematics import geometry_manifest
        if active_tool is None:
            if ball_id is not None or contact:raise ValueError('A selected active tool is required for TriCal contact')
            result=robot_profile(self.bundle)
        else:result=profile(self.bundle,active_tool,ball_id,contact)
        if engine is not None:
            if geometry_manifest(engine,'dvt2_locked_v1')['geometry_sha256']!=self._metadata['geometry']['geometry_sha256']:
                raise ValueError('Candidate geometry requires rebuilt collision assets')
            enabled=set(result.pairs);n=len(result.geometry)
            exclusions=[(i,j) for i in range(n) for j in range(i+1,n) if (i,j) not in enabled]
            clone=CollisionProfile(engine,result.geometry,exclusions,result.metadata)
            if hasattr(result,'pair_clearance_m'):clone.pair_clearance_m=result.pair_clearance_m.copy()
            result=clone
        return result

    def kinematic_config(self):
        """K1 observation model only, not a K2/K3 collector or a mastered device."""
        from .trical_assembly import asset_directory,asset_manifest
        from .kinematics import validate_config
        defaults=json.loads(files('casbot_calib').joinpath('assets/trical_measurement_defaults.json').read_text(encoding='utf-8'))
        assembly=asset_manifest(asset_directory(self.bundle));axes=assembly['axes_sensor']
        if not np.allclose(axes['left'],axes['right'],rtol=0,atol=1e-12):raise ValueError('This common-axis K1 profile requires separately handled left/right calibrations')
        indicators=deepcopy(defaults['indicators']);indicators['axes_sensor']=axes['left'];indicators['axes_source']='1105_03 CAD; zero/range remain synthetic Mock values'
        waist=np.array(assembly['setup']['cad_to_robot']['world_waist_m'])
        balls=[dict(id=b['id'],center_m=(waist+np.array(b['center_relative_waist_mm'])/1000).tolist()) for b in assembly['setup']['balls']]
        config=dict(schema_version=1,profile_id='dvt2_trical_k1_context_v1',robot_model_id=self.model_id,
            geometry_version='dvt2_1105_03_working_state_v2',urdf_sha256=self.source['sha256'],control_period_ns=2_000_000,
            synthetic_only=True,hardware_executable=False,root_frame='calib_world',tools=deepcopy(self._metadata['profile']['tools']),
            fixture=dict(id='trical_1105_03',frame='trical_artifact',world_translation_m=[0.,0.,0.],world_rotation_vector_rad=[0.,0.,0.],
                         ball_radius_m=.0127,balls=balls,provenance='Registered source CAD, not surveyed fixture metrology'),
            indicators=indicators,parameter_policy=deepcopy(defaults['parameter_policy']),model_context=self.identity)
        validate_config(config);return config

    def write_view_scene(self,output,*,active_tool=None):
        """Reuse frozen CAD visuals; remove tool/bench for bare dynamics view."""
        from .dvt2_bundle import viewer_adapter
        if active_tool not in {None,'left','right'}:raise ValueError('Unknown tool side')
        output=Path(output)
        if output.exists():raise FileExistsError(output)
        output.parent.mkdir(parents=True,exist_ok=True);source=self.bundle/('scene_'+(active_tool or 'left')+'.xml')
        tree=ET.parse(source);root=tree.getroot()
        if active_tool is None:
            for parent in root.iter():
                for item in list(parent):
                    if item.tag=='geom' and item.get('name','').startswith(('fixture_','datum_')):parent.remove(item)
                    elif item.tag=='site' and item.get('name','').endswith('_trical_tcp'):parent.remove(item)
            used={g.get('mesh') for g in root.iter('geom')}
            asset=root.find('asset')
            for item in list(asset):
                if item.tag=='mesh' and item.get('name') not in used:asset.remove(item)
        for item in root.findall('./asset/mesh'):
            if item.get('file'):item.set('file',Path(os.path.relpath((source.parent/item.get('file')).resolve(),output.parent)).as_posix())
        tree.write(output,encoding='unicode');return viewer_adapter(self.bundle)


def load_context():
    from .dvt2_bundle import load_bundle
    bundle=model_directory()/BUNDLE_RELATIVE;engine,metadata=load_bundle(bundle)
    if metadata['model_id']!=MODEL_ID or not metadata['assembly_confirmed']:raise ValueError('Selected DVT2 assembly identity differs')
    return ModelContext(bundle,engine.model,metadata)


def resolve_context(identity,context=None):
    context=load_context() if context is None else context
    if identity!=context.identity:raise ValueError('Artifact model context differs from selected DVT2 source')
    return context
