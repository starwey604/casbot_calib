"""Explicit source-model resources; no default assembly, reduction or physics loop."""
from dataclasses import dataclass
from hashlib import sha256
from importlib.resources import files
import json
from pathlib import Path, PurePosixPath
import xml.etree.ElementTree as ET


MODEL_ID='casbot_hl12p1_dvt2'


def model_directory():
    """Resource root in a normal source checkout or unpacked Python installation."""
    return Path(str(files('casbot_calib').joinpath('assets/models')))


def _inside(root,path):
    root=Path(root).resolve();result=(root/path).resolve()
    if not result.is_relative_to(root):raise ValueError('Model resource escapes its bundle: '+str(path))
    return result


def _resource(path):
    if not path.is_file():raise FileNotFoundError('Missing model resource: '+str(path))
    with path.open('rb') as stream:prefix=stream.read(80)
    if prefix.startswith(b'version https://git-lfs.github.com/spec/v1'):
        raise ValueError('Unhydrated Git LFS model resource; run git lfs pull before building/installing: '+str(path))
    return path


def source_catalog(*,models_directory=None):
    root=(model_directory() if models_directory is None else Path(models_directory))/MODEL_ID
    catalog=json.loads(_resource(root/'catalog.json').read_text(encoding='utf-8'))
    if catalog['schema_version']!='casbot_model_sources_v1' or catalog['model_id']!=MODEL_ID:
        raise ValueError('Unsupported model source catalog')
    return catalog


@dataclass(frozen=True)
class SourceModel:
    model_id:str
    variant:str
    directory:Path
    urdf:Path
    asset_root:Path
    reference_mjcf:Path
    urdf_sha256:str
    calibration_joint_names:tuple[str,...]

    def resolve_asset(self,filename):
        """Resolve the vendor's visual/... or collision/... URI, not process cwd."""
        path=PurePosixPath(filename)
        if path.is_absolute() or '..' in path.parts or ':' in filename or '\\' in filename:
            raise ValueError('Relative asset-root mesh path required: '+filename)
        return _resource(_inside(self.asset_root,path))


def dvt2_source(variant,*,models_directory=None):
    """Require an explicit source variant; none is a released calibration model."""
    directory=((model_directory() if models_directory is None else Path(models_directory))/MODEL_ID).resolve()
    catalog=source_catalog(models_directory=models_directory)
    if variant not in catalog['variants']:
        raise ValueError('Choose an explicit DVT2 source variant: '+', '.join(catalog['variants']))
    info=catalog['variants'][variant];urdf=_resource(_inside(directory,info['urdf']))
    # This digest binds the supplied URDF identity used by the model contracts.
    if sha256(urdf.read_bytes()).hexdigest()!=info['urdf_sha256']:
        raise ValueError('DVT2 source URDF differs from its catalog: '+variant)
    return SourceModel(MODEL_ID,variant,directory,urdf,_inside(directory,catalog['asset_root']),
        _resource(_inside(directory,info['reference_mjcf'])),info['urdf_sha256'],tuple(catalog['calibration_joint_names']))


def inspect_source(variant,*,models_directory=None):
    """Read URDF/MJCF references and OBJ material dependencies without simulating."""
    source=dvt2_source(variant,models_directory=models_directory)
    catalog=source_catalog(models_directory=models_directory);info=catalog['variants'][variant]
    root=ET.parse(source.urdf).getroot();joints=root.findall('joint')
    active=[j.get('name') for j in joints if j.get('type')!='fixed']
    if active!=info['active_joint_names'] or not set(source.calibration_joint_names)<=set(active):
        raise ValueError('DVT2 source joint mapping differs from catalog')
    references={source.resolve_asset(mesh.get('filename')) for mesh in root.findall('.//mesh')}
    urdf_mesh_count=len(references)
    reference=ET.parse(source.reference_mjcf).getroot();compiler=reference.find('compiler')
    meshdir=compiler.get('meshdir','') if compiler is not None else ''
    texturedir=compiler.get('texturedir',meshdir) if compiler is not None else meshdir
    for tag,directory in (('mesh',meshdir),('texture',texturedir)):
        for item in reference.findall('.//'+tag):
            if item.get('file'):
                references.add(_resource(_inside(source.directory,source.reference_mjcf.parent/directory/item.get('file'))))
    for item in reference.findall('.//include'):
        references.add(_resource(_inside(source.directory,source.reference_mjcf.parent/item.get('file'))))
    materials=set()
    for mesh in references:
        if mesh.suffix.lower()!='.obj':continue
        with mesh.open(encoding='utf-8') as stream:
            for row in stream:
                if row.startswith('mtllib '):
                    for name in row.split()[1:]:materials.add(_resource(_inside(source.directory,mesh.parent/name)))
    references.update(materials)
    return dict(passed=True,model_id=source.model_id,variant=variant,urdf=str(source.urdf),asset_root=str(source.asset_root),
        source_urdf_sha256=source.urdf_sha256,active_joint_names=active,calibration_joint_names=source.calibration_joint_names,
        urdf_meshes=urdf_mesh_count,referenced_files=len(references),material_files=len(materials),
        links_without_inertial=info['links_without_inertial'],nonpositive_mass_links=info['nonpositive_mass_links'],
        source_only=True,default_model_changed=False,hardware_executable=False,
        files=[str(path.relative_to(source.directory)) for path in sorted(references)])
