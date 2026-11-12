"""Display-only scheduling and disposable mailboxes; never schedules robot commands."""
import json
import os
from pathlib import Path
import time

VIEW_FPS = 10.
VIEW_ANGLE_EPSILON_RAD = 1e-4


def configure_graphics_environment():
    # Mesa software rasterization otherwise creates a worker per CPU, in BOTH
    # graphics processes. Respect an explicit deployment override. This does not
    # affect native physics, DDS, BLAS or OpenMP thread settings.
    os.environ.setdefault('LP_NUM_THREADS', '0')


def latest_json(path, value):
    """Atomic live-view replacement, not a durable evidence/seal write.

    Only one writer owns each mailbox. Readers see a complete old or new value.
    These files can be recreated after a crash and are excluded from snapshots.
    """
    path=Path(path);temporary=path.with_name(path.name+'.pending')
    with temporary.open('w',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,allow_nan=False)
        stream.write('\n')
    temporary.replace(path)


def geometry_changed(pose, previous):
    # Sub-pixel angular noise need not redraw 3D. Compare with the last rendered
    # pose (not the previous packet), so arbitrarily slow motion accumulates.
    if previous is None or pose.get('selected')!=previous.get('selected'):return True
    return any(abs(a-b)>VIEW_ANGLE_EPSILON_RAD for a,b in zip(pose['q'],previous['q'],strict=True))


def lightweight_scene(context, canonical):
    """Replace only visual mesh resources for the explicitly supported frozen scene.

    Body/joint transforms, geometry placement/scale/material and SDK mapping stay
    intact. The source bundle, URDF and collision meshes are never modified.
    Other contexts keep their canonical display geometry.
    """
    import xml.etree.ElementTree as ET
    assets=Path(__file__).parent/'assets/fieldwork_display/dvt2_v1'
    manifest=json.loads((assets/'manifest.json').read_text())
    if any(context.identity.get(key)!=manifest[key] for key in ('model_id','bundle_sha256','urdf_sha256')):
        return Path(canonical)
    tree=ET.parse(canonical);meshes={node.attrib['name']:node for node in tree.findall('./asset/mesh')}
    if set(meshes)!={row['name'] for row in manifest['meshes']}:
        raise ValueError('轻量显示网格清单与绑定模型不符')
    for row in manifest['meshes']:
        node=meshes[row['name']]
        if Path(node.attrib['file']).name!=row['source']:raise ValueError('轻量显示来源不符：'+row['name'])
        resource=assets/row['file']
        if not resource.is_file():raise ValueError('缺少轻量显示资源：'+str(resource))
        node.set('file',os.path.relpath(resource,Path(canonical).parent))
    output=Path(canonical).with_name('scene_display.xml')
    temporary=output.with_suffix('.pending');tree.write(temporary,encoding='unicode');temporary.replace(output)
    return output
