"""Preserve logical mesh/texture resources when an exported URDF changes folder."""
import os
from pathlib import Path


def resource_elements(root):
    return [*root.findall('.//mesh'),*root.findall('.//texture')]


def resolved_resource(filename,urdf):
    if '://' in filename:return filename
    path=Path(filename)
    return str((path if path.is_absolute() else Path(urdf).parent/path).resolve())


def rebase_resources(root,source,destination):
    for item in resource_elements(root):
        filename=item.get('filename')
        if filename and '://' not in filename:
            item.set('filename',Path(os.path.relpath(resolved_resource(filename,source),Path(destination).parent)).as_posix())
    return root


def canonical_resources(root,source):
    for item in resource_elements(root):
        if item.get('filename'):item.set('filename',resolved_resource(item.get('filename'),source))
    return root
