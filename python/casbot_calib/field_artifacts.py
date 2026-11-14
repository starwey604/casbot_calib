"""Immutable field bundles shared by SDK and SHM; no transport assumptions."""
import json
from pathlib import Path
from .data import write_json
from .artifact_io import sha256


class FieldContractError(ValueError):
    """Changed sealed inputs, execution mismatch or failed geometric proof."""


def seal(root):
    root = Path(root)
    write_json(root / 'manifest.json', {str(p.relative_to(root)): sha256(p)
        for p in sorted(root.rglob('*')) if p.is_file() and p != root / 'manifest.json'})


def output_outside(output, *inputs):
    path = Path(output).resolve()
    if any(path == Path(root).resolve() or Path(root).resolve() in path.parents for root in inputs):
        raise ValueError('Field output must be outside frozen input bundles')
    return path


def check_seal(root):
    root = Path(root); manifest = json.loads((root / 'manifest.json').read_text())
    # Static preset manifests wrap the map; field bundles use the map directly.
    entries = manifest.get('files', manifest)
    actual = {str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() and p != root / 'manifest.json'}
    if actual != set(entries): raise FieldContractError('Field manifest coverage changed')
    for name, digest in entries.items():
        path = (root / name).resolve()
        if not path.is_relative_to(root.resolve()) or sha256(path) != digest:
            raise FieldContractError('Frozen field input changed: ' + name)
