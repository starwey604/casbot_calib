"""Source-relative files needed for headless dynamics and TriCal collision checks."""
from pathlib import Path

MODELS = Path('python/casbot_calib/assets/models')
BUNDLE = MODELS / 'casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2'
TRICAL = MODELS / 'trical_1105_03_gauge_cover_v2'
FILES = tuple(BUNDLE / name for name in (
    'model.json', 'model.urdf', 'collision.json', 'collision.npz', 'trical_scene.json')) + tuple(
    TRICAL / name for name in ('manifest.json', 'collision.json', 'collision.npz'))

if __name__ == '__main__':
    print('\n'.join(map(str, FILES)))
