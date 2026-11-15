"""Project configuration/output root for source and installed-wheel execution."""
import os
from pathlib import Path

ROOT = Path(os.environ.get('CASBOT_PROJECT_ROOT', Path(__file__).resolve().parents[2])).resolve()
