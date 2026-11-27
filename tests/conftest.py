"""app.py reads config.yaml when it is imported: point it at a copy of the mock setup before that."""

import os
import shutil
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
folder = Path(tempfile.mkdtemp(prefix="fairinogale-test-"))
shutil.copy(ROOT / "mock" / "cell.yaml", folder / "config.yaml")  # teach points: copied by app.py
os.environ["FAIRINO_CONFIG"] = str(folder / "config.yaml")
