"""app.py reads config.yaml when it is imported: point it at a mock setup before that."""

import os
import shutil
import tempfile
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
folder = Path(tempfile.mkdtemp(prefix="fairinogale-test-"))
config = yaml.safe_load((ROOT / "config.example.yaml").read_text())
config["robot"]["mock"] = True
config["vision"] = {"tag_size_mm": 30, "cameras": {"wrist": {
    "source": "mock", "fov_deg": 70, "mount": [0, -60, -80, -15, 0, 0]}}}
config["stations"]["opentrons"]["vision"] = {
    "camera": "wrist", "look": "move6", "tags": [10, 11], "carrier_tags": [100],
    "correct": ["move7", "move8", "move9"]}
(folder / "config.yaml").write_text(yaml.safe_dump(config))
shutil.copytree(ROOT / "control-points", folder / "control-points")
os.environ["FAIRINO_CONFIG"] = str(folder / "config.yaml")
