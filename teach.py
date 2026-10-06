"""Save teach points from the Fairino controller into a station file.

Name the poses in the Fairino teach interface first, then:

    uv run teach.py control-points/my_station.json home move1 move2 move3
    docker exec -it <container> python teach.py /config/control-points/my_station.json home move1

The points are written in the given order, which is the order the arm drives them. The robot's
IP comes from config.yaml (FAIRINO_CONFIG). This only reads: it does not enable or move the arm.
"""

import json
import sys
from pathlib import Path

from app import cfg, rpc_class

out, names = Path(sys.argv[1]) if len(sys.argv) > 1 else None, sys.argv[2:]
if not names:
    sys.exit(__doc__)
rbt = rpc_class()(cfg["robot"]["ip"])
points = {}
for name in names:
    error, data = rbt.GetRobotTeachingPoint(name)
    if error != 0 or not data:
        sys.exit(f"Could not read teach point {name!r} (error {error})")
    points[name] = data
    print(f"{name}: {data}")
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(json.dumps(points, indent=4))
print(f"Saved {len(points)} points to {out}")
