"""The app in Arkitekt's local runtime (no server): the trajectories state follows the moves, and
the stations on offer follow the points folder without a restart."""

import json
import time

import app
from arkitekt.testing import local_app


@app.app.action
def peek(state: app.TrajectoryState) -> str:
    """Peek"""
    return f"{state.current_position} {state.gripping} {' '.join(state.available_trajectories)}"


def test_state_follows_moves_and_stations():
    with local_app(app.app) as run:
        assert run.call_local(peek).startswith("home False opentrons pickupstation ")

        run.call_local(app.pick_up, station="opentrons", correct=False)
        assert run.call_local(peek).startswith("opentrons True ")
        run.call_local(app.release_at, station="pickupstation", correct=False)
        assert run.call_local(peek).startswith("pickupstation False ")
        run.call_local(app.home_robot)
        assert run.call_local(peek).startswith("home False ")

        point = [0.0] * 20
        run.call_local(app.upload_points, station="extra", points=json.dumps({"home": point}))
        assert " extra" in run.call_local(peek)

        (app.POINTS / "dropped.json").write_text(json.dumps({"home": point}))
        deadline = time.monotonic() + app.RESCAN_S + 5
        while "dropped" not in run.call_local(peek) and time.monotonic() < deadline:
            time.sleep(0.5)
        assert " dropped" in run.call_local(peek)
