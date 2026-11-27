"""teach_station with the mock arm and camera: the points are recorded where the person guided the
arm, the reference is saved at the look point (asked again until the tags are in view), and
auto-mode cannot teach."""

from types import SimpleNamespace

import pytest
from arkitekt.testing import local_app
from arkitekt_spec.declare.blok.validate import validate_bsx

import app
import vision

STATION = "pickupstation"  # look point move5, tags 0 and 1 (mock/cell.yaml)


class Person:
    """Orkestrator and the person at the arm: before each approval the mock arm is put where the
    person guided it; None denies."""

    def __init__(self, *poses):
        self.poses, self.asked = list(poses), []

    def hold(self, message, prompt=None, **_):
        self.asked.append(prompt.title)
        pose = self.poses.pop(0)
        if pose is not None:
            rpc = app.connection()
            rpc.tcp_pose, rpc.joint_positions = list(pose[0:6]), list(pose[6:12])
        return SimpleNamespace(approved=pose is not None, auto=False)

    def progress(self, percent, message=""):
        pass


@pytest.fixture
def files():
    """The station's files, put back afterwards: test_arkitekt drives the station too."""
    points = app.station_file(STATION)[1]
    reference = vision.reference_file(points)
    kept = {f: f.read_bytes() for f in (points, reference) if f.exists()}
    yield points
    for f in (points, reference):
        f.write_bytes(kept[f]) if f in kept else f.unlink(missing_ok=True)


def test_points_and_reference_are_taught_where_the_arm_was_guided(files):
    taught = app.station_points(STATION)[2]
    move4 = [v + 1 for v in taught["move4"][0:12]]  # guided a little off the old pose
    away = [v + 2000 if i < 3 else v for i, v in enumerate(taught["move5"][0:12])]  # tags not in view
    person = Person(move4, away, taught["move5"])
    state = app.TrajectoryState()

    result = app.teach_station.__wrapped__(state, STATION, ["move4", "move5"], task=person)

    assert person.asked == [f"{STATION}: guide the arm to move4 (1 of 2)",
                            f"{STATION}: guide the arm to move5 (2 of 2)",
                            f"{STATION}: the camera's view from move5"]
    assert "camera reference of move5" in result
    now = app.station_points(STATION)[2]
    assert now["move4"][0:12] == move4 and now["move5"][0:12] == taught["move5"][0:12]
    assert {k: v for k, v in now.items() if k not in ("move4", "move5")} == \
           {k: v for k, v in taught.items() if k not in ("move4", "move5")}
    assert sorted(vision.load_references(files)["move5"]["tags"]) == ["0", "1"]
    assert state.teach_places == ["move4", "move5"] and state.teaching == ""
    assert state.teach_image.startswith("data:image/jpeg;base64,")

    seen = app.observe(STATION, "", "move5")  # the arm is still at the look point
    assert seen["error"] is None and seen["measurement"].shift_mm < 0.1


def test_auto_mode_does_not_teach(files):
    before = files.read_bytes()
    # a local run approves every hold by itself, like auto-mode
    with local_app(app.app) as run, pytest.raises(Exception, match="auto-mode"):
        run.call_local(app.teach_station, station=STATION, points=["move4"])
    assert files.read_bytes() == before


def test_teach_panel_is_valid_bsx():
    assert validate_bsx(app.TEACH_PANEL) == []  # also no warnings: components and props exist
