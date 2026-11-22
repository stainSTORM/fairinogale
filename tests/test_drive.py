"""pick_up and release_at with the mock arm and the simulated camera: the arm must go where the
station went, and stay out when the look says so."""

import copy

import numpy as np
import pytest

import app
import vision
from vision import T_from_pose

STATION, TAGS, CARRIER = "opentrons", [10, 11], [100]
TASK = app.ConsoleTask()


@pytest.fixture
def cell():
    """A freshly taught station: reference saved, carrier on the seat, arm at home."""
    rbt, cam = app.robot(), app.camera("wrist")
    rbt.move_time, rbt.gripper_stop = 0, None
    before = copy.deepcopy(app.cfg), dict(cam.scene)
    app.look_at(STATION, "", True, False, 20, 70, 10, TASK)
    rbt.moves.clear()
    yield rbt, cam
    cam.scene = before[1]
    app.cfg.clear()
    app.cfg.update(before[0])


def move_station(cam, dx, dy, yaw, ids=TAGS + CARRIER):
    """Moves the station in its own plane (user frame); returns the 4x4 transform."""
    plane = cam.scene[TAGS[0]]
    D = plane @ T_from_pose([dx, dy, 0, 0, 0, yaw]) @ np.linalg.inv(plane)
    cam.move(ids, D)
    return D


def taught(name):
    return app.station_points(STATION)[2][name][0:6]


def linear_moves(rbt):
    return [pose for kind, pose in rbt.moves if kind == "L"]


def test_unmoved_station_is_driven_as_taught(cell):
    rbt, _ = cell
    app.drive(STATION, "", False, 20, 70, 10, TASK)
    assert linear_moves(rbt)[:3] == [taught("move7"), taught("move8"), taught("move9")]


def test_pick_follows_the_station(cell):
    rbt, cam = cell
    D = move_station(cam, 2.5, -1.5, 1.0)
    app.drive(STATION, "", False, 20, 70, 10, TASK)
    for pose, name in zip(linear_moves(rbt), ["move7", "move8", "move9"]):
        expected = D @ T_from_pose(taught(name))
        assert np.linalg.norm(T_from_pose(pose)[:3, 3] - expected[:3, 3]) < 0.3, name
    assert rbt.moves[-1][1] == taught("home")


def test_release_follows_the_station_too(cell):
    rbt, cam = cell
    del cam.scene[100]  # the seat is free, the carrier is in the gripper
    D = move_station(cam, -2.0, 1.0, 0.0, TAGS)
    app.drive(STATION, "", True, 20, 70, 10, TASK)
    expected = D @ T_from_pose(taught("move8"))
    assert np.linalg.norm(T_from_pose(linear_moves(rbt)[1])[:3, 3] - expected[:3, 3]) < 0.3


def test_station_too_far_off_backs_out(cell):
    rbt, cam = cell
    move_station(cam, 9.0, 0.0, 0.0)
    with pytest.raises(app.StationMoved, match="shift"):
        app.drive(STATION, "", False, 20, 70, 10, TASK)
    assert not linear_moves(rbt) and rbt.moves[-1][1] == taught("home")
    assert rbt.gripper_state["position"] != app.cfg["gripper"]["close"]


def test_hidden_tag_fails_or_drives_uncorrected(cell):
    rbt, cam = cell
    del cam.scene[11]
    with pytest.raises(app.TagsNotSeen, match="11"):
        app.drive(STATION, "", False, 20, 70, 10, TASK)
    app.cfg["stations"][STATION]["vision"]["on_no_tag"] = "uncorrected"
    rbt.moves.clear()
    app.drive(STATION, "", False, 20, 70, 10, TASK)
    assert linear_moves(rbt)[:3] == [taught("move7"), taught("move8"), taught("move9")]


def test_empty_seat_stops_a_pick_and_taken_seat_a_release(cell):
    rbt, cam = cell
    with pytest.raises(app.CarrierMisplaced, match="no carrier should be"):
        app.drive(STATION, "", True, 20, 70, 10, TASK)
    del cam.scene[100]
    with pytest.raises(app.CarrierMisplaced, match="a carrier should be"):
        app.drive(STATION, "", False, 20, 70, 10, TASK)
    assert not linear_moves(rbt)


def test_carrier_off_its_seat_stops_a_pick(cell):
    _, cam = cell
    cam.move(CARRIER, T_from_pose([4.0, 0, 0, 0, 0, 0]))
    with pytest.raises(app.CarrierMisplaced, match="mm from the seat"):
        app.drive(STATION, "", False, 20, 70, 10, TASK)


def test_correct_false_ignores_the_camera(cell):
    rbt, cam = cell
    move_station(cam, 9.0, 0.0, 0.0)
    app.drive(STATION, "", False, 20, 70, 10, TASK, correct=False)
    assert linear_moves(rbt)[:3] == [taught("move7"), taught("move8"), taught("move9")]


def test_check_station_reports_instead_of_failing(cell):
    _, cam = cell
    move_station(cam, 9.0, 0.0, 0.0)
    check = app.station_check(app.look_at(STATION, "", False, False, 20, 70, 10, TASK))
    assert not check.ok and "shift" in check.problem and check.carrier_on_seat


def test_no_reference_is_an_error(cell):
    vision.reference_file(app.station_points(STATION)[1]).unlink()
    with pytest.raises(app.VisionError, match="No reference"):
        app.drive(STATION, "", False, 20, 70, 10, TASK)


def test_gripper_closing_on_air_fails_the_pick(cell):
    rbt, _ = cell
    app.cfg["gripper"]["holding"] = [80, 88]
    with pytest.raises(app.GripFailed):
        app.drive(STATION, "", False, 20, 70, 10, TASK)
    rbt.gripper_stop = 84
    app.drive(STATION, "", False, 20, 70, 10, TASK)
