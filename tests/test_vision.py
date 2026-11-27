"""The vision maths against the simulated camera: known station moves must come back out.

    uv run pytest
"""

import cv2
import numpy as np
import pytest

import vision
from mock.MockCamera import MockCamera
from vision import T_from_pose, pose_from_T

SIZE = 30.0
LOOK = [84.3, 108.5, -171.0, 140.4, 0.6, 90.6]  # a look pose with a tilted tool, like the Jena points
MOUNT = [0, -60, -80, -15, 0, 0]
STATION, CARRIER = [10, 11], [100]


def scene(**spec):
    """A camera at the look pose over two station tags and a carrier tag; returns it and the reference."""
    camera = MockCamera("wrist", {"mount": MOUNT, "fov_deg": 70, **spec}, lambda: LOOK, SIZE)
    camera.place(STATION, LOOK)
    camera.place(CARRIER, LOOK, row=45)
    image = camera.grab()
    found = vision.detect(image)
    reference = {"size": [image.shape[1], image.shape[0]], "carrier": found[100].tolist(),
                 "tags": {str(i): found[i].tolist() for i in STATION}}
    return camera, reference


def planar_move(camera, dx, dy, yaw_deg):
    """A move of the station inside its own plane, about the middle of its tags (user frame)."""
    a, b = camera.scene[10], camera.scene[11]
    middle = np.eye(4)
    middle[:3, :3], middle[:3, 3] = a[:3, :3], (a[:3, 3] + b[:3, 3]) / 2
    return middle @ T_from_pose([dx, dy, 0, 0, 0, yaw_deg]) @ np.linalg.inv(middle)


def test_pose_round_trip():
    for pose in ([10, -20, 30, 140.4, 0.6, 90.6], [0, 0, 0, -109.8, 0.1, -178.8], [1, 2, 3, 20, 90, 0]):
        again = pose_from_T(T_from_pose(pose))
        assert np.allclose(T_from_pose(again), T_from_pose(pose), atol=1e-9)


def test_detects_the_rendered_tags():
    camera, _ = scene()
    assert sorted(vision.detect(camera.grab())) == [10, 11, 100]


def test_no_move_measures_nothing():
    camera, reference = scene()
    m = vision.measure(reference, vision.detect(camera.grab()), camera, LOOK, SIZE, STATION, CARRIER)
    assert m.shift_mm < 0.02 and abs(m.yaw_deg) < 0.02 and m.carrier_mm < 0.05


@pytest.mark.parametrize("dx, dy, yaw", [(2.0, -1.5, 0.0), (-3.0, 2.0, 1.5), (0.0, 0.0, -2.0)])
def test_station_move_is_recovered(dx, dy, yaw):
    camera, reference = scene()
    D = planar_move(camera, dx, dy, yaw)
    camera.move(STATION + CARRIER, D)
    m = vision.measure(reference, vision.detect(camera.grab()), camera, LOOK, SIZE, STATION, CARRIER)
    assert m.shift_mm == pytest.approx(np.hypot(dx, dy), abs=0.15)
    assert abs(m.yaw_deg) == pytest.approx(abs(yaw), abs=0.1)
    assert abs(m.lift_mm) < 0.3 and m.tilt_deg < 0.3
    assert m.carrier_mm < 0.5  # the carrier moved with its station: still on the seat
    # A taught pose 150 mm from the tags must land where the station took it.
    taught = pose_from_T(camera.scene[10] @ T_from_pose([150, 40, 20, 180, 0, 30]))
    expected = D @ T_from_pose(taught)
    got = T_from_pose(m.corrected(taught))
    assert np.linalg.norm(got[:3, 3] - expected[:3, 3]) < 0.3
    assert np.degrees(np.linalg.norm(vision.rotation_vector(got[:3, :3].T @ expected[:3, :3]))) < 0.15


def test_guessed_field_of_view_is_enough():
    """The camera is not calibrated: 62 deg assumed where 70 is true."""
    camera, reference = scene()
    D = planar_move(camera, 3.0, -2.0, 1.0)
    camera.move(STATION, D)
    guessed = vision.Camera("wrist", {"mount": MOUNT, "fov_deg": 62})
    m = vision.measure(reference, vision.detect(camera.grab()), guessed, LOOK, SIZE, STATION)
    assert m.shift_mm == pytest.approx(np.hypot(3.0, 2.0), abs=0.3)
    assert m.yaw_deg == pytest.approx(1.0, abs=0.15) or m.yaw_deg == pytest.approx(-1.0, abs=0.15)


def test_carrier_off_its_seat():
    camera, reference = scene()
    camera.move(CARRIER, T_from_pose([2.0, 0, 0, 0, 0, 0]))
    m = vision.measure(reference, vision.detect(camera.grab()), camera, LOOK, SIZE, STATION, CARRIER)
    assert m.carrier_tag == 100 and m.carrier_mm == pytest.approx(2.0, abs=0.6)  # one small tag: depth is the weak axis
    assert m.shift_mm < 0.05


def test_no_carrier_in_view():
    camera, reference = scene()
    del camera.scene[100]
    m = vision.measure(reference, vision.detect(camera.grab()), camera, LOOK, SIZE, STATION, CARRIER)
    assert m.carrier_mm is None


def test_reference_file(tmp_path):
    camera, _ = scene()
    points = tmp_path / "station.json"
    vision.save_reference(points, "move6", camera, camera.grab(), STATION, CARRIER)
    saved = vision.load_references(points)["move6"]
    assert sorted(saved["tags"]) == ["10", "11"] and saved["carrier"] is not None
    assert (tmp_path / "station.vision.move6.jpg").exists()
    with pytest.raises(RuntimeError, match="does not see"):
        vision.save_reference(points, "move6", camera, camera.grab(), [10, 12])


def test_camera_angles_from_jogs():
    """debug.py mount: six straight jogs give the camera's angles in the tool frame."""
    pose = list(LOOK)
    camera = MockCamera("wrist", {"mount": [0, -60, -80, -15, 4, 30], "fov_deg": 70}, lambda: pose, SIZE)
    camera.place(STATION, LOOK)
    K, start, jogs = camera.matrix(camera.width, camera.height), vision.detect(camera.grab()), []
    for axis in range(3):
        for sign in (10.0, -10.0):
            jog = [sign if k == axis else 0.0 for k in range(3)]
            pose[:3] = [a + b for a, b in zip(LOOK[:3], jog)]
            jogs.append((jog, vision.displacement(start, vision.detect(camera.grab()), K, camera.dist, SIZE)[0]))
    angles, misfit = vision.mount_rotation(jogs, LOOK)
    assert angles == pytest.approx([-15, 4, 30], abs=1.0) and misfit < 0.3
