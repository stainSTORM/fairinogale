"""Where a station is now, compared with when its points were taught.

The stations carry AprilTags (family 36h11). A camera on the gripper looks at them from one of the
station's teach points, the look point. When the station is taught, the tag corners in that image
are saved as the reference. The same look later gives the rigid motion of the tags since then, and
the points at the station are shifted by it. Only the difference to the taught situation is
measured, so the teach points stay the truth and the camera needs no exact calibration.

Frames: a pose is the Fairino [x, y, z, rx, ry, rz] in mm and degrees, R = Rz(rz) Ry(ry) Rx(rx)
(checked against the pose and joint values of the shipped teach points). The camera frame is
OpenCV's: x right, y down, z along the view. All lengths are mm.

This module knows nothing about the robot or Arkitekt: app.py drives, debug.py and vision_ui.py
show what the camera sees.
"""

import json
import threading
import time
import urllib.request
from dataclasses import asdict, dataclass, field
from pathlib import Path

import cv2
import numpy as np

DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)


# --- poses -------------------------------------------------------------------------------------

def T_from_pose(pose) -> np.ndarray:
    """4x4 transform of a Fairino pose [x, y, z, rx, ry, rz] (mm, degrees)."""
    x, y, z, rx, ry, rz = (float(v) for v in pose)
    (sa, sb, sc), (ca, cb, cc) = np.sin(np.radians([rx, ry, rz])), np.cos(np.radians([rx, ry, rz]))
    T = np.eye(4)
    T[:3, :3] = [[cc * cb, cc * sb * sa - sc * ca, cc * sb * ca + sc * sa],
                 [sc * cb, sc * sb * sa + cc * ca, sc * sb * ca - cc * sa],
                 [-sb, cb * sa, cb * ca]]
    T[:3, 3] = x, y, z
    return T


def pose_from_T(T: np.ndarray) -> list[float]:
    """The Fairino pose of a 4x4 transform."""
    R = T[:3, :3]
    if abs(R[2, 0]) > 1 - 1e-9:  # ry = +-90 degrees: rx and rz turn about the same axis
        rx, ry, rz = 0.0, -np.arcsin(np.clip(R[2, 0], -1, 1)), np.arctan2(-R[0, 1], R[1, 1])
    else:
        rx, ry, rz = np.arctan2(R[2, 1], R[2, 2]), -np.arcsin(R[2, 0]), np.arctan2(R[1, 0], R[0, 0])
    return [float(v) for v in (*T[:3, 3], *np.degrees([rx, ry, rz]))]


def rotation_vector(R: np.ndarray) -> np.ndarray:
    """Axis times angle (rad) of a rotation matrix."""
    return cv2.Rodrigues(np.asarray(R, dtype=float))[0].ravel()


# --- cameras -----------------------------------------------------------------------------------

class Camera:
    """One camera from config.yaml (vision.cameras.<name>).

    source: a camera index (0), the URL of a JPEG snapshot (the webcam container:
    http://localhost/webcam/snapshot) or an image file (dry runs).
    """

    def __init__(self, name: str, spec: dict):
        self.name, self.spec = name, spec
        self.mount = T_from_pose(spec.get("mount", [0, 0, 0, 0, 0, 0]))  # camera in the tool frame
        self.dist = np.array(spec.get("dist", [0, 0, 0, 0, 0]), dtype=float)
        self._lock, self._capture = threading.Lock(), None

    def matrix(self, width: int, height: int) -> np.ndarray:
        """The camera matrix: vision.cameras.<name>.matrix [fx, fy, cx, cy], else from fov_deg."""
        if "matrix" in self.spec:
            fx, fy, cx, cy = self.spec["matrix"]
        else:  # a guess is enough to start with: see docs/vision.md, "Camera calibration"
            fx = fy = width / 2 / np.tan(np.radians(self.spec.get("fov_deg", 70)) / 2)
            cx, cy = width / 2, height / 2
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)

    def grab(self) -> np.ndarray:
        """A fresh BGR image."""
        source = self.spec.get("source", 0)
        with self._lock:
            if isinstance(source, str) and source.startswith(("http://", "https://")):
                with urllib.request.urlopen(source, timeout=5) as reply:
                    image = cv2.imdecode(np.frombuffer(reply.read(), np.uint8), cv2.IMREAD_COLOR)
            elif isinstance(source, str):
                image = cv2.imread(source)
            else:
                if self._capture is None:
                    self._capture = cv2.VideoCapture(int(source))
                    for key, prop in (("width", cv2.CAP_PROP_FRAME_WIDTH),
                                      ("height", cv2.CAP_PROP_FRAME_HEIGHT)):
                        if key in self.spec:
                            self._capture.set(prop, self.spec[key])
                for _ in range(4):  # the driver buffers frames: drop the ones from before now
                    self._capture.grab()
                ok, image = self._capture.read()
                if not ok:
                    self._capture.release()
                    self._capture = image = None
        if image is None:
            raise RuntimeError(f"Camera {self.name!r}: no image from {source!r}")
        return image


def detect(image: np.ndarray) -> dict[int, np.ndarray]:
    """The AprilTags in an image: {id: 4x2 corner pixels, clockwise from the tag's top left}."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.ArucoDetector(DICTIONARY, parameters).detectMarkers(gray)
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2).astype(float) for i, c in zip(ids.ravel(), corners)}


# --- measuring ---------------------------------------------------------------------------------

def tag_points(corners, K, dist, size: float) -> tuple[np.ndarray, np.ndarray]:
    """The four corners of one tag in the camera frame (4x3, mm) and the tag's normal."""
    s = size / 2
    square = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=float)
    ok, rvec, tvec = cv2.solvePnP(square, np.asarray(corners, dtype=float), K, dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise RuntimeError("Could not compute the pose of a tag")
    R = cv2.Rodrigues(rvec)[0]
    return square @ R.T + tvec.ravel(), R[:, 2]


def model(tags: dict, K, dist, size: float) -> tuple[np.ndarray, np.ndarray]:
    """The corners of a set of tags in the camera frame (4n x 3, in the order of sorted ids) and
    the normal of the face they sit on.

    One small tag gives its distance and tilt only roughly. Tags on one flat face are therefore
    put on a common plane, fitted through all their corners: each corner moves along its line of
    sight onto that plane. Tags that are not on one face (more than 1.5 mm off the plane) stay as
    they were found.
    """
    ids = sorted(tags)
    placed = [tag_points(tags[i], K, dist, size) for i in ids]
    points, normal = np.vstack([p for p, _ in placed]), np.mean([n for _, n in placed], axis=0)
    normal /= np.linalg.norm(normal)
    if len(ids) > 1:
        centre = points.mean(axis=0)
        fitted = np.linalg.svd(points - centre)[2][2]  # direction of least spread
        if np.sqrt((((points - centre) @ fitted) ** 2).mean()) < 1.5:
            normal = fitted if fitted @ normal > 0 else -fitted
            seen = np.vstack([np.asarray(tags[i], dtype=float) for i in ids]).reshape(-1, 1, 2)
            rays = np.hstack([cv2.undistortPoints(seen, K, dist).reshape(-1, 2), np.ones((len(seen), 1))])
            points = rays * ((centre @ normal) / (rays @ normal))[:, None]
    return points, normal


def motion(reference: dict, now: dict, K, dist, size: float) -> tuple[np.ndarray, float]:
    """The rigid motion of a set of tags between the reference image and now.

    Returns the 4x4 transform in the camera frame (p_now = M p_reference) and the reprojection
    error in pixels. The corners of the reference tags, placed in space (model), are fitted to the
    corners seen now; with tags far apart the turn is measured well.
    """
    ids = sorted(set(reference) & set(now))
    if not ids:
        raise RuntimeError("No tag is in both the reference and the image")
    points = model({i: reference[i] for i in ids}, K, dist, size)[0]
    seen = np.vstack([np.asarray(now[i], dtype=float) for i in ids])
    ok, rvec, tvec = cv2.solvePnP(points, seen, K, dist, rvec=np.zeros((3, 1)), tvec=np.zeros((3, 1)),
                                  useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("Could not fit the reference tags to the image")
    projected = cv2.projectPoints(points, rvec, tvec, K, dist)[0].reshape(-1, 2)
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = cv2.Rodrigues(rvec)[0], tvec.ravel()
    return M, float(np.sqrt(((projected - seen) ** 2).sum(axis=1).mean()))


@dataclass
class Measurement:
    """What one look at a station found. Lengths in mm, angles in degrees, robot user frame."""

    tags: list[int]                 # station tags in both the reference and the image
    shift: list[float]              # the station's move in its own plane, [dx, dy, dz]
    shift_mm: float                 # its length
    yaw_deg: float                  # the station's turn about its normal
    lift_mm: float                  # move along the normal: not corrected, only limited
    tilt_deg: float                 # turn out of the plane: not corrected, only limited
    error_px: float                 # reprojection error of the fit
    carrier_mm: float | None = None  # distance of a carrier tag from its seat; None: none in view
    carrier_tag: int | None = None
    correction: list[list[float]] = field(default_factory=lambda: np.eye(4).tolist(), repr=False)

    def corrected(self, pose) -> list[float]:
        """A taught pose, moved with the station."""
        return pose_from_T(np.array(self.correction) @ T_from_pose(pose))

    def summary(self) -> str:
        text = (f"shift {self.shift_mm:.2f} mm ({self.shift[0]:+.2f}, {self.shift[1]:+.2f}, "
                f"{self.shift[2]:+.2f}), yaw {self.yaw_deg:+.2f} deg, lift {self.lift_mm:+.2f} mm, "
                f"tilt {self.tilt_deg:.2f} deg, fit {self.error_px:.2f} px, tags {self.tags}")
        if self.carrier_mm is not None:
            text += f", carrier tag {self.carrier_tag} {self.carrier_mm:.2f} mm from its seat"
        return text

    def as_dict(self) -> dict:
        return asdict(self)


def measure(reference: dict, now: dict, camera: Camera, look_pose, size: float,
            station_tags=None, carrier_tags=()) -> Measurement:
    """Compares the tags seen now with a saved reference.

    reference: one entry of a .vision.json file (see save_reference). now: detect() of an image
    taken with the tool at look_pose, the pose the reference was taken from.
    """
    width, height = reference["size"]
    K, dist = camera.matrix(width, height), camera.dist
    saved = {int(i): np.array(c) for i, c in reference["tags"].items()}
    if station_tags:
        saved = {i: c for i, c in saved.items() if i in station_tags}
    M, error = motion(saved, now, K, dist, size)
    ids = sorted(set(saved) & set(now))

    # The station's plane, from the reference tags: where they lie and which way they face.
    placed, normal_cam = model({i: saved[i] for i in ids}, K, dist, size)
    origin_cam = placed.mean(axis=0)
    X = T_from_pose(look_pose) @ camera.mount  # camera in the robot's user frame, at the look
    D = X @ M @ np.linalg.inv(X)               # the same motion, in the user frame
    origin = (X @ np.append(origin_cam, 1))[:3]
    normal = X[:3, :3] @ normal_cam
    normal /= np.linalg.norm(normal)

    turn = rotation_vector(D[:3, :3])
    yaw = float(turn @ normal)
    move = (D @ np.append(origin, 1))[:3] - origin
    lift = float(move @ normal)
    shift = move - lift * normal
    P = np.eye(4)  # the correction: the part of D inside the station's plane
    P[:3, :3] = cv2.Rodrigues(yaw * normal)[0]
    P[:3, 3] = origin + shift - P[:3, :3] @ origin
    result = Measurement(tags=ids, shift=[float(v) for v in shift],
                         shift_mm=float(np.linalg.norm(shift)), yaw_deg=float(np.degrees(yaw)),
                         lift_mm=lift, tilt_deg=float(np.degrees(np.linalg.norm(turn - yaw * normal))),
                         error_px=error, correction=P.tolist())

    # A carrier: how far is its tag from where the carrier's tag sat at teaching, on this station?
    if reference.get("carrier") is not None:
        seat = (M @ np.append(tag_points(reference["carrier"], K, dist, size)[0].mean(axis=0), 1))[:3]
        for i in set(carrier_tags) & set(now):
            distance = float(np.linalg.norm(tag_points(now[i], K, dist, size)[0].mean(axis=0) - seat))
            if result.carrier_mm is None or distance < result.carrier_mm:
                result.carrier_mm, result.carrier_tag = distance, i
    return result


def displacement(reference: dict, now: dict, K, dist, size: float) -> tuple[np.ndarray, float]:
    """How far the middle of a set of tags moved between two images (mm, camera frame), and the
    reprojection error of the fit."""
    M, error = motion(reference, now, K, dist, size)
    middle = model({i: reference[i] for i in set(reference) & set(now)}, K, dist, size)[0].mean(axis=0)
    return (M @ np.append(middle, 1))[:3] - middle, error


def mount_rotation(jogs, look_pose) -> tuple[list[float], float]:
    """The camera's angles in the tool frame, from straight jogs of the arm away from a look point.

    jogs: [(the jog [dx, dy, dz] in the user frame, displacement() of the tags between the look
    point and the jogged pose)]. A camera moved by t sees the scene move by -t, so the directions
    the tags appear to move in give the camera's orientation. Returns [rx, ry, rz] for the
    camera's mount in config.yaml and the misfit in mm (large: wrong tag size, or the tags moved).
    """
    moved = np.array([-np.asarray(jog, dtype=float) for jog, _ in jogs])  # user frame
    seen = np.array([d for _, d in jogs])                                 # camera frame
    U, _, Vt = np.linalg.svd(moved.T @ seen)  # the rotation R with R @ seen = moved, least squares
    R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    misfit = float(np.sqrt(((seen @ R.T - moved) ** 2).sum(axis=1).mean()))
    T = np.eye(4)
    T[:3, :3] = T_from_pose(look_pose)[:3, :3].T @ R
    return pose_from_T(T)[3:], misfit


# --- references --------------------------------------------------------------------------------

def reference_file(points_file: Path) -> Path:
    """The references of a teach-point file: opentrons.json -> opentrons.vision.json."""
    return Path(points_file).with_suffix(".vision.json")


def load_references(points_file: Path) -> dict:
    """{look point: reference} of a teach-point file; empty if nothing was saved yet."""
    file = reference_file(points_file)
    return json.loads(file.read_text()) if file.exists() else {}


def save_reference(points_file: Path, look: str, camera: Camera, image: np.ndarray,
                   station_tags=None, carrier_tags=()) -> dict:
    """Saves the tags in an image as the reference of a look point, with the image beside it."""
    found = detect(image)
    tags = {i: c for i, c in found.items() if (i in station_tags if station_tags else
                                                i not in carrier_tags)}
    missing = sorted(set(station_tags or ()) - set(tags))
    if missing or not tags:
        raise RuntimeError(f"The camera does not see the station's tags {missing or ''}: it sees "
                           f"{sorted(found)}")
    carriers = [i for i in found if i in carrier_tags]
    if len(carriers) > 1:
        raise RuntimeError(f"More than one carrier tag in view ({carriers}): leave one carrier, on "
                           f"this station's seat")
    reference = {"camera": camera.name, "size": [image.shape[1], image.shape[0]],
                 "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
                 "tags": {str(i): np.round(c, 2).tolist() for i, c in sorted(tags.items())},
                 "carrier": np.round(found[carriers[0]], 2).tolist() if carriers else None}
    file = reference_file(points_file)
    references = load_references(points_file)
    references[look] = reference
    file.write_text(json.dumps(references, indent=1))
    cv2.imwrite(str(file.with_suffix(f".{look}.jpg")), image)
    return reference


def annotate(image: np.ndarray, now: dict, reference: dict | None = None) -> np.ndarray:
    """A copy of the image with the tags seen now (green) and the reference's tags (orange)."""
    out = image.copy()
    if reference:
        outlines = [np.array(c) for c in reference["tags"].values()]
        if reference.get("carrier") is not None:
            outlines.append(np.array(reference["carrier"]))
        cv2.polylines(out, [c.astype(np.int32) for c in outlines], True, (0, 140, 255), 2, cv2.LINE_AA)
    for i, c in now.items():
        cv2.polylines(out, [c.astype(np.int32)], True, (0, 220, 0), 2, cv2.LINE_AA)
        cv2.circle(out, tuple(int(v) for v in c[0]), 4, (0, 220, 0), -1)
        cv2.putText(out, str(i), tuple(int(v) for v in c.mean(axis=0)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 255), 2, cv2.LINE_AA)
    return out
