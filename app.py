"""Fairino robot arm as an Arkitekt app.

The arm carries slides between stations (pickup station, Opentrons, microscope). A station is a
teach-point file; config.yaml lists them, with the robot's IP and the speeds. Two layouts work:

- round trip (fairinogale): every pose in driving order, including the way back (r_ points).
  Name suffixes set the behaviour: ..._danger is driven slowly, ..._open-gripper and
  ..._close-gripper act on the gripper. Release drives the file backwards with open and close
  swapped.
- out and back (Jena): home, move1..moveN. The arm drives out to moveN and back the same way.
  The station lists which points are driven linearly and slowly, and where the gripper opens and
  closes, separately for pick and release.

Every JSON file in the points folder is a station too, driven as a round trip unless config.yaml
says otherwise. upload_points, teach_point and import_teach_points write those files, so a station
can be taught or replaced while the app runs. The folder sits next to config.yaml (on the Pi a bind
mount), and a replaced file is kept in its .history/ folder.

A station with a vision block is looked at before the arm goes in: a camera on the gripper sees the
station's AprilTags from one of the teach points, and the points behind it are shifted by what the
station moved since teaching (vision.py, docs/vision.md). debug.py calls the same functions from a
terminal, without Arkitekt.

Run:  uv run app.py   then open http://localhost:8080/ to bind it.
FAIRINO_CONFIG picks another config.yaml (default: next to this file).
"""

import asyncio
import json
import os
import platform
import re
import shutil
import sys
import threading
import time
from contextlib import contextmanager
from dataclasses import field
from datetime import datetime
from fnmatch import fnmatch
from pathlib import Path
from typing import Annotated

import koil
import numpy as np
import yaml
from arkitekt import App, Description, Effects, Task
from arkitekt.widgets import ParagraphWidget, withStateChoices

import vision
from arkitekt_bind import serve

HERE = Path(__file__).resolve().parent
CONFIG = Path(os.getenv("FAIRINO_CONFIG", HERE / "config.yaml"))

if not CONFIG.exists():  # first start: copy the defaults where they can be edited
    CONFIG.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(HERE / "config.example.yaml", CONFIG)
cfg = yaml.safe_load(CONFIG.read_text())
POINTS = (CONFIG.parent / cfg.get("points_dir", "control-points")).resolve()
if not POINTS.exists():  # start from the shipped teach points
    shutil.copytree(HERE / "control-points", POINTS)
HOME_FILE = cfg.get("home_file", "home_robot.json")
RESCAN_S = 5  # how often the points folder is looked at for new stations
VISION = cfg.get("vision") or {}
TAG_SIZE = VISION.get("tag_size_mm", 30.0)  # edge of the black square of the station tags
CARRIER_TAG_SIZE = VISION.get("carrier_tag_size_mm", TAG_SIZE)  # ... and of the carrier tags


def station_names() -> list[str]:
    """config.yaml's stations, then the other teach-point files in the points folder."""
    names = list(cfg["stations"])
    files = [st["file"].replace("{sample}", "*") for st in cfg["stations"].values()]
    for path in sorted(POINTS.glob("*.json")):
        if path.stem not in names and not any(fnmatch(path.name, f) for f in files + [HOME_FILE]):
            names.append(path.stem)
    return names


STATIONS = station_names()  # at start; the trajectories state follows the folder

# 2.0.0: the trajectories state, stations offered from it, and the vision actions.
app = App(cfg.get("app", "fairinogale"), "2.0.0", effects=Effects.IRREVERSIBLE)


@app.state(name="trajectories")  # the key the protocol app's arm demand asks for
class TrajectoryState:
    """The stations the arm can drive to, where it went last and whether it holds a slide."""

    available_trajectories: list[str] = field(default_factory=list)
    current_position: str = "home"
    gripping: bool = False


@app.startup
def initialize_trajectories() -> TrajectoryState:
    return TrajectoryState(available_trajectories=station_names())


@app.background
async def watch_stations(state: TrajectoryState) -> None:
    """Offers a teach-point file put into the points folder by hand without a restart."""
    while True:
        await asyncio.sleep(RESCAN_S)
        names = station_names()
        if names != state.available_trajectories:
            state.available_trajectories = names


Station = Annotated[str, withStateChoices("self.trajectories.available_trajectories"),
                    Description("Station from config.yaml or the points folder")]
AnyStation = Annotated[str, Description("A station, or a new name (letters, digits, - and _): "
                                        "its file is <name>.json in the points folder")]
Sample = Annotated[str, Description("Fills {sample} in the station's file name, e.g. Box1_A1")]
Speed = Annotated[int, Description("Joint speed in %, outside danger zones")]
Acceleration = Annotated[int, Description("Acceleration in %")]
DangerSpeed = Annotated[int, Description("Speed in % at danger, linear and gripper points")]
Points = Annotated[str, ParagraphWidget(), Description(
    'JSON in driving order: {"home": [x, y, z, rx, ry, rz, j1, j2, j3, j4, j5, j6, tool, user, '
    '...], "move1": [...]}, as export_points and the Fairino teach points give it')]
Merge = Annotated[bool, Description("Keep the station's other points: replace those with the "
                                    "same name in place and add new ones at the end")]
Correct = Annotated[bool, Description("Look at the station's tags first and shift the points "
                                      "(stations with a vision block)")]

_robot = None
_automatic = False  # the arm was switched to automatic mode by this process
_robot_lock = threading.Lock()
_moving = threading.Lock()  # held while the arm moves; a restart of the app waits for it
_cameras: dict = {}


class GripFailed(RuntimeError):
    """The gripper reports a fault, or a jaw position outside gripper.holding after a pick."""


class VisionError(RuntimeError):
    """The camera's look at a station says: do not go in."""


class TagsNotSeen(VisionError):
    """The station's tags are not all in view."""


class StationMoved(VisionError):
    """The station moved further since teaching than the limits allow to correct."""


class CarrierMisplaced(VisionError):
    """A carrier is on the seat where none should be, or the other way round."""


class NotAtLookPoint(RuntimeError):
    """A reference is to be saved, but the arm does not stand at the look point."""


class ConsoleTask:
    """Stands in for Arkitekt's Task when a terminal or the vision page calls an action."""

    def pausepoint(self) -> None:
        pass

    def progress(self, percent: int, message: str = "") -> None:
        print(f"{percent:3d} %  {message}", flush=True)

    def log(self, message: str) -> None:
        pass  # log() below has printed it


@contextmanager
def moving():
    """One move at a time: Arkitekt holds its own lock, the vision page and debug.py do not."""
    if not _moving.acquire(blocking=False):
        raise RuntimeError("The arm is busy with another move")
    try:
        yield
    finally:
        _moving.release()


def rpc_class():
    """The Fairino SDK's RPC, or the mock when robot.mock is set."""
    if cfg["robot"].get("mock"):
        from mock.MockRPC import MockRPC

        return MockRPC
    sdk = "windows" if platform.system() == "Windows" else "linux"
    sys.path.insert(0, os.getenv("FAIRINO_SDK_DIR", str(HERE / "fairino-python-sdk" / sdk / "fairino")))
    from Robot import RPC

    return RPC


def connection():
    """The connection to the arm, opened on first use so the app starts without it.

    Opening it changes nothing on the arm: teaching reads it while it is in manual mode.
    """
    global _robot
    with _robot_lock:
        if _robot is None:
            ip, RPC = cfg["robot"]["ip"], rpc_class()
            rpc = RPC(ip)
            if getattr(RPC, "is_connect", True) is False:
                rpc.CloseRPC()  # stops the SDK's receive threads
                raise ConnectionError(f"Cannot reach the Fairino robot at {ip} (robot.ip in {CONFIG})")
            _robot = rpc
        return _robot


def robot(force_automatic: bool = False):
    """The connection, with the arm enabled in automatic mode on first use.

    Later calls leave the mode alone: someone who switched the arm to manual mode to guide it by
    hand is not overridden by the next move. init_robot_and_gripper switches it back.
    """
    global _automatic
    rpc = connection()
    with _robot_lock:
        if force_automatic or not _automatic:
            rpc.RobotEnable(1)
            rpc.Mode(1)  # 0 = manual, 1 = automatic
            _automatic = True
    return rpc


def check(error, what: str) -> None:
    """The SDK returns 0 on success; anything else stops the sequence."""
    if error not in (0, None):
        raise RuntimeError(f"{what} failed with Fairino error code {error}")


def value(result, what: str):
    """A getter's value: the SDK returns (0, value), or only the error code when it fails."""
    if not isinstance(result, tuple) or result[0] != 0 or result[1] is None:
        code = result[0] if isinstance(result, tuple) else result
        raise RuntimeError(f"{what} failed with Fairino error code {code}")
    return result[1]


def log(task, message: str) -> None:
    """To the console and, when Arkitekt runs the action, to the task's log."""
    print(message, flush=True)
    try:
        task.log(message)
    except Exception:  # noqa: BLE001 - a log line must not stop a move
        pass


# --- gripper -----------------------------------------------------------------------------------

def gripper(position: int, velocity: int) -> float:
    """Moves the jaws, waits for them to stop and returns the position the controller reports (%)."""
    g, rbt = cfg["gripper"], connection()  # the caller sets the mode
    check(rbt.MoveGripper(g.get("jaw", 1), position, velocity, 30, 10000, 0, 0, 0, 0, 0),
          f"gripper to {position}")
    deadline = time.monotonic() + g.get("settle_s", 3.0)
    while time.monotonic() < deadline:  # MoveGripper blocks, but the reported state lags behind
        error, state = rbt.GetGripperMotionDone()
        if error == 0 and state and state[1] == 1:
            break
        time.sleep(0.05)
    return jaw_position()


def station_file(station: str, sample: str = "") -> tuple[dict, Path]:
    """The station's settings and teach-point file. A station config.yaml does not name is
    <station>.json in the points folder, driven as a round trip."""
    st = cfg["stations"].get(station)
    if st is None:
        if not re.fullmatch(r"[A-Za-z0-9_-]+", station or ""):
            raise ValueError(f"{station!r} is not a station: use letters, digits, - and _")
        st = {"file": f"{station}.json"}
    if "{sample}" in st["file"] and not sample:
        raise ValueError(f"Station {station!r} needs a sample (its file is {st['file']})")
    path = (POINTS / st["file"].format(sample=sample)).resolve()
    if not path.is_relative_to(POINTS):
        raise ValueError(f"{path} is outside the points folder {POINTS}")
    return st, path


def read_points(station: str, sample: str = "") -> dict:
    path = station_file(station, sample)[1]
    if not path.exists():
        raise FileNotFoundError(f"Station {station!r} has no points yet ({path} is missing)")
    return json.loads(path.read_text())


def check_points(points) -> dict:
    """{name: [x, y, z, rx, ry, rz, j1..j6, tool, user, ...]}: what drive() reads."""
    if not isinstance(points, dict) or not points:
        raise ValueError('Points must be a JSON object: {"home": [x, y, z, rx, ry, rz, '
                         'j1, j2, j3, j4, j5, j6, tool, user, ...], ...}')
    for name, values in points.items():
        if not isinstance(values, list) or len(values) < 14:
            raise ValueError(f"Point {name!r}: needs at least 14 values (pose x, y, z, rx, ry, "
                             f"rz, joints j1..j6, tool, user), got {values!r}")
        try:
            [float(v) for v in values]
        except (TypeError, ValueError):
            raise ValueError(f"Point {name!r}: every value must be a number, got {values!r}") from None
    return points


def write_points(station: str, sample: str, points: dict, merge: bool = False) -> str:
    """Store a station's points; the file it replaces goes to .history/."""
    path = station_file(station, sample)[1]
    check_points(points)
    if path.exists():
        if merge:
            points = {**json.loads(path.read_text()), **points}
        history = POINTS / ".history"
        history.mkdir(exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        name = path.relative_to(POINTS).with_suffix("").as_posix().replace("/", "_")
        shutil.copy2(path, history / f"{name}-{stamp}.json")
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(points, indent=4))
    tmp.replace(path)
    return f"Saved {len(points)} points of {station!r} to {path}"


def jaw_position() -> float:
    """The jaw position in % from the controller's state stream; a gripper fault raises."""
    _, fault, position = connection().GetGripperCurPosition()
    if fault:
        raise GripFailed(f"The gripper reports fault {fault}")
    return float(position)


def check_holding(position: float, when: str) -> None:
    """Fails a pick when the jaws are not where they stop on a carrier (gripper.holding)."""
    holding = cfg["gripper"].get("holding")
    if holding and not holding[0] <= position <= holding[1]:
        raise GripFailed(f"Jaw position {position:.0f} % {when} is outside gripper.holding "
                         f"{holding}: no carrier in the gripper")


# --- stations and routes -----------------------------------------------------------------------

def station_points(station: str, sample: str = "") -> tuple[dict, Path, dict]:
    """A station's config, its teach-point file and the points: {name: 20 values}."""
    st, file = station_file(station, sample)
    points = {name: [float(v) for v in values] for name, values in read_points(station, sample).items()}
    return st, file, points


def route(st: dict, points: dict, release: bool) -> tuple[list[str], int]:
    """The points in driving order, and how many of them are the way out (the gripper acts there)."""
    if st.get("out_and_back"):
        moves = [n for n in points if n != "home"]
        home = ["home"] if "home" in points else []
        return home + moves + moves[-2::-1] + home, len(home) + len(moves)
    order = list(points)[::-1] if release else list(points)
    return order, len(order)


def look_points(st: dict) -> list[str]:
    """The teach points of a station from where the camera sees its tags (vision.look)."""
    look = (st.get("vision") or {}).get("look", [])
    return [look] if isinstance(look, str) else list(look)


def look_route(st: dict, points: dict) -> list[str]:
    """The pick route up to and including the first look point."""
    order, looks = route(st, points, False)[0], look_points(st)
    reached = [order.index(look) for look in looks if look in order]
    if not reached:
        raise ValueError(f"No look point of {looks} is in the station's teach-point file")
    return order[:min(reached) + 1]


def go(name: str, values: list, linear: bool, vel: int, acc: int, pose: list | None = None) -> None:
    """One move to a teach point. A corrected pose replaces the taught one and is driven linearly:
    the taught joint values no longer belong to it, so the controller solves them (as for MoveL)."""
    taught, joints, tool, user = values[0:6], values[6:12], int(values[12]), int(values[13])
    if pose is not None or linear:
        check(robot().MoveL(pose or taught, tool=tool, user=user, vel=vel, acc=acc), f"MoveL to {name}")
    else:
        check(robot().MoveJ(joints, tool=tool, user=user, desc_pos=taught, vel=vel, acc=acc),
              f"MoveJ to {name}")


# --- vision ------------------------------------------------------------------------------------

def camera(name: str) -> vision.Camera:
    """A camera of config.yaml (vision.cameras), opened on first use."""
    if name not in _cameras:
        spec = VISION["cameras"][name]
        if spec.get("source") != "mock":
            _cameras[name] = vision.Camera(name, spec)
        else:  # no hardware: draw each station's tags in front of its look point
            from mock.MockCamera import MockCamera

            cam = _cameras[name] = MockCamera(name, spec, lambda: connection().tcp_pose, TAG_SIZE)
            dx, dy, yaw = spec.get("mock_shift", [0, 0, 0])
            for station, st in cfg["stations"].items():
                vis = st.get("vision") or {}
                if vis.get("camera") != name or "{sample}" in st["file"] or not vis.get("tags"):
                    continue
                look = station_points(station)[2][look_points(st)[0]][0:6]
                carrier = list(vis.get("carrier_tags", []))[:1]
                cam.place(vis["tags"], look)
                cam.place(carrier, look, row=45, size=CARRIER_TAG_SIZE)
                plane = cam.scene[vis["tags"][0]]
                cam.move(list(vis["tags"]) + carrier, plane @ vision.T_from_pose([dx, dy, 0, 0, 0, yaw])
                         @ np.linalg.inv(plane))
    return _cameras[name]


def observe(station: str, sample: str = "", look: str | None = None) -> dict:
    """One look at a station from where the arm stands. Nothing moves and nothing is raised:
    "error" holds what speaks against going in, "apply" whether the points would be corrected."""
    st, file, points = station_points(station, sample)
    vis = st.get("vision")
    if not vis or not look_points(st):
        raise ValueError(f"Station {station!r} has no vision block with a look point in config.yaml")
    look = look or look_points(st)[0]
    cam = camera(vis["camera"])
    image = cam.grab()
    now = vision.detect(image)
    reference = vision.load_references(file).get(look)
    seen = {"station": station, "sample": sample, "look": look, "image": image, "now": now,
            "reference": reference, "measurement": None, "travel_mm": None, "apply": False,
            "error": None, "on_seat": None}
    if reference is None:
        seen["error"] = VisionError(f"No reference saved for look point {look!r} of {file.name}: "
                                    f"save one with the station as taught (docs/vision.md)")
        return seen
    wanted = [int(i) for i in reference["tags"] if not vis.get("tags") or int(i) in vis["tags"]]
    missing = sorted(set(wanted) - set(now))
    if missing or not wanted:
        seen["error"] = TagsNotSeen(f"Station {station!r}: tags {missing} are not in view from "
                                    f"{look} (the camera sees {sorted(now)})")
        return seen
    m = seen["measurement"] = vision.measure(reference, now, cam, points[look][0:6], TAG_SIZE,
                                             vis.get("tags"), vis.get("carrier_tags", ()),
                                             CARRIER_TAG_SIZE)
    # How far the correction takes the points it applies to: limits and deadband refer to that.
    P = np.array(m.correction)
    travel = [np.linalg.norm((P @ vision.T_from_pose(points[n][0:6]))[:3, 3] - points[n][0:3])
              for n in vis.get("correct", []) if n in points]
    travel = seen["travel_mm"] = float(max(travel, default=m.shift_mm))
    max_shift, max_yaw = vis.get("max_shift_mm", 5.0), vis.get("max_yaw_deg", 3.0)
    over = [f"{what} {value:.2f} {unit} (limit {limit:g})" for what, value, limit, unit in (
        ("shift", travel, max_shift, "mm"), ("yaw", abs(m.yaw_deg), max_yaw, "deg"),
        ("lift", abs(m.lift_mm), vis.get("max_lift_mm", max_shift), "mm"),
        ("tilt", m.tilt_deg, vis.get("max_tilt_deg", max_yaw), "deg"),
        ("fit error", m.error_px, vis.get("max_error_px", 2.0), "px")) if value > limit]
    if over:
        seen["error"] = StationMoved(f"Station {station!r} is not where it was taught: "
                                     f"{', '.join(over)}")
    seen["apply"] = not over and travel >= vis.get("deadband_mm", 0.3)
    if vis.get("carrier_tags") and reference.get("carrier") is not None:
        seen["on_seat"] = m.carrier_mm is not None and m.carrier_mm <= vis.get("max_seat_mm", 1.5)
    return seen


def expect_carrier(seen: dict, on_seat: bool, when: str) -> None:
    """Fails when the seat is taken where it should be free, or free where a carrier should sit."""
    if seen["on_seat"] is None or seen["on_seat"] == on_seat:
        return
    m = seen["measurement"]
    found = ("no carrier tag is in view" if m.carrier_mm is None else
             f"carrier tag {m.carrier_tag} is {m.carrier_mm:.1f} mm from the seat")
    raise CarrierMisplaced(f"Station {seen['station']!r} {when}: "
                           f"{'a carrier should be' if on_seat else 'no carrier should be'} "
                           f"on the seat, but {found}")


def save_reference(station: str, sample: str = "", look: str | None = None, force: bool = False) -> dict:
    """Saves what the camera sees now as the reference of a look point. The arm must stand at
    that point (compared by joint angles) unless force is set."""
    st, file, points = station_points(station, sample)
    vis = st.get("vision") or {}
    look = look or (look_points(st) or [None])[0]
    if look not in points:
        raise ValueError(f"Station {station!r} has no look point {look!r} in {file.name}")
    if not force:
        reply = connection().GetActualJointPosDegree()  # reading: leaves the mode alone
        joints = reply[1] if isinstance(reply, tuple) and reply[0] == 0 else None
        if joints is None or max(abs(a - b) for a, b in zip(joints, points[look][6:12])) > 0.5:
            raise NotAtLookPoint(f"The arm is not at look point {look!r} of {file.name}: drive "
                                 f"it there first")
    cam = camera(vis["camera"])
    vision.save_reference(file, look, cam, cam.grab(), vis.get("tags"), vis.get("carrier_tags", ()))
    return observe(station, sample, look)


def look_at(station: str, sample: str, save: bool, stay: bool, speed: int, acceleration: int,
            danger_speed: int, task) -> dict:
    """Drives out to the station's first look point, saves the reference there or measures against
    it, and drives back the same way (stay: the arm stops at the look point; leave_look returns)."""
    st, _, points = station_points(station, sample)
    out, plan = look_route(st, points), st.get("pick") or {}
    with moving():
        for name in out:
            task.pausepoint()
            koil.check_cancelled()
            linear = name in plan.get("linear", [])
            go(name, points[name], linear, danger_speed if linear else speed, acceleration)
        time.sleep(st["vision"].get("settle_s", 0.3))
        try:
            if save:
                seen = save_reference(station, sample, out[-1], force=True)
            else:
                seen = observe(station, sample, out[-1])
        finally:
            if not stay:
                for name in out[-2::-1]:
                    go(name, points[name], name in plan.get("linear", []), speed, acceleration)
    log(task, f"{station} at {out[-1]}: " + (str(seen["error"]) if seen["error"] else
                                             seen["measurement"].summary()))
    return seen


def leave_look(station: str, sample: str, speed: int, acceleration: int) -> None:
    """Drives from the look point back to the start of the route (after look_at with stay)."""
    st, _, points = station_points(station, sample)
    plan = st.get("pick") or {}
    with moving():
        for name in look_route(st, points)[-2::-1]:
            go(name, points[name], name in plan.get("linear", []), speed, acceleration)


# --- driving -----------------------------------------------------------------------------------

def drive(station: str, sample: str, release: bool, speed: int, acceleration: int,
          danger_speed: int, task, correct: bool = True,
          state: TrajectoryState | None = None) -> None:
    st, _, points = station_points(station, sample)
    if state is not None:
        state.current_position = station
    plan = st.get("release" if release else "pick") or {}
    order, outward = route(st, points, release)
    vis = (st.get("vision") or {}) if correct else {}
    looks, doing = look_points(st) if vis else [], "release" if release else "pick"
    located, correction, late = False, None, None  # set at the first look point

    g = cfg["gripper"]
    with moving():
        for i, name in enumerate(order):
            task.pausepoint()
            koil.check_cancelled()  # a cancelled task stops at the next point, not after the route
            lower = name.lower()
            linear = name in plan.get("linear", [])
            pose = None
            if name in vis.get("correct", []):
                if not located:
                    raise RuntimeError(f"{name} is in vision.correct of station {station!r}, but the "
                                       f"route reaches it before a look point ({looks})")
                if correction is not None:
                    pose = correction.corrected(points[name][0:6])
            slow = linear or pose is not None or lower.endswith(("danger", "gripper"))
            go(name, points[name], linear, danger_speed if slow else speed, acceleration, pose)

            opens = closes = False
            if i < outward:
                opens, closes = name in plan.get("open", []), name in plan.get("close", [])
            if lower.endswith("open-gripper"):
                opens, closes = not release, release
            elif lower.endswith("close-gripper"):
                opens, closes = release, not release
            if opens:
                log(task, f"{name}: gripper open, jaws at {gripper(g['open'], 30):.0f} %")
                if state is not None:
                    state.gripping = False
            if closes:
                position = gripper(g["close"], 50)
                log(task, f"{name}: gripper closed, jaws at {position:.0f} %")
                if state is not None:
                    state.gripping = True
                if not release:
                    check_holding(position, f"after closing at {name}")

            if name in looks and not located:  # before the station: where is it, is the seat right?
                located = True
                time.sleep(vis.get("settle_s", 0.3))
                seen = observe(station, sample, name)
                try:
                    error = seen["error"]
                    if isinstance(error, TagsNotSeen) and vis.get("on_no_tag") == "uncorrected":
                        log(task, f"{error}. Driving the taught points (on_no_tag: uncorrected)")
                    elif error:
                        raise error
                    else:
                        log(task, f"{station} at {name}: {seen['measurement'].summary()}"
                            + (f"; points move up to {seen['travel_mm']:.2f} mm" if seen["apply"]
                               else "; inside the deadband, not corrected"))
                        expect_carrier(seen, not release, f"before the {doing}")
                        correction = seen["measurement"] if seen["apply"] else None
                except VisionError:
                    for back in order[i - 1::-1] if i else []:  # nothing touched yet: back out
                        go(back, points[back], back in plan.get("linear", []), speed, acceleration)
                    raise
            elif name in looks and vis.get("check_after"):  # after it: did the carrier change hands?
                time.sleep(vis.get("settle_s", 0.3))
                seen = observe(station, sample, name)
                try:
                    if seen["error"]:
                        raise seen["error"]
                    expect_carrier(seen, release, f"after the {doing}")
                except VisionError as e:  # the arm finishes its way back first
                    late = e
                    log(task, f"{e}. Finishing the route.")
            task.progress(int(100 * (i + 1) / len(order)), name)
    if late is not None:
        raise late
    if not release:  # a carrier lost on the way back lets the jaws close further
        check_holding(jaw_position(), "at the end of the pick")


# --- Arkitekt actions --------------------------------------------------------------------------

@app.model
class StationCheck:
    """What the camera found at a station, compared with the reference saved at teaching."""

    station: str
    look: str            # the teach point the camera looked from
    ok: bool             # False: problem says why the arm should not go in
    problem: str
    would_correct: bool  # the points would be shifted (beyond the deadband, inside the limits)
    travel_mm: float     # how far the corrected points would move
    shift_x: float       # the station's move in its plane, mm in the robot's user frame
    shift_y: float
    shift_z: float
    yaw_deg: float
    lift_mm: float
    tilt_deg: float
    error_px: float
    carrier_seen: bool   # a carrier tag is in view
    carrier_mm: float    # its distance from the seat (0 if none is in view)
    carrier_on_seat: bool


def station_check(seen: dict) -> StationCheck:
    m = seen["measurement"]
    values = (m.shift + [m.yaw_deg, m.lift_mm, m.tilt_deg, m.error_px]) if m else [0.0] * 7
    return StationCheck(seen["station"], seen["look"], seen["error"] is None, str(seen["error"] or ""),
                        seen["apply"], seen["travel_mm"] or 0.0, *values,
                        bool(m and m.carrier_mm is not None), (m.carrier_mm or 0.0) if m else 0.0,
                        bool(seen["on_seat"]))


@app.action(locks=["robot"])
def pick_up(state: TrajectoryState, station: Station, sample: Sample = "", speed: Speed = cfg["speed"],
            acceleration: Acceleration = cfg["acceleration"],
            danger_speed: DangerSpeed = cfg["danger_speed"], correct: Correct = True,
            *, task: Task) -> None:
    """Pick Up

    Drives to the station, grips the slide and comes back. Fails if the jaws do not stop where
    they stop on a carrier (gripper.holding in config.yaml). At a station with a vision block the
    arm first looks at the station's tags: it shifts the points, or backs out and fails when the
    station moved too far or the seat is empty.
    """
    drive(station, sample, False, speed, acceleration, danger_speed, task, correct, state)


@app.action(locks=["robot"])
def release_at(state: TrajectoryState, station: Station, sample: Sample = "", speed: Speed = cfg["speed"],
               acceleration: Acceleration = cfg["acceleration"],
               danger_speed: DangerSpeed = cfg["danger_speed"], correct: Correct = True,
               *, task: Task) -> None:
    """Release At

    Drives to the station, puts the slide down and comes back. At a station with a vision block
    the arm first looks at the station's tags, like Pick Up; it fails when the seat is taken.
    """
    drive(station, sample, True, speed, acceleration, danger_speed, task, correct, state)


@app.action(locks=["robot"])
def check_station(station: Station, sample: Sample = "", speed: Speed = cfg["speed"],
                  acceleration: Acceleration = cfg["acceleration"],
                  danger_speed: DangerSpeed = cfg["danger_speed"], *, task: Task) -> StationCheck:
    """Check Station

    Drives to the station's look point, compares its tags with the saved reference and comes
    back. Touches nothing. Does not fail on what it sees: ok and problem carry the result.
    """
    return station_check(look_at(station, sample, False, False, speed, acceleration, danger_speed, task))


@app.action(locks=["robot"])
def save_station_reference(station: Station, sample: Sample = "", speed: Speed = cfg["speed"],
                           acceleration: Acceleration = cfg["acceleration"],
                           danger_speed: DangerSpeed = cfg["danger_speed"],
                           *, task: Task) -> StationCheck:
    """Save Station Reference

    Drives to the station's look point and saves what the camera sees as the station's reference.
    Do this right after teaching the station's points, with a carrier on its seat.
    """
    return station_check(look_at(station, sample, True, False, speed, acceleration, danger_speed, task))


@app.action(locks=["robot"])
def init_robot_and_gripper() -> None:
    """Init Robot and Gripper

    Enables the arm in automatic mode (also after it was switched to manual mode for teaching),
    activates the gripper and moves it to its initial width.
    """
    rbt, g = robot(force_automatic=True), cfg["gripper"]
    with moving():
        rbt.SetGripperConfig(g.get("company", 6), g.get("device", 0), softversion=0, bus=0)
        check(rbt.ActGripper(g.get("jaw", 1), 1), "activating the gripper")
        gripper(g.get("initial", g["close"]), 30)


@app.action(locks=["robot"])
def open_grip(state: TrajectoryState) -> None:
    """Open Gripper"""
    with moving():
        robot()
        gripper(cfg["gripper"]["open"], 30)
    state.gripping = False


@app.action(locks=["robot"])
def close_grip(state: TrajectoryState) -> None:
    """Close Gripper"""
    with moving():
        robot()
        gripper(cfg["gripper"]["close"], 50)
    state.gripping = True


@app.action(locks=["robot"])
def home_robot(state: TrajectoryState, speed: Speed = 30, acceleration: Acceleration = 30) -> None:
    """Home Robot

    Moves straight to the home pose. The arm takes the shortest joint path: make sure it is clear,
    or move the arm by hand to a safe pose first.
    """
    home = json.loads((POINTS / HOME_FILE).read_text())["home"]
    with moving():
        go("home", [float(v) for v in home], False, speed, acceleration)
    state.current_position = "home"


# --- Teach points: these read the arm and write files, they never move it ---------------------


@app.action(locks=["robot"], effects=Effects.REPEATABLE)
def upload_points(state: TrajectoryState, station: AnyStation, points: Points, sample: Sample = "",
                  merge: Merge = False) -> str:
    """Upload Points

    Stores teach points for a station, e.g. coordinates edited after export_points. Replaces the
    station's file unless merge is set; the old file is kept in the points folder's .history/.
    Re-teach or check every point on the arm before driving it at speed.
    """
    try:
        parsed = json.loads(points)
    except json.JSONDecodeError as e:
        raise ValueError(f"Points are not valid JSON: {e}") from None
    message = write_points(station, sample, parsed, merge)
    state.available_trajectories = station_names()
    return message


@app.action(locks=["robot"], effects=Effects.REPEATABLE)
def teach_point(state: TrajectoryState, station: AnyStation,
                point: Annotated[str, Description("The point's name, e.g. home or move3")],
                sample: Sample = "") -> str:
    """Teach Point

    Stores where the arm is now as a point of the station: guide it there by hand or with the
    teach pendant, then call this. A point of the same name is replaced in place, a new one goes
    last (the arm drives the points in file order). Reads the arm, does not move it. For the
    pose home_robot drives to, use station home_robot and point home.
    """
    rpc = connection()
    pose = value(rpc.GetActualTCPPose(), "reading the tool pose")
    joints = value(rpc.GetActualJointPosDegree(), "reading the joint angles")
    tool = value(rpc.GetActualTCPNum(), "reading the tool number")
    user = value(rpc.GetActualWObjNum(), "reading the work-object number")
    # the layout of the controller's own teach points: pose, joints, tool, user, vel, acc, ext axes
    values = [float(v) for v in [*pose, *joints, tool, user, 100, 100, 0, 0, 0, 0]]
    try:
        points = read_points(station, sample)
    except FileNotFoundError:
        points = {}
    points[point] = values
    message = write_points(station, sample, points)
    state.available_trajectories = station_names()
    return message


def teach_points(rpc, names: list[str]) -> dict:
    """Points saved by name in the Fairino teach interface, read from the controller."""
    return {name: value(rpc.GetRobotTeachingPoint(name), f"reading teach point {name!r}")
            for name in names}


@app.action(locks=["robot"], effects=Effects.REPEATABLE)
def import_teach_points(state: TrajectoryState, station: AnyStation,
                        names: Annotated[list[str], Description(
                            "Teach points saved on the controller, in driving order, e.g. home, "
                            "move1, move2")],
                        sample: Sample = "", merge: Merge = False) -> str:
    """Import Teach Points

    Copies points saved by name in the Fairino teach interface into the station's file, in the
    given order. Replaces the file unless merge is set. Reads the arm, does not move it.
    """
    if not names:
        raise ValueError("Name at least one teach point")
    message = write_points(station, sample, teach_points(connection(), names), merge)
    state.available_trajectories = station_names()
    return message


@app.action(effects=Effects.NONE)
def export_points(station: AnyStation, sample: Sample = "") -> str:
    """Export Points

    The station's points as JSON, to check them or to edit and upload them again.
    """
    return json.dumps(read_points(station, sample), indent=4)


if __name__ == "__main__":
    if VISION:
        import vision_ui

        vision_ui.start(int(os.getenv("VISION_PORT", VISION.get("port", 8093))), sys.modules[__name__])
    serve(app, CONFIG, int(os.getenv("PORT", "8080")), busy=_moving)
