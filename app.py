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

A station with a vision block is looked at before the arm goes in: a camera on the gripper sees the
station's AprilTags from one of the teach points, and the points behind it are shifted by what the
station moved since teaching (vision.py, docs/vision.md). debug.py calls the same functions from a
terminal, without Arkitekt.

Run:  uv run app.py   then open http://localhost:8080/ to bind it.
FAIRINO_CONFIG picks another config.yaml (default: next to this file).
"""

import json
import os
import platform
import shutil
import sys
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Annotated

import koil
import numpy as np
import yaml
from arkitekt import App, Description, Effects, Task
from arkitekt.widgets import withChoices

import vision
from arkitekt_bind import serve

HERE = Path(__file__).resolve().parent
CONFIG = Path(os.getenv("FAIRINO_CONFIG", HERE / "config.yaml"))

if not CONFIG.exists():  # first start: copy the defaults where they can be edited
    CONFIG.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(HERE / "config.example.yaml", CONFIG)
cfg = yaml.safe_load(CONFIG.read_text())
POINTS = CONFIG.parent / cfg.get("points_dir", "control-points")
if not POINTS.exists():  # start from the shipped teach points
    shutil.copytree(HERE / "control-points", POINTS)
STATIONS = list(cfg["stations"])
VISION = cfg.get("vision") or {}
TAG_SIZE = VISION.get("tag_size_mm", 30.0)  # edge of the printed tag's black square

app = App(cfg.get("app", "fairinogale"), "1.0.0", effects=Effects.IRREVERSIBLE)

Station = Annotated[str, withChoices(*STATIONS), Description("Station from config.yaml")]
Sample = Annotated[str, Description("Fills {sample} in the station's file name, e.g. Box1_A1")]
Speed = Annotated[int, Description("Joint speed in %, outside danger zones")]
Acceleration = Annotated[int, Description("Acceleration in %")]
DangerSpeed = Annotated[int, Description("Speed in % at danger, linear and gripper points")]
Correct = Annotated[bool, Description("Look at the station's tags first and shift the points "
                                      "(stations with a vision block)")]

_robot = None
_robot_lock = threading.Lock()
_busy = threading.Lock()
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
    if not _busy.acquire(blocking=False):
        raise RuntimeError("The arm is busy with another move")
    try:
        yield
    finally:
        _busy.release()


def rpc_class():
    """The Fairino SDK's RPC, or the mock when robot.mock is set."""
    if cfg["robot"].get("mock"):
        from mock.MockRPC import MockRPC

        return MockRPC
    sdk = "windows" if platform.system() == "Windows" else "linux"
    sys.path.insert(0, os.getenv("FAIRINO_SDK_DIR", str(HERE / "fairino-python-sdk" / sdk / "fairino")))
    from Robot import RPC

    return RPC


def robot():
    """The connection to the arm, opened on first use so the app starts without it."""
    global _robot
    with _robot_lock:
        if _robot is None:
            ip, RPC = cfg["robot"]["ip"], rpc_class()
            rpc = RPC(ip)
            if getattr(RPC, "is_connect", True) is False:
                rpc.CloseRPC()  # stops the SDK's receive threads
                raise ConnectionError(f"Cannot reach the Fairino robot at {ip} (robot.ip in {CONFIG})")
            rpc.RobotEnable(1)
            rpc.Mode(1)  # 0 = manual, 1 = automatic
            _robot = rpc
        return _robot


def check(error, what: str) -> None:
    """The SDK returns 0 on success; anything else stops the sequence."""
    if error not in (0, None):
        raise RuntimeError(f"{what} failed with Fairino error code {error}")


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
    g, rbt = cfg["gripper"], robot()
    check(rbt.MoveGripper(g.get("jaw", 1), position, velocity, 30, 10000, 0, 0, 0, 0, 0),
          f"gripper to {position}")
    deadline = time.monotonic() + g.get("settle_s", 3.0)
    while time.monotonic() < deadline:  # MoveGripper blocks, but the reported state lags behind
        error, state = rbt.GetGripperMotionDone()
        if error == 0 and state and state[1] == 1:
            break
        time.sleep(0.05)
    return jaw_position()


def jaw_position() -> float:
    """The jaw position in % from the controller's state stream; a gripper fault raises."""
    _, fault, position = robot().GetGripperCurPosition()
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
    if station not in cfg["stations"]:
        raise ValueError(f"Unknown station {station!r}; config.yaml has {STATIONS}")
    st = cfg["stations"][station]
    if "{sample}" in st["file"] and not sample:
        raise ValueError(f"Station {station!r} needs a sample (its file is {st['file']})")
    file = POINTS / st["file"].format(sample=sample)
    points = {name: [float(v) for v in values] for name, values in json.loads(file.read_text()).items()}
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

            cam = _cameras[name] = MockCamera(name, spec, lambda: robot().tcp_pose, TAG_SIZE)
            dx, dy, yaw = spec.get("mock_shift", [0, 0, 0])
            for station, st in cfg["stations"].items():
                vis = st.get("vision") or {}
                if vis.get("camera") != name or "{sample}" in st["file"] or not vis.get("tags"):
                    continue
                look = station_points(station)[2][look_points(st)[0]][0:6]
                carrier = list(vis.get("carrier_tags", []))[:1]
                cam.place(vis["tags"], look)
                cam.place(carrier, look, row=45)
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
                                             vis.get("tags"), vis.get("carrier_tags", ()))
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
        reply = robot().GetActualJointPosDegree()
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
          danger_speed: int, task, correct: bool = True) -> None:
    st, _, points = station_points(station, sample)
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
            if closes:
                position = gripper(g["close"], 50)
                log(task, f"{name}: gripper closed, jaws at {position:.0f} %")
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
def pick_up(station: Station, sample: Sample = "", speed: Speed = cfg["speed"],
            acceleration: Acceleration = cfg["acceleration"],
            danger_speed: DangerSpeed = cfg["danger_speed"], correct: Correct = True,
            *, task: Task) -> None:
    """Pick Up

    Drives to the station, grips the slide and comes back. Fails if the jaws do not stop where
    they stop on a carrier (gripper.holding in config.yaml). At a station with a vision block the
    arm first looks at the station's tags: it shifts the points, or backs out and fails when the
    station moved too far or the seat is empty.
    """
    drive(station, sample, False, speed, acceleration, danger_speed, task, correct)


@app.action(locks=["robot"])
def release_at(station: Station, sample: Sample = "", speed: Speed = cfg["speed"],
               acceleration: Acceleration = cfg["acceleration"],
               danger_speed: DangerSpeed = cfg["danger_speed"], correct: Correct = True,
               *, task: Task) -> None:
    """Release At

    Drives to the station, puts the slide down and comes back. At a station with a vision block
    the arm first looks at the station's tags, like Pick Up; it fails when the seat is taken.
    """
    drive(station, sample, True, speed, acceleration, danger_speed, task, correct)


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

    Enables the arm in automatic mode, activates the gripper and moves it to its initial width.
    """
    rbt, g = robot(), cfg["gripper"]
    with moving():
        rbt.SetGripperConfig(g.get("company", 6), g.get("device", 0), softversion=0, bus=0)
        check(rbt.ActGripper(g.get("jaw", 1), 1), "activating the gripper")
        gripper(g.get("initial", g["close"]), 30)


@app.action(locks=["robot"])
def open_grip() -> None:
    """Open Gripper"""
    with moving():
        gripper(cfg["gripper"]["open"], 30)


@app.action(locks=["robot"])
def close_grip() -> None:
    """Close Gripper"""
    with moving():
        gripper(cfg["gripper"]["close"], 50)


@app.action(locks=["robot"])
def home_robot(speed: Speed = 30, acceleration: Acceleration = 30) -> None:
    """Home Robot

    Moves straight to the home pose. The arm takes the shortest joint path: make sure it is clear,
    or move the arm by hand to a safe pose first.
    """
    home = json.loads((POINTS / cfg.get("home_file", "home_robot.json")).read_text())["home"]
    with moving():
        go("home", [float(v) for v in home], False, speed, acceleration)


if __name__ == "__main__":
    if VISION:
        import vision_ui

        vision_ui.start(int(os.getenv("VISION_PORT", VISION.get("port", 8093))), sys.modules[__name__])
    serve(app, CONFIG, int(os.getenv("PORT", "8080")))
