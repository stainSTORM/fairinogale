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

Run:  uv run app.py   then open http://localhost:8080/ to bind it.
FAIRINO_CONFIG picks another config.yaml (default: next to this file).
"""

import json
import os
import platform
import re
import shutil
import sys
import threading
from datetime import datetime
from fnmatch import fnmatch
from pathlib import Path
from typing import Annotated

import koil
import yaml
from arkitekt import App, Description, Effects, Task
from arkitekt.widgets import ParagraphWidget, withChoices

from arkitekt_bind import restart_soon, serve

HERE = Path(__file__).resolve().parent
CONFIG = Path(os.getenv("FAIRINO_CONFIG", HERE / "config.yaml"))

if not CONFIG.exists():  # first start: copy the defaults where they can be edited
    CONFIG.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(HERE / "config.example.yaml", CONFIG)
    if not (CONFIG.parent / "control-points").exists():
        shutil.copytree(HERE / "control-points", CONFIG.parent / "control-points")
cfg = yaml.safe_load(CONFIG.read_text())
POINTS = (CONFIG.parent / cfg.get("points_dir", "control-points")).resolve()
HOME_FILE = cfg.get("home_file", "home_robot.json")


def station_names() -> list[str]:
    """config.yaml's stations, then the other teach-point files in the points folder."""
    names = list(cfg["stations"])
    files = [st["file"].replace("{sample}", "*") for st in cfg["stations"].values()]
    for path in sorted(POINTS.glob("*.json")):
        if path.stem not in names and not any(fnmatch(path.name, f) for f in files + [HOME_FILE]):
            names.append(path.stem)
    return names


STATIONS = station_names()

app = App(cfg.get("app", "fairinogale"), "1.0.0", effects=Effects.IRREVERSIBLE)

Station = Annotated[str, withChoices(*STATIONS), Description("Station from config.yaml or the points folder")]
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

_robot = None
_automatic = False  # the arm was switched to automatic mode by this process
_robot_lock = threading.Lock()
_moving = threading.Lock()  # held while the arm moves; a restart of the app waits for it


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


def gripper(position: int, velocity: int) -> None:
    g = cfg["gripper"]
    check(connection().MoveGripper(g.get("jaw", 1), position, velocity, 30, 10000, 0, 0, 0, 0, 0),
          f"gripper to {position}")


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
    """Store a station's points; the file it replaces goes to .history/. A new station appears in
    pick_up and release_at once the app has restarted, which it then does by itself."""
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
    message = f"Saved {len(points)} points of {station!r} to {path}"
    if station_names() != STATIONS:
        restart_soon(f"new station {station!r}")
        message += "; the app restarts so that pick_up and release_at offer it"
    return message


def drive(station: str, sample: str, release: bool, speed: int, acceleration: int,
          danger_speed: int, task: Task) -> None:
    st = station_file(station, sample)[0]
    points = read_points(station, sample)
    plan = st.get("release" if release else "pick") or {}

    if st.get("out_and_back"):
        moves = [n for n in points if n != "home"]
        home = ["home"] if "home" in points else []
        order = home + moves + moves[-2::-1] + home
        outward = len(home) + len(moves)  # the gripper acts on the way out only
    else:
        order = list(points)[::-1] if release else list(points)
        outward = len(order)

    rbt, g = robot(), cfg["gripper"]
    for i, name in enumerate(order):
        task.pausepoint()
        koil.check_cancelled()  # a cancelled task stops at the next point, not after the route
        values = [float(v) for v in points[name]]
        pose, joints, tool, user = values[0:6], values[6:12], int(values[12]), int(values[13])
        lower = name.lower()
        linear = name in plan.get("linear", [])
        vel = danger_speed if linear or lower.endswith(("danger", "gripper")) else speed
        if linear:
            check(rbt.MoveL(pose, tool=tool, user=user, vel=vel, acc=acceleration), f"MoveL to {name}")
        else:
            check(rbt.MoveJ(joints, tool=tool, user=user, desc_pos=pose, vel=vel, acc=acceleration),
                  f"MoveJ to {name}")

        opens = closes = False
        if i < outward:
            opens, closes = name in plan.get("open", []), name in plan.get("close", [])
        if lower.endswith("open-gripper"):
            opens, closes = not release, release
        elif lower.endswith("close-gripper"):
            opens, closes = release, not release
        if opens:
            gripper(g["open"], 30)
        if closes:
            gripper(g["close"], 50)
        task.progress(int(100 * (i + 1) / len(order)), name)


@app.action(locks=["robot"])
def pick_up(station: Station, sample: Sample = "", speed: Speed = cfg["speed"],
            acceleration: Acceleration = cfg["acceleration"],
            danger_speed: DangerSpeed = cfg["danger_speed"], *, task: Task) -> None:
    """Pick Up

    Drives to the station, grips the slide and comes back.
    """
    with _moving:
        drive(station, sample, False, speed, acceleration, danger_speed, task)


@app.action(locks=["robot"])
def release_at(station: Station, sample: Sample = "", speed: Speed = cfg["speed"],
               acceleration: Acceleration = cfg["acceleration"],
               danger_speed: DangerSpeed = cfg["danger_speed"], *, task: Task) -> None:
    """Release At

    Drives to the station, puts the slide down and comes back.
    """
    with _moving:
        drive(station, sample, True, speed, acceleration, danger_speed, task)


@app.action(locks=["robot"])
def init_robot_and_gripper() -> None:
    """Init Robot and Gripper

    Enables the arm in automatic mode (also after it was switched to manual mode for teaching),
    activates the gripper and moves it to its initial width.
    """
    with _moving:
        rbt, g = robot(force_automatic=True), cfg["gripper"]
        rbt.SetGripperConfig(g.get("company", 6), g.get("device", 0), softversion=0, bus=0)
        check(rbt.ActGripper(g.get("jaw", 1), 1), "activating the gripper")
        gripper(g.get("initial", g["close"]), 30)


@app.action(locks=["robot"])
def open_grip() -> None:
    """Open Gripper"""
    with _moving:
        robot()
        gripper(cfg["gripper"]["open"], 30)


@app.action(locks=["robot"])
def close_grip() -> None:
    """Close Gripper"""
    with _moving:
        robot()
        gripper(cfg["gripper"]["close"], 50)


@app.action(locks=["robot"])
def home_robot(speed: Speed = 30, acceleration: Acceleration = 30) -> None:
    """Home Robot

    Moves straight to the home pose. The arm takes the shortest joint path: make sure it is clear,
    or move the arm by hand to a safe pose first.
    """
    home = json.loads((POINTS / HOME_FILE).read_text())["home"]
    values = [float(v) for v in home]
    with _moving:
        check(robot().MoveJ(values[6:12], tool=int(values[12]), user=int(values[13]),
                            desc_pos=values[0:6], vel=speed, acc=acceleration), "MoveJ to home")


# --- Teach points: these read the arm and write files, they never move it ---------------------


@app.action(locks=["robot"], effects=Effects.REPEATABLE)
def upload_points(station: AnyStation, points: Points, sample: Sample = "",
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
    return write_points(station, sample, parsed, merge)


@app.action(locks=["robot"], effects=Effects.REPEATABLE)
def teach_point(station: AnyStation,
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
    return write_points(station, sample, points)


def teach_points(rpc, names: list[str]) -> dict:
    """Points saved by name in the Fairino teach interface, read from the controller."""
    return {name: value(rpc.GetRobotTeachingPoint(name), f"reading teach point {name!r}")
            for name in names}


@app.action(locks=["robot"], effects=Effects.REPEATABLE)
def import_teach_points(station: AnyStation,
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
    return write_points(station, sample, teach_points(connection(), names), merge)


@app.action(effects=Effects.NONE)
def export_points(station: AnyStation, sample: Sample = "") -> str:
    """Export Points

    The station's points as JSON, to check them or to edit and upload them again.
    """
    return json.dumps(read_points(station, sample), indent=4)


if __name__ == "__main__":
    serve(app, CONFIG, int(os.getenv("PORT", "8080")), busy=_moving)
