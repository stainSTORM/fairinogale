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

Run:  uv run app.py   then open http://localhost:8080/ to bind it.
FAIRINO_CONFIG picks another config.yaml (default: next to this file).
"""

import json
import os
import platform
import shutil
import sys
import threading
from pathlib import Path
from typing import Annotated

import koil
import yaml
from arkitekt import App, Description, Effects, Task
from arkitekt.widgets import withChoices

from arkitekt_bind import serve

HERE = Path(__file__).resolve().parent
CONFIG = Path(os.getenv("FAIRINO_CONFIG", HERE / "config.yaml"))

if not CONFIG.exists():  # first start: copy the defaults where they can be edited
    CONFIG.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(HERE / "config.example.yaml", CONFIG)
    if not (CONFIG.parent / "control-points").exists():
        shutil.copytree(HERE / "control-points", CONFIG.parent / "control-points")
cfg = yaml.safe_load(CONFIG.read_text())
POINTS = CONFIG.parent / cfg.get("points_dir", "control-points")
STATIONS = list(cfg["stations"])

app = App(cfg.get("app", "fairinogale"), "1.0.0", effects=Effects.IRREVERSIBLE)

Station = Annotated[str, withChoices(*STATIONS), Description("Station from config.yaml")]
Sample = Annotated[str, Description("Fills {sample} in the station's file name, e.g. Box1_A1")]
Speed = Annotated[int, Description("Joint speed in %, outside danger zones")]
Acceleration = Annotated[int, Description("Acceleration in %")]
DangerSpeed = Annotated[int, Description("Speed in % at danger, linear and gripper points")]

_robot = None
_robot_lock = threading.Lock()


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


def gripper(position: int, velocity: int) -> None:
    g = cfg["gripper"]
    check(robot().MoveGripper(g.get("jaw", 1), position, velocity, 30, 10000, 0, 0, 0, 0, 0),
          f"gripper to {position}")


def drive(station: str, sample: str, release: bool, speed: int, acceleration: int,
          danger_speed: int, task: Task) -> None:
    if station not in cfg["stations"]:
        raise ValueError(f"Unknown station {station!r}; config.yaml has {STATIONS}")
    st = cfg["stations"][station]
    if "{sample}" in st["file"] and not sample:
        raise ValueError(f"Station {station!r} needs a sample (its file is {st['file']})")
    points = json.loads((POINTS / st["file"].format(sample=sample)).read_text())
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
    drive(station, sample, False, speed, acceleration, danger_speed, task)


@app.action(locks=["robot"])
def release_at(station: Station, sample: Sample = "", speed: Speed = cfg["speed"],
               acceleration: Acceleration = cfg["acceleration"],
               danger_speed: DangerSpeed = cfg["danger_speed"], *, task: Task) -> None:
    """Release At

    Drives to the station, puts the slide down and comes back.
    """
    drive(station, sample, True, speed, acceleration, danger_speed, task)


@app.action(locks=["robot"])
def init_robot_and_gripper() -> None:
    """Init Robot and Gripper

    Enables the arm in automatic mode, activates the gripper and moves it to its initial width.
    """
    rbt, g = robot(), cfg["gripper"]
    rbt.SetGripperConfig(g.get("company", 6), g.get("device", 0), softversion=0, bus=0)
    check(rbt.ActGripper(g.get("jaw", 1), 1), "activating the gripper")
    gripper(g.get("initial", g["close"]), 30)


@app.action(locks=["robot"])
def open_grip() -> None:
    """Open Gripper"""
    gripper(cfg["gripper"]["open"], 30)


@app.action(locks=["robot"])
def close_grip() -> None:
    """Close Gripper"""
    gripper(cfg["gripper"]["close"], 50)


@app.action(locks=["robot"])
def home_robot(speed: Speed = 30, acceleration: Acceleration = 30) -> None:
    """Home Robot

    Moves straight to the home pose. The arm takes the shortest joint path: make sure it is clear,
    or move the arm by hand to a safe pose first.
    """
    home = json.loads((POINTS / cfg.get("home_file", "home_robot.json")).read_text())["home"]
    values = [float(v) for v in home]
    check(robot().MoveJ(values[6:12], tool=int(values[12]), user=int(values[13]),
                        desc_pos=values[0:6], vel=speed, acc=acceleration), "MoveJ to home")


if __name__ == "__main__":
    serve(app, CONFIG, int(os.getenv("PORT", "8080")))
