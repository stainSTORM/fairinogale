# -*- coding: utf-8 -*-

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple
import os
import sys
import json
import time
import threading
import platform
from pathlib import Path
from arkitekt_next import easy, register
from rekuest_next.actors.sync import SyncGroup
from dotenv import load_dotenv


global_sync = SyncGroup("robot_sync")

# ------------------------------------------------------------------------------
# Fairino SDK import (Windows/Linux) with optional FAIRINO_SDK_DIR override
# ------------------------------------------------------------------------------
def _resolve_sdk_path() -> Optional[Path]:
    # Allow explicit override for CI/containers: FAIRINO_SDK_DIR=/path/to/fairino/.../fairino
    env_dir = os.getenv(
        "FAIRINO_SDK_DIR", "/home/pi/fairinogale/fairino-python-sdk/linux/fairino"
    )  # /home/pi/fairinogale/linux/fairino
    if env_dir:
        return Path(env_dir)

    system_name = platform.system().lower()
    root = Path("fairino-python-sdk")
    if system_name == "windows":
        return (root / "windows" / "fairino").resolve()
    elif system_name == "linux":
        return (root / "linux" / "fairino").resolve()
    else:
        # Unsupported OS: caller will fall back to MockRPC
        return None


RPC = None  # will be assigned below

load_dotenv()

sdk_path = _resolve_sdk_path()
print(f"SDK path: {sdk_path}")
if sdk_path and sdk_path.exists():
    sys.path.append(str(sdk_path))
    try:
        from Robot import RPC as _RealRPC  # type: ignore

        RPC = _RealRPC
        print(f"Fairino SDK loaded from {sdk_path}")
    except Exception as e:
        print(f"Failed loading Fairino SDK from {sdk_path}: {e}")

if RPC is None:
    # Fallback to MockRPC for environments without the hardware SDK installed.
    try:
        from mock.MockRPC import MockRPC  # type: ignore
    except Exception:
        try:
            from MockRPC import MockRPC  # type: ignore
        except Exception:
            MockRPC = None  # No mock available

    if MockRPC is None:
        raise ImportError(
            "Neither Fairino SDK nor MockRPC is available. "
            "Install the SDK or provide a MockRPC for development."
        )
    RPC = MockRPC  # type: ignore
    print("Fairino SDK unavailable; using MockRPC fallback")


# ---------------------------------------------------------
# configuration
# ---------------------------------------------------------
# fairino5
ROBOT_IP = os.getenv("ROBOT_IP", "192.168.5.1")
SPEED = 30
TOOL = 0
USER = 0


# gripper settings
COMPANY = 6
DEVICE = 0
JAWNUMBER = 1
POS = 0  # percentage range [0~100]
FORCE = 50  # percentage [0~100]
VELOCITY = 30  # percentage [0 to 100]
MAXTIME = 10000  # waittime in ms [0~30000]
IS_BLOCKING = 0  # 0-blocking, 1-non-blocking
TYP = 0
ROT_NUM = 0
ROT_VEL = 0
ROT_TORQUE = 0


rbt = RPC(ROBOT_IP)

IS_INITIALIZED = False

# ---------------------------------------------------------
# initialization
# ---------------------------------------------------------
def init() -> bool:
    global IS_INITIALIZED
    if IS_INITIALIZED:
        return True
    print("Start initializing robot and gripper")
    if not init_robot():
        return False
    if not init_gripper():
        return False
    print("Robot and gripper initialized")
    IS_INITIALIZED = True
    return True


def init_robot():
    """Establishes the connection to the robot and activates it."""
    time.sleep(0.5)
    try:
        print(f"Connected to robot {ROBOT_IP}")
        print(f"Robot object: {rbt}")
        rbt.RobotEnable(1)
        rbt.Mode(1)  # 0 = Jog, 1 = Auto, 2 = Program
        print("Robot is ready.")
    except Exception as e:
        print("Initialization error:", e)
        rbt.RobotEnable(0)
        return False
    return True


def init_gripper(openingWidth: int = 95):
    """Connects to the gripper and activates it."""
    try:
        jawnumber = 1
        ret = rbt.SetGripperConfig(COMPANY, DEVICE, softversion=0, bus=0)
        print("Configure gripper ", ret)
        time.sleep(1)
        error = rbt.ActGripper(jawnumber, 1)
        print("Activate gripper ", error)
        time.sleep(2)
        rbt.MoveGripper(
            jawnumber, openingWidth, 30, 30, 10000, 0, 0, 0, 0, 0
        )
    except Exception as e:
        print("Error initializing gripper:", e)
        return False
    return True


# ---------------------------------------------------------
# loading taught points as JSON
# ---------------------------------------------------------
def load_teach_points(teach_points_file) -> Optional[Dict[str, List[float]]]:
    """Loads the stored teach points from a JSON file."""
    if not os.path.exists(teach_points_file):
        print(f"Error: File '{teach_points_file}' not found.")
        return None

    try:
        with open(teach_points_file, "r") as f:
            points = json.load(f)
        print(f"{len(points)} teach points loaded successfully.")
        return points
    except Exception as e:
        print("Error loading teach points:", e)
        return None


# ---------------------------------------------------------
# driving
# ---------------------------------------------------------

def open_gripper(openingWidth: int = 50):
    jawnumber = 1
    print("\n--- Starting gripper movement ---")
    error1 = rbt.MoveGripper(jawnumber, openingWidth, 30, 30, 10000, 0, 0, 0, 0, 0)
    print("Gripper command return code:", error1)


def close_gripper(openingWidth: int = 95):
    jawnumber = 1
    print("\n--- Starting gripper movement ---")
    error1 = rbt.MoveGripper(jawnumber, openingWidth, 50, 30, 10000, 0, 0, 0, 0, 0)
    print("Gripper command return code:", error1)


def execute_pick_up_movement(points: dict, speed: int=50, danger_speed: int=10, acceleration: int=30):
    points = load_teach_points(points)

    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]
            coordsLine = [float(x) for x in values[0:6]]
            move_speed = speed

            if point_name.lower().endswith("danger") or point_name.lower().endswith("gripper"):
                move_speed = danger_speed
            
            if point_name.lower().endswith("open-gripper"):
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=move_speed, acc=acceleration)
                open_gripper()
            elif point_name.lower().endswith("close-gripper"):
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=move_speed, acc=acceleration)
                close_gripper()
            else:
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=move_speed, acc=acceleration)
            
            print(f"Point '{point_name}' reached. Return value: {errorDrive}")
            
        return True

    except Exception as e:
        print(f"Error moving: {e}")
        return False
    

def execute_release_movement(points: dict, speed: int=50, danger_speed: int=10, acceleration: int=30):
    points = load_teach_points(points)
    points = dict(reversed(list(points.items())))

    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]
            coordsLine = [float(x) for x in values[0:6]]
            move_speed = speed

            if point_name.lower().endswith("danger") or point_name.lower().endswith("gripper"):
                move_speed = danger_speed
            
            if point_name.lower().endswith("open-gripper"):
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=move_speed, acc=acceleration)
                close_gripper()
            elif point_name.lower().endswith("close-gripper"):
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=move_speed, acc=acceleration)
                open_gripper()
            else:
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=move_speed, acc=acceleration)
            
            print(f"Point '{point_name}' reached. Return value: {errorDrive}")
            
        return True

    except Exception as e:
        print(f"Error moving: {e}")
        return False


def shutdown_robot():
    """Disconnects from the robot."""
    global IS_INITIALIZED
    try:
        rbt.RobotEnable(0)
        print("Robot shut down.")
        IS_INITIALIZED = False
    except:
        print("Error disconnecting.")


@register()
def release_at_opentrons(sample: str, speed: int = 40, acceleration: int = 30, dangerSpeed: int = 5):
    """Move the samples into the Opentrons."""
    if not init():
        print("Could not start routine. Exiting.")
        return
    # release samples in Opentrons
    execute_release_movement(
        points="./control-points/pick_up_opentrons_{}.json".format(sample), 
        speed=20,
        danger_speed=3,
        acceleration=30
        )


@register()
def pick_up_opentrons(sample: str, speed: int = 40, acceleration: int = 30, dangerSpeed: int = 5):
    """Pick up the samples from the Opentrons."""
    if not init():
        print("Could not start routine. Exiting.")
        return
    # pick up samples from Opentrons
    execute_pick_up_movement(
        points="./control-points/pick_up_opentrons_{}.json".format(sample), 
        speed=20,
        danger_speed=3,
        acceleration=30
        )


@register()
def release_at_frame(sample: str, speed: int = 40, acceleration: int = 30, dangerSpeed: int = 5):
    """Move the samples onto the FRAME."""
    if not init():
        print("Could not start routine. Exiting.")
        return
    execute_release_movement(
        points="./control-points/pick_up_frame.json",
        speed=40,
        danger_speed=5,
        acceleration=30
        )


@register()
def pick_up_frame(sample: str, speed: int = 40, acceleration: int = 30, dangerSpeed: int = 5):
    """Pick up the samples from the FRAME."""
    if not init():
        print("Could not start routine. Exiting.")
        return
    # pick up samples from FRAME
    execute_pick_up_movement(
        points="./control-points/pick_up_frame.json",
        speed=40,
        danger_speed=5,
        acceleration=30
        )
    

@register()
def init_robot_and_gripper():
    """Initialize the robot and gripper."""
    if not init():
        print("Initialization failed. Exiting.")
    init_robot()
    init_gripper()
    print("Robot and gripper initialized successfully.")


@register()
def open_grip():
    """Open the gripper."""
    if not init():
        print("Robot not initialized. Exiting.")
        return
    try:
        open_gripper()
        print("Gripper opened successfully.")
    except Exception as e:
        print(f"Error opening gripper: {e}")


@register()
def close_grip():
    """Close the gripper."""
    if not init():
        print("Robot not initialized. Exiting.")
        return
    try:
        close_gripper()
        print("Gripper closed successfully.")
    except Exception as e:
        print(f"Error closing gripper: {e}")


@register()
def home_robot(move_speed: int = 30, acceleration: int = 30):
    """Move the robot to the home position. ATTENTION: The robot will take the shortest path to the home position, so make sure that the way is clear, or move the arm manually to a safe position before homing."""
    points = load_teach_points("./control-points/home_robot.json")
    coords = [float(x) for x in points['home'][6:12]]

    if not init():
        print("Robot not initialized. Exiting.")
        return
    try:
        rbt.MoveJ(coords, tool=1, user=1, vel=move_speed, acc=acceleration)
        print("Robot homed successfully.")
    except Exception as e:
        print(f"Error homing robot: {e}")
    

if __name__ == "__main__":
    app_name = os.getenv("ARKITEKT_APPNAME", "farinogale")
    if app_name == "":
        print("ARKITEKT_APPNAME is not set. Please set the ARKITEKT_APPNAME environment variable. For example put it in .env file.")
        exit(1)
    app_url = os.getenv("ARKITEKT_URL", "go.arkitekt.live")
    app = easy(identifier=app_name, url=app_url, redeem_token=os.getenv("REDEEM_TOKEN"))
    app.enter()
    app.run()
