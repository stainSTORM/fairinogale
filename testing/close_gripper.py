import sys
import json
import time
import os
import platform
# import numpy as np
from pathlib import Path

# Detect the OS
system_name = platform.system().lower()

# Choose the correct path based on OS
if system_name == "windows":
    sdk_path = Path("fairino-python-sdk") / "windows" / "fairino"
elif system_name == "linux":
    sdk_path = Path("fairino-python-sdk") / "linux" / "fairino"
else:
    raise OSError(f"Unsupported operating system: {system_name}")

# Add the SDK path to sys.path
sys.path.append(str(sdk_path.resolve()))

# Import the RPC module
from Robot import RPC

# ---------------------------------------------------------
# config
# ---------------------------------------------------------
# fairino5
ROBOT_IP = "192.168.1.2"   
SPEED = 30  
TOOL = 0
USER = 0

rbt = RPC(ROBOT_IP)

# jodell
company=6
device=0
jawnumber = 1
pos = 0          # percentage range [0~100]
force = 50       # percentage [0~100]
vel = 30         # percentage [0 to 100]
maxtime = 10000  # waittime in ms [0~30000]
block = 0        # 0-blocking, 1-non-blocking
typ = 0
rotNum = 0
rotVel = 0
rotTorque = 0

# ---------------------------------------------------------
# initialize
# ---------------------------------------------------------
def init_robot():
    """Connect to the robot and enable it."""
    time.sleep(0.5)
    try:
        rbt.RobotEnable(1)
        rbt.Mode(1)  # 0 = Jog, 1 = Auto, 2 = Programm
    except Exception as e:
        print("Fehler bei der Initialisierung:", e)
        rbt.RobotEnable(0)
        sys.exit()
    print(f"Connected to robot {ROBOT_IP}")
    print("Robot is ready.")


def init_gripper(openingWidth: int=70):
    """Connects to the gripper and enables it."""
    jawnumber = 1
    ret = rbt.SetGripperConfig(company, device, softversion=0, bus=0)
    print("Configure gripper ", ret)
    time.sleep(1)
    error = rbt.ActGripper(jawnumber, 1)
    print("Activate gripper ", error)
    time.sleep(2)
    error = rbt.MoveGripper(jawnumber, openingWidth, 30, 30, 10000, 0, 0, 0, 0, 0)


# ---------------------------------------------------------
# looking for teached points as json
# ---------------------------------------------------------
def load_teach_points(teach_points_file):
    """Load the teach points from a JSON file."""
    if not os.path.exists(teach_points_file):
        print(f"Error: File '{teach_points_file}' not found.")
        sys.exit()

    try:
        with open(teach_points_file, "r") as f:
            points = json.load(f)
        print(f"{len(points)} Teach Points successfully loaded.")
        return points
    except Exception as e:
        print("Error loading Teach Points:", e)
        sys.exit()


# ---------------------------------------------------------
# driving
# ---------------------------------------------------------
def open_gripper(openingWidth: int=50):
    jawnumber = 1
    print("\n--- Open the gripper ---")
    error1 = rbt.MoveGripper(jawnumber, openingWidth, 30, 30, 10000, 0, 0, 0, 0, 0)
    print("Gripper command return code:", error1)


def close_gripper(openingWidth: int=90):
    jawnumber = 1
    print("\n--- Close the gripper ---")
    error1 = rbt.MoveGripper(jawnumber, openingWidth, 60, 30, 10000, 0, 0, 0, 0, 0)
    print("Gripper command return code:", error1)


def execute_movement(points: dict, speed: int=20, danger_speed: int=5, acceleration: int=30):
    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]
            coordsLine = [float(x) for x in values[0:6]]
            move_speed = speed

            if point_name.lower().endswith("danger") or point_name.lower().endswith("gripper"):
                move_speed = danger_speed
            
            errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=move_speed, acc=acceleration)
            print(f"Point '{point_name}' reached. Return value: {errorDrive}")
        
        return True
    
    except Exception as e:
        print(f"Error moving: {e}")
        return False


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
    """Close the connection to the robot."""
    try:
        rbt.RobotEnable(0)
        print("Robot disabled.")
    except:
        print("Error occurred while disconnecting the robot.")


# ---------------------------------------------------------
# Main
# ---------------------------------------------------------
if __name__ == "__main__":
    # initialize the robot and gripper
#    init_robot()
#    init_gripper()

#    open_gripper()
    close_gripper()
    sample = "Box1_A1"

    points = {
        "ot_1_hover-over": [
        "35.764999",
        "365.615997",
        "-100.390999",
        "-178.227997",
        "0.868000",
        "-0.535000",
        "22.159000",
        "-128.552002",
        "-104.870003",
        "-36.695000",
        "91.970001",
        "157.705002",
        "1.000000",
        "1.000000",
        "100.000000",
        "100.000000",
        "0.000000",
        "0.000000",
        "0.000000",
        "0.000000"
        ],
    }

#    execute_movement(
#        points=points,
#        speed=20,
#        danger_speed=5,
#        acceleration=30
#    )
    
    # release samples in Opentrons
    # execute_release_movement(
    #     points="./control-points/pick_up_opentrons_{}.json".format(sample), 
    #     speed=20,
    #     danger_speed=3,
    #     acceleration=30
    #     )
    
    # pick up samples from Opentrons
    # execute_pick_up_movement(
    #     points="./control-points/pick_up_opentrons_{}.json".format(sample), 
    #     speed=20,
    #     danger_speed=3,
    #     acceleration=30
    #     )
    
    # release samples on FRAME
    # execute_release_movement(
    #     points="./control-points/pick_up_frame.json",
    #     speed=40,
    #     danger_speed=5,
    #     acceleration=30
    #     )

    # pick up samples from FRAME
    # execute_pick_up_movement(
    #     points="./control-points/pick_up_frame.json",
    #     speed=40,
    #     danger_speed=5,
    #     acceleration=30
    #     )
    
    # Disconnect the robot
    # shutdown_robot()
