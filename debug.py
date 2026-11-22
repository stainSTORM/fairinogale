"""The arm and its camera from a terminal, without Arkitekt.

    uv run debug.py tags                     what the camera sees; writes debug.jpg
    uv run debug.py measure opentrons        compare the view with the saved reference (no move)
    uv run debug.py reference opentrons      save the view as the reference (arm at the look point)
    uv run debug.py look opentrons           drive to the look point, measure, drive back
    uv run debug.py look opentrons --save    ... and save the reference there
    uv run debug.py look opentrons --stay    ... and stop there (then: back)
    uv run debug.py back opentrons           drive back from the look point
    uv run debug.py mount opentrons          measure the camera's angles (arm at the look point)
    uv run debug.py pick opentrons           pick_up, as Arkitekt would call it (--taught: no camera)
    uv run debug.py release opentrons
    uv run debug.py page                     only the vision page, http://localhost:8093/

A station that has {sample} in its file name takes the sample as the next word: pick pickupstation
Box1_A1. Everything reads the same config.yaml as app.py (FAIRINO_CONFIG); with robot.mock and a
camera whose source is "mock" no hardware is needed. On the Pi:
docker exec -it <container> python debug.py tags
"""

import argparse
import sys
import time

import cv2

import app
import vision

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("command", choices=["tags", "measure", "reference", "look", "back", "mount",
                                        "pick", "release", "page"])
parser.add_argument("station", nargs="?", help="station from config.yaml (tags: the camera)")
parser.add_argument("sample", nargs="?", default="")
parser.add_argument("--save", action="store_true", help="look: save the reference at the look point")
parser.add_argument("--stay", action="store_true", help="look: do not drive back")
parser.add_argument("--force", action="store_true", help="reference: do not ask the arm where it is")
parser.add_argument("--taught", action="store_true", help="pick, release: taught points, no camera")
parser.add_argument("--jog", type=float, default=10.0, help="mount: length of the jogs in mm")
parser.add_argument("--yes", action="store_true", help="do not ask before the arm moves")
args = parser.parse_args()

task = app.ConsoleTask()
speeds = app.cfg["speed"], app.cfg["acceleration"], app.cfg["danger_speed"]


def confirm(what: str) -> None:
    if not args.yes and input(f"{what} Is the way clear? [y/N] ").strip().lower() != "y":
        sys.exit("Nothing moved.")


def report(seen: dict) -> None:
    cv2.imwrite("debug.jpg", vision.annotate(seen["image"], seen["now"], seen["reference"]))
    print(f"Tags in view: {sorted(seen['now'])} (debug.jpg)")
    if seen["measurement"]:
        print(seen["measurement"].summary())
    if seen["error"]:
        print(f"Stay out: {seen['error']}")
    elif seen["apply"]:
        print(f"Would correct: the points move up to {seen['travel_mm']:.2f} mm")
    else:
        print(f"As taught: inside the deadband ({seen['travel_mm']:.2f} mm)")
    if seen["on_seat"] is not None:
        print("Carrier on the seat" if seen["on_seat"] else "No carrier on the seat")


if args.command == "tags":
    cameras = app.VISION.get("cameras") or sys.exit("config.yaml has no vision.cameras")
    cam = app.camera(args.station or next(iter(cameras)))
    image = cam.grab()
    found = vision.detect(image)
    K = cam.matrix(image.shape[1], image.shape[0])
    for i, corners in sorted(found.items()):
        centre = vision.tag_points(corners, K, cam.dist, app.TAG_SIZE)[0].mean(axis=0)
        print(f"tag {i:3d}: {centre[2]:6.0f} mm away, {abs(corners[0] - corners[1]).max():4.0f} px wide")
    cv2.imwrite("debug.jpg", vision.annotate(image, found))
    print(f"{len(found)} tags in a {image.shape[1]} x {image.shape[0]} image (debug.jpg)")
elif args.command == "page":
    import vision_ui

    vision_ui.start(int(app.VISION.get("port", 8093)), app)
    while True:
        time.sleep(1)
elif not args.station:
    parser.error(f"{args.command} needs a station: {app.STATIONS}")
elif args.command == "measure":
    report(app.observe(args.station, args.sample))
elif args.command == "reference":
    report(app.save_reference(args.station, args.sample, force=args.force))
    print("Reference saved.")
elif args.command == "look":
    confirm(f"The arm drives to the look point of {args.station}.")
    report(app.look_at(args.station, args.sample, args.save, args.stay, *speeds, task))
    if args.save:
        print("Reference saved.")
elif args.command == "back":
    confirm("The arm drives back from the look point.")
    app.leave_look(args.station, args.sample, *speeds[:2])
elif args.command in ("pick", "release"):
    confirm(f"The arm drives to {args.station}.")
    app.drive(args.station, args.sample, args.command == "release", *speeds, task, not args.taught)
elif args.command == "mount":
    # Six straight jogs away from the look point and back; the tags must stay in view.
    st, _, points = app.station_points(args.station, args.sample)
    look = app.look_points(st)[0]
    values, d = points[look], args.jog
    cam = app.camera(st["vision"]["camera"])
    confirm(f"The arm stands at look point {look} and jogs {d:g} mm in +-x, +-y and +-z from there.")
    with app.moving():
        app.go(look, values, True, speeds[2], speeds[1])
        time.sleep(0.5)
        image = cam.grab()
        K, start, jogs = cam.matrix(image.shape[1], image.shape[0]), vision.detect(image), []
        try:
            for axis in range(3):
                for sign in (1, -1):
                    jog = [sign * d if k == axis else 0.0 for k in range(3)]
                    pose = [a + b for a, b in zip(values[0:3], jog)] + values[3:6]
                    app.go(f"jog {jog}", values, True, speeds[2], speeds[1], pose)
                    time.sleep(0.5)
                    moved, error = vision.displacement(start, vision.detect(cam.grab()), K, cam.dist,
                                                       app.TAG_SIZE)
                    print(f"jog {jog}: the tags moved {moved.round(2)} mm for the camera, fit {error:.2f} px")
                    jogs.append((jog, moved))
        finally:
            app.go(look, values, True, speeds[2], speeds[1])
    angles, misfit = vision.mount_rotation(jogs, values[0:6])
    old = cam.spec.get("mount", [0, 0, 0, 0, 0, 0])
    print(f"Misfit {misfit:.2f} mm over {d:g} mm jogs (above about {d / 20:.1f} mm: check tag_size_mm "
          f"and that nothing else moved)")
    print(f"config.yaml, vision.cameras.{cam.name}:")
    print(f"  mount: [{old[0]:g}, {old[1]:g}, {old[2]:g}, {angles[0]:.1f}, {angles[1]:.1f}, {angles[2]:.1f}]")
