# Vision: tags on the stations, a camera on the gripper

The arm drives taught points. When a station is bumped, a holder is put back a little differently
or the bench settles, the points no longer fit and a pick fails. With this feature the arm looks
at the station before it goes in:

- A camera on the gripper sees AprilTags fixed to the station, from one of the station's teach
  points (the **look point**).
- When the station is taught, that view is saved as the station's **reference**.
- Before every pick and release the arm compares the view with the reference. It shifts the points
  at the station by what the station moved, or backs out and fails when the station moved too far,
  the tags are hidden, or the seat is not as expected (empty before a pick, taken before a
  release).

Only the difference to the taught view is measured. The teach points stay the truth, and neither
the camera nor its position on the gripper has to be calibrated exactly.

**State of this draft:** everything below runs against a mock arm and a simulated camera
(`uv run pytest`, 24 tests). Nothing has run on the real arm or a real camera yet. See
[What is not verified](#what-is-not-verified) before the first real pick.

## 1. Try it without hardware

`mock/cell.yaml` is a ready setup: a mock arm, and a camera that draws the stations' tags as seen
from the arm's pose.

```bash
uv sync
export FAIRINO_CONFIG=mock/cell.yaml        # PowerShell: $env:FAIRINO_CONFIG = "mock/cell.yaml"
uv run debug.py look opentrons --save --yes # drive to the look point, save the reference, drive back
uv run debug.py pick opentrons --yes        # "inside the deadband, not corrected"
uv run app.py                               # the vision page: http://localhost:8093/
```

Then set `mock_shift: [2, -1, 1]` in `mock/cell.yaml` (the stations moved by 2 mm, -1 mm and 1
degree) and run the pick again: the log shows the measured shift and the three points at the
station are driven corrected. `mock_shift: [9, 0, 0]` is beyond the limit: the arm backs out and
the pick fails.

On the page, press **Drive to the look point** first: the mock arm starts at no particular pose
and the camera sees nothing from there.

## 2. What you need

- **A USB camera on the gripper.** Fixed focus, or autofocus and auto-zoom switched off: a lens
  that refocuses changes the image scale. 1280 x 720 is enough. It must see the station's tags
  from the look point, at 150 to 300 mm, both with an empty gripper and with a carrier in it.
- **Printed tags**: [tags/stations-30mm.pdf](../tags/stations-30mm.pdf) and
  [tags/carriers-15mm.pdf](../tags/carriers-15mm.pdf).
- Light that does not change much. Glossy tags under a spot lamp fail; matte paper works.

## 3. Print and place the tags

Print the PDFs at 100 % ("actual size", not "fit to page") and measure the 100 mm bar on the
sheet. The tag size is the edge of the black square: 30 mm for stations, 15 mm for carriers. If
your print is off, put the measured sizes into `config.yaml`. Cut along the grey line: the white
margin belongs to the tag.

| Ids | Where | Size |
|---|---|---|
| 0 to 3 | pickup station (0 and 1; 2 and 3 are spares) | 30 mm |
| 10 to 13 | Opentrons | 30 mm |
| 20 to 23 | microscope | 30 mm |
| 30 to 33 | another station | 30 mm |
| 100 to 147 | carriers, one each | 15 mm |

`uv run tags.py --ids 40 41 --size 20 --name extra` makes a sheet of your own.

**Station tags**

- Two per station, on one flat and stiff face that is part of the station itself: the adapter
  plate that holds the seat, not the bench, not the Opentrons lid, not the carrier.
- As far apart as the view allows, 60 mm or more. The turn of the station is measured over that
  distance.
- Flat. A tag that curls measures wrong. Glue the paper on, or print the tag into the part
  (black and white filament) or laser-engrave it.
- Each id once per cell.

**Carrier tags** (optional, for the seat check)

- One per carrier, at the same place on every carrier, visible from the look point when the
  carrier sits on its seat.
- Without carrier tags the arm still corrects the station; it only cannot tell whether a carrier
  is there.

## 4. Mount the camera and describe it

Mount the camera rigidly on the gripper adapter so that, at the look point, both station tags are
well inside the image. A camera that looks roughly square onto the tags measures best.

`config.yaml`:

```yaml
vision:
  tag_size_mm: 30
  carrier_tag_size_mm: 15
  port: 8093
  cameras:
    wrist:
      source: 0                       # camera index, or a JPEG URL, see "On the Raspberry Pi"
      width: 1280
      height: 720
      fov_deg: 70                     # horizontal field of view from the camera's data sheet
      mount: [0, -60, -80, -15, 0, 0] # the camera in the tool frame: see below
```

`mount` is the camera's pose in the tool frame of the look point, as `[x, y, z, rx, ry, rz]` in mm
and degrees, like a Fairino pose. The camera looks along its z axis; x points right in the image
and y down.

1. Check that the camera sees tags: `uv run debug.py tags` prints the ids and their distance and
   writes `debug.jpg`.
2. Put the three lengths in from the CAD or a ruler, within a centimetre. They matter little: an
   error of 20 mm costs 0.35 mm per degree the station turned.
3. Let the arm measure the three angles. Drive it to the look point (`uv run debug.py look
   <station> --stay`), then `uv run debug.py mount <station>`: it jogs 10 mm in six directions,
   watches how the tags move in the image and prints the `mount:` line. A misfit above 0.5 mm
   means a wrong tag size or something else moved.

For orientation, from the shipped teach points: user frame 1 is the base frame shifted by about
(557, -224, 19) mm, not turned. Tool 3 (the Jena points) has its tip 252 mm in front of the
flange, turned by -20 degrees about x and 45 degrees about z; tool 1 (`pick_up_frame.json`) 148 mm,
turned 135 degrees about z.

## 5. Add the station's vision block

```yaml
stations:
  opentrons:
    file: jena/opentrons.json
    out_and_back: true
    pick: {linear: [move7, move8, move9], open: [move6], close: [move9]}
    release: {linear: [move7, move8], open: [move8]}
    vision:
      camera: wrist
      look: move6                     # the teach point the camera looks from
      tags: [10, 11]                  # the tags fixed to this station
      correct: [move7, move8, move9]  # the points that move with the station
      carrier_tags: [100, 101, 102]   # optional: all carrier ids
```

- **`look`** is a point of the route before the station: typically the last point above it, where
  the gripper has not yet entered anything. In a round-trip file that is also driven backwards for
  a release, give two: `look: [approach, r_leave]`. The first one reached is used.
- **`correct`** lists the points at the station: the approach into the seat, the grip point, the
  lift. They are driven linearly at `danger_speed`, also on the way back. Every point after the
  look point that is nearer than a hand's width to the station belongs in the list.

More keys, with their defaults:

| Key | Default | Meaning |
|---|---|---|
| `max_shift_mm` | 5 | further than this: back out and fail |
| `max_yaw_deg` | 3 | same, for the turn |
| `max_lift_mm`, `max_tilt_deg` | as the two above | the station moved out of its plane: never corrected, only limited |
| `max_error_px` | 2 | the tags do not fit the reference (a tag came loose, or is half hidden) |
| `deadband_mm` | 0.3 | nearer than this: taught points, no correction |
| `on_no_tag` | `fail` | `uncorrected`: drive the taught points when the tags are not in view |
| `max_seat_mm` | 1.5 | a carrier tag within this distance of its seat counts as "on the seat" |
| `check_after` | `false` | look again on the way back: after a pick the seat must be free, after a release taken |
| `settle_s` | 0.3 | wait at the look point before the image is taken |

## 6. Teach

Do this once per station, right after its points are taught (or checked), with the station where
it belongs and one carrier on its seat.

**From Orkestrator, with the points:** `teach_station` saves the reference when you record the
look point. Its panel shows the camera's view, and it asks again until the station's tags are in it.

**With the page** (`http://<host>:8093/`): choose the station, bring the arm to the look point
(the **Drive to the look point** button with `vision.allow_motion: true`, or the pendant), check
that both station tags are outlined in green, press **Save reference**. The page refuses when the
arm is not at the look point, and asks whether to save anyway.

**From a terminal:** `uv run debug.py look opentrons --save` drives there, saves and drives back.

**From Arkitekt:** the action `save_station_reference`.

The reference is saved next to the teach points: `opentrons.vision.json` (tag corners per look
point) and `opentrons.vision.move6.jpg` (the image, for you). Keep them with the teach points.
Re-teaching a station's points means saving its reference again. So does moving the camera.

## 7. Run

- `uv run debug.py look opentrons` (or the action `check_station`) drives to the look point,
  measures and drives back. It touches nothing. Do this a few times first and watch the numbers:
  with nothing moved they should stay below the deadband. If they do not, raise `deadband_mm` or
  find what moves (camera mount, light, autofocus).
- Move the station by a millimetre or two on purpose and look again: the shift must have the
  right size and direction. Then `uv run debug.py pick opentrons` with your hand on the stop
  button.
- `pick_up` and `release_at` correct by themselves from then on. `correct: false` in a call
  drives the taught points, as before.

What the arm does at the look point:

| It sees | It does |
|---|---|
| station within `deadband_mm` | drives the taught points |
| station moved, inside the limits | shifts the `correct` points, logs the shift |
| station moved beyond a limit | backs out the way it came, fails with `StationMoved` |
| a station tag not in view | backs out, fails with `TagsNotSeen` (or `on_no_tag: uncorrected`) |
| pick: no carrier on the seat; release: seat taken | backs out, fails with `CarrierMisplaced` |
| no reference saved | backs out, fails |

Nothing has been gripped or put down when the arm backs out, so the workflow can retry, call
`check_station`, skip the slide or ask a person. Independent of the camera, a pick also fails
when the gripper closes on air: set `gripper.holding` (see `config.example.yaml`).

## 8. On the Raspberry Pi

The container needs the camera and the page's port.

- **Camera.** The simplest way is the webcam container that the Pi image already runs: plug the
  gripper camera in as its camera and set `source: http://localhost/webcam/snapshot`. The robot
  container uses the host's network, so `localhost` is the Pi. With a second camera for the cell
  overview, run a second webcam deployment with `CAM_INDEX` set to the other device.
  To let this container open the camera itself (`source: 0`), add to `deployment.compose.yml` of
  `fairino.pkg`, as `webcam.pkg` does:

  ```yaml
      volumes:
        - /dev:/dev
      device_cgroup_rules:
        - 'c 81:* rmw'
  ```

- **Page.** With host networking the page is at `http://<pi>:8093/`. For `http://<pi>/fairino-vision/`
  add a route like the one for `/fairino/` in `frontend.compose.yml`, to `host.docker.internal:8093`.
- **Terminal.** `docker exec -it <container> python debug.py tags`

## 9. Accuracy

In the simulation (two 30 mm tags 80 mm apart, 220 mm from a 1280 x 720 camera with a 70 degree
lens, viewed 20 degrees off square): shift within 0.15 mm, turn within 0.1 degree, a point 150 mm
from the tags within 0.3 mm. A field of view guessed 8 degrees wrong costs less than 0.3 mm on a
3.6 mm shift. A real camera adds blur, lens distortion and light; expect two to three times that
until measured. The carrier's distance from its seat comes from one small tag and is good to
about half a millimetre.

If that is not enough: tags further apart, a camera nearer or with more pixels, and a calibrated
camera (`matrix` and `dist` in `config.yaml`, from OpenCV's chessboard calibration; there is no
command for it yet).

## What is not verified

- **The real controller.** A corrected point is sent as `MoveL` with a pose only, so the
  controller solves the joints itself. That is the same call the taught linear points use today,
  but with a pose that was computed, not taught. Start slowly.
- **The gripper read-back.** The SDK reports the jaw position from the controller's state stream.
  Whether the Jodell gripper fills it in through the Fairino controller is not checked: watch the
  "jaws at ... %" lines in the log before setting `gripper.holding`.
- **The view.** Whether both station tags are visible from a look point with a carrier in the
  gripper depends on where the camera sits. If the carrier hides them, a release cannot be
  corrected from that point.
- **The container.** The image has not been built with OpenCV in it yet; GitHub Actions does that
  on the first push.
- **The Arkitekt side.** The new actions and the `StationCheck` result are declared (the
  definitions build), but no workflow has called them.
