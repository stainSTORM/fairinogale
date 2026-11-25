# fairinogale

**The Fairino robot arm as an [Arkitekt](https://arkitekt.live) app.** It moves slides between the
stations of the staining workflow (pickup station, Opentrons, microscope), so that a workflow such
as [protocolapp](https://github.com/stainSTORM/protocolapp) can call it.

It runs as a container on the openUC2 OS Raspberry Pi image (see
[Raspberry Pi](#raspberry-pi)), or with `uv` on any computer that can reach the arm.

## Actions

| Action | Arguments | What it does |
|---|---|---|
| `pick_up` | `station`, `sample`, `speed`, `acceleration`, `danger_speed` | Drives to the station, grips the slide, comes back |
| `release_at` | same | Drives to the station, puts the slide down, comes back |
| `init_robot_and_gripper` | | Enables the arm in automatic mode (also after teaching in manual mode), activates the gripper |
| `open_grip`, `close_grip` | | |
| `home_robot` | `speed`, `acceleration` | Straight to the home pose (shortest joint path: clear the way first) |
| `teach_point` | `station`, `point`, `sample` | Stores where the arm is now as a named point of the station. Does not move the arm |
| `import_teach_points` | `station`, `names`, `sample`, `merge` | Copies points saved by name on the controller into the station. Does not move the arm |
| `upload_points` | `station`, `points` (JSON), `sample`, `merge` | Stores pasted coordinates for the station |
| `export_points` | `station`, `sample` | The station's points as JSON |

`station` in `pick_up` and `release_at` is a dropdown of the stations (see
[Stations and teach points](#stations-and-teach-points)). `sample` fills `{sample}` in a station's
file name (for example `Box1_A1`); stations without `{sample}` ignore it. The speeds default to the
values in `config.yaml`.

- Every action except `export_points` holds one `robot` lock, so calls run one at a time.
- A move the controller rejects (non-zero error code) stops the sequence and fails the action.
  The old code printed the error and carried on.
- Cancelling the task in Arkitekt stops the arm at the next teach point.

## Configuration

Everything is in `config.yaml`: the arm's IP, the gripper positions, the default speeds and the
stations. On the first start it is copied from [config.example.yaml](config.example.yaml), which
explains each key. `robot.mock: true` runs without an arm and only prints the moves.

### Stations and teach points

A station is a teach-point file in `control-points/`: the stations `config.yaml` lists, and every
other `.json` file there, under its file name. Two layouts work:

- **Round trip** (the original fairinogale layout, e.g. `pick_up_frame.json`): every pose in
  driving order, including the way back (`r_...` points). Name suffixes set the behaviour:
  `_danger` is driven at `danger_speed`; `_open-gripper` and `_close-gripper` act on the gripper.
  `release_at` drives the file backwards, with open and close swapped.
- **Out and back** (the Jena layout, e.g. `jena/opentrons.json`): `home`, `move1` … `moveN`. The arm
  drives out to `moveN` and back the same way. The station lists, separately for pick and release,
  the points driven linearly (MoveL at `danger_speed`) and where the gripper opens and closes on
  the way out.

```yaml
stations:
  frame:
    file: pick_up_frame.json
  opentrons:
    file: jena/opentrons.json
    out_and_back: true
    pick: {linear: [move7, move8, move9], open: [move6], close: [move9]}
    release: {linear: [move7, move8], open: [move8]}
```

A station `config.yaml` does not list is a round trip. Give it a `config.yaml` entry for the out
and back layout or another file name.

The shipped teach points come from two installations (`frame` from the fairinogale setup,
`opentrons` and `pickupstation` from the Jena setup). Teach points belong to one arm and one
bench, so re-teach them for yours before driving the arm.

### Teaching

`control-points/` sits next to `config.yaml`: on the Pi that is
`/home/pi/arkitekt/fairino/control-points/`, a bind mount, so taught points survive container
updates. The app reads a station's file at every move, so a changed station applies at once. A
**new** station shows up in the `pick_up` and `release_at` dropdowns after a restart, which the app
then does by itself (after the current move, if one is running). Every file that is replaced is
kept in `control-points/.history/`.

Three ways to set a station's points from Arkitekt. None of them moves the arm:

- **`teach_point(station, point)`**: guide the arm to a pose (by hand in manual mode, or with the
  teach pendant), then call it with the point's name, e.g. `home`, then `move1`, `move2` … A
  point of the same name is replaced where it is; a new one goes last, and the arm drives the
  points in file order. For the pose `home_robot` drives to, use station `home_robot`, point
  `home`. Call `init_robot_and_gripper` afterwards to switch the arm back to automatic mode.
- **`import_teach_points(station, names)`**: name the poses in the Fairino teach interface, then
  copy them from the controller in the order given (what `get_points.py` did).
- **`upload_points(station, points)`**: paste coordinates as JSON, e.g. from `export_points`
  after editing, or from another installation:

  ```json
  {"home":  [x, y, z, rx, ry, rz, j1, j2, j3, j4, j5, j6, tool, user, 100, 100, 0, 0, 0, 0],
   "move1": [...]}
  ```

  A point needs at least 14 numbers: the tool pose, the joint angles in degrees, the tool and
  the work-object number. The controller's teach points add speed, acceleration and the external
  axes, which the app does not use. `merge` keeps the station's other points.

The same from a shell, without Arkitekt:

```bash
uv run teach.py control-points/my_station.json home move1 move2 move3
docker exec -it <container> python teach.py /config/control-points/my_station.json home move1 …
```

## Running locally (uv)

```bash
uv sync
uv run app.py                 # config.yaml next to app.py; FAIRINO_CONFIG=... for another one
```

Open <http://localhost:8080/> to bind it (see below). Set `robot.mock: true` to try it without an
arm. On the Pi, stop the container first (`forklift plt disable-depl fairino && forklift plt apply`)
so that the two do not drive the arm at the same time. Then point `FAIRINO_CONFIG` at
`/home/pi/arkitekt/fairino/config.yaml` to reuse its config and stored login.

## Binding to an Arkitekt server

The page on port 8080 (on the Pi: `http://<pi>/fairino/`):

1. Enter the server (e.g. `https://go.arkitekt.live`, or a local one) and press **Bind**.
2. The page shows a code and an approval link. Open the link in any browser, sign in and check that
   the code matches.
3. The app registers its actions and stays connected. The login is stored in `.state/` next to
   `config.yaml`, so the app reconnects by itself after a restart.

**Disconnect** keeps the stored login; **Unbind** forgets it. Tick *allow plain http* for a server
without TLS (e.g. a NAS); the session token then travels unencrypted. The page also edits
`config.yaml`: **Save and restart** checks the YAML, writes it and restarts the app.

`arkitekt_bind.py` is the same file in fairinogale and OT2windy_service. Keep the copies identical.

## Docker

```bash
docker build -t fairinogale .
docker run -p 8080:8080 -v ~/fairino:/config fairinogale
```

GitHub Actions ([.github/workflows/docker.yml](.github/workflows/docker.yml)) builds
`ghcr.io/stainstorm/fairinogale` for arm64 and amd64 on every push, tagged `sha-<commit>` and with
the branch name.

## Raspberry Pi

The Arkitekt robot pallet (branch
[`arkitekt-service` of openUC2/os-rpi](https://github.com/openUC2/os-rpi/tree/arkitekt-service))
deploys this container as `fairino`, switched on, in its `os-rpi-arkitekt` SD card image. The
microscope pallet on `edge` does not have it. The page is linked from the machine's home page.

On the Pi, the config and teach points live in `/home/pi/arkitekt/fairino/`. To ship a new version,
push here, then set the new `sha-<commit>` tag in that branch's `deployments/fairino.pkg/deployment.compose.yml`.

## fairino-python-sdk

The vendored [Fairino Python SDK](https://fair-documentation.readthedocs.io/en/latest/SDKManual/python_intro.html)
(`fairino-python-sdk/`). The container uses only `linux/fairino/Robot.py`, which is pure Python.
`FAIRINO_SDK_DIR` points the app at another copy.

## Project structure

```
fairinogale/
├── app.py                 # the Arkitekt app: actions, stations, robot connection
├── arkitekt_bind.py       # binding page and connection (same file in OT2windy_service)
├── teach.py               # read teach points from the controller into a station file (shell)
├── config.example.yaml    # copied to config.yaml on the first start
├── control-points/        # teach-point files (jena/: the Jena installation)
├── mock/MockRPC.py        # stand-in for the SDK when robot.mock is set
├── testing/               # older standalone test scripts
└── fairino-python-sdk/    # vendored Fairino SDK
```
