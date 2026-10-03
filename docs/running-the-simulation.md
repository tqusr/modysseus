# Running the simulation

How to build, launch, and test the full Modysseus autonomy stack (localization,
perception, navigation, docking, mission) against the Gazebo arena simulation
and the Saga AI mock delivery-dispatch server.

## Prerequisites

- **ROS 2 Jazzy on Ubuntu Noble**, already installed and working
  (`/opt/ros/jazzy/setup.bash` exists).
- **The `einride-mini-truck` vendor repo**, checked out as a sibling directory
  next to this repo (`../einride-mini-truck`). It provides the robot
  description, the Gazebo world, and the sim/hardware bridge. See its own
  `README.md` for details; this guide only covers building and sourcing it.
- **[`uv`](https://docs.astral.sh/uv/)** installed, for running the Saga AI
  mock server.
- **This repo**, checked out on the `full-stack` branch (or `main`, once this
  work has landed), with the stack's packages under `src/`: `mission`,
  `localization`, `perception`, `navigation`.

## One-time setup

### 1. Build the einride-mini-truck vendor workspace

This is its own, separate colcon workspace — its packages sit at the repo
root, not under a `src/` directory:

```bash
cd ~/einride-mini-truck
source /opt/ros/jazzy/setup.bash
sudo rosdep install --from-paths . --ignore-src -r -i -y --rosdistro jazzy  # first time only
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

### 2. Build this workspace on top of it

```bash
cd ~/modysseus
colcon build
source install/setup.bash
```

Every new terminal you use to launch or interact with this stack needs all
three `source` lines, in this order (ROS core, then the vendor workspace,
then this workspace), since this workspace's `mission` package launches the
vendor workspace's `einride_mini_truck_bringup` package by name at runtime.

### 3. Set up the Saga AI mock server (one-time)

```bash
cd ~/ai-powered-autonomous-challenge-server
uv sync
uv run python -m app.credentials generate   # writes credentials.json: admin login + 4 truck tokens
```

`credentials.json` persists across restarts, so you only need to generate it
once (`generate --force` replaces it and invalidates the old tokens).

## Running the simulation

The whole real stack — Gazebo + the robot bridge, EKF, `slam_toolbox`
localization, AprilTag detection, the camera self-detection filter,
`opennav_docking`, the full Nav2 stack, and `mission` — comes up from one
command:

```bash
ros2 launch mission full_stack.launch.py saga_token:=<truck-token>
```

`<truck-token>` is one of the four truck bearer tokens in the Saga server's
`credentials.json` (see [Running a delivery route](#running-a-delivery-route-end-to-end)
for how to pull it out without printing it to your terminal history). There
is no usable default for this argument — the Saga server rejects an
unauthenticated truck, so without a real token `mission_node` will just poll
the server and fail every second (see [Known quirks](#known-quirks--things-to-watch-for)).

Useful launch arguments:

| Argument | Default | What it does |
|---|---|---|
| `world` | `arena` | World file basename (no `.sdf`) to load in Gazebo. **Leave this at `arena`** — see the quirks section below. |
| `headless` | `true` | Run the Gazebo server without the GUI. |
| `rviz` | `false` | Open RViz. Off by default to keep the compute budget for the stack itself; pass `rviz:=true` on a dev machine with headroom. |
| `saga_url` | `http://localhost:8000` | Base URL of the Saga AI server. |
| `saga_token` | `''` (empty) | Bearer token for this truck. No safe default — see above. |

Startup takes roughly 25-30 seconds from launch until `mission_node` is
actually driving the robot: the launch file deliberately staggers Gazebo,
localization, perception, docking, navigation, and mission over that window
rather than starting ~15 heavyweight processes at once. This is expected —
see [Known quirks](#known-quirks--things-to-watch-for) for what's normal to
see in the logs during that window.

Shut the stack down with Ctrl-C in the terminal running `ros2 launch`.

## Running a delivery route end-to-end

### 1. Start the Saga AI mock server

```bash
cd ~/ai-powered-autonomous-challenge-server
uv run python -m app
```

This listens on `0.0.0.0:8000`: the truck-facing API is under `/api/v1`, the
organizer admin page is at `/admin`, the audience score page is at `/score`,
and interactive API docs are at `/docs`.

### 2. Pull the admin credentials and a truck token out of credentials.json

Do this without echoing the actual values to your terminal:

```bash
cd ~/ai-powered-autonomous-challenge-server
ADMIN_USER=$(python3 -c "import json; print(json.load(open('credentials.json'))['admin']['username'])")
ADMIN_PASS=$(python3 -c "import json; print(json.load(open('credentials.json'))['admin']['password'])")
TRUCK_TOKEN=$(python3 -c "import json; print(json.load(open('credentials.json'))['trucks']['1'])")  # truck 1; use '2'-'4' for a different truck
```

(equivalently, with `jq`: `jq -r '.admin.username' credentials.json`, etc.)

### 3. Launch the stack with that truck's token

```bash
ros2 launch mission full_stack.launch.py saga_token:="$TRUCK_TOKEN"
```

### 4. Start the event

```bash
curl -u "$ADMIN_USER:$ADMIN_PASS" -X POST http://localhost:8000/admin/event/start
```

This assigns every truck its first route. Confirm it's live:

```bash
curl -H "Authorization: Bearer $TRUCK_TOKEN" http://localhost:8000/api/v1/route
```

### 5. Watch the robot work the route

`mission_node` drives the behavior tree on its own from here: it navigates
(Nav2) to the route's source dock and docks (`opennav_docking`) for real, no
further input needed. The route only advances once you confirm
loading/unloading — this stands in for an organizer pressing the dashboard
button at the physical dock:

```bash
# once the robot has arrived and docked at the source dock:
curl -u "$ADMIN_USER:$ADMIN_PASS" -X POST http://localhost:8000/admin/trucks/1/actions/arrived_at_source
sleep 4   # the "complete" action unlocks ~3s after arrival
curl -u "$ADMIN_USER:$ADMIN_PASS" -X POST http://localhost:8000/admin/trucks/1/actions/loading_complete

# the robot then undocks and navigates to the destination dock on its own;
# once it has arrived and docked there:
curl -u "$ADMIN_USER:$ADMIN_PASS" -X POST http://localhost:8000/admin/trucks/1/actions/arrived_at_destination
sleep 4
curl -u "$ADMIN_USER:$ADMIN_PASS" -X POST http://localhost:8000/admin/trucks/1/actions/unloading_complete
```

### 6. Check status and completion

```bash
curl -H "Authorization: Bearer $TRUCK_TOKEN" http://localhost:8000/api/v1/status
curl -H "Authorization: Bearer $TRUCK_TOKEN" http://localhost:8000/api/v1/routes/<route_id>
```

`unloading_complete` increments `completed_routes` and immediately assigns a
new route (watch `route_id` change) — `mission_node` picks it up on its own
within a second, no restart needed. A full route (both legs, both unlock
waits) takes several minutes of real wall-clock time, since the robot is
physically navigating and docking, not stubbed.

## Running the test suite

The `mission` package's pytest suite is unit/integration-level — it runs
against the stub action servers, not the real Gazebo stack, so Gazebo does
not need to be running:

```bash
cd ~/modysseus
colcon build --packages-select mission
source install/setup.bash
python3 -m pytest src/mission/test/ -v
```

## Known quirks / things to watch for

- **The world must stay `arena`.** `full_stack.launch.py` defaults `world` to
  `arena`, but the vendor `einride_mini_truck_bringup`'s own
  `simulation.launch.py` (which it includes) defaults to the smaller
  `einride_mini_truck` world when launched directly. Don't override `world`
  by habit from the vendor README's examples. It isn't just a cosmetic
  choice: `arena` is the only world whose dock layout matches the saved
  localization map (`src/localization/maps/arena.posegraph`/`.yaml`) and
  `mission`'s dock config (`src/mission/config/docks.yaml`) — a different
  world silently breaks localization and docking, not just the visuals.

- **`saga_token` has no safe default.** If you omit it, `mission_node` polls
  the Saga server with an empty token every second, which it rejects (401).
  This doesn't fail the launch — you'll just see repeated `Problem talking to
  Saga AI` warnings in `mission_node`'s log instead of the robot doing
  anything. If you see that, re-launch with a real `saga_token`.

- **Lifecycle activation can occasionally fail silently.** `slam_toolbox`
  (inside `localize.launch.py`) and `docking_server`
  (`docking.launch.py`) are ROS 2 managed lifecycle nodes, and in this
  environment their `change_state` service responses are sometimes lost.
  Both launch files run a watchdog that polls the node's state directly and
  retries instead of waiting on that response, then reports the outcome
  explicitly in the log: look for `slam_toolbox activation CONFIRMED -
  localization is live.` and `docking_server activation CONFIRMED -
  DockRobot is live.` a few seconds after each subsystem starts. If you
  instead see `ERROR: ... never reached "active"`, that node is not
  processing any scans/dock goals — kill the whole `full_stack.launch.py`
  (Ctrl-C) and relaunch rather than trying to fix the one node in place.

- **Startup takes ~25-30 seconds, and that's normal.** The launch file
  staggers simulation, localization, perception, docking, navigation, and
  mission over that window to avoid overloading the machine. `mission_node`
  itself starts earlier than that and will sit in a `safe_stop` loop
  (stop, report fault, retry) until the rest of the stack — particularly
  `slam_toolbox`'s `/map` — is ready. A `TF lookup map<-odom failed` message
  from `mission_node` in the first ~10-15 seconds is expected and resolves on
  its own; it is not a hang.

- **Docking and undocking bypass the normal safety chain.** Everywhere else,
  velocity commands go through `controller_server`/`behavior_server` →
  `velocity_smoother` → `collision_monitor` before reaching `/cmd_vel`.
  `docking_server` publishes `/cmd_vel` directly during dock and undock legs,
  bypassing both. Collision protection during those legs comes solely from
  `docking_server`'s own `controller.use_collision_detection` check
  (`src/perception/config/docking.yaml`) against the local costmap.

- **AprilTag detection runs but isn't wired into anything yet.**
  `apriltag.launch.py` publishes `/detections` and tag TF frames, but docking
  uses fixed dock poses (`use_external_detection_pose: false`), and nothing
  in localization or mission subscribes to tag detections. Don't expect
  camera-visible AprilTags to visibly change navigation or docking behavior.

- **Occasional transient Nav2 recovery events are expected, not failures.**
  A `FollowPath` action occasionally aborts mid-route and `bt_navigator`
  retries/recovers automatically within a few seconds with no intervention
  needed. Only treat it as a real problem if the robot stays stuck (no
  further `Begin navigating` or `Goal succeeded` log lines) for longer than
  that.
