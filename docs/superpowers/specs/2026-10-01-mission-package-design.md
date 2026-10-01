# Mission package — design

Date: 2026-10-01
Status: approved for implementation planning

## Goal

Build the `mission` package: the first slice of the Modysseus ROS 2 stack,
covering the "Mission & integration" work package from the team's Einride
Hackathon plan. It is a thin vertical slice — mission logic wired end-to-end
against stub Nav2/docking actions and the real (locally-hosted) Saga AI mock
server — so the rest of the team can replace stubs with real parts
incrementally without the stack ever being fully disconnected.

Success for this pass: start the local Saga AI mock server, start an event,
launch the mission stack, and watch it actually loop — drive (stub) → load
→ drive (stub) → unload → report — through several real routes end-to-end,
confirmed via the mock server's `/admin` page and the mission node's logs.

## Workspace layout

`/home/albin/modysseus` becomes its own colcon workspace, overlaid on top of
`einride-mini-truck`'s install space (two independent repos, never pushed
without explicit instruction):

```
modysseus/
  src/
    mission/
      mission/                  # Python package
        __init__.py
        bt_nodes.py              # py_trees Behaviour/Condition implementations
        mission_tree.py          # tree assembly (Mission, Route, GoToDock, WaitUntil)
        saga_client.py           # Saga AI HTTP polling client
        stub_actions.py          # stub NavigateToPose / DockRobot action servers
      launch/
        mission.launch.py
      config/
        docks.yaml
      package.xml
      setup.py / setup.cfg
      test/
```

Build/run:
```
source /home/albin/einride-mini-truck/install/setup.bash
cd /home/albin/modysseus
colcon build
source install/setup.bash
ros2 launch mission mission.launch.py
```

## Dock file (shared interface)

`src/mission/config/docks.yaml`, matching Saga AI's `tag36h11` IDs 0-7 for
docks A-H (per `docs/saga-ai.md` in einride-mini-truck):

```yaml
docks:
  A: {tag_id: 0, dock_pose: {x: 0.0, y: 0.0, theta: 0.0}, staging_pose: {x: 0.0, y: 0.0, theta: 0.0}}
  B: {tag_id: 1, dock_pose: {...}, staging_pose: {...}}
  # ... C-H
```
Poses are placeholders until localization/arena-mapping owners provide real
ones; mission only needs the schema and the letter↔tag_id mapping today.

## Stub action servers (`stub_actions.py`)

Two ROS 2 action servers standing in for Nav2/docking until those owners'
real implementations land — the exact two actions the hackathon plan names
as mission's only contract with the rest of the stack:
- `NavigateToPose` (`nav2_msgs/action/NavigateToPose`): accepts a goal pose,
  waits a short fixed delay, succeeds.
- `DockRobot` (`nav2_msgs/action/DockRobot`): accepts a dock id, waits a
  short fixed delay, succeeds.

## Saga AI client (`saga_client.py`)

Wraps the polling API in `docs/saga-ai.md`: `GET /api/v1/route` (truck's
current route: `source`, `destination`, `status`, `next_dock`, `route_id`),
bearer-token auth, plus whatever report-completion the route lifecycle
needs. ROS 2 parameters: `server_url`, `truck_token` (defaults point at our
local mock server; override for the real event server on the day). Exposes
plain Python methods the BT action nodes call — `get_current_route()`,
`report_route_done()`, `report_fault()` — no ROS service/action wrapper
needed since it's all internal to this one package for now.

## Mission behavior tree

Implemented with `py_trees` + `py_trees_ros`. Node structure is taken
directly from the team's "Behaviour Tree Einride v2.drawio" diagram in
Drive, decoded and verified (base64 + raw-deflate + XML parse) for
`Mission`, `VehicleStatusOK`, and `WaitUntil`. **`GoToDock`, `Localize`, and
`Mapping` failed to decompress from the v2 file** (corrupted payload,
confirmed on two independent decode attempts) — those three are
reconstructed from the older pre-v2 diagram, cross-checked against the v2
Mission page's own changelog notes about what changed. This is called out
explicitly below wherever it applies; it is not something to quietly paper
over, and whoever owns Localize/Mapping should re-open the v2 drawio file
directly (not via this spec) before trusting those two beyond today's stub.

```
Repeat forever
└─ Fallback: "Mission or safe stop"
   ├─ ReactiveSequence: "Mission" (guards re-checked every tick)
   │  ├─ SubTree: VehicleStatusOK   [STUBBED today — see below]
   │  ├─ SubTree: Localize          [STUBBED today — see below]
   │  └─ Sequence(memory=True): "Route"
   │     ├─ Fallback: "Have route"
   │     │  ├─ Condition: HasRoute
   │     │  └─ Action: GetRouteFromSaga → sets blackboard {source},{destination},{route_id}
   │     ├─ Fallback: "Load"
   │     │  ├─ Condition: HoldsPackage
   │     │  └─ Sequence: [GoToDock({source}), WaitUntil(SagaLoaded, timeout=5min), set HoldsPackage]
   │     ├─ Fallback: "Deliver"
   │     │  ├─ Condition: PackageDelivered
   │     │  └─ Sequence: [GoToDock({destination}), WaitUntil(SagaUnloaded, timeout=5min), set PackageDelivered]
   │     ├─ Action: ReportRouteDone (via saga_client)
   │     └─ Action: ClearRoute (reset HasRoute/HoldsPackage/PackageDelivered)
   └─ Sequence: "Safe stop"
      ├─ Action: StopRobot (publish zero /cmd_vel)
      ├─ Action: ReportFaultToSaga
      └─ Sleep 2s
```

**GoToDock(dock)** (v1-inferred, with the v2 ordering fix applied — the
Mission page's note says the only real v1→v2 change here is checking
"at dock" *before* re-triggering movement, so a `ReactiveSequence`
re-ticking the tree doesn't yank an already-docked robot back off the dock):
```
Fallback: "At dock already?"
├─ Condition: AtDock(dock)
└─ Sequence
   ├─ Fallback: "At pre-dock?"
   │  ├─ Condition: AtPreDock(dock)
   │  └─ Action: NavigateToPose(staging_pose(dock))   [stub action]
   ├─ WaitUntil(DockIsFree(dock), timeout=60s, poll=1s)   [stubbed condition: always free, today]
   └─ Action: DockRobot(dock)                             [stub action]
```

**WaitUntil(condition, timeout, poll)** — confirmed generic reusable
subtree. Ported to a single py_trees-idiomatic `Behaviour` rather than
literally composing BT.CPP's `Timeout`/`RetryUntilSuccessful`/`ForceFailure`
decorators (same semantics, simpler in py_trees since we chose it over
BT.CPP): polls `condition` each tick, returns `RUNNING` until it's true
(→ `SUCCESS`) or `timeout` elapses (→ `FAILURE`), sleeping `poll` between
checks without blocking the tree tick.

**VehicleStatusOK** and **Localize** — confirmed/inferred structure is
documented for the future owners (battery/thermal/motors/LiDAR/AprilTag
checks for the former; slam_toolbox-primary-with-AMCL-backup for the
latter), but **today's implementation is a single stub Condition node per
subtree that always returns `SUCCESS`**. None of the real signals
(hardware status topics, EKF/SLAM nodes) exist yet in this stub-only slice;
building fake versions of their internals now would be wasted work. Each
stub is clearly marked as the integration point for its respective owner.

**Mapping** — out of scope for the `mission` package entirely (it belongs
to localization). Flagging only: the pre-v2 diagram contains a literal
`"Manual intervention????"` branch the team had not resolved as of that
diagram — a known-open design question, not something this spec decides.

## Testing

- Unit tests for `saga_client.py` against a mocked HTTP layer (route
  parsing, auth header, error handling) — no network needed.
- Unit tests for the `WaitUntil` behavior (condition-true, timeout) in
  isolation using `py_trees`' test harness (tick manually, assert status
  transitions) — no ROS runtime needed.
- Integration: launch `mission.launch.py` against the real local Saga AI
  mock server (already set up and verified separately), start an event, and
  confirm via the mock server's `/admin` page and mission node logs that
  routes complete and the route counter advances across at least 2-3
  consecutive routes.

## Out of scope for this pass

- Real `NavigateToPose`/`DockRobot` implementations (navigation/perception
  owners).
- Real `VehicleStatusOK`/`Localize` subtree internals (relevant owners).
- `Mapping` subtree (localization owner) — including its unresolved manual-
  intervention branch.
- Real dock/staging poses in `docks.yaml` (localization/arena-mapping).
