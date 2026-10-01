# Mission Package Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the `mission` ROS 2 package: a thin vertical slice of the Modysseus stack — mission behavior tree, Saga AI client, stub `NavigateToPose`/`DockRobot` action servers, and the dock file — that runs end-to-end in simulation today, so other work packages can swap stubs for real parts later without ever breaking the build.

**Architecture:** An `ament_python` package (`src/mission`) built with `py_trees` + `py_trees_ros`. The mission behavior tree (ticked at 1 Hz by a `rclpy` timer node) polls a read-only Saga AI HTTP client for the current route, drives it via two stub ROS 2 action servers standing in for Nav2/docking, and tracks dock arrival on a blackboard key until real localization replaces that bookkeeping.

**Tech Stack:** ROS 2 Jazzy, `py_trees` 2.5 / `py_trees_ros` 2.5 (apt: `ros-jazzy-py-trees`, `ros-jazzy-py-trees-ros`), `rclpy`, `nav2_msgs` (for the `NavigateToPose`/`DockRobot` action definitions — confirmed present via `ros2 interface list` on this machine), `requests` (already installed), `PyYAML`, `pytest`.

**Spec:** `docs/superpowers/specs/2026-10-01-mission-package-design.md`

## Global Constraints

- Work only in `/home/albin/modysseus` — **never `git push`** without explicit instruction (standing rule for this repo).
- `modysseus` is a colcon workspace overlaid on top of `einride-mini-truck`'s install space: always `source /home/albin/einride-mini-truck/install/setup.bash` before `source /opt/ros/jazzy/setup.bash`'s sibling workspace build/install steps below — concretely: `source /opt/ros/jazzy/setup.bash && source /home/albin/einride-mini-truck/install/setup.bash` before building or running anything in `modysseus`.
- Mission talks to the rest of the stack **only** through `NavigateToPose` and `DockRobot` actions (`nav2_msgs/action/NavigateToPose`, `nav2_msgs/action/DockRobot` — verified present in this ROS 2 Jazzy install).
- **Correction vs. the spec:** the Saga AI API (`einride-mini-truck/docs/saga-ai.md`, confirmed by reading it in full) is **read-only from the truck's side** — there is no endpoint for the truck to report route completion or faults; status is advanced by organizers pressing buttons on their own admin page. `ReportRouteDone`/`ReportFaultToSaga` are therefore implemented as local log statements only, not API calls. `SagaApi` only ever does `GET`.
- Saga AI polling: once per second, 0.5 s request timeout, never block a ROS callback on it (per `docs/saga-ai.md`'s own "Good habits" section) — the tree's 1 Hz tick rate satisfies this directly, no extra rate-limiting needed.
- `VehicleStatusOK` and `Localize` subtrees are stub Conditions that always return `SUCCESS` this pass (no real signals exist yet) — do not build out their internals.
- `Mapping` subtree is out of scope entirely (localization owner's package).

---

## File Structure

```
modysseus/src/mission/
  package.xml
  setup.py
  setup.cfg
  resource/mission
  mission/
    __init__.py
    docks.py              # Task 2: dock file loader
    saga_client.py         # Task 3: read-only Saga AI HTTP client
    bt_behaviours.py       # Tasks 4+6: WaitUntil, CallAction, blackboard flag helpers
    stub_action_servers.py # Task 5: stub NavigateToPose / DockRobot action servers
    mission_tree.py        # Task 7: tree assembly (GoToDock, Route, Mission, safe-stop)
    mission_node.py        # Task 8: rclpy node ticking the tree at 1 Hz
  config/
    docks.yaml             # Task 2
  launch/
    mission.launch.py      # Task 8
  test/
    test_docks.py          # Task 2
    test_saga_client.py    # Task 3
    test_bt_behaviours.py  # Tasks 4+6
    test_stub_action_servers.py  # Task 5
    test_mission_tree.py   # Task 7
```

---

### Task 1: Package scaffolding

**Files:**
- Create: `src/mission/package.xml`
- Create: `src/mission/setup.py`
- Create: `src/mission/setup.cfg`
- Create: `src/mission/resource/mission`
- Create: `src/mission/mission/__init__.py`
- Create: `src/mission/test/test_docks.py` (placeholder-free smoke test, extended for real in Task 2)

**Interfaces:**
- Produces: an importable `mission` Python package, installed by colcon, that every later task builds into.

- [ ] **Step 1: Install the two missing apt packages (needs sudo — run this yourself)**

Run in your own terminal (not through Claude — sudo needs an interactive password):
```bash
sudo apt update || true
sudo apt install -y ros-jazzy-py-trees ros-jazzy-py-trees-ros
```
Verify: `python3 -c "import py_trees; print(py_trees.__version__)"` after `source /opt/ros/jazzy/setup.bash` should print `2.5.0` (or newer).

- [ ] **Step 2: Create the workspace skeleton**

```bash
mkdir -p /home/albin/modysseus/src/mission/mission
mkdir -p /home/albin/modysseus/src/mission/resource
mkdir -p /home/albin/modysseus/src/mission/test
mkdir -p /home/albin/modysseus/src/mission/config
mkdir -p /home/albin/modysseus/src/mission/launch
touch /home/albin/modysseus/src/mission/mission/__init__.py
touch /home/albin/modysseus/src/mission/resource/mission
```

- [ ] **Step 3: Write `package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mission</name>
  <version>0.0.0</version>
  <description>Mission behavior tree, Saga AI client, and stub Nav2/docking actions for Modysseus.</description>
  <maintainer email="albin@stenboda.se">Albin</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>py_trees</depend>
  <depend>py_trees_ros</depend>
  <depend>nav2_msgs</depend>
  <depend>action_msgs</depend>
  <depend>python3-requests</depend>
  <depend>python3-yaml</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 4: Write `setup.py`**

```python
from setuptools import find_packages, setup

package_name = 'mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/docks.yaml']),
        ('share/' + package_name + '/launch', ['launch/mission.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Albin',
    maintainer_email='albin@stenboda.se',
    description='Mission behavior tree, Saga AI client, and stub Nav2/docking actions for Modysseus.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = mission.mission_node:main',
            'stub_action_servers = mission.stub_action_servers:main',
        ],
    },
)
```

Note: `setup.py`'s `data_files` references `config/docks.yaml` and `launch/mission.launch.py`, which don't exist yet — that's fine, they're created in Tasks 2 and 8 respectively, before this package is ever built. Create an empty placeholder now so `colcon build` in Step 6 doesn't fail on a missing file:

```bash
touch /home/albin/modysseus/src/mission/config/docks.yaml
touch /home/albin/modysseus/src/mission/launch/mission.launch.py
```

- [ ] **Step 5: Write `setup.cfg`**

```ini
[develop]
script_dir=$base/lib/mission
[install]
install_scripts=$base/lib/mission
```

- [ ] **Step 6: Write the smoke test**

`src/mission/test/test_docks.py`:
```python
def test_mission_package_importable():
    import mission  # noqa: F401
```

- [ ] **Step 7: Build and verify**

```bash
source /opt/ros/jazzy/setup.bash
source /home/albin/einride-mini-truck/install/setup.bash
cd /home/albin/modysseus
colcon build --packages-select mission
source install/setup.bash
python3 -m pytest src/mission/test/test_docks.py -v
```
Expected: build succeeds, 1 test passes.

- [ ] **Step 8: Commit**

```bash
cd /home/albin/modysseus
git add src/mission
git commit -m "Scaffold the mission package"
```

---

### Task 2: Dock file loader

**Files:**
- Create: `src/mission/config/docks.yaml` (overwrite the Task 1 placeholder)
- Create: `src/mission/mission/docks.py`
- Modify: `src/mission/test/test_docks.py`

**Interfaces:**
- Produces: `mission.docks.Dock` (dataclass: `letter: str`, `tag_id: int`, `dock_pose: dict`, `staging_pose: dict`), `mission.docks.load_docks(path: str) -> dict[str, Dock]`.

- [ ] **Step 1: Write the failing tests**

`src/mission/test/test_docks.py` (replace the Task 1 placeholder entirely):
```python
import textwrap

from mission.docks import load_docks


def test_mission_package_importable():
    import mission  # noqa: F401


def test_load_docks_parses_all_fields(tmp_path):
    yaml_path = tmp_path / "docks.yaml"
    yaml_path.write_text(textwrap.dedent("""\
        docks:
          A:
            tag_id: 0
            dock_pose: {x: 1.0, y: 2.0, theta: 0.0}
            staging_pose: {x: 0.5, y: 2.0, theta: 0.0}
          B:
            tag_id: 1
            dock_pose: {x: 3.0, y: 2.0, theta: 1.57}
            staging_pose: {x: 2.5, y: 2.0, theta: 1.57}
        """))

    docks = load_docks(str(yaml_path))

    assert set(docks.keys()) == {"A", "B"}
    assert docks["A"].letter == "A"
    assert docks["A"].tag_id == 0
    assert docks["A"].dock_pose == {"x": 1.0, "y": 2.0, "theta": 0.0}
    assert docks["B"].staging_pose == {"x": 2.5, "y": 2.0, "theta": 1.57}
```

- [ ] **Step 2: Run tests to verify the new one fails**

```bash
python3 -m pytest src/mission/test/test_docks.py -v
```
Expected: `test_load_docks_parses_all_fields` FAILS with `ModuleNotFoundError: No module named 'mission.docks'`.

- [ ] **Step 3: Write `mission/docks.py`**

```python
from dataclasses import dataclass

import yaml


@dataclass(frozen=True)
class Dock:
    letter: str
    tag_id: int
    dock_pose: dict
    staging_pose: dict


def load_docks(path: str) -> dict[str, Dock]:
    with open(path) as handle:
        raw = yaml.safe_load(handle)
    docks = {}
    for letter, entry in raw["docks"].items():
        docks[letter] = Dock(
            letter=letter,
            tag_id=entry["tag_id"],
            dock_pose=entry["dock_pose"],
            staging_pose=entry["staging_pose"],
        )
    return docks
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest src/mission/test/test_docks.py -v
```
Expected: both tests PASS.

- [ ] **Step 5: Write the real `config/docks.yaml`** (placeholder poses — real ones come from localization/arena-mapping later; letters/tag_ids match `einride-mini-truck/docs/saga-ai.md`'s A-H / 0-7 mapping)

```yaml
docks:
  A: {tag_id: 0, dock_pose: {x: 0.0, y: 0.0, theta: 0.0}, staging_pose: {x: 0.0, y: 0.0, theta: 0.0}}
  B: {tag_id: 1, dock_pose: {x: 0.0, y: 0.0, theta: 0.0}, staging_pose: {x: 0.0, y: 0.0, theta: 0.0}}
  C: {tag_id: 2, dock_pose: {x: 0.0, y: 0.0, theta: 0.0}, staging_pose: {x: 0.0, y: 0.0, theta: 0.0}}
  D: {tag_id: 3, dock_pose: {x: 0.0, y: 0.0, theta: 0.0}, staging_pose: {x: 0.0, y: 0.0, theta: 0.0}}
  E: {tag_id: 4, dock_pose: {x: 0.0, y: 0.0, theta: 0.0}, staging_pose: {x: 0.0, y: 0.0, theta: 0.0}}
  F: {tag_id: 5, dock_pose: {x: 0.0, y: 0.0, theta: 0.0}, staging_pose: {x: 0.0, y: 0.0, theta: 0.0}}
  G: {tag_id: 6, dock_pose: {x: 0.0, y: 0.0, theta: 0.0}, staging_pose: {x: 0.0, y: 0.0, theta: 0.0}}
  H: {tag_id: 7, dock_pose: {x: 0.0, y: 0.0, theta: 0.0}, staging_pose: {x: 0.0, y: 0.0, theta: 0.0}}
```

- [ ] **Step 6: Rebuild and run the full package's tests**

```bash
colcon build --packages-select mission
source install/setup.bash
python3 -m pytest src/mission/test/ -v
```
Expected: all tests PASS.

- [ ] **Step 7: Commit**

```bash
git add src/mission/config/docks.yaml src/mission/mission/docks.py src/mission/test/test_docks.py
git commit -m "Add dock file loader"
```

---

### Task 3: Saga AI client

**Files:**
- Create: `src/mission/mission/saga_client.py`
- Create: `src/mission/test/test_saga_client.py`

**Interfaces:**
- Produces: `mission.saga_client.SagaApi(server_url: str, token: str, timeout: float = 0.5)` with methods `get_route() -> dict | None`, `get_status() -> dict`, `get_route_by_id(route_id: int) -> dict`, `get_docks() -> list[dict]`. All raise `requests.RequestException` on network/HTTP errors (caller's responsibility to catch — see Task 7).

- [ ] **Step 1: Write the failing tests**

`src/mission/test/test_saga_client.py`:
```python
from unittest.mock import MagicMock, patch

import pytest
import requests

from mission.saga_client import SagaApi


def _fake_response(json_body, status_ok=True):
    response = MagicMock()
    response.json.return_value = json_body
    if status_ok:
        response.raise_for_status.return_value = None
    else:
        response.raise_for_status.side_effect = requests.HTTPError("401")
    return response


@patch("mission.saga_client.requests.get")
def test_get_route_returns_the_route_field(mock_get):
    mock_get.return_value = _fake_response({
        "truck_id": 1,
        "event_state": "running",
        "route": {"route_id": 5, "source": {"letter": "C", "tag_id": 2},
                   "destination": {"letter": "F", "tag_id": 5}},
    })

    api = SagaApi("http://localhost:8000", "secret-token")
    route = api.get_route()

    assert route["route_id"] == 5
    mock_get.assert_called_once_with(
        "http://localhost:8000/api/v1/route",
        headers={"Authorization": "Bearer secret-token"},
        timeout=0.5,
    )


@patch("mission.saga_client.requests.get")
def test_get_route_returns_none_before_event_starts(mock_get):
    mock_get.return_value = _fake_response({"truck_id": 1, "event_state": "not_started", "route": None})

    api = SagaApi("http://localhost:8000", "secret-token")

    assert api.get_route() is None


@patch("mission.saga_client.requests.get")
def test_wrong_token_raises(mock_get):
    mock_get.return_value = _fake_response({}, status_ok=False)

    api = SagaApi("http://localhost:8000", "wrong-token")

    with pytest.raises(requests.HTTPError):
        api.get_route()


@patch("mission.saga_client.requests.get")
def test_server_url_trailing_slash_is_stripped(mock_get):
    mock_get.return_value = _fake_response({"truck_id": 1, "event_state": "running", "route": None})

    api = SagaApi("http://localhost:8000/", "secret-token")
    api.get_route()

    mock_get.assert_called_once_with(
        "http://localhost:8000/api/v1/route",
        headers={"Authorization": "Bearer secret-token"},
        timeout=0.5,
    )
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest src/mission/test/test_saga_client.py -v
```
Expected: FAIL with `ModuleNotFoundError: No module named 'mission.saga_client'`.

- [ ] **Step 3: Write `mission/saga_client.py`**

```python
import requests


class SagaApi:
    """Read-only client for the Saga AI event server.

    The truck never reports anything back — route status is advanced by
    organizers pressing buttons on their own admin page. Every method here
    is a GET.
    """

    def __init__(self, server_url: str, token: str, timeout: float = 0.5):
        self.server_url = server_url.rstrip('/')
        self.headers = {'Authorization': f'Bearer {token}'}
        self.timeout = timeout

    def _get(self, path: str) -> dict:
        response = requests.get(
            self.server_url + path,
            headers=self.headers,
            timeout=self.timeout,
        )
        response.raise_for_status()
        return response.json()

    def get_route(self):
        """The truck's current route, or None if the event hasn't started."""
        return self._get('/api/v1/route')['route']

    def get_status(self) -> dict:
        return self._get('/api/v1/status')

    def get_route_by_id(self, route_id: int) -> dict:
        return self._get(f'/api/v1/routes/{route_id}')

    def get_docks(self) -> list:
        return self._get('/api/v1/docks')
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest src/mission/test/test_saga_client.py -v
```
Expected: all 4 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/mission/mission/saga_client.py src/mission/test/test_saga_client.py
git commit -m "Add read-only Saga AI client"
```

---

### Task 4: WaitUntil behaviour

**Files:**
- Create: `src/mission/mission/bt_behaviours.py`
- Create: `src/mission/test/test_bt_behaviours.py`

**Interfaces:**
- Produces: `mission.bt_behaviours.WaitUntil(name: str, condition: Callable[[], bool], timeout_sec: float, poll_sec: float = 1.0, clock: Callable[[], float] = time.monotonic)`, a `py_trees.behaviour.Behaviour`.
- Also produces (same file, used by Task 6): `BlackboardFlagTrue(name, key)` and `SetBlackboardFlag(name, key, value)` — written in Step 5 below, generic reusable blackboard-flag condition/action pair used by Task 7 for `HoldsPackage`/`PackageDelivered`.

- [ ] **Step 1: Write the failing tests for `WaitUntil`**

`src/mission/test/test_bt_behaviours.py`:
```python
import py_trees

from mission.bt_behaviours import WaitUntil


class _FakeClock:
    def __init__(self, start=0.0):
        self.now = start

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def test_wait_until_succeeds_once_condition_is_true():
    clock = _FakeClock()
    condition = {"value": False}
    node = WaitUntil(
        "wait",
        condition=lambda: condition["value"],
        timeout_sec=10.0,
        poll_sec=1.0,
        clock=clock,
    )
    node.initialise()

    assert node.update() == py_trees.common.Status.RUNNING

    condition["value"] = True
    clock.advance(1.0)  # past the 1s poll interval, so the next tick re-checks
    assert node.update() == py_trees.common.Status.SUCCESS


def test_wait_until_stays_running_between_polls_even_if_true():
    clock = _FakeClock()
    node = WaitUntil("wait", condition=lambda: True, timeout_sec=10.0, poll_sec=1.0, clock=clock)
    node.initialise()

    assert node.update() == py_trees.common.Status.RUNNING  # first poll happens immediately...
    assert node.update() == py_trees.common.Status.SUCCESS  # ...and succeeds right away


def test_wait_until_fails_after_timeout():
    clock = _FakeClock()
    node = WaitUntil("wait", condition=lambda: False, timeout_sec=5.0, poll_sec=1.0, clock=clock)
    node.initialise()

    node.update()
    clock.advance(6.0)
    assert node.update() == py_trees.common.Status.FAILURE
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest src/mission/test/test_bt_behaviours.py -v
```
Expected: FAIL with `ModuleNotFoundError: No module named 'mission.bt_behaviours'`.

- [ ] **Step 3: Write `WaitUntil` in `mission/bt_behaviours.py`**

```python
import time

import py_trees


class WaitUntil(py_trees.behaviour.Behaviour):
    """Polls `condition` until it's true or `timeout_sec` elapses.

    Non-blocking: each tick either returns immediately (no sleeping) or
    reports RUNNING. `clock` is injectable so tests don't need real time.
    """

    def __init__(self, name, condition, timeout_sec, poll_sec=1.0, clock=time.monotonic):
        super().__init__(name)
        self.condition = condition
        self.timeout_sec = timeout_sec
        self.poll_sec = poll_sec
        self.clock = clock
        self._start_time = None
        self._last_poll = None

    def initialise(self):
        self._start_time = self.clock()
        self._last_poll = None

    def update(self):
        now = self.clock()
        if now - self._start_time > self.timeout_sec:
            return py_trees.common.Status.FAILURE
        if self._last_poll is not None and (now - self._last_poll) < self.poll_sec:
            return py_trees.common.Status.RUNNING
        self._last_poll = now
        if self.condition():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest src/mission/test/test_bt_behaviours.py -v
```
Expected: all 3 `WaitUntil` tests PASS.

- [ ] **Step 5: Write the failing tests for the blackboard flag helpers**

Append to `src/mission/test/test_bt_behaviours.py`:
```python
from mission.bt_behaviours import BlackboardFlagTrue, SetBlackboardFlag


def test_blackboard_flag_true_fails_when_unset():
    node = BlackboardFlagTrue("flag?", key="holds_package")
    node.setup()
    assert node.update() == py_trees.common.Status.FAILURE


def test_set_then_check_blackboard_flag():
    setter = SetBlackboardFlag("set flag", key="holds_package", value=True)
    checker = BlackboardFlagTrue("flag?", key="holds_package")
    setter.setup()
    checker.setup()

    assert setter.update() == py_trees.common.Status.SUCCESS
    assert checker.update() == py_trees.common.Status.SUCCESS
```

- [ ] **Step 6: Run tests to verify the new ones fail**

```bash
python3 -m pytest src/mission/test/test_bt_behaviours.py -v
```
Expected: the two new tests FAIL with `ImportError` (names not defined yet).

- [ ] **Step 7: Add `BlackboardFlagTrue` and `SetBlackboardFlag` to `mission/bt_behaviours.py`**

```python
class BlackboardFlagTrue(py_trees.behaviour.Behaviour):
    """Generic reusable condition: SUCCESS if blackboard[key] is truthy."""

    def __init__(self, name, key):
        super().__init__(name)
        self.key = key
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    def update(self):
        value = self.blackboard.get(self.key) if self.blackboard.exists(self.key) else False
        return py_trees.common.Status.SUCCESS if value else py_trees.common.Status.FAILURE


class SetBlackboardFlag(py_trees.behaviour.Behaviour):
    """Generic reusable action: writes `value` to blackboard[key], always SUCCESS."""

    def __init__(self, name, key, value):
        super().__init__(name)
        self.key = key
        self.value = value
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def update(self):
        self.blackboard.set(self.key, self.value)
        return py_trees.common.Status.SUCCESS
```

- [ ] **Step 8: Run tests to verify they pass**

```bash
python3 -m pytest src/mission/test/test_bt_behaviours.py -v
```
Expected: all 5 tests PASS.

- [ ] **Step 9: Commit**

```bash
git add src/mission/mission/bt_behaviours.py src/mission/test/test_bt_behaviours.py
git commit -m "Add WaitUntil and blackboard flag behaviours"
```

---

### Task 5: Stub action servers

**Files:**
- Create: `src/mission/mission/stub_action_servers.py`
- Create: `src/mission/test/test_stub_action_servers.py`

**Interfaces:**
- Produces: `mission.stub_action_servers.StubActionServers`, an `rclpy.node.Node` subclass that serves `NavigateToPose` on action name `navigate_to_pose` and `DockRobot` on action name `dock_robot`, each accepting any goal, waiting `delay_sec` (constructor param, default `0.5`), and succeeding. Also a `main()` entry point (`ros2 run mission stub_action_servers`).

- [ ] **Step 1: Write the failing test**

`src/mission/test/test_stub_action_servers.py`:
```python
import rclpy
from nav2_msgs.action import DockRobot, NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from mission.stub_action_servers import StubActionServers


def _call_action_and_wait(executor, client, goal, timeout_sec=5.0):
    send_future = client.send_goal_async(goal)
    executor.spin_until_future_complete(send_future, timeout_sec=timeout_sec)
    goal_handle = send_future.result()
    assert goal_handle.accepted
    result_future = goal_handle.get_result_async()
    executor.spin_until_future_complete(result_future, timeout_sec=timeout_sec)
    return result_future.result()


def test_navigate_to_pose_succeeds():
    rclpy.init()
    try:
        server = StubActionServers(delay_sec=0.05)
        client_node = rclpy.create_node("test_client")
        client = ActionClient(client_node, NavigateToPose, "navigate_to_pose")
        executor = SingleThreadedExecutor()
        executor.add_node(server)
        executor.add_node(client_node)

        assert client.wait_for_server(timeout_sec=5.0)
        result = _call_action_and_wait(executor, client, NavigateToPose.Goal())

        assert result.status == 4  # GoalStatus.STATUS_SUCCEEDED
    finally:
        rclpy.shutdown()


def test_dock_robot_succeeds():
    rclpy.init()
    try:
        server = StubActionServers(delay_sec=0.05)
        client_node = rclpy.create_node("test_client")
        client = ActionClient(client_node, DockRobot, "dock_robot")
        executor = SingleThreadedExecutor()
        executor.add_node(server)
        executor.add_node(client_node)

        assert client.wait_for_server(timeout_sec=5.0)
        result = _call_action_and_wait(executor, client, DockRobot.Goal())

        assert result.status == 4  # GoalStatus.STATUS_SUCCEEDED
    finally:
        rclpy.shutdown()
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
python3 -m pytest src/mission/test/test_stub_action_servers.py -v
```
Expected: FAIL with `ModuleNotFoundError: No module named 'mission.stub_action_servers'`.

- [ ] **Step 3: Write `mission/stub_action_servers.py`**

```python
import time

import rclpy
from nav2_msgs.action import DockRobot, NavigateToPose
from rclpy.action import ActionServer
from rclpy.node import Node


class StubActionServers(Node):
    """Stands in for Nav2's NavigateToPose and the docking server's DockRobot
    until those owners' real implementations land. Accepts any goal, waits
    `delay_sec`, succeeds.
    """

    def __init__(self, delay_sec: float = 2.0):
        super().__init__('stub_action_servers')
        self.delay_sec = delay_sec
        self._navigate_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose', self._execute_navigate)
        self._dock_server = ActionServer(
            self, DockRobot, 'dock_robot', self._execute_dock)

    def _execute_navigate(self, goal_handle):
        self.get_logger().info('Stub NavigateToPose: driving...')
        time.sleep(self.delay_sec)
        goal_handle.succeed()
        return NavigateToPose.Result()

    def _execute_dock(self, goal_handle):
        self.get_logger().info('Stub DockRobot: docking...')
        time.sleep(self.delay_sec)
        goal_handle.succeed()
        return DockRobot.Result()


def main():
    rclpy.init()
    node = StubActionServers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
python3 -m pytest src/mission/test/test_stub_action_servers.py -v
```
Expected: both tests PASS. (`ActionServer`'s default single-goal executor runs goals on a callback thread, so `time.sleep` inside `_execute_navigate`/`_execute_dock` does not block the executor spinning the action protocol itself — if this surprises you with a hang, confirm by running with `-s` to see the log lines and check `ActionServer`'s `execute_callback` threading model.)

- [ ] **Step 5: Commit**

```bash
git add src/mission/mission/stub_action_servers.py src/mission/test/test_stub_action_servers.py
git commit -m "Add stub NavigateToPose/DockRobot action servers"
```

---

### Task 6: CallAction behaviour (BT ↔ ROS2 action bridge)

**Files:**
- Modify: `src/mission/mission/bt_behaviours.py`
- Modify: `src/mission/test/test_bt_behaviours.py`

**Interfaces:**
- Consumes: `mission.stub_action_servers.StubActionServers` (Task 5, as the test target).
- Produces: `mission.bt_behaviours.CallAction(name, node, action_type, action_name, goal_builder: Callable[[], Goal], on_success: Callable[[], None] | None = None)`, a `py_trees.behaviour.Behaviour`. `goal_builder` is called once per `initialise()` to build the goal message; `on_success` (if given) is called once when the action reports `STATUS_SUCCEEDED`, before returning `SUCCESS` — this is how Task 7 updates `current_position` bookkeeping.

- [ ] **Step 1: Write the failing test**

Append to `src/mission/test/test_bt_behaviours.py`:
```python
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.executors import SingleThreadedExecutor

from mission.bt_behaviours import CallAction
from mission.stub_action_servers import StubActionServers


def test_call_action_reaches_success_and_fires_on_success_callback():
    rclpy.init()
    try:
        server = StubActionServers(delay_sec=0.05)
        client_node = rclpy.create_node("test_bt_client")
        executor = SingleThreadedExecutor()
        executor.add_node(server)
        executor.add_node(client_node)

        fired = {"value": False}
        node = CallAction(
            "navigate",
            node=client_node,
            action_type=NavigateToPose,
            action_name="navigate_to_pose",
            goal_builder=NavigateToPose.Goal,
            on_success=lambda: fired.__setitem__("value", True),
        )
        node.client.wait_for_server(timeout_sec=5.0)
        node.initialise()

        status = py_trees.common.Status.RUNNING
        deadline = time.monotonic() + 5.0
        while status == py_trees.common.Status.RUNNING and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            status = node.update()

        assert status == py_trees.common.Status.SUCCESS
        assert fired["value"] is True
    finally:
        rclpy.shutdown()
```

Add `import time` to the top of `test_bt_behaviours.py` alongside the existing `import py_trees` if not already present.

- [ ] **Step 2: Run the test to verify it fails**

```bash
python3 -m pytest src/mission/test/test_bt_behaviours.py -v -k call_action
```
Expected: FAIL with `ImportError: cannot import name 'CallAction'`.

- [ ] **Step 3: Add `CallAction` to `mission/bt_behaviours.py`**

```python
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient


class CallAction(py_trees.behaviour.Behaviour):
    """Sends a ROS 2 action goal on initialise(); RUNNING until a result
    arrives, then SUCCESS/FAILURE based on it. `on_success` fires once,
    exactly when the result is SUCCEEDED, before returning SUCCESS.
    """

    def __init__(self, name, node, action_type, action_name, goal_builder, on_success=None):
        super().__init__(name)
        self.node = node
        self.goal_builder = goal_builder
        self.on_success = on_success
        self.client = ActionClient(node, action_type, action_name)
        self._send_future = None
        self._result_future = None

    def initialise(self):
        self._result_future = None
        self._send_future = self.client.send_goal_async(self.goal_builder())

    def update(self):
        if self._send_future is not None:
            if not self._send_future.done():
                return py_trees.common.Status.RUNNING
            goal_handle = self._send_future.result()
            self._send_future = None
            if not goal_handle.accepted:
                return py_trees.common.Status.FAILURE
            self._result_future = goal_handle.get_result_async()

        if self._result_future is not None:
            if not self._result_future.done():
                return py_trees.common.Status.RUNNING
            result = self._result_future.result()
            self._result_future = None
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                if self.on_success is not None:
                    self.on_success()
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
python3 -m pytest src/mission/test/test_bt_behaviours.py -v
```
Expected: all tests PASS (6 from before + this new one).

- [ ] **Step 5: Commit**

```bash
git add src/mission/mission/bt_behaviours.py src/mission/test/test_bt_behaviours.py
git commit -m "Add CallAction behaviour bridging py_trees to ROS2 actions"
```

---

### Task 7: Mission tree assembly

**Files:**
- Create: `src/mission/mission/mission_tree.py`
- Create: `src/mission/test/test_mission_tree.py`

**Interfaces:**
- Consumes: `mission.docks.Dock`/`load_docks` (Task 2), `mission.saga_client.SagaApi` (Task 3), `mission.bt_behaviours.{WaitUntil, BlackboardFlagTrue, SetBlackboardFlag, CallAction}` (Tasks 4+6).
- Produces: `mission.mission_tree.build_mission_tree(node, saga_client, docks) -> py_trees.behaviour.Behaviour` — the root `Selector("Mission or safe stop")`, ready to be wrapped in a `py_trees.trees.BehaviourTree` and ticked by Task 8's node. Blackboard keys used (module-level constants, importable for tests): `BB_ROUTE_ID`, `BB_SOURCE`, `BB_DESTINATION`, `BB_HOLDS_PACKAGE`, `BB_PACKAGE_DELIVERED`, `BB_CURRENT_POSITION`.

- [ ] **Step 1: Write the failing structural test**

This test builds the real tree against a **fake** `SagaApi` (no network) and a **fake** ROS node using the real stub action servers from Task 5, and ticks it manually to verify the tree actually drives through one full route cycle.

`src/mission/test/test_mission_tree.py`:
```python
import time

import py_trees
import rclpy
from rclpy.executors import SingleThreadedExecutor

from mission.docks import Dock
from mission.mission_tree import (
    BB_HOLDS_PACKAGE,
    BB_PACKAGE_DELIVERED,
    BB_ROUTE_ID,
    build_mission_tree,
)
from mission.stub_action_servers import StubActionServers


class _FakeSagaApi:
    """Serves one route, then a different one, like the real server does
    once a delivery completes."""

    def __init__(self):
        self._routes = [
            {"route_id": 1, "source": {"letter": "A", "tag_id": 0},
             "destination": {"letter": "B", "tag_id": 1}, "status": "loading_pending"},
            {"route_id": 2, "source": {"letter": "C", "tag_id": 2},
             "destination": {"letter": "D", "tag_id": 3}, "status": "loading_pending"},
        ]
        self.route_index = 0

    def get_route(self):
        return self._routes[self.route_index]

    def get_status(self):
        return {"status": self._routes[self.route_index]["status"]}

    def advance_to_next_route(self):
        self.route_index = min(self.route_index + 1, len(self._routes) - 1)


def _docks():
    return {
        letter: Dock(letter=letter, tag_id=i,
                     dock_pose={"x": 0.0, "y": 0.0, "theta": 0.0},
                     staging_pose={"x": 0.0, "y": 0.0, "theta": 0.0})
        for i, letter in enumerate("ABCD")
    }


def test_mission_tree_completes_a_route_cycle():
    rclpy.init()
    try:
        action_server = StubActionServers(delay_sec=0.05)
        client_node = rclpy.create_node("test_mission_tree")
        executor = SingleThreadedExecutor()
        executor.add_node(action_server)
        executor.add_node(client_node)

        saga = _FakeSagaApi()
        root = build_mission_tree(client_node, saga, _docks())
        tree = py_trees.trees.BehaviourTree(root)
        tree.setup(node=client_node, timeout=5.0)

        observer = py_trees.blackboard.Client(name="observer")
        observer.register_key(key=BB_ROUTE_ID, access=py_trees.common.Access.READ)
        observer.register_key(key=BB_PACKAGE_DELIVERED, access=py_trees.common.Access.READ)

        # After loading completes, the fake server advances to the next route
        # — simulate the organizer pressing the admin button.
        deadline = time.monotonic() + 15.0
        route_ids_seen = set()
        while time.monotonic() < deadline and len(route_ids_seen) < 2:
            executor.spin_once(timeout_sec=0.05)
            tree.tick()
            if observer.exists(BB_ROUTE_ID):
                route_ids_seen.add(observer.get(BB_ROUTE_ID))
            if observer.exists(BB_PACKAGE_DELIVERED) and observer.get(BB_PACKAGE_DELIVERED):
                saga.advance_to_next_route()
            time.sleep(0.05)

        assert route_ids_seen == {1, 2}, f"expected to see both routes, saw {route_ids_seen}"
    finally:
        rclpy.shutdown()
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
python3 -m pytest src/mission/test/test_mission_tree.py -v
```
Expected: FAIL with `ModuleNotFoundError: No module named 'mission.mission_tree'`.

- [ ] **Step 3: Write `mission/mission_tree.py`**

```python
import py_trees

from mission.bt_behaviours import BlackboardFlagTrue, CallAction, SetBlackboardFlag, WaitUntil
from nav2_msgs.action import DockRobot, NavigateToPose

BB_ROUTE_ID = "route_id"
BB_SOURCE = "source_dock"
BB_DESTINATION = "destination_dock"
BB_HOLDS_PACKAGE = "holds_package"
BB_PACKAGE_DELIVERED = "package_delivered"
# Mission's own stand-in for real localization until that owner replaces it:
# "dock:<letter>" | "predock:<letter>" | unset.
BB_CURRENT_POSITION = "current_position"


class HasRoute(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=BB_ROUTE_ID, access=py_trees.common.Access.READ)

    def update(self):
        exists = self.blackboard.exists(BB_ROUTE_ID)
        return py_trees.common.Status.SUCCESS if exists else py_trees.common.Status.FAILURE


class GetRouteFromSaga(py_trees.behaviour.Behaviour):
    def __init__(self, name, saga_client):
        super().__init__(name)
        self.saga_client = saga_client
        self.blackboard = self.attach_blackboard_client(name=name)
        for key in (BB_ROUTE_ID, BB_SOURCE, BB_DESTINATION):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def update(self):
        try:
            route = self.saga_client.get_route()
        except Exception as error:  # requests.RequestException in production
            self.logger.warning(f"Saga AI unreachable: {error}")
            return py_trees.common.Status.FAILURE
        if route is None:
            return py_trees.common.Status.FAILURE
        self.blackboard.set(BB_ROUTE_ID, route["route_id"])
        self.blackboard.set(BB_SOURCE, route["source"]["letter"])
        self.blackboard.set(BB_DESTINATION, route["destination"]["letter"])
        return py_trees.common.Status.SUCCESS


class ClearRoute(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        for key in (BB_ROUTE_ID, BB_HOLDS_PACKAGE, BB_PACKAGE_DELIVERED):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def update(self):
        if self.blackboard.exists(BB_ROUTE_ID):
            self.blackboard.unset(BB_ROUTE_ID)
        self.blackboard.set(BB_HOLDS_PACKAGE, False)
        self.blackboard.set(BB_PACKAGE_DELIVERED, False)
        return py_trees.common.Status.SUCCESS


class ReportRouteDone(py_trees.behaviour.Behaviour):
    """Saga AI has no report endpoint (truck is read-only) — this just logs."""

    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=BB_ROUTE_ID, access=py_trees.common.Access.READ)

    def update(self):
        route_id = self.blackboard.get(BB_ROUTE_ID) if self.blackboard.exists(BB_ROUTE_ID) else None
        self.logger.info(f"Route {route_id} complete")
        return py_trees.common.Status.SUCCESS


def _at_position(name, dock_key, prefix):
    class _AtPosition(py_trees.behaviour.Behaviour):
        def __init__(self):
            super().__init__(name)
            self.blackboard = self.attach_blackboard_client(name=name)
            self.blackboard.register_key(key=dock_key, access=py_trees.common.Access.READ)
            self.blackboard.register_key(key=BB_CURRENT_POSITION, access=py_trees.common.Access.READ)

        def update(self):
            dock = self.blackboard.get(dock_key)
            current = self.blackboard.get(BB_CURRENT_POSITION) \
                if self.blackboard.exists(BB_CURRENT_POSITION) else None
            return py_trees.common.Status.SUCCESS if current == f"{prefix}:{dock}" \
                else py_trees.common.Status.FAILURE

    return _AtPosition()


def build_goto_dock_subtree(node, dock_key, docks):
    """dock_key is BB_SOURCE or BB_DESTINATION — resolved at tick time, so the
    same subtree definition serves both the Load and Deliver branches."""

    position_writer = py_trees.blackboard.Client(name=f"position-writer-{dock_key}")
    position_writer.register_key(key=dock_key, access=py_trees.common.Access.READ)
    position_writer.register_key(key=BB_CURRENT_POSITION, access=py_trees.common.Access.WRITE)

    def mark_predock():
        position_writer.set(BB_CURRENT_POSITION, f"predock:{position_writer.get(dock_key)}")

    def mark_dock():
        position_writer.set(BB_CURRENT_POSITION, f"dock:{position_writer.get(dock_key)}")

    at_dock = _at_position("At dock?", dock_key, "dock")
    at_predock = _at_position("At pre-dock?", dock_key, "predock")

    def navigate_goal():
        dock = docks[position_writer.get(dock_key)]
        goal = NavigateToPose.Goal()
        goal.pose.pose.position.x = dock.staging_pose["x"]
        goal.pose.pose.position.y = dock.staging_pose["y"]
        return goal

    def dock_goal():
        return DockRobot.Goal()

    navigate = CallAction(
        "NavigateToPose(staging)", node=node, action_type=NavigateToPose,
        action_name="navigate_to_pose", goal_builder=navigate_goal, on_success=mark_predock,
    )
    dock_is_free = WaitUntil(
        "DockIsFree?", condition=lambda: True, timeout_sec=60.0, poll_sec=1.0,
    )  # stubbed free-check — real multi-robot occupancy comes from perception later
    dock_action = CallAction(
        "DockRobot", node=node, action_type=DockRobot,
        action_name="dock_robot", goal_builder=dock_goal, on_success=mark_dock,
    )

    move_and_dock = py_trees.composites.Sequence(name="Move and dock", memory=True, children=[
        py_trees.composites.Selector(name="At pre-dock?", memory=False, children=[at_predock, navigate]),
        dock_is_free,
        dock_action,
    ])

    return py_trees.composites.Selector(name="GoToDock", memory=False, children=[at_dock, move_and_dock])


def build_mission_tree(node, saga_client, docks):
    vehicle_status_ok = py_trees.behaviours.Success(name="VehicleStatusOK (stub)")
    localize = py_trees.behaviours.Success(name="Localize (stub)")

    have_route = py_trees.composites.Selector(name="Have route", memory=False, children=[
        HasRoute("HasRoute?"),
        GetRouteFromSaga("GetRouteFromSaga", saga_client),
    ])

    load = py_trees.composites.Selector(name="Load", memory=False, children=[
        BlackboardFlagTrue("HoldsPackage?", key=BB_HOLDS_PACKAGE),
        py_trees.composites.Sequence(name="Load at source", memory=True, children=[
            build_goto_dock_subtree(node, BB_SOURCE, docks),
            WaitUntil("SagaLoaded?", condition=lambda: saga_client.get_status()["status"] == "loading_complete",
                      timeout_sec=300.0, poll_sec=1.0),
            SetBlackboardFlag("Set HoldsPackage", key=BB_HOLDS_PACKAGE, value=True),
        ]),
    ])

    deliver = py_trees.composites.Selector(name="Deliver", memory=False, children=[
        BlackboardFlagTrue("PackageDelivered?", key=BB_PACKAGE_DELIVERED),
        py_trees.composites.Sequence(name="Unload at destination", memory=True, children=[
            build_goto_dock_subtree(node, BB_DESTINATION, docks),
            WaitUntil("SagaUnloaded?", condition=lambda: saga_client.get_status()["status"] == "unloading_complete",
                      timeout_sec=300.0, poll_sec=1.0),
            SetBlackboardFlag("Set PackageDelivered", key=BB_PACKAGE_DELIVERED, value=True),
        ]),
    ])

    route = py_trees.composites.Sequence(name="Route", memory=True, children=[
        have_route, load, deliver, ReportRouteDone("ReportRouteDone"), ClearRoute("ClearRoute"),
    ])

    mission = py_trees.composites.ReactiveSequence(name="Mission", children=[
        vehicle_status_ok, localize, route,
    ])

    safe_stop = py_trees.composites.Sequence(name="Safe stop", memory=True, children=[
        py_trees.behaviours.Success(name="StopRobot (stub)"),
        py_trees.behaviours.Success(name="ReportFaultToSaga (log only)"),
    ])

    return py_trees.composites.Selector(name="Mission or safe stop", memory=False, children=[mission, safe_stop])
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
python3 -m pytest src/mission/test/test_mission_tree.py -v
```
Expected: PASS within the 15s deadline, having seen both route IDs `{1, 2}`.

If it fails on a `py_trees` API mismatch (e.g. `py_trees.behaviours.Success` constructor, `Selector`/`Sequence`'s exact keyword names, or blackboard client method names), this is expected risk flagged in the plan's Tech Stack note — check `python3 -c "import py_trees; help(py_trees.composites.Selector)"` (and similarly for `Sequence`, `py_trees.behaviours.Success`, `py_trees.blackboard.Client`) and adjust the constructor calls above to match the installed version, then re-run.

- [ ] **Step 5: Commit**

```bash
git add src/mission/mission/mission_tree.py src/mission/test/test_mission_tree.py
git commit -m "Assemble the mission behavior tree"
```

---

### Task 8: Mission node, launch file, and end-to-end verification

**Files:**
- Create: `src/mission/mission/mission_node.py`
- Create: `src/mission/launch/mission.launch.py` (overwrite the Task 1 placeholder)

**Interfaces:**
- Consumes: `mission.mission_tree.build_mission_tree` (Task 7), `mission.saga_client.SagaApi` (Task 3), `mission.docks.load_docks` (Task 2).
- Produces: a runnable `ros2 launch mission mission.launch.py` bringing up the stub action servers and the ticking mission tree together.

- [ ] **Step 1: Write `mission/mission_node.py`**

```python
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from mission.docks import load_docks
from mission.mission_tree import build_mission_tree
from mission.saga_client import SagaApi

import py_trees


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.declare_parameter('saga_url', 'http://localhost:8000')
        self.declare_parameter('saga_token', '')
        self.declare_parameter('docks_file', get_package_share_directory('mission') + '/config/docks.yaml')
        self.declare_parameter('tick_hz', 1.0)

        saga_client = SagaApi(
            self.get_parameter('saga_url').value,
            self.get_parameter('saga_token').value,
        )
        docks = load_docks(self.get_parameter('docks_file').value)

        root = build_mission_tree(self, saga_client, docks)
        self.tree = py_trees.trees.BehaviourTree(root)
        self.tree.setup(node=self, timeout=15.0)

        tick_hz = self.get_parameter('tick_hz').value
        self.create_timer(1.0 / tick_hz, self.tree.tick)


def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Write `launch/mission.launch.py`**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('saga_url', default_value='http://localhost:8000'),
        DeclareLaunchArgument('saga_token', default_value=''),
        Node(
            package='mission',
            executable='stub_action_servers',
            name='stub_action_servers',
            output='screen',
        ),
        Node(
            package='mission',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[{
                'saga_url': LaunchConfiguration('saga_url'),
                'saga_token': LaunchConfiguration('saga_token'),
            }],
        ),
    ])
```

- [ ] **Step 3: Rebuild**

```bash
cd /home/albin/modysseus
colcon build --packages-select mission
source install/setup.bash
```
Expected: build succeeds.

- [ ] **Step 4: Run the full test suite one more time**

```bash
python3 -m pytest src/mission/test/ -v
```
Expected: every test from Tasks 1-7 still PASSES.

- [ ] **Step 5: End-to-end verification against the real local Saga AI mock server**

Terminal 1 — start the mock server fresh and start an event:
```bash
cd /home/albin/ai-powered-autonomous-challenge-server
export PATH="$HOME/.local/bin:$PATH"
uv run python -m app
```
Open `http://localhost:8000/admin` in a browser, log in with the credentials printed earlier (or re-run `uv run python -m app.credentials generate` if lost), and press **Start event**. Note truck 1's token from that same admin page (or `credentials.json`).

Terminal 2 — launch mission against it:
```bash
cd /home/albin/modysseus
source /opt/ros/jazzy/setup.bash
source /home/albin/einride-mini-truck/install/setup.bash
source install/setup.bash
ros2 launch mission mission.launch.py saga_token:=<truck 1's token>
```

Expected, within the first ~30 seconds:
- `mission_node` logs show a route being picked up (`GetRouteFromSaga` succeeding, non-trivial `route_id`).
- Stub action server logs show `Stub NavigateToPose: driving...` and `Stub DockRobot: docking...` firing for the source dock.
- On the `/admin` page, truck 1's card shows it "arrived" is not organizer-driven here (organizer still presses the Saga buttons manually in this manual test — mission only reacts to status, it doesn't fake real arrival detection). Press **Loading at X complete** on the admin page after mission's logs show it reached the source dock; mission should then report `Route N complete` in its logs once it also finishes the destination leg, and pick up a new route automatically (different `route_id` in the next `GetRouteFromSaga` log line).
- Repeat for 2-3 routes to confirm the loop really continues (this is the spec's stated success criterion).

- [ ] **Step 6: Commit**

```bash
git add src/mission/mission/mission_node.py src/mission/launch/mission.launch.py
git commit -m "Add mission node, launch file, and end-to-end verification"
```

---

## Self-Review Notes

- **Spec coverage:** workspace/overlay layout (Task 1), dock file (Task 2), Saga AI client — corrected to read-only per the full `docs/saga-ai.md` (Task 3), stub action servers (Task 5), mission BT incl. `WaitUntil`/`GoToDock`/stubbed `VehicleStatusOK`/`Localize` (Tasks 4, 6, 7), launch + end-to-end success criterion (Task 8). `Mapping` is explicitly out of scope per the spec, no task needed.
- **Corrected from spec:** `ReportRouteDone`/`ReportFaultToSaga` do not call any Saga API (none exists) — implemented as log-only behaviours in Task 7. This was caught by reading `docs/saga-ai.md` in full while writing this plan; flagged here since it changes `SagaApi`'s public surface from what the spec implied.
- **Verified against the actual machine, not assumed:** `nav2_msgs/action/DockRobot` (not `opennav_docking_msgs`) via `ros2 interface list`; `ros-jazzy-py-trees`/`ros-jazzy-py-trees-ros` exist via `apt-cache policy` (not yet installed — Task 1 Step 1 is a required manual sudo step).
- **Known risk:** exact `py_trees` 2.5 constructor signatures (`Selector`/`Sequence` keyword names, `py_trees.behaviours.Success`, blackboard client methods) are used to the best of available knowledge but not executed/verified on this machine yet (package only becomes installable after the Task 1 sudo step). Task 7 Step 4 calls this out explicitly with the command to check real signatures if a test fails on an API mismatch — this is a normal TDD red/green loop, not a gap in the plan.
