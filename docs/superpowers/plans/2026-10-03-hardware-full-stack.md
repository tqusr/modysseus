# Hardware Full Stack Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `ros2 launch mission full_stack_hardware.launch.py saga_token:=...` starts the full stack on the real robot, with each layer switchable.

**Architecture:** Make the five sim-hardcoded launches take `use_sim_time` (default true) as a typed bool, extract the hardware odometry nodes into one shared include, then write the hardware full-stack launch as the sim one minus Gazebo. All tests are in-process launch-description tests; nothing is launched.

**Tech Stack:** ROS 2 Jazzy launch / launch_ros, pytest, `perception/test/launch_helpers.py`.

**Spec:** `docs/superpowers/specs/2026-10-03-hardware-full-stack-design.md`

## Global Constraints

- Defaults reproduce simulation exactly: `use_sim_time` default `'true'`, `map` default `arena`; `full_stack.launch.py` and `mapping_mission.launch.py` are not edited.
- Node `use_sim_time` parameters are Python `bool`s (`_as_bool(text)`: `text.strip().lower() in ('true', '1')`), never the argument's text. Included launch arguments are strings (`'false'`).
- No new package and no `package.xml` change. Cross-package includes use `FindPackageShare` (perception must not be depended on by localization: cycle).
- `mapping_mission_hardware.launch.py` behaviour must not change.
- Do not launch Gazebo, `ros2 launch`, or any node. Unit tests and static checks only; say plainly that on-robot verification was not done.
- Conventional Commits, no Co-Authored-By or AI trailer.
- Build and test (all four dirs exist except `src/navigation/test`, created in Task 2):
  `cd /home/albin/modysseus && source /opt/ros/jazzy/setup.bash && colcon build --packages-select perception localization navigation mission && source install/setup.bash && python3 -m pytest -q src/perception/test src/localization/test src/navigation/test src/mission/test`
  Baseline at plan time: builds clean (the `mission` build failure a previous agent saw did not reproduce; likely a stale or un-sourced shell, not a code fault) and 248 tests pass.

## Review Focus

- `use_sim_time:=false` given in any spelling (`False`, `0`) must still yield a bool `False` at every node; `true`/`1` yield `True`.
- `map:=arena_real` when the posegraph does not exist: slam_toolbox fails at configure; the watchdog already reports it loudly (Task 4 test pins the path; no new handling).
- Switch `nav:=false` must also drop `oak_points_filter` (only Nav2 reads it), and `docking:=false` must not drop anything else.
- A numeric-looking `saga_token` (e.g. `12345`) must reach `mission_node` as a string, not an int.
- `apriltag:=false` must not require `perception`'s apriltag file (lazy `FindPackageShare`).

---

### Task 1: use_sim_time / map in localize, docking, oak_points_filter

**Files:**
- Modify: `src/perception/test/launch_helpers.py` (`describe_node`)
- Modify: `src/localization/launch/localize.launch.py`, `src/perception/launch/docking.launch.py`, `src/perception/launch/oak_points_filter.launch.py`
- Create: `src/localization/test/test_localize_launch.py`, `src/perception/test/test_sim_time_launches.py`

**Interfaces:**
- Produces: `describe_node(...)['inline_parameters']` (dict of the inline parameter dicts, substitutions performed to text, plain values untouched). Launch args `use_sim_time` (all three) and `map` (localize).

- [ ] **Step 1: Extend `describe_node`** in `launch_helpers.py`: inside the `isinstance(param, dict)` branch also fill `inline[text(key)] = perform_substitutions(context, normalize_to_list_of_substitutions(value)) if isinstance(value, Substitution) else value` (import `Substitution` from `launch.substitutions`), start with `inline = {}`, and add `'inline_parameters': inline` to the returned dict.

- [ ] **Step 2: Write failing tests.** `src/perception/test/test_sim_time_launches.py`:

```python
"""use_sim_time reaches the docking and oak_points_filter nodes as a bool."""
import os

import pytest

from launch_helpers import (describe_node, expand, load_launch_module,
                            make_context, nodes_of)

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _nodes(launch_file, **args):
    description = load_launch_module(
        os.path.join(PKG, 'launch', launch_file)).generate_launch_description()
    context = make_context(description, **args)
    return [describe_node(n, context)
            for n in nodes_of(expand(description.entities, context))]


@pytest.mark.parametrize('launch_file', ['docking.launch.py',
                                         'oak_points_filter.launch.py'])
@pytest.mark.parametrize('args,expected', [
    ({}, True), ({'use_sim_time': 'true'}, True),
    ({'use_sim_time': 'false'}, False), ({'use_sim_time': 'False'}, False),
    ({'use_sim_time': '0'}, False)])
def test_use_sim_time_is_a_typed_bool(launch_file, args, expected):
    nodes = _nodes(launch_file, **args)
    assert nodes
    for node in nodes:
        assert node['inline_parameters']['use_sim_time'] is expected
```

`src/localization/test/test_localize_launch.py` (load helpers by path as `test_mapping_mission_hardware_launch.py` does: copy its `HELPERS_PATH` / `_load_helpers` block):

```python
"""localize.launch.py: sim defaults unchanged, hardware arguments reach nodes."""
import os

import pytest

# ... HELPERS_PATH / _load_helpers() / `h = _load_helpers()` as in
# test_mapping_mission_hardware_launch.py ...

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_PATH = os.path.join(PKG, 'launch', 'localize.launch.py')


def _nodes(**args):
    description = h.load_launch_module(LAUNCH_PATH).generate_launch_description()
    context = h.make_context(description, **args)
    return {n['name']: n for n in (
        h.describe_node(x, context)
        for x in h.nodes_of(h.expand(description.entities, context)))}


def test_defaults_are_simulation_on_the_arena_map():
    nodes = _nodes()
    assert sorted(nodes) == ['scan_filter', 'slam_toolbox']
    assert nodes['scan_filter']['inline_parameters']['use_sim_time'] is True
    slam = nodes['slam_toolbox']['inline_parameters']
    assert slam['use_sim_time'] is True
    assert slam['map_file_name'].endswith(os.path.join('maps', 'arena'))


def test_map_argument_selects_the_posegraph_basename():
    slam = _nodes(map='arena_real')['slam_toolbox']['inline_parameters']
    assert slam['map_file_name'].endswith(os.path.join('maps', 'arena_real'))


@pytest.mark.parametrize('spelling', ['false', 'False', '0'])
def test_use_sim_time_false_reaches_both_nodes_as_bool(spelling):
    for node in _nodes(use_sim_time=spelling).values():
        assert node['inline_parameters']['use_sim_time'] is False
```

- [ ] **Step 3: Run, expect FAIL** (no `use_sim_time` argument; KeyError/assert): `python3 -m pytest -q src/perception/test/test_sim_time_launches.py src/localization/test/test_localize_launch.py`

- [ ] **Step 4: Implement.** In each file add `from launch.actions import DeclareLaunchArgument, OpaqueFunction`, `from launch.substitutions import LaunchConfiguration`, and
```python
def _as_bool(text):
    """Launch arguments are text; node parameters must be typed."""
    return text.strip().lower() in ('true', '1')
```
Turn the current body of `generate_launch_description` into `_launch_nodes(context)` that starts with `use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))`, replaces every `True` for `use_sim_time` with it, and **returns the list** that was in `LaunchDescription([...])`. Then:
```python
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_launch_nodes),
    ])
```
Per file: `localize`: also `DeclareLaunchArgument('map', default_value='arena', description='Pose graph basename in localization/maps')`, and `map_file_name = os.path.join(share, 'maps', LaunchConfiguration('map').perform(context))` (keep the `change_state`/watchdog/report code inside `_launch_nodes` unchanged). `docking`: `parameters=[config, {'use_sim_time': use_sim_time}]`. `oak_points_filter`: both nodes `parameters=[config, {'use_sim_time': use_sim_time}]` (the YAML sets `true`; the later dict wins). Keep the module docstrings; add one sentence each saying the argument exists for hardware.

- [ ] **Step 5: Run the same tests, expect PASS**, then the full suite command from Global Constraints.

- [ ] **Step 6: Commit**
```bash
git add src/perception src/localization/launch/localize.launch.py src/localization/test/test_localize_launch.py
git commit -m "feat(launch): make localize, docking and oak_points_filter use_sim_time configurable"
```

---

### Task 2: use_sim_time in navigation and mission

**Files:**
- Modify: `src/navigation/launch/navigation.launch.py`, `src/mission/launch/mission.launch.py`
- Create: `src/navigation/test/test_navigation_launch.py`, `src/mission/test/test_mission_launch.py`

**Interfaces:**
- Consumes: `launch_helpers` and `describe_node(...)['inline_parameters']` (Task 1), loaded by path from `src/perception/test/launch_helpers.py`.
- Produces: `use_sim_time` argument on both launches; `mission_node` gets `saga_url`/`saga_token` as `str`.

- [ ] **Step 1: Failing tests.** Both test files use the same `_load_helpers()` by-path block as Task 1 (path `../../perception/test/launch_helpers.py`). `test_navigation_launch.py`:

```python
import os

import pytest
# ... _load_helpers block, `h = _load_helpers()` ...

LAUNCH_PATH = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'launch', 'navigation.launch.py')


def _nodes(**args):
    description = h.load_launch_module(LAUNCH_PATH).generate_launch_description()
    context = h.make_context(description, **args)
    return [h.describe_node(n, context)
            for n in h.nodes_of(h.expand(description.entities, context))]


@pytest.mark.parametrize('args,expected', [
    ({}, True), ({'use_sim_time': 'false'}, False), ({'use_sim_time': '0'}, False)])
def test_every_nav2_node_gets_a_bool_use_sim_time(args, expected):
    nodes = _nodes(**args)
    assert len(nodes) == 7  # six servers + lifecycle manager
    assert all(n['inline_parameters']['use_sim_time'] is expected for n in nodes)
```
`test_mission_launch.py` (same shape, `mission/launch/mission.launch.py`, helpers at `../../perception/test/launch_helpers.py`):
```python
def test_defaults_are_sim_time_with_string_saga_arguments():
    (node,) = _nodes(saga_token='12345')
    p = node['inline_parameters']
    assert p['use_sim_time'] is True
    assert p['saga_token'] == '12345' and isinstance(p['saga_token'], str)
    assert p['saga_url'] == 'http://localhost:8000'


def test_use_sim_time_false_is_a_bool():
    (node,) = _nodes(use_sim_time='false')
    assert node['inline_parameters']['use_sim_time'] is False
```
(`navigation.launch.py` also returns a `SetEnvironmentVariable`; `nodes_of` ignores it.)

- [ ] **Step 2: Run, expect FAIL:** `python3 -m pytest -q src/navigation/test src/mission/test/test_mission_launch.py`

- [ ] **Step 3: Implement** with the Task 1 pattern (`_as_bool`, `_launch_nodes(context)`, `DeclareLaunchArgument('use_sim_time', default_value='true')`, `OpaqueFunction`). Navigation: `common_args['parameters'] = [config, {'use_sim_time': use_sim_time}]` and the lifecycle manager's inline dict; keep `SetEnvironmentVariable` first in the returned list. Mission: keep the `saga_url`/`saga_token` declarations, and in `_launch_nodes` use `LaunchConfiguration('saga_url').perform(context)` / `...('saga_token').perform(context)` for the node parameters; keep the existing comment on why mission_node needs sim time, adding that hardware passes `false`.

- [ ] **Step 4: Run, expect PASS**, then the full suite command.

- [ ] **Step 5: Commit**
```bash
git add src/navigation src/mission/launch/mission.launch.py src/mission/test/test_mission_launch.py
git commit -m "feat(launch): make navigation and mission use_sim_time configurable"
```

---

### Task 3: Extract hardware_odometry.launch.py

**Files:**
- Create: `src/localization/launch/hardware_odometry.launch.py`, `src/localization/test/test_hardware_odometry_launch.py`
- Modify: `src/localization/launch/mapping_mission_hardware.launch.py`, `src/localization/test/test_mapping_mission_hardware_launch.py` (last test only)

**Interfaces:**
- Produces: `hardware_odometry.launch.py`, no arguments; starts `wheel_odometry_node` (params: `hardware.yaml`, `use_sim_time` False, `publish_tf` False) then includes `localization.launch.py` with `{'use_sim_time': 'false', 'publish_tf': 'true'}`.

- [ ] **Step 1: Failing test** `test_hardware_odometry_launch.py` (same by-path helpers block):
```python
def test_wheel_odometry_then_fusion_include():
    description = h.load_launch_module(LAUNCH_PATH).generate_launch_description()
    context = h.make_context(description)
    entities = h.expand(description.entities, context)
    (node,) = [h.describe_node(n, context) for n in h.nodes_of(entities)]
    assert node['name'] == 'wheel_odometry_node'
    assert node['inline_parameters'] == {'use_sim_time': False, 'publish_tf': False}
    assert node['parameter_files'][0].endswith('hardware.yaml')
    (include,) = [e for e in entities if isinstance(e, IncludeLaunchDescription)]
    info = h.describe_include(include, context)
    assert info['file'].endswith('localization.launch.py')
    assert info['args'] == {'use_sim_time': 'false', 'publish_tf': 'true'}
    assert entities.index(include) > entities.index(
        h.nodes_of(entities)[0])  # odometry node first, as before
```
(import `IncludeLaunchDescription` from `launch.actions`.) Expected FAIL: file missing.

- [ ] **Step 2: Implement.** Move the `wheel_odometry` Node and `fusion` include, with their comments and the `USE_SIM_TIME`/`USE_SIM_TIME_ARG` note, verbatim into `hardware_odometry.launch.py` (`generate_launch_description` returns `LaunchDescription([wheel_odometry, fusion])`). In `mapping_mission_hardware.launch.py` replace the two with one `hardware_odometry = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hardware_odometry.launch.py')))` at the same position in the returned list (`hardware_odometry, mapping, ...`); keep `USE_SIM_TIME_ARG` for the mapping/apriltag includes, drop now-unused names.

- [ ] **Step 3: Update the one existing test** `test_hardware_launch_still_runs_on_wall_clock_without_the_camera_nodes`: it must find `wheel_odometry_node` one include deeper, so evaluate the hardware_odometry include with `h.describe_include`, load that file, and assert the same two facts (`name == 'wheel_odometry_node'`, `use_sim_time is False`) on its node; also assert the launch's own top-level nodes are now empty. All other tests in that file stay untouched and must pass.

- [ ] **Step 4: Run** `python3 -m pytest -q src/localization/test` and the full suite. Expected: all PASS.

- [ ] **Step 5: Commit**
```bash
git add src/localization
git commit -m "refactor(localization): extract hardware odometry into a shared launch"
```

---

### Task 4: full_stack_hardware.launch.py

**Files:**
- Create: `src/mission/launch/full_stack_hardware.launch.py`, `src/mission/test/test_full_stack_hardware_launch.py`

**Interfaces:**
- Consumes: all launches above; `hardware_odometry.launch.py` (no args); `use_sim_time`/`map` args (Tasks 1-2); `perception/apriltag.launch.py` args `use_sim_time`, `image_topic`, `rectify`.
- Produces: args `map` (`arena_real`), `saga_url` (`http://localhost:8000`), `saga_token` (`''`), `apriltag`/`nav`/`docking`/`mission` (`true`).

- [ ] **Step 1: Failing tests** `test_full_stack_hardware_launch.py` (by-path helpers, path `../launch/full_stack_hardware.launch.py`):

```python
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node


def _is_false(value):
    return value is False or (isinstance(value, str) and value.lower() in ('false', '0'))


def _walk(entities, context):
    """Yield ('node', info) and ('include', file) for everything that would run."""
    for e in h.expand(entities, context):
        if isinstance(e, TimerAction):
            yield from _walk(e.actions, context)
        elif isinstance(e, IncludeLaunchDescription):
            if e.condition is not None and not e.condition.evaluate(context):
                continue
            info = h.describe_include(e, context)
            yield 'include', info['file']
            sub = h.load_launch_module(info['file']).generate_launch_description()
            yield from _walk(sub.entities, h.make_context(sub, **info['args']))
        elif isinstance(e, Node):
            yield 'node', h.describe_node(e, context)


def _run(**args):
    description = h.load_launch_module(LAUNCH_PATH).generate_launch_description()
    context = h.make_context(description, **args)
    return list(_walk(description.entities, context))


def _includes(**args):
    return [os.path.basename(x) for kind, x in _run(**args) if kind == 'include']


def _nodes(**args):
    return [x for kind, x in _run(saga_token='t', **args) if kind == 'node']


ALL = ['hardware_odometry.launch.py', 'localization.launch.py',
       'apriltag.launch.py', 'oak_points_filter.launch.py', 'localize.launch.py',
       'docking.launch.py', 'navigation.launch.py', 'mission.launch.py']


def test_all_layers_on_by_default_and_no_simulator():
    includes = _includes()
    assert sorted(includes) == sorted(ALL)
    assert 'simulation.launch.py' not in includes


def test_every_node_is_on_wall_clock():
    nodes = _nodes()
    assert len(nodes) >= 12
    for node in nodes:
        # oak filter / apriltag / etc. all set it; a node without it would
        # silently run with its YAML's `true`.
        assert _is_false(node['inline_parameters'].get('use_sim_time')), node['name']


@pytest.mark.parametrize('switch,dropped', [
    ('apriltag', ['apriltag.launch.py']),
    ('nav', ['navigation.launch.py', 'oak_points_filter.launch.py']),
    ('docking', ['docking.launch.py']),
    ('mission', ['mission.launch.py'])])
def test_switch_false_drops_only_its_includes(switch, dropped):
    assert sorted(_includes(**{switch: 'false'})) == sorted(
        x for x in ALL if x not in dropped)


def test_map_and_saga_token_are_forwarded():
    nodes = {n['name']: n for n in _nodes(map='foo', saga_token='12345')}
    assert nodes['slam_toolbox']['inline_parameters']['map_file_name'].endswith(
        os.path.join('maps', 'foo'))
    assert nodes['mission_node']['inline_parameters']['saga_token'] == '12345'


def test_default_map_is_arena_real():
    nodes = {n['name']: n for n in _nodes()}
    assert nodes['slam_toolbox']['inline_parameters']['map_file_name'].endswith(
        os.path.join('maps', 'arena_real'))


def test_apriltag_is_rectified_on_the_oak_raw_image():
    nodes = {n['name']: n for n in _nodes()}
    assert nodes['rectify_node']['remappings']['image'] == '/oak/rgb/image_raw'
```
Expected: FAIL (file missing).

- [ ] **Step 2: Implement** the launch. Module docstring: what it is, the stagger table, "needs the `einride-mini-truck` service already running", initial pose via `/initialpose` (RViz 2D Pose Estimate), link to the spec. Code:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

HARDWARE = {'use_sim_time': 'false'}  # launch arguments are text


def _include(package, launch_file, arguments=None, switch=None):
    """Include `package`'s launch file, resolved when it runs, optionally gated."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=(arguments or {}).items(),
        condition=IfCondition(LaunchConfiguration(switch)) if switch else None)


def generate_launch_description():
    odometry = _include('localization', 'hardware_odometry.launch.py')
    apriltag = _include('perception', 'apriltag.launch.py', {
        **HARDWARE, 'image_topic': '/oak/rgb/image_raw', 'rectify': 'true'},
        switch='apriltag')
    oak_filter = _include('perception', 'oak_points_filter.launch.py', HARDWARE, 'nav')
    localize = _include('localization', 'localize.launch.py',
                        {**HARDWARE, 'map': LaunchConfiguration('map')})
    docking = _include('perception', 'docking.launch.py', HARDWARE, 'docking')
    navigation = _include('navigation', 'navigation.launch.py', HARDWARE, 'nav')
    mission = _include('mission', 'mission.launch.py', {
        **HARDWARE, 'saga_url': LaunchConfiguration('saga_url'),
        'saga_token': LaunchConfiguration('saga_token')}, 'mission')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='arena_real', description=...),
        DeclareLaunchArgument('saga_url', default_value='http://localhost:8000'),
        DeclareLaunchArgument('saga_token', default_value='', description=...),
        *[DeclareLaunchArgument(s, default_value='true', description=...)
          for s in ('apriltag', 'nav', 'docking', 'mission')],
        odometry,
        TimerAction(period=5.0, actions=[apriltag, oak_filter]),
        TimerAction(period=7.0, actions=[localize]),
        TimerAction(period=10.0, actions=[docking]),
        TimerAction(period=18.0, actions=[navigation]),
        TimerAction(period=25.0, actions=[mission]),
    ])
```
Replace each `description=...` with a real one-line description when writing the file. `describe_include` must be able to evaluate `LaunchConfiguration('map')` values: they are performed against the same context, which holds the declared defaults.

- [ ] **Step 3: Run** the new test file, then the full suite command. Expected: all PASS. Also `python3 -m flake8 src/mission/launch src/localization/launch src/perception/launch src/navigation/launch` if flake8 is installed (the packages declare `ament_flake8`); fix what the new code adds.

- [ ] **Step 4: Commit**
```bash
git add src/mission/launch/full_stack_hardware.launch.py src/mission/test/test_full_stack_hardware_launch.py
git commit -m "feat(mission): add full_stack_hardware launch for the real robot"
```

- [ ] **Step 5: Docs.** Call the `code-documenter` agent: add a "Full stack on the robot" section to `docs/running-the-simulation.md`'s neighbour (new `docs/running-on-hardware.md` if that file is sim-only) with the command, the switches, saving `arena_real`, the initial-pose procedure and the staged checklist from the spec. Commit as `docs: describe running the full stack on the robot`.

- [ ] **Step 6: Push**
```bash
git push -u origin hardware-full-stack
```
