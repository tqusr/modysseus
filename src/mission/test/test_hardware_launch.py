"""hardware.launch.py: the robot-side layers (no docking, no mission), on wall-clock time.

The launch files are evaluated in-process through includes and timers (see
perception/test/launch_helpers.py); nothing is started. Needs the four packages
built and sourced.
"""
import importlib.util
import os

import pytest
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_PATH = os.path.join(PKG, 'launch', 'hardware.launch.py')
HELPERS_PATH = os.path.join(
    PKG, '..', 'perception', 'test', 'launch_helpers.py')


def _load_helpers():
    """perception's launch_helpers, loaded by path (no package dependency)."""
    spec = importlib.util.spec_from_file_location('launch_helpers', HELPERS_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


h = _load_helpers()


@pytest.fixture(autouse=True)
def robot_pose_graph_exists(monkeypatch):
    """`arena_real` only exists on the robot; pretend its files do here."""
    real_exists = os.path.exists
    monkeypatch.setattr(os.path, 'exists', lambda path: (
        True if 'arena_real' in str(path) else real_exists(path)))


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
    return [x for kind, x in _run(**args) if kind == 'node']


ALL = ['hardware_odometry.launch.py', 'localization.launch.py',
       'apriltag.launch.py', 'oak_points_filter.launch.py', 'localize.launch.py',
       'navigation.launch.py', 'wheel_drive.launch.py']


def test_all_layers_on_by_default():
    includes = _includes()
    assert sorted(includes) == sorted(ALL)


def test_every_node_is_on_wall_clock():
    nodes = _nodes()
    assert sorted(n['name'] for n in nodes) == [
        'apriltag_node', 'behavior_server', 'bt_navigator', 'collision_monitor',
        'controller_server', 'ekf_filter_node',
        'gyro_bias_node', 'image_gate', 'lifecycle_manager_navigation',
        # the oak filter: its stages are components
        # inside this container (perception's own tests cover them)
        'oak_cloud_container', 'planner_server',
        'rectify_node', 'scan_filter', 'slam_toolbox',
        'velocity_smoother',
        'wheel_drive_node', 'wheel_odometry_node']
    for node in nodes:
        # oak filter / apriltag / etc. all set it; a node without it would
        # silently run with its YAML's `true`.
        assert node['inline_parameters'].get('use_sim_time') is False, node['name']


@pytest.mark.parametrize('switch,dropped', [
    ('apriltag', ['apriltag.launch.py']),
    ('nav', ['navigation.launch.py', 'oak_points_filter.launch.py'])])
def test_switch_false_drops_only_its_includes(switch, dropped):
    assert sorted(_includes(**{switch: 'false'})) == sorted(
        x for x in ALL if x not in dropped)


def test_docking_and_mission_are_not_here():
    # They run from mission.launch.py in a second terminal.
    names = {n['name'] for n in _nodes()}
    assert not names & {'docking_server', 'tag_dock_server', 'mission_node'}
    assert 'docking.launch.py' not in _includes()
    assert 'mission.launch.py' not in _includes()


def test_map_is_forwarded():
    nodes = {n['name']: n for n in _nodes(map='arena_real_alt')}
    assert nodes['slam_toolbox']['inline_parameters']['map_file_name'].endswith(
        os.path.join('maps', 'arena_real_alt'))


def test_map_start_pose_is_forwarded():
    nodes = {n['name']: n for n in _nodes(map_start_pose='1.0, 2.0, 0.5')}
    assert nodes['slam_toolbox']['inline_parameters']['map_start_pose'] == (1.0, 2.0, 0.5)


def test_default_map_is_arena_real():
    nodes = {n['name']: n for n in _nodes()}
    assert nodes['slam_toolbox']['inline_parameters']['map_file_name'].endswith(
        os.path.join('maps', 'arena_real'))


def test_apriltag_is_rectified_on_the_oak_raw_image():
    nodes = {n['name']: n for n in _nodes()}
    assert nodes['rectify_node']['remappings']['image'] == '/oak/rgb/image_raw'


def _nav_include_args(**args):
    return next(i for kind, i in _include_infos(**args)
                if kind == 'include_info' and i['file'].endswith('navigation.launch.py'))['args']


def _include_infos(**args):
    description = h.load_launch_module(LAUNCH_PATH).generate_launch_description()
    context = h.make_context(description, **args)
    for e in h.expand(description.entities, context):
        if isinstance(e, TimerAction):
            for a in h.expand(e.actions, context):
                if isinstance(a, IncludeLaunchDescription):
                    yield 'include_info', h.describe_include(a, context)


def test_wheel_drive_always_starts_and_nav2_turn_rate_matches():
    runs = _run()
    assert any(kind == 'include' and f.endswith('wheel_drive.launch.py')
               for kind, f in runs)
    assert float(_nav_include_args()['wz_max']) == 0.75
