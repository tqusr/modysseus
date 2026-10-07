"""full_stack_hardware.launch.py: the sim stack minus Gazebo, on wall-clock time.

The launch files are evaluated in-process, recursing through includes and
timers (see perception/test/launch_helpers.py); nothing is started. Needs the
four packages built and their install sourced, as the includes are located
through the share directories.
"""
import importlib.util
import os

import pytest
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_PATH = os.path.join(PKG, 'launch', 'full_stack_hardware.launch.py')
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
    return [x for kind, x in _run(**{'saga_token': 't', **args}) if kind == 'node']


ALL = ['hardware_odometry.launch.py', 'localization.launch.py',
       'apriltag.launch.py', 'oak_points_filter.launch.py', 'localize.launch.py',
       'docking.launch.py', 'navigation.launch.py', 'mission.launch.py']


def test_all_layers_on_by_default_and_no_simulator():
    includes = _includes()
    assert sorted(includes) == sorted(ALL)
    assert 'simulation.launch.py' not in includes


def test_every_node_is_on_wall_clock():
    nodes = _nodes()
    assert sorted(n['name'] for n in nodes) == [
        'apriltag_node', 'behavior_server', 'bt_navigator', 'collision_monitor',
        'controller_server', 'docking_server', 'ekf_filter_node',
        'gyro_bias_node', 'image_gate', 'lifecycle_manager_navigation', 'mission_node',
        # the oak filter in source:=depth mode: its stages are components
        # inside this container (perception's own tests cover them)
        'oak_cloud_container', 'planner_server',
        'rectify_node', 'scan_filter', 'slam_toolbox', 'tag_dock_server',
        'velocity_smoother',
        'wheel_odometry_node']
    for node in nodes:
        # oak filter / apriltag / etc. all set it; a node without it would
        # silently run with its YAML's `true`.
        assert node['inline_parameters'].get('use_sim_time') is False, node['name']


@pytest.mark.parametrize('switch,dropped', [
    ('apriltag', ['apriltag.launch.py']),
    ('nav', ['navigation.launch.py', 'oak_points_filter.launch.py']),
    ('docking', ['docking.launch.py']),
    ('mission', ['mission.launch.py'])])
def test_switch_false_drops_only_its_includes(switch, dropped):
    assert sorted(_includes(**{switch: 'false'})) == sorted(
        x for x in ALL if x not in dropped)


def test_map_and_saga_token_are_forwarded():
    nodes = {n['name']: n for n in _nodes(map='arena_real_alt', saga_token='12345')}
    assert nodes['slam_toolbox']['inline_parameters']['map_file_name'].endswith(
        os.path.join('maps', 'arena_real_alt'))
    assert nodes['mission_node']['inline_parameters']['saga_token'] == '12345'


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


def _warnings(**args):
    description = h.load_launch_module(LAUNCH_PATH).generate_launch_description()
    context = h.make_context(description, **args)
    return [perform_substitutions(context, e.msg)
            for e in h.expand(description.entities, context)
            if isinstance(e, LogInfo)
            and (e.condition is None or e.condition.evaluate(context))]


def test_no_warnings_by_default():
    assert _warnings() == []


def test_docking_without_nav_warns():
    warnings = _warnings(docking='true', nav='false')
    assert len(warnings) == 2  # mission is on by default as well
    assert any('docking needs nav' in w for w in warnings)


def test_mission_without_nav_warns():
    warnings = _warnings(docking='false', nav='false')
    assert len(warnings) == 1 and 'mission:=true' in warnings[0]


def test_no_warnings_when_dependents_are_off_too():
    assert _warnings(nav='false', docking='false', mission='false') == []


def test_mission_node_loads_the_hardware_health_topics():
    nodes = {n['name']: n for n in _nodes()}
    (path,) = nodes['mission_node']['parameter_files']
    assert path.endswith(os.path.join('mission', 'config', 'health_hardware.yaml'))


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


def test_wheel_drive_is_off_by_default():
    runs = _run()
    assert not any(kind == 'include' and f.endswith('wheel_drive.launch.py')
                   for kind, f in runs if kind == 'include')
    nav = _nav_include_args()
    assert nav['cmd_vel_out'] == 'cmd_vel'
    assert nav['wz_max'] == ''


def test_wheel_drive_starts_the_node_and_retargets_nav2():
    runs = _run(wheel_drive='true')
    assert any(kind == 'include' and f.endswith('wheel_drive.launch.py')
               for kind, f in runs)
    nav = _nav_include_args(wheel_drive='true')
    assert nav['cmd_vel_out'] == 'cmd_vel_ctrl'
    assert float(nav['wz_max']) == 0.75
