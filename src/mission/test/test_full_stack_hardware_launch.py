"""full_stack_hardware.launch.py: the sim stack minus Gazebo, on wall-clock time.

The launch files are evaluated in-process, recursing through includes and
timers (see perception/test/launch_helpers.py); nothing is started. Needs the
four packages built and their install sourced, as the includes are located
through the share directories.
"""
import importlib.util
import os

import pytest
from launch.actions import IncludeLaunchDescription, TimerAction
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
