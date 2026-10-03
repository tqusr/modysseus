"""Launch-description tests for launch/mapping_mission_hardware.launch.py.

Pins the `apriltag` argument: on (the default) the real-robot mapping run also
starts AprilTag detection - rectified, on wall-clock time - and off it is the
lidar-only run it used to be. The launch files are imported and evaluated
in-process (see perception/test/launch_helpers.py); nothing is started.

Needs `localization` and `perception` built and their install sourced, as the
launch file locates the perception launch through the share directory.
"""

import importlib.util
import os

import pytest
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_PATH = os.path.join(PKG, 'launch', 'mapping_mission_hardware.launch.py')
HELPERS_PATH = os.path.join(
    PKG, '..', 'perception', 'test', 'launch_helpers.py')


def _load_helpers():
    """perception's launch_helpers, loaded by path: localization must not
    depend on perception (perception already depends on localization)."""
    spec = importlib.util.spec_from_file_location('launch_helpers', HELPERS_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


h = _load_helpers()


def _description_and_context(**launch_arguments):
    description = h.load_launch_module(LAUNCH_PATH).generate_launch_description()
    return description, h.make_context(description, **launch_arguments)


def _apriltag_timers(description):
    """The delayed actions that include a launch file.

    The included file is not matched by name here: before a launch context
    evaluates it, an include's `location` is only the repr of its
    substitutions. The file is asserted on, evaluated, in the tests below.
    """
    return [
        e for e in description.entities
        if isinstance(e, TimerAction)
        and any(isinstance(a, IncludeLaunchDescription) for a in e.actions)]


def _apriltag_include(timer, context):
    (include,) = [a for a in timer.actions
                  if isinstance(a, IncludeLaunchDescription)]
    return h.describe_include(include, context)


def test_apriltag_defaults_to_on_staggered_by_five_seconds():
    description, context = _description_and_context()

    (timer,) = _apriltag_timers(description)

    assert isinstance(timer.condition, IfCondition)
    assert timer.condition.evaluate(context) is True
    assert timer.period == 5.0


def test_apriltag_include_passes_the_hardware_arguments():
    description, context = _description_and_context()
    (timer,) = _apriltag_timers(description)

    include = _apriltag_include(timer, context)

    assert include['file'].endswith(
        os.path.join('perception', 'launch', 'apriltag.launch.py'))
    assert include['args'] == {
        'use_sim_time': 'false',
        'image_topic': '/oak/rgb/image_raw',
        'rectify': 'true',
    }


def test_apriltag_false_skips_the_include():
    description, context = _description_and_context(apriltag='false')

    (timer,) = _apriltag_timers(description)

    assert timer.condition.evaluate(context) is False


@pytest.mark.parametrize('spelling', ['False', 'FALSE', '0'])
def test_apriltag_false_spelling_variants_skip_the_include(spelling):
    description, context = _description_and_context(apriltag=spelling)

    (timer,) = _apriltag_timers(description)

    assert timer.condition.evaluate(context) is False


def _apriltag_log_infos(description):
    """LogInfos that only appear when AprilTag detection is on."""
    return [e for e in description.entities
            if isinstance(e, LogInfo) and e.condition is not None]


def test_apriltag_announcement_follows_the_apriltag_argument():
    description, context = _description_and_context()
    (log,) = _apriltag_log_infos(description)
    assert isinstance(log.condition, IfCondition)
    assert log.condition.evaluate(context) is True

    description, context = _description_and_context(apriltag='false')
    (log,) = _apriltag_log_infos(description)
    assert log.condition.evaluate(context) is False


def test_apriltag_include_is_resolved_lazily_through_find_package_share():
    # A substitution, not a path computed while the description is built, so
    # apriltag:=false does not need perception installed.
    description, _ = _description_and_context()
    (timer,) = _apriltag_timers(description)
    (include,) = [a for a in timer.actions
                  if isinstance(a, IncludeLaunchDescription)]
    (location,) = include.launch_description_source._LaunchDescriptionSource__location
    assert isinstance(location, PathJoinSubstitution)
    assert 'FindPackageShare' in repr(location)


def test_apriltag_nodes_are_rectified_and_on_wall_clock():
    description, context = _description_and_context()
    (timer,) = _apriltag_timers(description)
    include = _apriltag_include(timer, context)

    # Evaluate the included launch itself with those arguments, to see the
    # nodes it would really start.
    apriltag_launch = h.load_launch_module(include['file'])
    apriltag_description = apriltag_launch.generate_launch_description()
    apriltag_context = h.make_context(apriltag_description, **include['args'])
    nodes = {
        info['name']: info
        for info in (
            h.describe_node(n, apriltag_context)
            for n in h.nodes_of(h.expand(
                apriltag_description.entities, apriltag_context)))}

    assert sorted(nodes) == ['apriltag_node', 'rectify_node']
    assert all(n['use_sim_time'] is False for n in nodes.values())
    assert nodes['apriltag_node']['remappings']['image_rect'] == (
        '/oak/rgb/image_rect')
    assert nodes['rectify_node']['remappings']['image'] == '/oak/rgb/image_raw'


def test_hardware_launch_still_runs_on_wall_clock_without_the_camera_nodes():
    description, context = _description_and_context()

    entities = h.expand(description.entities, context)
    own_nodes = [h.describe_node(n, context) for n in h.nodes_of(entities)]

    assert [n['name'] for n in own_nodes] == ['wheel_odometry_node']
    assert own_nodes[0]['use_sim_time'] is False
