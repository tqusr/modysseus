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
    assert h.nodes_of(entities) == []  # the odometry node lives one include down

    (odometry_include,) = [
        h.describe_include(e, context) for e in entities
        if isinstance(e, IncludeLaunchDescription)
        and h.describe_include(e, context)['file'].endswith(
            'hardware_odometry.launch.py')]
    odometry_description = h.load_launch_module(
        odometry_include['file']).generate_launch_description()
    odometry_context = h.make_context(
        odometry_description, **odometry_include['args'])
    odometry_nodes = [
        h.describe_node(n, odometry_context)
        for n in h.nodes_of(h.expand(
            odometry_description.entities, odometry_context))]

    assert [n['name'] for n in odometry_nodes] == ['wheel_odometry_node']
    assert odometry_nodes[0]['use_sim_time'] is False


MAPPING_PATH = os.path.join(PKG, 'launch', 'mapping.launch.py')


def _mapping_include(context, description):
    (include,) = [
        h.describe_include(e, context)
        for e in h.expand(description.entities, context)
        if isinstance(e, IncludeLaunchDescription)
        and h.describe_include(e, context)['file'].endswith('mapping.launch.py')]
    return include


def _slam_parameter_files(**args):
    description = h.load_launch_module(MAPPING_PATH).generate_launch_description()
    context = h.make_context(description, **args)
    (slam,) = [
        info for info in (
            h.describe_node(n, context)
            for n in h.nodes_of(h.expand(description.entities, context)))
        if info['name'] == 'slam_toolbox']
    return slam['parameter_files']


def test_mapping_includes_with_no_arguments_and_uses_the_hardware_params_file():
    description, context = _description_and_context()
    include = _mapping_include(context, description)

    assert include['args'] == {}
    (path,) = _slam_parameter_files()
    assert path.endswith(
        os.path.join('localization', 'config', 'mapper_params_hardware.yaml'))
    assert os.path.isfile(path)


def test_mapping_params_keep_the_narrow_search_window_and_transform_timeout():
    import yaml
    with open(os.path.join(PKG, 'config', 'mapper_params_hardware.yaml')) as f:
        hw = yaml.safe_load(f)['slam_toolbox']['ros__parameters']
    assert hw['correlation_search_space_dimension'] == 0.5
    assert hw['transform_timeout'] == 1.0
