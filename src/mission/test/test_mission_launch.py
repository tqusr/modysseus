"""mission.launch.py: docking + mission in the second terminal, wall clock."""
import importlib.util
import os

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_PATH = os.path.join(PKG, 'launch', 'mission.launch.py')
HELPERS_PATH = os.path.join(
    PKG, '..', 'perception', 'test', 'launch_helpers.py')


def _load_helpers():
    """perception's launch_helpers, loaded by path (no package dependency)."""
    spec = importlib.util.spec_from_file_location('launch_helpers', HELPERS_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


h = _load_helpers()


def _expand_all(entities, context):
    """Entities of a launch description, recursing through its includes."""
    from launch.actions import IncludeLaunchDescription
    for e in h.expand(entities, context):
        if isinstance(e, IncludeLaunchDescription):
            info = h.describe_include(e, context)
            sub = h.load_launch_module(info['file']).generate_launch_description()
            yield from _expand_all(sub.entities, h.make_context(sub, **info['args']))
        else:
            yield e, context


def _nodes(**args):
    from launch_ros.actions import Node
    description = h.load_launch_module(LAUNCH_PATH).generate_launch_description()
    context = h.make_context(description, **args)
    return [h.describe_node(e, c) for e, c in _expand_all(description.entities, context)
            if isinstance(e, Node)]


def test_defaults_are_wall_clock_with_string_saga_arguments():
    node = next(n for n in _nodes(saga_token='12345') if n['name'] == 'mission_node')
    p = node['inline_parameters']
    assert p['use_sim_time'] is False
    assert p['saga_token'] == '12345' and isinstance(p['saga_token'], str)
    assert p['saga_url'] == 'http://localhost:8000'


def test_no_parameter_files():
    node = next(n for n in _nodes() if n['name'] == 'mission_node')
    assert node['parameter_files'] == []


def test_runs_the_docking_servers_and_the_mission_node():
    names = sorted(n['name'] for n in _nodes(saga_token='t'))
    assert names == ['docking_server', 'mission_node', 'tag_dock_server']


def test_mission_false_runs_only_the_docking_servers():
    names = sorted(n['name'] for n in _nodes(mission='false'))
    assert names == ['docking_server', 'tag_dock_server']


def test_mission_false_spellings():
    for spelling in ('False', 'FALSE', '0'):
        assert 'mission_node' not in {n['name'] for n in _nodes(mission=spelling)}


def test_every_node_is_on_wall_clock():
    for node in _nodes(saga_token='t'):
        assert node['inline_parameters'].get('use_sim_time') is False, node['name']


def test_does_not_start_nav2_or_the_hardware_layers():
    names = {n['name'] for n in _nodes(saga_token='t')}
    assert not names & {'bt_navigator', 'controller_server', 'slam_toolbox',
                        'ekf_filter_node', 'wheel_drive_node'}
