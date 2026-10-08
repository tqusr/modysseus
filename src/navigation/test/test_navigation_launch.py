"""navigation.launch.py: use_sim_time reaches every Nav2 node as a bool."""
import importlib.util
import os

import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_PATH = os.path.join(PKG, 'launch', 'navigation.launch.py')
HELPERS_PATH = os.path.join(
    PKG, '..', 'perception', 'test', 'launch_helpers.py')


def _load_helpers():
    """perception's launch_helpers, loaded by path (no package dependency)."""
    spec = importlib.util.spec_from_file_location('launch_helpers', HELPERS_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


h = _load_helpers()


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


def test_nav_to_pose_replans_only_when_the_path_becomes_invalid():
    bt = next(n for n in _nodes() if n['name'] == 'bt_navigator')
    xml = bt['inline_parameters']['default_nav_to_pose_bt_xml']
    assert xml.endswith(
        'navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml')
    assert os.path.isfile(xml)


def _node(name, **args):
    return next(n for n in _nodes(**args) if n['name'] == name)


def test_wz_max_defaults_to_the_files_values():
    controller = _node('controller_server')['inline_parameters']
    smoother = _node('velocity_smoother')['inline_parameters']
    assert 'FollowPath.wz_max' not in controller
    assert 'max_velocity' not in smoother


def test_wz_max_overrides_every_angular_limit_together():
    controller = _node('controller_server', wz_max='0.75')['inline_parameters']
    smoother = _node('velocity_smoother', wz_max='0.75')['inline_parameters']
    assert controller['FollowPath.wz_max'] == 0.75
    assert controller['FollowPath.rotate_to_heading_angular_vel'] == 0.75
    assert list(smoother['max_velocity']) == [0.3, 0.0, 0.75]
    assert list(smoother['min_velocity']) == [-0.15, 0.0, -0.75]


def test_the_safety_chain_ends_in_cmd_vel_ctrl():
    params = _node('collision_monitor')['inline_parameters']
    assert params['cmd_vel_out_topic'] == 'cmd_vel_ctrl'


def test_the_files_limits_the_override_replaces_are_the_ones_we_think():
    """If the file's wz or vx change, the override's [0.3, 0, wz] must follow."""
    import yaml
    with open(os.path.join(PKG, 'config', 'nav2_params.yaml')) as f:
        params = yaml.safe_load(f)
    smoother = params['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'][:2] == [0.3, 0.0]
    assert smoother['min_velocity'][:2] == [-0.15, 0.0]
