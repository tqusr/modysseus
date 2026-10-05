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
