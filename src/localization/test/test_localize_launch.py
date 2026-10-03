"""localize.launch.py: sim defaults unchanged, hardware arguments reach nodes."""
import importlib.util
import os

import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_PATH = os.path.join(PKG, 'launch', 'localize.launch.py')
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
