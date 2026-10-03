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


def _expand(**args):
    description = h.load_launch_module(LAUNCH_PATH).generate_launch_description()
    context = h.make_context(description, **args)
    return h.expand(description.entities, context), context


@pytest.fixture
def robot_pose_graph_exists(monkeypatch):
    """`arena_real` only exists on the robot; pretend its files do here."""
    real_exists = os.path.exists
    monkeypatch.setattr(os.path, 'exists', lambda path: (
        True if 'arena_real' in str(path) else real_exists(path)))


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


def test_map_argument_selects_the_posegraph_basename(robot_pose_graph_exists):
    slam = _nodes(map='arena_real')['slam_toolbox']['inline_parameters']
    assert slam['map_file_name'].endswith(os.path.join('maps', 'arena_real'))


@pytest.mark.parametrize('spelling', ['false', 'False', '0'])
def test_use_sim_time_false_reaches_both_nodes_as_bool(spelling):
    for node in _nodes(use_sim_time=spelling).values():
        assert node['inline_parameters']['use_sim_time'] is False


def test_missing_pose_graph_fails_the_launch_naming_the_paths():
    with pytest.raises(RuntimeError) as error:
        _expand(map='no_such_map')
    message = str(error.value)
    base = os.path.join('maps', 'no_such_map')
    assert base + '.posegraph' in message and base + '.data' in message
    assert 'serialize_map' in message


def test_existing_pose_graph_does_not_raise():
    _expand(map='arena')
