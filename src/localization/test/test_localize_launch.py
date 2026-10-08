"""localize.launch.py: wall clock, the arena_real map, start pose reaches slam_toolbox."""
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


def test_defaults_are_wall_clock_on_the_arena_real_map(robot_pose_graph_exists):
    nodes = _nodes()
    assert sorted(nodes) == ['scan_filter', 'slam_toolbox']
    assert nodes['scan_filter']['inline_parameters']['use_sim_time'] is False
    slam = nodes['slam_toolbox']['inline_parameters']
    assert slam['use_sim_time'] is False
    assert slam['map_file_name'].endswith(os.path.join('maps', 'arena_real'))


def test_map_argument_selects_the_posegraph_basename(robot_pose_graph_exists):
    slam = _nodes(map='arena_real')['slam_toolbox']['inline_parameters']
    assert slam['map_file_name'].endswith(os.path.join('maps', 'arena_real'))


def test_missing_pose_graph_fails_the_launch_naming_the_paths():
    with pytest.raises(RuntimeError) as error:
        _expand(map='no_such_map')
    message = str(error.value)
    base = os.path.join('maps', 'no_such_map')
    assert base + '.posegraph' in message and base + '.data' in message
    assert 'serialize_map' in message


def test_existing_pose_graph_does_not_raise(robot_pose_graph_exists):
    _expand()


def test_start_pose_defaults_to_the_origin_so_the_graph_loads(robot_pose_graph_exists):
    """Without map_start_pose slam_toolbox skips loading the pose graph."""
    slam = _nodes()['slam_toolbox']['inline_parameters']
    assert slam['map_start_pose'] == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('spelling', ['1.5, -2, 3.14', '[1.5, -2, 3.14]', '1.5 -2 3.14'])
def test_start_pose_argument_reaches_slam_toolbox_as_floats(spelling, robot_pose_graph_exists):
    slam = _nodes(map_start_pose=spelling)['slam_toolbox']['inline_parameters']
    assert slam['map_start_pose'] == (1.5, -2.0, 3.14)


@pytest.mark.parametrize('spelling', ['1, 2', '1, 2, 3, 4', 'a, b, c', ''])
def test_malformed_start_pose_fails_the_launch(spelling, robot_pose_graph_exists):
    with pytest.raises(RuntimeError, match='map_start_pose'):
        _expand(map_start_pose=spelling)
