"""mission.launch.py: sim time by default, saga arguments passed as strings."""
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


def _nodes(**args):
    description = h.load_launch_module(LAUNCH_PATH).generate_launch_description()
    context = h.make_context(description, **args)
    return [h.describe_node(n, context)
            for n in h.nodes_of(h.expand(description.entities, context))]


def test_defaults_are_sim_time_with_string_saga_arguments():
    (node,) = _nodes(saga_token='12345')
    p = node['inline_parameters']
    assert p['use_sim_time'] is True
    assert p['saga_token'] == '12345' and isinstance(p['saga_token'], str)
    assert p['saga_url'] == 'http://localhost:8000'


def test_use_sim_time_false_is_a_bool():
    (node,) = _nodes(use_sim_time='false')
    assert node['inline_parameters']['use_sim_time'] is False
