"""hardware_odometry.launch.py: wheel odometry, then the fusion include."""
import importlib.util
import os

from launch.actions import IncludeLaunchDescription

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_PATH = os.path.join(PKG, 'launch', 'hardware_odometry.launch.py')
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


def test_wheel_odometry_then_fusion_include():
    description = h.load_launch_module(LAUNCH_PATH).generate_launch_description()
    context = h.make_context(description)
    entities = h.expand(description.entities, context)
    (node,) = [h.describe_node(n, context) for n in h.nodes_of(entities)]
    assert node['name'] == 'wheel_odometry_node'
    assert node['inline_parameters'] == {'use_sim_time': False, 'publish_tf': False}
    assert node['parameter_files'][0].endswith('hardware.yaml')
    (include,) = [e for e in entities if isinstance(e, IncludeLaunchDescription)]
    info = h.describe_include(include, context)
    assert info['file'].endswith('localization.launch.py')
    assert info['args'] == {}
    assert entities.index(include) > entities.index(
        h.nodes_of(entities)[0])  # odometry node first, as before
