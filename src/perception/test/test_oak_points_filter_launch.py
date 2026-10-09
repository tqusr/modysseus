"""oak_points_filter.launch.py's depth chain publishes /oak/points_filtered."""
import os

from launch.utilities import (normalize_to_list_of_substitutions,
                              perform_substitutions)
from launch_ros.actions import ComposableNodeContainer

from launch_helpers import expand, load_launch_module, make_context

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _entities(**args):
    description = load_launch_module(os.path.join(
        PKG, 'launch', 'oak_points_filter.launch.py')).generate_launch_description()
    context = make_context(description, **args)
    return expand(description.entities, context), context


def _components(**args):
    """Each component as {plugin, name, remappings, inline, files}, in order."""
    entities, context = _entities(**args)
    [container] = [e for e in entities if isinstance(e, ComposableNodeContainer)]

    def text(value):
        return perform_substitutions(
            context, normalize_to_list_of_substitutions(value))

    out = []
    for c in container._ComposableNodeContainer__composable_node_descriptions:
        inline, files = {}, []
        for param in c.parameters:
            if isinstance(param, dict):
                inline.update({text(k): v for k, v in param.items()})
            else:  # a ParameterFile, holding a path or substitutions
                path = param.param_file
                files.append(str(path) if isinstance(path, os.PathLike)
                             else text(path))
        out.append({
            'plugin': text(c.node_plugin),
            'name': text(c.node_name),
            'remappings': {text(a): text(b) for a, b in c.remappings},
            'inline': inline,
            'files': files,
        })
    return out


def test_builds_points_filtered_straight_from_the_depth_image():
    # No resize, voxel or CropBox: the camera publishes small depth itself and
    # does not see the robot.
    [stage] = _components()
    assert stage['plugin'] == 'depth_image_proc::PointCloudXyzNode'
    assert stage['remappings'] == {
        'image_rect': '/oak/stereo/image_raw',
        'camera_info': '/oak/stereo/camera_info',
        'points': '/oak/points_filtered',
    }


def test_keeps_the_filters_yaml_names():
    # The YAML is keyed by node name; a renamed component would silently run
    # on the defaults.
    [stage] = _components()
    assert stage['name'] == 'oak_depth_points'
    assert [os.path.basename(f) for f in stage['files']] == [
        'oak_points_filter.yaml']


def test_runs_on_wall_clock():
    [stage] = _components()
    assert stage['inline']['use_sim_time'] is False
