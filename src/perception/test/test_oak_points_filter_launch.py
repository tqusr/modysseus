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
            'extra': {text(k): v for arg in c.extra_arguments
                      for k, v in arg.items()},
        })
    return out


def test_chains_depth_image_to_points_filtered():
    # No CropBox: the real camera does not see the robot, so nothing near it
    # is cut.
    stages = _components()
    assert [s['plugin'] for s in stages] == [
        'image_proc::ResizeNode', 'depth_image_proc::PointCloudXyzNode',
        'pcl_ros::VoxelGrid']
    resize, points, voxel = (s['remappings'] for s in stages)
    assert resize['image/image_raw'] == '/oak/stereo/image_raw'
    assert resize['image/camera_info'] == '/oak/stereo/camera_info'
    assert points['image_rect'] == resize['resize/image_raw']
    assert points['camera_info'] == resize['resize/camera_info']
    assert voxel['input'] == points['points']
    assert voxel['output'] == '/oak/points_filtered'


def test_keeps_the_filters_yaml_names():
    # The YAML is keyed by node name; a renamed component would silently run
    # on pcl_ros defaults (and VoxelGrid's default range cut is 1 m).
    names = [s['name'] for s in _components()]
    assert names == ['oak_depth_resize', 'oak_depth_points',
                     'filter_voxel_grid_node']
    for stage in _components():
        assert [os.path.basename(f) for f in stage['files']] == [
            'oak_points_filter.yaml']


def test_every_stage_runs_on_wall_clock_in_process():
    for stage in _components():
        assert stage['inline']['use_sim_time'] is False, stage['name']
        assert stage['extra'] == {'use_intra_process_comms': True}
