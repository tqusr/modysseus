"""Launch-description tests for launch/apriltag.launch.py.

Plain pytest: the launch file is imported and its description evaluated
in-process (see launch_helpers). No process is started, no camera.
Needs `perception` built and its install sourced, because the launch file
resolves its params file through the package share directory.
"""

import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

from launch_helpers import (describe_node, expand, load_launch_module,
                            make_context, nodes_of)

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_PATH = os.path.join(PKG, 'launch', 'apriltag.launch.py')
YAML_PATH = os.path.join(PKG, 'config', 'apriltag.yaml')


def _nodes(**launch_arguments):
    description = load_launch_module(LAUNCH_PATH).generate_launch_description()
    context = make_context(description, **launch_arguments)
    return {
        info['name']: info
        for info in (describe_node(n, context)
                     for n in nodes_of(expand(description.entities, context)))}


def test_defaults_rectify_then_detect_on_wall_clock():
    nodes = _nodes()

    assert sorted(nodes) == ['apriltag_node', 'rectify_node']
    assert nodes['apriltag_node']['package'] == 'apriltag_ros'
    assert nodes['apriltag_node']['executable'] == 'apriltag_node'
    assert nodes['apriltag_node']['use_sim_time'] is False
    assert nodes['rectify_node']['use_sim_time'] is False
    rectify = nodes['rectify_node']
    assert rectify['package'] == 'image_proc'
    assert rectify['executable'] == 'rectify_node'
    assert rectify['use_sim_time'] is False
    assert rectify['remappings'] == {
        'image': '/oak/rgb/image_raw',
        'image_rect': '/oak/rgb/image_rect',
    }
    apriltag = nodes['apriltag_node']
    assert apriltag['use_sim_time'] is False
    assert apriltag['remappings'] == {
        'image_rect': '/oak/rgb/image_rect',
    }


def test_custom_topics_without_rectify_follow_the_arguments():
    nodes = _nodes(image_topic='/cam/image_raw', rectify='false')

    assert list(nodes) == ['apriltag_node']
    assert nodes['apriltag_node']['remappings'] == {
        'image_rect': '/cam/image_raw',
    }


def test_custom_topics_with_rectify_put_image_rect_next_to_the_raw_topic():
    nodes = _nodes(image_topic='/cam/image_raw', rectify='true')

    assert nodes['rectify_node']['remappings']['image_rect'] == '/cam/image_rect'
    assert nodes['apriltag_node']['remappings'] == {
        'image_rect': '/cam/image_rect',
    }


@pytest.mark.parametrize('spelling', ['True', 'TRUE', 'true', '1'])
def test_rectify_accepts_any_true_spelling(spelling):
    assert 'rectify_node' in _nodes(rectify=spelling)


def test_root_namespace_topic_rectifies_to_root_image_rect():
    nodes = _nodes(image_topic='/image_raw', rectify='true')

    assert nodes['rectify_node']['remappings']['image_rect'] == '/image_rect'
    assert nodes['apriltag_node']['remappings']['image_rect'] == '/image_rect'


def test_camera_info_is_not_remapped_or_a_launch_argument():
    # image_transport::CameraSubscriber ignores a camera_info remapping, so
    # the launch must not pretend to support one.
    for nodes in (_nodes(), _nodes(rectify='false')):
        for node in nodes.values():
            assert 'camera_info' not in node['remappings']
    description = load_launch_module(LAUNCH_PATH).generate_launch_description()
    declared = {e.name for e in description.entities
                if isinstance(e, DeclareLaunchArgument)}
    assert declared == {'image_topic', 'rectify', 'gated'}


@pytest.mark.parametrize('arguments', [{}, {'rectify': 'true'}])
def test_apriltag_node_gets_the_params_file(arguments):
    apriltag = _nodes(**arguments)['apriltag_node']

    assert [os.path.normpath(f) for f in apriltag['parameter_files']] == [
        os.path.normpath(os.path.join(
            get_package_share_directory('perception'),
            'config', 'apriltag.yaml'))]


def test_params_file_no_longer_hardcodes_use_sim_time():
    with open(YAML_PATH) as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']

    assert 'use_sim_time' not in params
    assert params['family'] == '36h11'
    assert params['size'] == 0.100


def test_gated_detector_reads_the_gate_output_not_the_camera():
    nodes = _nodes(gated='true')

    assert sorted(nodes) == ['apriltag_node', 'image_gate', 'rectify_node']
    assert nodes['image_gate']['package'] == 'perception'
    assert nodes['image_gate']['inline_parameters'] == {
        'use_sim_time': False,
        'source_topic': '/oak/rgb/image_rect',
        'output_topic': '/oak/rgb/image_gated',
    }
    assert nodes['apriltag_node']['remappings'] == {
        'image_rect': '/oak/rgb/image_gated'}


def test_gated_without_rectify_gates_the_raw_image():
    nodes = _nodes(gated='true', rectify='false')

    assert nodes['image_gate']['inline_parameters']['source_topic'] == '/oak/rgb/image_raw'
    assert nodes['apriltag_node']['remappings'] == {
        'image_rect': '/oak/rgb/image_gated'}
