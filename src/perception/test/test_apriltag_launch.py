"""Launch-description tests for launch/apriltag.launch.py.

Plain pytest: the launch file is imported and its description evaluated
in-process (see launch_helpers). No process is started, no Gazebo, no camera.
Needs `perception` built and its install sourced, because the launch file
resolves its params file through the package share directory.
"""

import os

import pytest
import yaml

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


def test_sim_defaults_run_one_apriltag_node_on_the_raw_sim_topics():
    nodes = _nodes()

    assert list(nodes) == ['apriltag_node']
    apriltag = nodes['apriltag_node']
    assert apriltag['package'] == 'apriltag_ros'
    assert apriltag['executable'] == 'apriltag_node'
    assert apriltag['use_sim_time'] is True
    assert apriltag['remappings'] == {
        'image_rect': '/oak/rgb/image_raw',
        'camera_info': '/oak/rgb/camera_info',
    }


def test_hardware_arguments_rectify_then_detect_on_wall_clock():
    nodes = _nodes(use_sim_time='false', rectify='true')

    assert sorted(nodes) == ['apriltag_node', 'rectify_node']
    rectify = nodes['rectify_node']
    assert rectify['package'] == 'image_proc'
    assert rectify['executable'] == 'rectify_node'
    assert rectify['use_sim_time'] is False
    assert rectify['remappings'] == {
        'image': '/oak/rgb/image_raw',
        'camera_info': '/oak/rgb/camera_info',
        'image_rect': '/oak/rgb/image_rect',
    }
    apriltag = nodes['apriltag_node']
    assert apriltag['use_sim_time'] is False
    assert apriltag['remappings'] == {
        'image_rect': '/oak/rgb/image_rect',
        'camera_info': '/oak/rgb/camera_info',
    }


def test_custom_topics_without_rectify_follow_the_arguments():
    nodes = _nodes(image_topic='/cam/image_raw',
                   camera_info_topic='/cam/info', rectify='false')

    assert list(nodes) == ['apriltag_node']
    assert nodes['apriltag_node']['remappings'] == {
        'image_rect': '/cam/image_raw',
        'camera_info': '/cam/info',
    }


def test_custom_topics_with_rectify_put_image_rect_next_to_the_raw_topic():
    nodes = _nodes(image_topic='/cam/image_raw',
                   camera_info_topic='/cam/info', rectify='true')

    assert nodes['rectify_node']['remappings']['image_rect'] == '/cam/image_rect'
    assert nodes['apriltag_node']['remappings'] == {
        'image_rect': '/cam/image_rect',
        'camera_info': '/cam/info',
    }


@pytest.mark.parametrize('spelling', ['True', 'TRUE', 'true', '1'])
def test_rectify_accepts_any_true_spelling(spelling):
    assert 'rectify_node' in _nodes(rectify=spelling)


@pytest.mark.parametrize('spelling', ['false', 'False', 'FALSE', '0'])
def test_use_sim_time_false_spellings_reach_nodes_as_bool(spelling):
    nodes = _nodes(use_sim_time=spelling, rectify='true')

    assert nodes['apriltag_node']['use_sim_time'] is False
    assert nodes['rectify_node']['use_sim_time'] is False


def test_root_namespace_topic_rectifies_to_root_image_rect():
    nodes = _nodes(image_topic='/image_raw', rectify='true')

    assert nodes['rectify_node']['remappings']['image_rect'] == '/image_rect'
    assert nodes['apriltag_node']['remappings']['image_rect'] == '/image_rect'


def test_params_file_no_longer_hardcodes_use_sim_time():
    with open(YAML_PATH) as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']

    assert 'use_sim_time' not in params
    assert params['family'] == '36h11'
    assert params['size'] == 0.100
