"""use_sim_time reaches the docking and oak_points_filter nodes as a bool."""
import os

import pytest

from launch_helpers import (describe_node, expand, load_launch_module,
                            make_context, nodes_of)

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _nodes(launch_file, **args):
    description = load_launch_module(
        os.path.join(PKG, 'launch', launch_file)).generate_launch_description()
    context = make_context(description, **args)
    return [describe_node(n, context)
            for n in nodes_of(expand(description.entities, context))]


@pytest.mark.parametrize('launch_file', ['docking.launch.py',
                                         'oak_points_filter.launch.py'])
@pytest.mark.parametrize('args,expected', [
    ({}, True), ({'use_sim_time': 'true'}, True),
    ({'use_sim_time': 'false'}, False), ({'use_sim_time': 'False'}, False),
    ({'use_sim_time': '0'}, False)])
def test_use_sim_time_is_a_typed_bool(launch_file, args, expected):
    nodes = _nodes(launch_file, **args)
    assert nodes
    for node in nodes:
        assert node['inline_parameters']['use_sim_time'] is expected
