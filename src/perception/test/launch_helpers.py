"""Evaluate a launch description in-process, without starting any node.

Used by the launch-description tests of this package and of `localization`
(whose hardware mapping launch includes perception's apriltag launch).

Technique: a `LaunchContext` is filled with the launch configurations the
description declares (their defaults, then the test's overrides). Entities
whose content depends on those values are `OpaqueFunction`s; calling
`execute(context)` on one returns the actions it builds. Nothing is ever
passed to a launch service, so no process starts and no ROS graph is needed.
`Node` keeps its remappings and parameters in private attributes until it is
executed, so `describe_node` reads those, evaluating their substitutions.
"""

import importlib.util

from launch import LaunchContext
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def load_launch_module(path):
    """Import a `*.launch.py` file by path (its name is not importable)."""
    spec = importlib.util.spec_from_file_location('launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_context(description, **overrides):
    """A context holding every declared argument's default, then `overrides`."""
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = perform_substitutions(
                context, entity.default_value)
    context.launch_configurations.update(overrides)
    return context


def expand(entities, context):
    """Flatten `entities`, running every OpaqueFunction against `context`."""
    flat = []
    for entity in entities:
        if isinstance(entity, OpaqueFunction):
            flat.extend(expand(entity.execute(context) or [], context))
        else:
            flat.append(entity)
    return flat


def nodes_of(entities):
    return [e for e in entities if isinstance(e, Node)]


def describe_node(node, context):
    """The facts a test cares about, as plain Python values."""
    def text(substitutions):
        return perform_substitutions(context, list(substitutions))

    remappings = {text(src): text(dst) for src, dst in node._Node__remappings}
    use_sim_time = None
    for param in node._Node__parameters:
        if isinstance(param, dict):  # the inline {'use_sim_time': ...} dict
            for key, value in param.items():
                if text(key) == 'use_sim_time':
                    use_sim_time = value
    return {
        'package': node.node_package,
        'executable': node.node_executable,
        'name': node._Node__node_name,
        'remappings': remappings,
        'use_sim_time': use_sim_time,
    }


def describe_include(include, context):
    """An IncludeLaunchDescription's file path and its launch arguments."""
    args = {
        perform_substitutions(context, list(key)):
            perform_substitutions(context, list(value))
        for key, value in include.launch_arguments}
    return {'file': include.launch_description_source.location, 'args': args}
