"""Helpers used by subsystem launch files to consume resolved configuration."""

from __future__ import annotations

from typing import Any

from launch_ros.actions import Node

from .config_resolution import ResolvedSystemConfig


def configured_node(resolved: ResolvedSystemConfig, *, section: str, package: str, executable: str, node_name: str, namespace: str = "pickcell", extra_parameters: dict[str, Any] | None = None, **node_kwargs: Any, ) -> Node:
    """Construct a Node receiving only its resolved ROS parameter mapping."""
    parameters = resolved.parameters_for(section, node_name)
    if extra_parameters:
        parameters.update(extra_parameters)
    return Node(package=package, executable=executable, name=node_name, namespace=namespace, parameters=[parameters], **node_kwargs, )
