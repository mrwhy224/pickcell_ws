"""Tests for configured ROS node construction."""

from pathlib import Path

from launch_ros.actions import Node

from pickcell_bringup.config_resolution import ResolvedSection
from pickcell_bringup.config_resolution import ResolvedSystemConfig
from pickcell_bringup.launch_helpers import configured_node


def test_configured_node_returns_launch_node(tmp_path, monkeypatch) -> None:
    """Subsystem assembly creates a normal launch_ros Node action."""
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    resolved = ResolvedSystemConfig(
        mode="mock",
        use_sim_time=False,
        configuration_hash="abc123",
        sections={
            "perception": ResolvedSection(
                name="perception",
                profile="default",
                values={},
                loaded_files=(Path("defaults.yaml.dist"),),
            )
        },
    )
    node = configured_node(
        resolved,
        section="perception",
        package="demo_nodes_py",
        executable="listener",
        node_name="perception_server",
    )
    assert isinstance(node, Node)
