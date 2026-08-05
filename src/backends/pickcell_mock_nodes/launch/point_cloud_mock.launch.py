"""Launch the deterministic point-cloud and target-point mock."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the mock publisher with configurable path-planning points."""
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="mock"),
        DeclareLaunchArgument("frame_id", default_value="cell"),
        DeclareLaunchArgument("publish_rate_hz", default_value="2.0"),
        DeclareLaunchArgument("final_point", default_value="[0.45, 0.0, 0.10]"),
        DeclareLaunchArgument("target_point", default_value="[0.80, 0.0, 0.20]"),
        Node(
            package="pickcell_mock_nodes",
            executable="point_cloud_mock",
            name="point_cloud_mock",
            namespace=LaunchConfiguration("namespace"),
            parameters=[{
                "frame_id": LaunchConfiguration("frame_id"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "final_point": LaunchConfiguration("final_point"),
                "target_point": LaunchConfiguration("target_point"),
            }],
            output="screen",
        ),
    ])
