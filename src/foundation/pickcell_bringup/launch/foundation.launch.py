"""Launch the PickCell description using system-wide launch settings."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description() -> LaunchDescription:
    """Create the foundation launch description."""
    description_share = Path(
        get_package_share_directory("pickcell_description")
    )
    robot_description = xacro.process_file(
        str(description_share / "urdf" / "pickcell.urdf.xacro")
    ).toxml()

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="pickcell"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=LaunchConfiguration("namespace"),
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            output="screen",
        ),
    ])
