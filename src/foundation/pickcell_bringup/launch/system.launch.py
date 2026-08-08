"""Launch every node listed in pickcell_config/config/application.yaml."""

import hashlib
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
import yaml
import xacro


def generate_launch_description() -> LaunchDescription:
    """Read the one application file and launch its configured nodes."""
    config_path = (
        Path(get_package_share_directory("pickcell_config"))
        / "config"
        / "application.yaml"
    )
    with config_path.open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    system = config["system"]
    namespace = system["namespace"]
    use_sim_time = bool(system["use_sim_time"])
    config_hash = hashlib.sha256(
        json.dumps(config, sort_keys=True).encode("utf-8")
    ).hexdigest()

    actions = [LogInfo(msg=f"PickCell config: {config_path}")]
    tf_config = system["tf"]
    if tf_config["enabled"]:
        description_path = (
            Path(get_package_share_directory(
                tf_config["description_package"]
            ))
            / tf_config["xacro_file"]
        )
        robot_description = xacro.process_file(
            str(description_path)
        ).toxml()
        actions.extend([
            LogInfo(msg=f"TF description: {description_path}"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=namespace,
                parameters=[{
                    "robot_description": robot_description,
                    "use_sim_time": use_sim_time,
                }],
                output="screen",
            ),
        ])
    for node_name, node_config in config["nodes"].items():
        parameters = dict(node_config["parameters"])
        parameters.update({
            "system_mode": node_config["mode"],
            "use_sim_time": use_sim_time,
            "configuration_hash": config_hash,
        })
        actions.extend([
            LogInfo(msg=f"{node_name}: mode={node_config['mode']}"),
            Node(
                package=node_config["package"],
                executable=node_config["executable"],
                name=node_name,
                namespace=namespace,
                parameters=[parameters],
                output="screen",
            ),
        ])
    return LaunchDescription(actions)
