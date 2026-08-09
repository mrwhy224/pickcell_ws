"""Launch every node listed in pickcell_config/config/application.yaml."""

import hashlib
import json
import math
from pathlib import Path
import tempfile

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
    robot_config = system["robot"]
    tf_config = system["tf"]
    robot_description = None
    if tf_config["enabled"]:
        description_path = (
            Path(get_package_share_directory(
                tf_config["description_package"]
            ))
            / tf_config["xacro_file"]
        )
        robot_description = xacro.process_file(
            str(description_path),
            mappings={
                key: str(value)
                for key, value in robot_config.get(
                    "xacro_arguments", {}
                ).items()
            },
        ).toxml()
        actions.extend([
            LogInfo(
                msg=(
                    f"TF description: {description_path}; "
                    f"robot_model={robot_config['model']}; "
                    f"robot_mode={robot_config['mode']}"
                )
            ),
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
        if robot_config["joint_state_source"] == "default":
            joint_description_path = (
                Path(tempfile.gettempdir())
                / f"pickcell_robot_description_{config_hash}.urdf"
            )
            joint_description_path.write_text(
                robot_description, encoding="utf-8"
            )
            joint_state_parameters = {
                "rate": int(robot_config.get("joint_state_rate_hz", 10)),
                "use_sim_time": use_sim_time,
            }
            joint_state_parameters.update({
                f"zeros.{joint_name}": math.radians(float(position_deg))
                for joint_name, position_deg in robot_config.get(
                    "initial_joint_positions_deg", {}
                ).items()
            })
            actions.append(
                Node(
                    package="joint_state_publisher",
                    executable="joint_state_publisher",
                    name="joint_state_publisher",
                    namespace=namespace,
                    arguments=[str(joint_description_path)],
                    parameters=[joint_state_parameters],
                    output="screen",
                )
            )
    moveit_config = system.get("moveit", {})
    if moveit_config.get("enabled", False):
        if robot_description is None:
            raise RuntimeError("MoveIt requires system.tf.enabled")
        moveit_share = Path(get_package_share_directory(
            moveit_config["config_package"]
        ))
        semantic_path = moveit_share / moveit_config["semantic_file"]
        kinematics_path = moveit_share / moveit_config["kinematics_file"]
        joint_limits_path = moveit_share / moveit_config["joint_limits_file"]
        pipeline_name = moveit_config["planning_pipeline"]
        pipeline_path = (
            moveit_share / moveit_config["planning_pipeline_file"]
        )
        robot_description_semantic = semantic_path.read_text(
            encoding="utf-8"
        )
        with kinematics_path.open("r", encoding="utf-8") as config_file:
            kinematics = yaml.safe_load(config_file)
        with joint_limits_path.open("r", encoding="utf-8") as config_file:
            joint_limits = yaml.safe_load(config_file)
        with pipeline_path.open("r", encoding="utf-8") as config_file:
            planning_pipeline = yaml.safe_load(config_file)
        actions.extend([
            LogInfo(
                msg=(
                    f"MoveIt config: {moveit_share}; "
                    f"pipeline={pipeline_name}"
                )
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                namespace=namespace,
                parameters=[{
                    "robot_description": robot_description,
                    "robot_description_semantic": robot_description_semantic,
                    "robot_description_kinematics": kinematics,
                    "robot_description_planning": joint_limits,
                    "planning_pipelines": [pipeline_name],
                    "default_planning_pipeline": pipeline_name,
                    pipeline_name: planning_pipeline,
                    "allow_trajectory_execution": bool(moveit_config[
                        "allow_trajectory_execution"
                    ]),
                    "publish_monitored_planning_scene": bool(moveit_config[
                        "publish_monitored_planning_scene"
                    ]),
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
    visualization_config = system.get("visualization", {})
    if visualization_config.get("enabled", False):
        rviz_config_path = (
            Path(get_package_share_directory(
                visualization_config["package"]
            ))
            / visualization_config["rviz_config"]
        )
        actions.extend([
            LogInfo(msg=f"RViz config: {rviz_config_path}"),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                namespace=namespace,
                arguments=["-d", str(rviz_config_path)],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ])
    return LaunchDescription(actions)
