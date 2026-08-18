"""Launch headless Isaac Sim to generate a randomized bag dataset."""

import hashlib
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
import yaml
import xacro


def launch_generator(context):
    """Expand the scene and construct the Isaac dataset process."""
    config_path = (
        Path(get_package_share_directory("pickcell_config"))
        / "config" / "application.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    system = config["system"]
    simulator = system["simulator"]
    robot = system["robot"]

    description_share = Path(
        get_package_share_directory("pickcell_description")
    )
    description_path = description_share / system["tf"]["xacro_file"]
    mappings = {
        key: str(value)
        for key, value in robot.get("xacro_arguments", {}).items()
    }
    robot_description = xacro.process_file(
        str(description_path), mappings=mappings
    ).toxml()
    description_hash = hashlib.sha256(
        robot_description.encode("utf-8")
    ).hexdigest()
    urdf_path = (
        Path(tempfile.gettempdir())
        / f"pickcell_dataset_description_{description_hash}.urdf"
    )
    urdf_path.write_text(robot_description, encoding="utf-8")

    isaac_root = Path(os.environ.get(
        "PICKCELL_ISAAC_SIM_ROOT", simulator["install_root"]
    ))
    isaac_script = (
        Path(get_package_share_directory("pickcell_isaac_sim"))
        / ".." / ".." / "lib" / "pickcell_isaac_sim"
        / "run_pickcell_scene.py"
    ).resolve()
    command = [
        str(isaac_root / "python.sh"),
        str(isaac_script),
        "--urdf", str(urdf_path),
        "--usd-dir", str(simulator["usd_directory"]),
        "--pickcell-description", str(description_share),
        "--kuka-description", str(Path(
            get_package_share_directory("kuka_quantec_support")
        )),
        "--headless",
        "--dataset-samples",
        LaunchConfiguration("sample_count").perform(context),
        "--dataset-output",
        LaunchConfiguration("output_directory").perform(context),
        "--dataset-seed", LaunchConfiguration("seed").perform(context),
        "--min-bags", LaunchConfiguration("min_bags").perform(context),
        "--max-bags", LaunchConfiguration("max_bags").perform(context),
    ]
    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description() -> LaunchDescription:
    """Declare generation controls and start Isaac Sim."""
    return LaunchDescription([
        DeclareLaunchArgument("sample_count", default_value="100"),
        DeclareLaunchArgument(
            "output_directory",
            default_value="/home/mahdi/pickcell_ws/datasets/bags_synthetic",
        ),
        DeclareLaunchArgument("seed", default_value="42"),
        DeclareLaunchArgument("min_bags", default_value="1"),
        DeclareLaunchArgument("max_bags", default_value="9"),
        OpaqueFunction(function=launch_generator),
    ])
