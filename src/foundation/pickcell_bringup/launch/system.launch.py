"""Resolve configuration and assemble the top-level PickCell system."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from pickcell_bringup.config_resolution import DEFAULT_PROFILES
from pickcell_bringup.config_resolution import resolve_system_config


def _assemble(context: LaunchContext) -> list:
    """Resolve launch arguments once, then construct configured actions."""
    mode = LaunchConfiguration("mode").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time_text = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time = use_sim_time_text.lower() in {"1", "true", "yes", "on"}
    profiles = {
        section: LaunchConfiguration(f"{section}_profile").perform(context)
        for section in DEFAULT_PROFILES
    }
    resolved = resolve_system_config(
        mode,
        profiles=profiles,
        use_sim_time=use_sim_time,
    )

    share = Path(get_package_share_directory("pickcell_bringup"))
    foundation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / "launch" / "foundation.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": str(resolved.use_sim_time).lower(),
        }.items(),
    )
    selected = ", ".join(
        f"{name}={section.profile}"
        for name, section in sorted(resolved.sections.items())
    )
    return [
        LogInfo(msg=(
            f"PickCell mode={resolved.mode}; profiles: {selected}; "
            f"configuration_hash={resolved.configuration_hash}"
        )),
        foundation,
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare system selectors and defer resolution to launch execution."""
    arguments = [
        DeclareLaunchArgument(
            "mode",
            default_value="mock",
            choices=["mock", "sim", "real"],
        ),
        DeclareLaunchArgument("namespace", default_value="pickcell"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]
    arguments.extend(
        DeclareLaunchArgument(
            f"{section}_profile",
            default_value=profile,
        )
        for section, profile in DEFAULT_PROFILES.items()
    )
    return LaunchDescription([*arguments, OpaqueFunction(function=_assemble)])
