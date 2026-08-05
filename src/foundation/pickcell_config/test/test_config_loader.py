from pathlib import Path
from typing import Any

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

import pickcell_config.config_loader as config_loader_module
from pickcell_config.config_loader import ConfigResolver


def write_yaml(path: Path, data: dict[str, Any]) -> None:
    """
    Create the parent directories and write a YAML configuration file.
    """

    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="utf-8") as file:
        yaml.safe_dump(
            data,
            file,
            sort_keys=False,
            allow_unicode=True,
        )


@pytest.fixture
def fake_config_environment(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> dict[str, Path]:
    """
    Create a complete temporary configuration environment.

    Structure:

    temporary_package_share/
    └── config/
        └── perception/
            ├── common.yaml.dist
            ├── modes/
            │   ├── mock.yaml.dist
            │   ├── sim.yaml.dist
            │   └── real.yaml.dist
            └── profiles/
                └── pcl_cluster_pose.yaml.dist

    temporary_local_config/
    └── perception/
        ├── common.yaml
        ├── modes/
        │   └── sim.yaml
        └── profiles/
            └── pcl_cluster_pose.yaml
    """

    fake_package_share = (
        tmp_path
        / "share"
        / "pickcell_config"
    )

    package_config_root = (
        fake_package_share
        / "config"
    )

    local_config_root = (
        tmp_path
        / "local_config"
    )

    # ---------------------------------------------------------
    # Replace ROS package lookup with our temporary directory.
    # ---------------------------------------------------------

    monkeypatch.setattr(
        config_loader_module,
        "get_package_share_directory",
        lambda package_name: str(fake_package_share),
    )

    # ---------------------------------------------------------
    # Tracked package defaults
    # ---------------------------------------------------------

    write_yaml(
        package_config_root
        / "perception"
        / "common.yaml.dist",
        {
            "perception_server": {
                "ros__parameters": {
                    "canonical_frame": "cell",
                    "input_timeout": 0.5,
                    "publish_debug": False,
                    "maximum_objects": 10,
                    "algorithm": {
                        "voxel_leaf_size": 0.005,
                        "cluster_tolerance": 0.01,
                    },
                }
            }
        },
    )

    write_yaml(
        package_config_root
        / "perception"
        / "modes"
        / "mock.yaml.dist",
        {
            "perception_server": {
                "ros__parameters": {
                    "backend": "generated_data",
                    "input_topic": "mock/points",
                    "mode_name": "mock",
                }
            }
        },
    )

    write_yaml(
        package_config_root
        / "perception"
        / "modes"
        / "sim.yaml.dist",
        {
            "perception_server": {
                "ros__parameters": {
                    "backend": "isaac_rgbd",
                    "input_topic": "sensors/front_camera/points",
                    "mode_name": "sim",
                }
            }
        },
    )

    write_yaml(
        package_config_root
        / "perception"
        / "modes"
        / "real.yaml.dist",
        {
            "perception_server": {
                "ros__parameters": {
                    "backend": "stereo_camera",
                    "input_topic": "sensors/stereo/points",
                    "mode_name": "real",
                }
            }
        },
    )

    write_yaml(
        package_config_root
        / "perception"
        / "profiles"
        / "pcl_cluster_pose.yaml.dist",
        {
            "perception_server": {
                "ros__parameters": {
                    "algorithm_plugin": (
                        "pickcell_perception/PclClusterPose"
                    ),
                    "algorithm": {
                        "remove_plane": True,
                        "plane_distance_threshold": 0.008,
                        "cluster_tolerance": 0.015,
                        "minimum_cluster_points": 100,
                    },
                }
            }
        },
    )

    # ---------------------------------------------------------
    # Local/manual overrides
    # ---------------------------------------------------------

    write_yaml(
        local_config_root
        / "perception"
        / "common.yaml",
        {
            "perception_server": {
                "ros__parameters": {
                    # Overrides common.yaml.dist
                    "input_timeout": 1.5,

                    # New local value
                    "developer_name": "mahdi",
                }
            }
        },
    )

    write_yaml(
        local_config_root
        / "perception"
        / "modes"
        / "sim.yaml",
        {
            "perception_server": {
                "ros__parameters": {
                    # Overrides the default false value
                    "publish_debug": True,

                    # Overrides sim.yaml.dist
                    "input_topic": "custom/simulated/points",
                }
            }
        },
    )

    write_yaml(
        local_config_root
        / "perception"
        / "profiles"
        / "pcl_cluster_pose.yaml",
        {
            "perception_server": {
                "ros__parameters": {
                    "algorithm": {
                        # Overrides profile default
                        "cluster_tolerance": 0.025,

                        # Adds a new value
                        "maximum_cluster_points": 50000,
                    }
                }
            }
        },
    )

    return {
        "fake_package_share": fake_package_share,
        "package_config_root": package_config_root,
        "local_config_root": local_config_root,
    }


def test_sim_mode_reads_and_merges_all_files(
    fake_config_environment: dict[str, Path],
) -> None:
    """
    Verify that common, sim, profile and local files are merged.
    """

    resolver = ConfigResolver(
        mode="sim",
        local_root=fake_config_environment["local_config_root"],
    )

    config, loaded_files = resolver.load_section(
        section="perception",
        profile="pcl_cluster_pose",
    )

    parameters = config[
        "perception_server"
    ][
        "ros__parameters"
    ]

    # ---------------------------------------------------------
    # Values inherited from common.yaml.dist
    # ---------------------------------------------------------

    assert parameters["canonical_frame"] == "cell"
    assert parameters["maximum_objects"] == 10

    # ---------------------------------------------------------
    # Values loaded from sim.yaml.dist
    # ---------------------------------------------------------

    assert parameters["backend"] == "isaac_rgbd"
    assert parameters["mode_name"] == "sim"

    # ---------------------------------------------------------
    # Values loaded from profile
    # ---------------------------------------------------------

    assert (
        parameters["algorithm_plugin"]
        == "pickcell_perception/PclClusterPose"
    )

    assert parameters["algorithm"]["remove_plane"] is True
    assert (
        parameters["algorithm"]["plane_distance_threshold"]
        == 0.008
    )

    # ---------------------------------------------------------
    # Values overridden by local files
    # ---------------------------------------------------------

    assert parameters["input_timeout"] == 1.5
    assert parameters["publish_debug"] is True
    assert parameters["input_topic"] == "custom/simulated/points"
    assert parameters["developer_name"] == "mahdi"

    assert (
        parameters["algorithm"]["cluster_tolerance"]
        == 0.025
    )

    assert (
        parameters["algorithm"]["maximum_cluster_points"]
        == 50000
    )

    # ---------------------------------------------------------
    # Verify that unrelated values were preserved after merge.
    # ---------------------------------------------------------

    assert (
        parameters["algorithm"]["voxel_leaf_size"]
        == 0.005
    )

    assert (
        parameters["algorithm"]["minimum_cluster_points"]
        == 100
    )

    print("\nLoaded configuration files:")

    for loaded_file in loaded_files:
        print(f"  - {loaded_file}")

    print("\nFinal merged parameters:")

    print(
        yaml.safe_dump(
            parameters,
            sort_keys=False,
            allow_unicode=True,
        )
    )


def test_files_are_loaded_in_expected_priority_order(
    fake_config_environment: dict[str, Path],
) -> None:
    """
    Verify that files are loaded from low priority to high priority.
    """

    resolver = ConfigResolver(
        mode="sim",
        local_root=fake_config_environment["local_config_root"],
    )

    _, loaded_files = resolver.load_section(
        section="perception",
        profile="pcl_cluster_pose",
    )

    loaded_paths = [
        str(path)
        for path in loaded_files
    ]

    assert loaded_paths[0].endswith(
        "perception/common.yaml.dist"
    )

    assert loaded_paths[1].endswith(
        "perception/modes/sim.yaml.dist"
    )

    assert loaded_paths[2].endswith(
        "perception/profiles/pcl_cluster_pose.yaml.dist"
    )

    assert loaded_paths[3].endswith(
        "perception/common.yaml"
    )

    assert loaded_paths[4].endswith(
        "perception/modes/sim.yaml"
    )

    assert loaded_paths[5].endswith(
        "perception/profiles/pcl_cluster_pose.yaml"
    )


def test_mock_mode_does_not_load_sim_or_real_values(
    fake_config_environment: dict[str, Path],
) -> None:
    """
    Verify that selecting mock only loads mock mode configuration.
    """

    resolver = ConfigResolver(
        mode="mock",
        local_root=fake_config_environment["local_config_root"],
    )

    config, loaded_files = resolver.load_section(
        section="perception",
        profile="pcl_cluster_pose",
    )

    parameters = config[
        "perception_server"
    ][
        "ros__parameters"
    ]

    assert parameters["mode_name"] == "mock"
    assert parameters["backend"] == "generated_data"
    assert parameters["input_topic"] == "mock/points"

    loaded_paths = [
        str(path)
        for path in loaded_files
    ]

    assert any(
        path.endswith("modes/mock.yaml.dist")
        for path in loaded_paths
    )

    assert not any(
        path.endswith("modes/sim.yaml.dist")
        for path in loaded_paths
    )

    assert not any(
        path.endswith("modes/real.yaml.dist")
        for path in loaded_paths
    )


def test_real_mode_reads_real_configuration(
    fake_config_environment: dict[str, Path],
) -> None:
    """
    Verify that real mode uses the real hardware configuration.
    """

    resolver = ConfigResolver(
        mode="real",
        local_root=fake_config_environment["local_config_root"],
    )

    config, _ = resolver.load_section(
        section="perception",
        profile="pcl_cluster_pose",
    )

    parameters = config[
        "perception_server"
    ][
        "ros__parameters"
    ]

    assert parameters["mode_name"] == "real"
    assert parameters["backend"] == "stereo_camera"
    assert parameters["input_topic"] == "sensors/stereo/points"


def test_get_reads_nested_value(
    fake_config_environment: dict[str, Path],
) -> None:
    """
    Verify that ConfigResolver.get() reads a dotted key path.
    """

    resolver = ConfigResolver(
        mode="sim",
        local_root=fake_config_environment["local_config_root"],
    )

    value = resolver.get(
        section="perception",
        profile="pcl_cluster_pose",
        key_path=(
            "perception_server."
            "ros__parameters."
            "algorithm."
            "cluster_tolerance"
        ),
        default=-1.0,
    )

    assert value == 0.025


def test_get_returns_default_for_missing_value(
    fake_config_environment: dict[str, Path],
) -> None:
    """
    Verify that a missing key returns the requested default.
    """

    resolver = ConfigResolver(
        mode="sim",
        local_root=fake_config_environment["local_config_root"],
    )

    value = resolver.get(
        section="perception",
        profile="pcl_cluster_pose",
        key_path=(
            "perception_server."
            "ros__parameters."
            "missing_parameter"
        ),
        default="not_found",
    )

    assert value == "not_found"


def test_manifest_override_has_highest_priority(
    fake_config_environment: dict[str, Path],
) -> None:
    """
    Verify that manifest overrides are applied after all files.
    """

    resolver = ConfigResolver(
        mode="sim",
        local_root=fake_config_environment["local_config_root"],
    )

    config, _ = resolver.load_section(
        section="perception",
        profile="pcl_cluster_pose",
        manifest_overrides={
            "perception_server": {
                "ros__parameters": {
                    "input_timeout": 9.0,
                    "publish_debug": False,
                    "algorithm": {
                        "cluster_tolerance": 0.1,
                    },
                }
            }
        },
    )

    parameters = config[
        "perception_server"
    ][
        "ros__parameters"
    ]

    assert parameters["input_timeout"] == 9.0
    assert parameters["publish_debug"] is False

    assert (
        parameters["algorithm"]["cluster_tolerance"]
        == 0.1
    )

    # Other values must remain available after the override.
    assert parameters["canonical_frame"] == "cell"
    assert parameters["backend"] == "isaac_rgbd"


@pytest.mark.parametrize(
    "mode",
    [
        "invalid",
        "simulation",
        "hardware",
        "",
    ],
)
def test_invalid_mode_is_rejected(mode: str) -> None:
    """
    Verify that unsupported mode names are rejected.
    """

    with pytest.raises(ValueError):
        ConfigResolver(mode=mode)


def test_missing_section_raises_file_not_found(
    fake_config_environment: dict[str, Path],
) -> None:
    """
    Verify that requesting a completely missing section fails clearly.
    """

    resolver = ConfigResolver(
        mode="sim",
        local_root=fake_config_environment["local_config_root"],
    )

    with pytest.raises(FileNotFoundError):
        resolver.load_section(
            section="unknown_section",
            profile="unknown_profile",
        )


def test_installed_config_files_exist() -> None:
    """
    Integration test:

    Verify that setup.py installs configuration resources into the
    ROS package share directory.
    """

    package_share = Path(
        get_package_share_directory("pickcell_config")
    )

    config_root = package_share / "config"

    assert package_share.is_dir()
    assert config_root.is_dir()

    expected_files = [
        config_root
        / "perception"
        / "common.yaml.dist",

        config_root
        / "perception"
        / "modes"
        / "mock.yaml.dist",

        config_root
        / "perception"
        / "modes"
        / "sim.yaml.dist",

        config_root
        / "perception"
        / "modes"
        / "real.yaml.dist",

        config_root
        / "perception"
        / "profiles"
        / "pcl_cluster_pose.yaml.dist",
    ]

    for expected_file in expected_files:
        assert expected_file.is_file(), (
            f"Installed configuration file was not found: "
            f"{expected_file}"
        )
