from __future__ import annotations

import os
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

import yaml
from ament_index_python.packages import get_package_share_directory


VALID_MODES = {"mock", "sim", "real"}


def deep_merge(base: Mapping[str, Any], override: Mapping[str, Any], ) -> dict[str, Any]:
    """
    Recursively merge override into base.

    Dictionary values are merged recursively.
    Other values, including lists, are replaced completely.
    """
    result = deepcopy(dict(base))

    for key, override_value in override.items():
        base_value = result.get(key)

        if isinstance(base_value, Mapping) and isinstance(
            override_value, Mapping
        ):
            result[key] = deep_merge(base_value, override_value)
        else:
            result[key] = deepcopy(override_value)

    return result


def load_yaml_file(path: Path) -> dict[str, Any]:
    """Load a YAML file and require its root value to be a dictionary."""

    try:
        with path.open("r", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}
    except OSError as exc:
        raise ValueError(f"Cannot read configuration file '{path}': {exc}") from exc
    except yaml.YAMLError as exc:
        raise ValueError(f"Invalid YAML in '{path}': {exc}") from exc

    if not isinstance(data, dict):
        raise ValueError(
            f"Configuration root must be a dictionary: '{path}'"
        )

    return data


class ConfigResolver:
    def __init__(self, mode: str | None = None, package_name: str = "pickcell_config", local_root: str | Path | None = None, ) -> None:
        selected_mode = mode or os.getenv("PICKCELL_MODE")

        if not selected_mode:
            raise ValueError(
                "System mode is not set. Pass mode='mock', 'sim' or 'real', "
                "or define PICKCELL_MODE."
            )

        selected_mode = selected_mode.strip().lower()

        if selected_mode not in VALID_MODES:
            raise ValueError(
                f"Invalid mode '{selected_mode}'. "
                f"Expected one of: {sorted(VALID_MODES)}"
            )

        self.mode = selected_mode

        self.package_config_root = (
            Path(get_package_share_directory(package_name)) / "config"
        )

        configured_local_root = (
            local_root
            or os.getenv("PICKCELL_CONFIG_HOME")
            or Path.home() / ".config" / "pickcell"
        )

        self.local_root = Path(configured_local_root).expanduser()

    def load_section(self, section: str, profile: str | None = None, manifest_overrides: Mapping[str, Any] | None = None, explicit_override_file: str | Path | None = None, ) -> tuple[dict[str, Any], list[Path]]:
        """
        Load and merge configuration for one subsystem.

        Example:
            section='perception'
            profile='pcl_cluster_pose'
            mode='sim'
        """

        package_section = self.package_config_root / section
        local_section = self.local_root / section

        candidates: list[Path] = [
            # Tracked defaults
            package_section / "common.yaml.dist",
            package_section / "modes" / f"{self.mode}.yaml.dist",
        ]

        if profile:
            candidates.append(
                package_section / "profiles" / f"{profile}.yaml.dist"
            )

        candidates.extend([
            # Untracked/manual overrides
            local_section / "common.yaml",
            local_section / "modes" / f"{self.mode}.yaml",
        ])

        if profile:
            candidates.append(
                local_section / "profiles" / f"{profile}.yaml"
            )

        if explicit_override_file:
            candidates.append(
                Path(explicit_override_file).expanduser()
            )

        merged: dict[str, Any] = {}
        loaded_files: list[Path] = []

        for path in candidates:
            if not path.is_file():
                continue

            file_data = load_yaml_file(path)
            merged = deep_merge(merged, file_data)
            loaded_files.append(path)

        if manifest_overrides:
            merged = deep_merge(merged, manifest_overrides)

        if not loaded_files and not manifest_overrides:
            raise FileNotFoundError(
                f"No configuration found for section='{section}', "
                f"mode='{self.mode}', profile='{profile}'."
            )

        return merged, loaded_files

    def get(self, section: str, key_path: str, *, profile: str | None = None, default: Any = None, ) -> Any:
        config, _ = self.load_section(
            section=section,
            profile=profile,
        )

        value: Any = config

        for key in filter(None, key_path.split(".")):
            if not isinstance(value, Mapping) or key not in value:
                return default

            value = value[key]

        return value