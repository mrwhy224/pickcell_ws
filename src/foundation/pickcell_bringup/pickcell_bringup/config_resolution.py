"""Resolve system configuration once and prepare parameters for ROS nodes."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from pickcell_config.config_loader import ConfigResolver


DEFAULT_PROFILES = {
    "perception": "default",
    "grasp": "default",
    "motion": "default",
    "tasks": "default",
    "execution": "default",
    "safety": "default",
    "qos": "default",
}


@dataclass(frozen=True)
class ResolvedSection:
    """Effective values and provenance for one configuration section."""

    name: str
    profile: str | None
    values: dict[str, Any]
    loaded_files: tuple[Path, ...]


@dataclass(frozen=True)
class ResolvedSystemConfig:
    """Immutable launch-time snapshot shared by node construction helpers."""

    mode: str
    use_sim_time: bool
    sections: dict[str, ResolvedSection]
    configuration_hash: str

    def parameters_for(self, section: str, node_name: str, ) -> dict[str, Any]:
        """Return effective ROS parameters for one node in one section."""
        if section not in self.sections:
            raise KeyError(f"Configuration section was not resolved: {section}")

        values = self.sections[section].values
        parameters: dict[str, Any] = {}

        wildcard = values.get("/**", {})
        if isinstance(wildcard, Mapping):
            wildcard_parameters = wildcard.get("ros__parameters", {})
            if isinstance(wildcard_parameters, Mapping):
                parameters.update(wildcard_parameters)

        node_values = values.get(node_name, {})
        if isinstance(node_values, Mapping):
            node_parameters = node_values.get("ros__parameters", {})
            if isinstance(node_parameters, Mapping):
                parameters.update(node_parameters)

        parameters.update({
            "system_mode": self.mode,
            "use_sim_time": self.use_sim_time,
            "configuration_hash": self.configuration_hash,
        })
        return parameters

    def as_dict(self) -> dict[str, Any]:
        """Return a serializable representation for logs and diagnostics."""
        return {
            "mode": self.mode,
            "use_sim_time": self.use_sim_time,
            "configuration_hash": self.configuration_hash,
            "sections": {
                name: {
                    "profile": section.profile,
                    "values": section.values,
                    "loaded_files": [str(path) for path in section.loaded_files],
                }
                for name, section in sorted(self.sections.items())
            },
        }


def _configuration_hash(mode: str, use_sim_time: bool, sections: Mapping[str, ResolvedSection], ) -> str:
    payload = {
        "mode": mode,
        "use_sim_time": use_sim_time,
        "sections": {
            name: section.values for name, section in sorted(sections.items())
        },
    }
    encoded = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        default=str,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def resolve_system_config(mode: str, *, profiles: Mapping[str, str | None] | None = None, use_sim_time: bool | None = None, local_root: str | Path | None = None, override_files: Mapping[str, str | Path] | None = None, manifest_overrides: Mapping[str, Mapping[str, Any]] | None = None, ) -> ResolvedSystemConfig:
    """Resolve selected sections through one ConfigResolver instance."""
    selected_profiles = dict(DEFAULT_PROFILES)
    if profiles:
        selected_profiles.update(profiles)

    resolver = ConfigResolver(mode=mode, local_root=local_root)
    effective_sim_time = mode == "sim" if use_sim_time is None else use_sim_time
    sections: dict[str, ResolvedSection] = {}

    for section_name, profile in selected_profiles.items():
        values, loaded_files = resolver.load_section(
            section=section_name,
            profile=profile or None,
            manifest_overrides=(manifest_overrides or {}).get(section_name),
            explicit_override_file=(override_files or {}).get(section_name),
        )
        sections[section_name] = ResolvedSection(
            name=section_name,
            profile=profile or None,
            values=values,
            loaded_files=tuple(loaded_files),
        )

    digest = _configuration_hash(mode, effective_sim_time, sections)
    return ResolvedSystemConfig(
        mode=resolver.mode,
        use_sim_time=effective_sim_time,
        sections=sections,
        configuration_hash=digest,
    )
