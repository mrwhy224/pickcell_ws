"""Tests for centralized launch-time configuration resolution."""

from pathlib import Path

import pickcell_bringup.config_resolution as resolution_module
from pickcell_bringup.config_resolution import resolve_system_config


class FakeResolver:
    """Small ConfigResolver replacement for deterministic unit tests."""

    def __init__(self, mode: str, local_root: str | Path | None = None) -> None:
        del local_root
        self.mode = mode

    def load_section(
        self,
        section: str,
        profile: str | None = None,
        manifest_overrides: dict | None = None,
        explicit_override_file: str | Path | None = None,
    ) -> tuple[dict, list[Path]]:
        del explicit_override_file
        values = {
            f"{section}_server": {
                "ros__parameters": {
                    "profile": profile,
                    "section_value": section,
                }
            }
        }
        if manifest_overrides:
            values.update(manifest_overrides)
        return values, [Path(f"/{section}/{profile}.yaml.dist")]


def install_fake_resolver(monkeypatch) -> None:
    """Replace the installed resolver for one unit test."""
    monkeypatch.setattr(resolution_module, "ConfigResolver", FakeResolver)


def test_resolves_all_selected_sections_once(monkeypatch) -> None:
    """The system snapshot contains every explicitly selected profile."""
    install_fake_resolver(monkeypatch)
    resolved = resolve_system_config(
        "mock",
        profiles={"perception": "pose_injector", "motion": "ompl"},
    )
    assert resolved.mode == "mock"
    assert resolved.sections["perception"].profile == "pose_injector"
    assert resolved.sections["motion"].profile == "ompl"
    assert len(resolved.configuration_hash) == 64


def test_node_receives_resolved_and_system_parameters(monkeypatch) -> None:
    """Runtime nodes receive values, not a ConfigResolver Python object."""
    install_fake_resolver(monkeypatch)
    resolved = resolve_system_config("sim")
    parameters = resolved.parameters_for("perception", "perception_server")
    assert parameters["profile"] == "default"
    assert parameters["section_value"] == "perception"
    assert parameters["system_mode"] == "sim"
    assert parameters["use_sim_time"] is True
    assert parameters["configuration_hash"] == resolved.configuration_hash


def test_configuration_hash_is_deterministic(monkeypatch) -> None:
    """Equivalent effective configuration produces the same hash."""
    install_fake_resolver(monkeypatch)
    first = resolve_system_config("real", use_sim_time=False)
    second = resolve_system_config("real", use_sim_time=False)
    assert first.configuration_hash == second.configuration_hash


def test_missing_section_is_rejected(monkeypatch) -> None:
    """Requesting parameters for an unresolved section fails clearly."""
    install_fake_resolver(monkeypatch)
    resolved = resolve_system_config("mock")
    try:
        resolved.parameters_for("unknown", "node")
    except KeyError as error:
        assert "unknown" in str(error)
    else:
        raise AssertionError("missing configuration section was accepted")
