"""Run the ROS docstring style linter."""

from ament_pep257.main import main
from pathlib import Path
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257() -> None:
    """Check Python source for public API documentation."""
    package_root = Path(__file__).resolve().parents[1]
    assert main(argv=[str(package_root)]) == 0
