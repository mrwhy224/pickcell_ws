"""Run the ROS Python style linter."""

from ament_flake8.main import main_with_errors
from pathlib import Path
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8() -> None:
    """Check Python source for style and syntax errors."""
    package_root = Path(__file__).resolve().parents[1]
    return_code, errors = main_with_errors(argv=[str(package_root)])
    assert return_code == 0, "\n".join(errors)
