"""Run the ROS Python style linter."""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8() -> None:
    """Check Python source for style and syntax errors."""
    return_code, errors = main_with_errors(argv=[])
    assert return_code == 0, "\n".join(errors)
