"""Run the ROS docstring style linter."""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257() -> None:
    """Check Python source for public API documentation."""
    assert main(argv=["."]) == 0
