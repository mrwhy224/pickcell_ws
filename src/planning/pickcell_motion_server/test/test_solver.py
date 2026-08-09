"""Unit tests for inverse-kinematics solution enumeration."""

import math

from geometry_msgs.msg import PoseStamped
import pytest

from pickcell_motion_server.solver import InverseKinematicsSolver
from pickcell_motion_server.solver import JointConfiguration
from pickcell_motion_server.solver import JointLimit


class FakeBackend:
    """Return a fixed candidate set for solver unit tests."""

    def __init__(self, candidates):
        self.candidates = candidates

    def sample(self, target):
        """Return the configured candidates."""
        return self.candidates


def make_target() -> PoseStamped:
    """Build a valid target pose in the cell frame."""
    target = PoseStamped()
    target.header.frame_id = "cell"
    target.pose.orientation.w = 1.0
    return target


def test_solver_returns_all_unique_valid_configurations() -> None:
    """Remove malformed, nonfinite, duplicate, and out-of-limit results."""
    backend = FakeBackend([
        (0.1, 0.2),
        (0.1 + 1.0e-7, 0.2),
        (-0.5, 0.8),
        (2.0, 0.0),
        (math.nan, 0.0),
        (0.1,),
    ])
    solver = InverseKinematicsSolver(
        ("joint_1", "joint_2"),
        {
            "joint_1": JointLimit(-1.0, 1.0),
            "joint_2": JointLimit(-1.0, 1.0),
        },
        backend,
    )

    assert solver.solve(make_target()) == (
        JointConfiguration((0.1, 0.2)),
        JointConfiguration((-0.5, 0.8)),
    )


def test_solver_exposes_joint_names_for_optimizer_input() -> None:
    """A result can be converted to an explicit joint-name mapping."""
    solver = InverseKinematicsSolver(
        ("joint_1", "joint_2"),
        {
            "joint_1": JointLimit(-1.0, 1.0),
            "joint_2": JointLimit(-1.0, 1.0),
        },
        FakeBackend([]),
    )

    assert solver.as_joint_map(JointConfiguration((0.25, -0.5))) == {
        "joint_1": 0.25,
        "joint_2": -0.5,
    }


@pytest.mark.parametrize("frame_id", ["", None])
def test_solver_rejects_target_without_frame(frame_id) -> None:
    """IK targets must identify their coordinate frame."""
    target = make_target()
    target.header.frame_id = frame_id or ""
    solver = InverseKinematicsSolver(
        ("joint_1",),
        {"joint_1": JointLimit(-1.0, 1.0)},
        FakeBackend([]),
    )

    with pytest.raises(ValueError, match="frame_id"):
        solver.solve(target)


def test_solver_rejects_zero_orientation() -> None:
    """A zero quaternion does not define a target orientation."""
    target = make_target()
    target.pose.orientation.w = 0.0
    solver = InverseKinematicsSolver(
        ("joint_1",),
        {"joint_1": JointLimit(-1.0, 1.0)},
        FakeBackend([]),
    )

    with pytest.raises(ValueError, match="quaternion"):
        solver.solve(target)


@pytest.mark.parametrize("tolerance", [math.nan, math.inf, -1.0])
def test_solver_rejects_invalid_duplicate_tolerance(tolerance) -> None:
    """Deduplication requires a finite, nonnegative tolerance."""
    with pytest.raises(ValueError, match="duplicate tolerance"):
        InverseKinematicsSolver(
            ("joint_1",),
            {"joint_1": JointLimit(-1.0, 1.0)},
            FakeBackend([]),
            duplicate_tolerance=tolerance,
        )
