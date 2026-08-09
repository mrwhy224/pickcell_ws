"""Unit tests for inverse-kinematics candidate selection."""

import pytest

from pickcell_motion_server.optimizer import JointDistanceOptimizer
from pickcell_motion_server.solver import JointConfiguration


def test_optimizer_selects_lowest_weighted_joint_displacement() -> None:
    """Weights alter which valid candidate is considered best."""
    optimizer = JointDistanceOptimizer((10.0, 1.0))
    current = JointConfiguration((0.0, 0.0))
    candidates = (
        JointConfiguration((0.5, 0.0)),
        JointConfiguration((0.0, 1.0)),
    )

    assert optimizer.choose(candidates, current) == candidates[1]


def test_optimizer_returns_none_without_candidates() -> None:
    """An unreachable pose has no configuration to select."""
    optimizer = JointDistanceOptimizer((1.0,))

    assert optimizer.choose((), JointConfiguration((0.0,))) is None


@pytest.mark.parametrize("weights", [(), (-1.0,), (float("nan"),), (0.0,)])
def test_optimizer_rejects_invalid_weights(weights) -> None:
    """Quality weights must define a usable finite metric."""
    with pytest.raises(ValueError, match="weight"):
        JointDistanceOptimizer(weights)
