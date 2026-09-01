"""Tests for the safe pallet-to-box motion-cycle topology."""

import pytest

from pickcell_motion_server.cycle import CartesianPoseABC
from pickcell_motion_server.cycle import MotionType
from pickcell_motion_server.cycle import PickPlaceCycleConfig
from pickcell_motion_server.cycle import PickPlaceCyclePlanner


def config() -> PickPlaceCycleConfig:
    """Return representative taught points behind the robot."""
    return PickPlaceCycleConfig(
        camera_clear_home=CartesianPoseABC(-0.55, 0.0, 1.20, 0, 180, 0),
        transfer_waypoint=CartesianPoseABC(-0.90, 0.0, 1.20, 0, 180, 0),
        box_approach=CartesianPoseABC(-1.40, 0.0, 1.05, 0, 180, 0),
        box_drop=CartesianPoseABC(-1.40, 0.0, 0.72, 0, 180, 0),
    )


def test_dynamic_pick_precedes_fixed_box_transfer() -> None:
    """Plan only the bag-dependent leg before entering the taught path."""
    planner = PickPlaceCyclePlanner(config())
    bag = CartesianPoseABC(1.40, 0.20, 0.90, 0, 180, 0)

    steps = planner.build(bag)

    assert steps[1].name == "plan_to_pick_approach"
    assert steps[1].target.z == pytest.approx(1.10)
    assert steps[4].target.z == pytest.approx(1.20)
    assert steps[5].motion_type is MotionType.PLANNED
    assert all(step.reusable for step in steps[6:])


def test_fixed_return_ends_at_camera_clear_home() -> None:
    """Return through the transfer point to the safe starting posture."""
    planner = PickPlaceCyclePlanner(config())
    steps = planner.build(CartesianPoseABC(1.4, 0, 0.9, 0, 180, 0))

    assert steps[-2].target == config().transfer_waypoint
    assert steps[-1].target == config().camera_clear_home


def test_safe_points_must_be_on_rear_side() -> None:
    """Reject a home point that could enter the overhead camera workspace."""
    with pytest.raises(ValueError, match="behind the robot"):
        PickPlaceCycleConfig(
            camera_clear_home=CartesianPoseABC(0.5, 0, 1, 0, 180, 0),
            transfer_waypoint=CartesianPoseABC(-0.9, 0, 1, 0, 180, 0),
            box_approach=CartesianPoseABC(-1.4, 0, 1, 0, 180, 0),
            box_drop=CartesianPoseABC(-1.4, 0, 0.7, 0, 180, 0),
        )
