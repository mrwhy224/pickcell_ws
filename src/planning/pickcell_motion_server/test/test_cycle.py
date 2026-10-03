"""Tests for the two-anchor cycle and bag-specific local pick poses."""

import pytest

from pickcell_motion_server.cycle import CartesianPoseABC, MotionType
from pickcell_motion_server.cycle import PickPlaceCycleConfig, PickPlaceCyclePlanner


def config() -> PickPlaceCycleConfig:
    """Return representative pallet and box anchors."""
    return PickPlaceCycleConfig(
        pick_anchor=CartesianPoseABC(1.40, 0.0, 1.20, 0, 180, 0),
        drop_anchor=CartesianPoseABC(-1.40, 0.0, 0.72, 0, 180, 0),
    )


def test_local_pick_is_distinct_and_returns_to_exact_pick_anchor() -> None:
    """Hover, contact, and lift are temporary poses before fixed transfer."""
    steps = PickPlaceCyclePlanner(config()).build(
        CartesianPoseABC(1.415, 0.005, 0.97, 0, 180, 0)
    )

    assert [step.name for step in steps] == [
        "establish_pick_anchor",
        "plan_to_pick_approach",
        "descend_to_bag",
        "close_gripper",
        "lift_bag",
        "return_to_pick_anchor",
        "fixed_loaded_to_drop_anchor",
        "open_gripper",
        "fixed_return_to_pick_anchor",
    ]
    assert steps[1].target.z == pytest.approx(1.17)
    assert steps[2].target.z == pytest.approx(0.97)
    assert steps[4].target.z == pytest.approx(1.27)
    assert steps[5].target == config().pick_anchor
    assert steps[6].target == config().drop_anchor
    assert steps[8].target == config().pick_anchor
    assert steps[2].motion_type is MotionType.LINEAR


def test_only_two_reusable_pose_values_exist() -> None:
    """Temporary bag poses must not become persistent task anchors."""
    steps = PickPlaceCyclePlanner(config()).build(
        CartesianPoseABC(1.4, 0.2, 0.8, 0, 180, 0)
    )
    reusable = {step.target for step in steps if step.reusable and step.target}

    assert reusable == {config().pick_anchor, config().drop_anchor}


def test_anchors_must_be_distinct_and_on_opposite_sides() -> None:
    """Reject renamed legacy points that do not span pallet and box sides."""
    with pytest.raises(ValueError, match="opposite"):
        PickPlaceCycleConfig(
            pick_anchor=CartesianPoseABC(1.4, 0, 1.2, 0, 180, 0),
            drop_anchor=CartesianPoseABC(0.9, 0, 0.72, 0, 180, 0),
        )
