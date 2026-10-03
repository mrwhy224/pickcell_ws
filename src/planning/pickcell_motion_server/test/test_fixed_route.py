"""Tests for canonical lateral sweep geometry and sampled validation."""

import math

import pytest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pickcell_motion_server.cycle import CartesianPoseABC
from pickcell_motion_server.fixed_route import build_joint_sweep_trajectory
from pickcell_motion_server.fixed_route import FixedTransferRoute
from pickcell_motion_server.fixed_route import same_joint_geometry
from pickcell_motion_server.fixed_route import validate_joint_trajectory
from pickcell_motion_server.fixed_route import validate_lateral_sweep


def pose(x, y, z):
    """Build a downward-facing Cartesian test pose."""
    return CartesianPoseABC(x, y, z, 0.0, 180.0, 0.0)


def route(direction="positive"):
    """Return a two-anchor lateral route contract."""
    return FixedTransferRoute(
        pick_anchor=pose(1.4, 0.0, 1.2),
        drop_anchor=pose(-1.4, 0.0, 0.72),
        base_yaw_joint="joint_1",
        base_yaw_direction=direction,
        minimum_base_yaw_sweep_rad=math.radians(150.0),
        maximum_other_joint_excursion_rad=math.radians(75.0),
        axial_wrist_joint="joint_6",
        maximum_axial_wrist_excursion_rad=math.radians(190.0),
        maximum_tcp_height_m=1.35,
    )


def sweep(samples=81):
    """Build a representative primarily base-yaw trajectory."""
    return build_joint_sweep_trajectory(
        (0.0, -0.8), (math.pi, -0.3), ("joint_1", "joint_2"),
        maximum_velocity=(0.35, 0.35),
        maximum_acceleration=(0.5, 0.5),
        maximum_jerk=(2.0, 2.0),
        sample_count=samples,
    )


def test_sweep_is_monotonic_and_within_dynamic_limits():
    """The canonical path rotates laterally without a base-yaw reversal."""
    trajectory = sweep()

    validate_lateral_sweep(trajectory, route())
    validate_joint_trajectory(
        trajectory,
        trajectory.joint_names,
        (-3.3, -2.0),
        (3.3, 1.0),
        maximum_velocity=(0.35, 0.35),
        maximum_acceleration=(0.5, 0.5),
        maximum_jerk=(2.0, 2.0),
    )


def test_return_geometry_matches_after_progress_resampling():
    """Comparison is geometric even when two trajectories sample differently."""
    assert same_joint_geometry(sweep(41), sweep(81), tolerance_rad=0.002)


def test_wrong_base_direction_and_large_arm_excursion_are_rejected():
    """An over-the-top branch cannot masquerade as the fixed lateral sweep."""
    with pytest.raises(ValueError, match="direction"):
        validate_lateral_sweep(sweep(), route("negative"))
    trajectory = sweep()
    trajectory.points[-1].positions[1] = 1.0
    with pytest.raises(ValueError, match="non-base"):
        validate_lateral_sweep(trajectory, route())


def test_full_route_validation_rejects_joint_limit_violation():
    """A valid endpoint cannot hide an out-of-limit swept sample."""
    result = JointTrajectory(joint_names=["joint_1", "joint_2"])
    for seconds, positions in ((0, (0.0, 0.0)), (1, (0.1, 3.0)), (2, (0.2, 0.2))):
        point = JointTrajectoryPoint(positions=list(positions))
        point.time_from_start.sec = seconds
        result.points.append(point)

    with pytest.raises(ValueError, match="joint limit"):
        validate_joint_trajectory(
            result, result.joint_names, (-1.0, -1.0), (1.0, 1.0),
            maximum_velocity=(5.0, 5.0),
            maximum_acceleration=(10.0, 10.0),
            maximum_jerk=(20.0, 20.0),
        )
