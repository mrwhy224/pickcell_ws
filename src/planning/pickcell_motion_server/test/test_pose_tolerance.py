"""Tests for verifying a configured initial state before skipping home motion."""

import math

from geometry_msgs.msg import Pose

from pickcell_motion_server.pose_tolerance import within_pose_tolerance


def pose(x=0.0, y=0.0, z=0.0, yaw=0.0) -> Pose:
    """Make a compact Z-axis test pose."""
    result = Pose()
    result.position.x, result.position.y, result.position.z = x, y, z
    result.orientation.z = math.sin(yaw / 2.0)
    result.orientation.w = math.cos(yaw / 2.0)
    return result


def test_initial_home_check_accepts_only_a_nearby_matching_pose() -> None:
    """A proven home pose can safely suppress the redundant startup motion."""
    assert within_pose_tolerance(
        pose(0.005, 0.0, 0.0, math.radians(1.0)), pose(),
        position_tolerance_m=0.01,
        orientation_tolerance_rad=math.radians(2.0),
    )


def test_initial_home_check_rejects_a_different_tcp_pose() -> None:
    """Do not treat a merely valid initial joint vector as camera-clear home."""
    assert not within_pose_tolerance(
        pose(0.02), pose(), position_tolerance_m=0.01,
        orientation_tolerance_rad=math.radians(2.0),
    )
