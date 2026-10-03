"""Tests for geometric mock-gripper gates and task orientation checks."""

import math
import time

from geometry_msgs.msg import Pose
from pickcell_interfaces.msg import GripperCommand
import pytest

from pickcell_motion_server.bag_transfer_visualizer import BagTransferVisualizer
from pickcell_motion_server.bag_transfer_visualizer import pose_errors


def downward_pose(x, y, z):
    """Return a TCP pose whose local Z axis points down."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.y = 1.0
    return pose


def gated_node(actual_pose):
    """Create a callback-only visualizer with one accepted pallet bag."""
    node = BagTransferVisualizer.__new__(BagTransferVisualizer)
    node._active = 0
    node._active_cycle_id = 41
    node._states = ["pallet"]
    node._active_surface = (1.4, 0.0, 0.9)
    node._active_grasp_pose = downward_pose(1.4, 0.0, 0.9)
    node._last_joint_state_monotonic = time.monotonic()
    node._state_timeout = 0.5
    node._position_tolerance = 0.025
    node._axis_tolerance = math.radians(8.0)
    node._contact_xy_tolerance = 0.10
    node._contact_z_tolerance = 0.03
    node._actual_tcp_pose = lambda: actual_pose
    return node


def command_at_grasp():
    """Return the matching close command for the accepted cycle."""
    command = GripperCommand()
    command.cycle_id = 41
    command.command_id = 7
    command.close = True
    command.expected_tcp_pose = downward_pose(1.4, 0.0, 0.9)
    return command


def test_close_at_pick_anchor_cannot_attach_bag():
    """A_PICK arrival is not bag contact and must reject simulated grasp."""
    node = gated_node(downward_pose(1.4, 0.0, 1.2))

    reason = node._close_rejection_reason(command_at_grasp())

    assert reason is not None
    assert "position error" in reason


def test_close_at_matching_contact_pose_is_accepted():
    """Fresh executed geometry at the selected surface passes all gates."""
    node = gated_node(downward_pose(1.4, 0.0, 0.9))

    assert node._close_rejection_reason(command_at_grasp()) is None


def test_suction_axis_ignores_yaw_but_rejects_tilt():
    """The symmetric suction task constrains approach normal, not tool yaw."""
    expected = downward_pose(0.0, 0.0, 0.0)
    yawed = Pose()
    yawed.orientation.x = math.sqrt(0.5)
    yawed.orientation.y = math.sqrt(0.5)
    tilted = Pose()
    tilted.orientation.w = 1.0

    assert pose_errors(yawed, expected)[1] == 0.0
    assert pose_errors(tilted, expected)[1] == pytest.approx(math.pi)
