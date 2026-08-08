"""Unit tests for displaying received planning target poses."""

import math

from geometry_msgs.msg import PoseStamped

from pickcell_motion_server.target_listener import describe_target


def test_describe_target_includes_frame_and_xyzabc() -> None:
    """A displayed target contains its frame, position, and orientation."""
    target = PoseStamped()
    target.header.frame_id = "cell"
    target.pose.position.x = 1.234
    target.pose.position.y = -0.25
    target.pose.position.z = 0.2
    target.pose.orientation.z = math.sqrt(0.5)
    target.pose.orientation.w = math.sqrt(0.5)

    assert describe_target(target) == (
        "Received planning target "
        "frame='cell' x=1.234 y=-0.250 z=0.200 "
        "a=90.000 b=0.000 c=0.000 deg"
    )
