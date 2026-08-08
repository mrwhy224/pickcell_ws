"""Unit tests for displaying received planning targets."""

from geometry_msgs.msg import PointStamped

from pickcell_motion_server.target_listener import describe_target


def test_describe_target_includes_frame_and_xyz() -> None:
    """A displayed target contains its coordinate frame and position."""
    target = PointStamped()
    target.header.frame_id = "cell"
    target.point.x = 1.234
    target.point.y = -0.25
    target.point.z = 0.2

    assert describe_target(target) == (
        "Received target position "
        "frame='cell' x=1.234 y=-0.250 z=0.200"
    )
