"""Unit tests for deterministic mock point-cloud messages."""

import struct

from builtin_interfaces.msg import Time
import pytest

from pickcell_mock_nodes.point_cloud_mock import _triples
from pickcell_mock_nodes.point_cloud_mock import make_point_cloud
from pickcell_mock_nodes.point_cloud_mock import make_pose_stamped
from pickcell_mock_nodes.point_cloud_mock import make_target_transform


def test_pose_and_cloud_share_frame_and_stamp() -> None:
    """Path-planning inputs carry aligned metadata."""
    stamp = Time(sec=4, nanosec=5)
    pose = make_pose_stamped("cell", (1.0, 2.0, 3.0, 0, 0, 0), stamp)
    cloud = make_point_cloud("cell", ((1.0, 2.0, 3.0),), stamp)
    assert pose.header.frame_id == cloud.header.frame_id == "cell"
    assert pose.header.stamp == cloud.header.stamp
    assert pose.pose.position.x == 1.0
    assert pose.pose.orientation.w == 1.0
    assert cloud.width == 1
    assert struct.unpack("<fff", bytes(cloud.data)) == (1.0, 2.0, 3.0)


def test_kuka_a_angle_becomes_ros_yaw() -> None:
    """KUKA A is rotation about Z in the Z-Y-X ABC convention."""
    pose = make_pose_stamped("cell", (0, 0, 0, 90, 0, 0), Time())
    assert pose.pose.orientation.z == pytest.approx(2 ** -0.5)
    assert pose.pose.orientation.w == pytest.approx(2 ** -0.5)


def test_target_transform_matches_published_pose() -> None:
    """The target TF uses the pose frame, timestamp, and full transform."""
    stamp = Time(sec=4, nanosec=5)
    pose = make_pose_stamped("cell", (1, 2, 3, 90, 0, 0), stamp)
    transform = make_target_transform(pose, "motion_planning_target")

    assert transform.header.frame_id == "cell"
    assert transform.header.stamp == stamp
    assert transform.child_frame_id == "motion_planning_target"
    assert transform.transform.translation.x == 1.0
    assert transform.transform.translation.y == 2.0
    assert transform.transform.translation.z == 3.0
    assert transform.transform.rotation == pose.pose.orientation


def test_empty_target_frame_is_rejected() -> None:
    """TF requires a non-empty child frame name."""
    pose = make_pose_stamped("cell", (0, 0, 0, 0, 0, 0), Time())
    with pytest.raises(ValueError, match="target_frame_id"):
        make_target_transform(pose, "")


def test_flat_cloud_parameter_is_grouped() -> None:
    """The YAML-friendly flat coordinate parameter becomes XYZ triples."""
    assert _triples([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]) == (
        (1.0, 2.0, 3.0),
        (4.0, 5.0, 6.0),
    )


@pytest.mark.parametrize("values", [[], [1.0, 2.0], [1.0, 2.0, 3.0, 4.0]])
def test_invalid_cloud_parameter_is_rejected(values) -> None:
    """Malformed point arrays fail before a node starts publishing."""
    with pytest.raises(ValueError):
        _triples(values)
