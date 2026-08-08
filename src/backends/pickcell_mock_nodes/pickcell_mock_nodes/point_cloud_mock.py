"""Publish deterministic point-cloud and path-planning target fixtures."""

from __future__ import annotations

from collections.abc import Sequence
import math
import struct

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, PointField


def _triples(values: Sequence[float]) -> tuple[tuple[float, float, float], ...]:
    """Convert a flat parameter list into XYZ triples."""
    if len(values) % 3:
        raise ValueError("cloud_points must contain a multiple of three values")
    points = tuple(
        (float(values[index]), float(values[index + 1]), float(values[index + 2]))
        for index in range(0, len(values), 3)
    )
    if not points:
        raise ValueError("cloud_points must contain at least one point")
    return points


def make_pose_stamped(
    frame_id: str, pose_abc: Sequence[float], stamp,
) -> PoseStamped:
    """Build a pose from XYZ and KUKA-style ABC angles in degrees."""
    if len(pose_abc) != 6:
        raise ValueError("target_pose must contain X Y Z A B C")
    x, y, z, a_deg, b_deg, c_deg = map(float, pose_abc)
    yaw = math.radians(a_deg)
    pitch = math.radians(b_deg)
    roll = math.radians(c_deg)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)

    message = PoseStamped()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.position.z = z
    message.pose.orientation.x = sr * cp * cy - cr * sp * sy
    message.pose.orientation.y = cr * sp * cy + sr * cp * sy
    message.pose.orientation.z = cr * cp * sy - sr * sp * cy
    message.pose.orientation.w = cr * cp * cy + sr * sp * sy
    return message


def make_point_cloud(
    frame_id: str, points: Sequence[Sequence[float]], stamp,
) -> PointCloud2:
    """Build an unorganized XYZ float32 point cloud."""
    normalized = tuple(tuple(map(float, point)) for point in points)
    if not normalized or any(len(point) != 3 for point in normalized):
        raise ValueError("points must contain at least one XYZ triple")
    message = PointCloud2()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.height = 1
    message.width = len(normalized)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.is_dense = True
    message.data = b"".join(
        struct.pack("<fff", *point) for point in normalized
    )
    return message


class PointCloudMockNode(Node):
    """Publish a repeatable cloud and one complete planning target pose."""

    def __init__(self) -> None:
        super().__init__("point_cloud_mock")
        self.declare_parameter("frame_id", Parameter.Type.STRING)
        self.declare_parameter("publish_rate_hz", Parameter.Type.DOUBLE)
        self.declare_parameter("cloud_points", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("target_pose", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("point_cloud_topic", Parameter.Type.STRING)
        self.declare_parameter("target_topic", Parameter.Type.STRING)

        frame_id = str(self.get_parameter("frame_id").value)
        cloud_points = _triples(self.get_parameter("cloud_points").value)
        target_pose = self.get_parameter("target_pose").value
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")

        self._frame_id = frame_id
        self._cloud_points = cloud_points
        self._target_pose = target_pose
        self._cloud_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("point_cloud_topic").value),
            10,
        )
        self._target_publisher = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("target_topic").value),
            10,
        )
        self._timer = self.create_timer(1.0 / rate, self._publish_fixture)
        self.get_logger().info(
            f"Publishing target XYZABC={tuple(map(float, target_pose))} "
            f"in frame '{frame_id}'"
        )

    def _publish_fixture(self) -> None:
        """Publish all related messages with one common timestamp."""
        stamp = self.get_clock().now().to_msg()
        self._cloud_publisher.publish(
            make_point_cloud(self._frame_id, self._cloud_points, stamp)
        )
        self._target_publisher.publish(
            make_pose_stamped(self._frame_id, self._target_pose, stamp)
        )


def main(args: list[str] | None = None) -> None:
    """Run the mock point-cloud node."""
    rclpy.init(args=args)
    node = PointCloudMockNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
