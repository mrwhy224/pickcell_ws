"""Publish deterministic point-cloud and path-planning target fixtures."""

from __future__ import annotations

import struct
from collections.abc import Sequence

from geometry_msgs.msg import PointStamped
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


def make_point_stamped(frame_id: str, point: Sequence[float], stamp, ) -> PointStamped:
    """Build a frame-stamped point with a caller-provided timestamp."""
    if len(point) != 3:
        raise ValueError("point must contain exactly three coordinates")
    message = PointStamped()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.point.x, message.point.y, message.point.z = map(float, point)
    return message


def make_point_cloud(frame_id: str, points: Sequence[Sequence[float]], stamp, ) -> PointCloud2:
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
    """Publish a repeatable cloud, final point, and target point."""

    def __init__(self) -> None:
        super().__init__("point_cloud_mock")
        self.declare_parameter("frame_id", Parameter.Type.STRING)
        self.declare_parameter("publish_rate_hz", Parameter.Type.DOUBLE)
        self.declare_parameter("cloud_points", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("final_point", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("target_point", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("point_cloud_topic", Parameter.Type.STRING)
        self.declare_parameter("final_point_topic", Parameter.Type.STRING)
        self.declare_parameter("target_topic", Parameter.Type.STRING)

        frame_id = str(self.get_parameter("frame_id").value)
        cloud_points = _triples(self.get_parameter("cloud_points").value)
        final_point = self.get_parameter("final_point").value
        target_point = self.get_parameter("target_point").value
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")

        self._frame_id = frame_id
        self._cloud_points = cloud_points
        self._final_point = final_point
        self._target_point = target_point
        self._cloud_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("point_cloud_topic").value),
            10,
        )
        self._final_publisher = self.create_publisher(
            PointStamped,
            str(self.get_parameter("final_point_topic").value),
            10,
        )
        self._target_publisher = self.create_publisher(
            PointStamped,
            str(self.get_parameter("target_topic").value),
            10,
        )
        self._timer = self.create_timer(1.0 / rate, self._publish_fixture)
        self.get_logger().info(
            f"Publishing target {tuple(map(float, target_point))} "
            f"in frame '{frame_id}'"
        )

    def _publish_fixture(self) -> None:
        """Publish all related messages with one common timestamp."""
        stamp = self.get_clock().now().to_msg()
        self._cloud_publisher.publish(make_point_cloud(self._frame_id, self._cloud_points, stamp))
        self._final_publisher.publish(make_point_stamped(self._frame_id, self._final_point, stamp))
        self._target_publisher.publish(
            make_point_stamped(self._frame_id, self._target_point, stamp)
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
