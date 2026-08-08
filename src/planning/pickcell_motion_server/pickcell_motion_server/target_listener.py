"""Receive and display the configured path-planning target."""

from __future__ import annotations

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


def describe_target(message: PointStamped) -> str:
    """Return a compact, deterministic description of a target point."""
    point = message.point
    return (
        "Received target position "
        f"frame='{message.header.frame_id}' "
        f"x={point.x:.3f} y={point.y:.3f} z={point.z:.3f}"
    )


class TargetListenerNode(Node):
    """Subscribe to the configured target topic and display each update."""

    def __init__(self) -> None:
        super().__init__("target_listener")
        self.declare_parameter("system_mode", Parameter.Type.STRING)
        self.declare_parameter("target_topic", Parameter.Type.STRING)
        mode = str(self.get_parameter("system_mode").value)
        target_topic = str(self.get_parameter("target_topic").value)
        self._target_point: PointStamped | None = None
        self._target_subscription = self.create_subscription(
            PointStamped,
            target_topic,
            self._target_callback,
            10,
        )
        self.get_logger().info(
            f"Planning mode='{mode}'; waiting for target positions on "
            f"'{target_topic}'"
        )

    def _target_callback(self, message: PointStamped) -> None:
        """Retain and report the most recent planning target."""
        self._target_point = message
        self.get_logger().info(describe_target(message))


def main(args: list[str] | None = None) -> None:
    """Run the target listener."""
    rclpy.init(args=args)
    node = TargetListenerNode()
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
