"""Receive and display the configured path-planning target."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


def describe_target(message: PoseStamped) -> str:
    """Return the target as XYZ and KUKA-style ABC angles in degrees."""
    point = message.pose.position
    orientation = message.pose.orientation
    sin_c_cos_b = 2.0 * (
        orientation.w * orientation.x
        + orientation.y * orientation.z
    )
    cos_c_cos_b = 1.0 - 2.0 * (
        orientation.x ** 2 + orientation.y ** 2
    )
    c_deg = math.degrees(math.atan2(sin_c_cos_b, cos_c_cos_b))
    sin_b = 2.0 * (
        orientation.w * orientation.y
        - orientation.z * orientation.x
    )
    b_deg = math.degrees(math.asin(max(-1.0, min(1.0, sin_b))))
    sin_a_cos_b = 2.0 * (
        orientation.w * orientation.z
        + orientation.x * orientation.y
    )
    cos_a_cos_b = 1.0 - 2.0 * (
        orientation.y ** 2 + orientation.z ** 2
    )
    a_deg = math.degrees(math.atan2(sin_a_cos_b, cos_a_cos_b))
    return (
        "Received planning target "
        f"frame='{message.header.frame_id}' "
        f"x={point.x:.3f} y={point.y:.3f} z={point.z:.3f} "
        f"a={a_deg:.3f} b={b_deg:.3f} c={c_deg:.3f} deg"
    )


class TargetListenerNode(Node):
    """Subscribe to the configured target topic and display each update."""

    def __init__(self) -> None:
        super().__init__("target_listener")
        self.declare_parameter("system_mode", Parameter.Type.STRING)
        self.declare_parameter("target_topic", Parameter.Type.STRING)
        mode = str(self.get_parameter("system_mode").value)
        target_topic = str(self.get_parameter("target_topic").value)
        self._target_pose: PoseStamped | None = None
        self._target_subscription = self.create_subscription(
            PoseStamped,
            target_topic,
            self._target_callback,
            10,
        )
        self.get_logger().info(
            f"Planning mode='{mode}'; waiting for target poses on "
            f"'{target_topic}'"
        )

    def _target_callback(self, message: PoseStamped) -> None:
        """Retain and report the most recent planning target."""
        self._target_pose = message
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
