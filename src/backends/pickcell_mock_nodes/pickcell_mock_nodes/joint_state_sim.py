"""Apply selected configurations to the simulated robot joint state."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def positions_in_order(
    message: JointState, joint_names: tuple[str, ...],
) -> tuple[float, ...] | None:
    """Normalize a complete named joint state or reject an incomplete one."""
    positions = dict(zip(message.name, message.position))
    if not all(name in positions for name in joint_names):
        return None
    return tuple(float(positions[name]) for name in joint_names)


class JointStateSimulationNode(Node):
    """Publish initial state and apply solver selections without a controller."""

    def __init__(self) -> None:
        """Configure the selected-state input and simulated-state output."""
        super().__init__("joint_state_sim")
        self.declare_parameter(
            "joint_names", [f"joint_{index}" for index in range(1, 7)]
        )
        self.declare_parameter("initial_positions_deg", [0.0] * 6)
        self._joint_names = tuple(self.get_parameter("joint_names").value)
        initial = tuple(
            math.radians(float(value))
            for value in self.get_parameter("initial_positions_deg").value
        )
        if len(initial) != len(self._joint_names):
            raise ValueError("joint names and initial positions must align")
        self._positions = initial
        self._publisher = self.create_publisher(JointState, "joint_states", 10)
        self._subscription = self.create_subscription(
            JointState,
            "planning/selected_joint_configuration",
            self._apply,
            10,
        )
        self._timer = self.create_timer(0.1, self._publish)

    def _apply(self, message: JointState) -> None:
        selected = positions_in_order(message, self._joint_names)
        if selected is None:
            self.get_logger().error("Rejected incomplete selected configuration")
            return
        self._positions = selected
        self.get_logger().info("Applied selected configuration to simulation")

    def _publish(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self._joint_names)
        message.position = list(self._positions)
        self._publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    """Run the simulated joint-state applier."""
    rclpy.init(args=args)
    node = JointStateSimulationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
