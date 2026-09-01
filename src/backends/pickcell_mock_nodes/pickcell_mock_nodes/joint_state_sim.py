"""Apply selected configurations to the simulated robot joint state."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


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
        self._trajectory = None
        self._trajectory_started = 0.0
        self._publisher = self.create_publisher(JointState, "joint_states", 10)
        self._subscription = self.create_subscription(
            JointState,
            "planning/selected_joint_configuration",
            self._apply,
            10,
        )
        self._trajectory_subscription = self.create_subscription(
            JointTrajectory,
            "planning/joint_trajectory",
            self._start_trajectory,
            1,
        )
        # Keep the fallback arm visible before Isaac starts publishing /clock.
        # Message stamps still use the node's ROS clock, but publication must
        # not stall merely because simulated time has not started yet.
        self._wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self._timer = self.create_timer(
            0.1, self._publish, clock=self._wall_clock
        )

    def _apply(self, message: JointState) -> None:
        selected = positions_in_order(message, self._joint_names)
        if selected is None:
            self.get_logger().error("Rejected incomplete selected configuration")
            return
        self._positions = selected
        self.get_logger().info("Applied selected configuration to simulation")

    def _publish(self) -> None:
        self._update_trajectory()
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self._joint_names)
        message.position = list(self._positions)
        self._publisher.publish(message)

    def _start_trajectory(self, message: JointTrajectory) -> None:
        if not message.points or tuple(message.joint_names) != self._joint_names:
            self.get_logger().error("Rejected incomplete joint trajectory")
            return
        self._trajectory = message
        self._trajectory_started = time.monotonic()

    def _update_trajectory(self) -> None:
        if self._trajectory is None:
            return
        elapsed = time.monotonic() - self._trajectory_started
        points = self._trajectory.points
        times = [
            point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            for point in points
        ]
        if elapsed >= times[-1]:
            self._positions = tuple(points[-1].positions)
            self._trajectory = None
            return
        upper = next(index for index, value in enumerate(times) if value >= elapsed)
        if upper == 0:
            self._positions = tuple(points[0].positions)
            return
        lower = upper - 1
        fraction = (elapsed - times[lower]) / (times[upper] - times[lower])
        self._positions = tuple(
            first + fraction * (second - first)
            for first, second in zip(
                points[lower].positions, points[upper].positions
            )
        )


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
