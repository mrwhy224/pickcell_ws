"""Apply selected configurations to the simulated robot joint state."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt64
from trajectory_msgs.msg import JointTrajectory


class JointStateSimulationNode(Node):
    """Publish initial state and interpolate the sole demo trajectory topic."""

    def __init__(self) -> None:
        """Configure the demo trajectory input and simulated-state output."""
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
        self._active_command_id = None
        self._publisher = self.create_publisher(JointState, "joint_states", 10)
        self._complete_publisher = self.create_publisher(
            UInt64, "planning/trajectory_complete_id", 1
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

    def _publish(self) -> None:
        self._update_trajectory()
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self._joint_names)
        message.position = list(self._positions)
        self._publisher.publish(message)

    def _start_trajectory(self, message: JointTrajectory) -> None:
        if (
            not message.points
            or tuple(message.joint_names) != self._joint_names
            or not self._valid_trajectory(message)
        ):
            self.get_logger().error("Rejected incomplete joint trajectory")
            return
        self._trajectory = message
        self._trajectory_started = time.monotonic()
        self._active_command_id = self._command_id(message)

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
            self._complete_publisher.publish(UInt64(
                data=self._active_command_id
            ))
            self._active_command_id = None
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

    def _valid_trajectory(self, message: JointTrajectory) -> bool:
        """Reject malformed timing or an externally injected incomplete path."""
        if self._command_id(message) is None:
            return False
        previous = -1.0
        for point in message.points:
            current = (
                point.time_from_start.sec
                + point.time_from_start.nanosec / 1e9
            )
            if (
                len(point.positions) != len(self._joint_names)
                or not all(math.isfinite(value) for value in point.positions)
                or current <= previous
            ):
                return False
            previous = current
        return True

    @staticmethod
    def _command_id(message: JointTrajectory) -> int | None:
        """Read the executor command ID carried in the trajectory header."""
        prefix = "pickcell_trajectory/"
        frame_id = message.header.frame_id
        if not frame_id.startswith(prefix):
            return None
        suffix = frame_id[len(prefix):]
        if not suffix.isdecimal() or int(suffix) <= 0:
            return None
        return int(suffix)


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
