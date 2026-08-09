"""Resolve target TF, select an IK candidate, and apply it in simulation."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from .moveit_backend import MoveItIKBackend
from .optimizer import JointDistanceOptimizer
from .solver import InverseKinematicsSolver, JointConfiguration, JointLimit


class MotionSolverNode(Node):
    """Sample IK candidates and expose the best one as a simulated state."""

    def __init__(self) -> None:
        """Configure TF input, IK sampling, selection, and state output."""
        super().__init__("motion_solver")
        self._group = ReentrantCallbackGroup()
        self._declare_parameters()
        self._joint_names = tuple(self.get_parameter("joint_names").value)
        lower = tuple(self.get_parameter("joint_lower_limits").value)
        upper = tuple(self.get_parameter("joint_upper_limits").value)
        initial = tuple(
            math.radians(value)
            for value in self.get_parameter("initial_positions_deg").value
        )
        if not (
            len(self._joint_names)
            == len(lower)
            == len(upper)
            == len(initial)
        ):
            raise ValueError(
                "joint names, limits, and initial positions must align"
            )
        self._current = JointConfiguration.from_iterable(initial)
        limits = {
            name: JointLimit(lo, hi)
            for name, lo, hi in zip(self._joint_names, lower, upper)
        }
        seeds = self._make_seeds(lower, upper, self._current)
        client = self.create_client(
            GetPositionIK, "compute_ik", callback_group=self._group
        )
        backend = MoveItIKBackend(
            client,
            self._joint_names,
            seeds,
            group_name=str(self.get_parameter("group_name").value),
            end_effector_link=str(
                self.get_parameter("end_effector_link").value
            ),
            timeout_seconds=float(
                self.get_parameter("ik_timeout_seconds").value
            ),
            avoid_collisions=True,
        )
        self._solver = InverseKinematicsSolver(
            self._joint_names, limits, backend
        )
        self._optimizer = JointDistanceOptimizer(
            self.get_parameter("optimizer_weights").value
        )
        self._client = client
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._solved = False
        self._solving = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._selected_publisher = self.create_publisher(
            JointState,
            "planning/selected_joint_configuration",
            1,
        )
        self._state_subscription = self.create_subscription(
            JointState,
            "joint_states",
            self._receive_joint_state,
            10,
            callback_group=self._group,
        )
        self.create_timer(0.5, self._try_solve, callback_group=self._group)

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "joint_names",
            [f"joint_{index}" for index in range(1, 7)],
        )
        self.declare_parameter("joint_lower_limits", [0.0] * 6)
        self.declare_parameter("joint_upper_limits", [0.0] * 6)
        self.declare_parameter("initial_positions_deg", [0.0] * 6)
        self.declare_parameter("optimizer_weights", [1.0] * 6)
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("end_effector_link", "gripper_tcp")
        self.declare_parameter("base_frame", "robot_base")
        self.declare_parameter("target_frame", "motion_planning_target")
        self.declare_parameter("ik_timeout_seconds", 0.05)

    @staticmethod
    def _make_seeds(lower, upper, current):
        seeds = [current, JointConfiguration.from_iterable(
            (lo + hi) / 2.0 for lo, hi in zip(lower, upper)
        )]
        for index in range(len(current.positions)):
            for fraction in (0.1, 0.9):
                values = list(current.positions)
                values[index] = lower[index] + fraction * (
                    upper[index] - lower[index]
                )
                seeds.append(JointConfiguration.from_iterable(values))
        return tuple(seeds)

    def _try_solve(self) -> None:
        if (
            self._solved
            or self._solving
            or not self._client.service_is_ready()
        ):
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, self._target_frame, rclpy.time.Time()
            )
        except TransformException:
            return
        target = PoseStamped()
        target.header = transform.header
        target.header.frame_id = self._base_frame
        target.pose.position.x = transform.transform.translation.x
        target.pose.position.y = transform.transform.translation.y
        target.pose.position.z = transform.transform.translation.z
        target.pose.orientation = transform.transform.rotation
        self._solving = True
        try:
            candidates = self._solver.solve(target)
        finally:
            self._solving = False
        selected = self._optimizer.choose(candidates, self._current)
        if selected is None:
            self.get_logger().error(
                "No valid IK candidate found for target TF"
            )
            self._solved = True
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self._joint_names)
        message.position = list(selected.positions)
        self._selected_publisher.publish(message)
        self._solved = True
        values = ", ".join(
            f"{name}={value:.6f}"
            for name, value in zip(
                self._joint_names, selected.positions
            )
        )
        self.get_logger().info(
            f"Selected and applied simulated configuration: {values}"
        )

    def _receive_joint_state(self, message: JointState) -> None:
        """Track the current simulated state for seed and quality costs."""
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in self._joint_names):
            self._current = JointConfiguration.from_iterable(
                positions[name] for name in self._joint_names
            )


def main(args=None) -> None:
    """Run the motion solver with service callbacks on a second thread."""
    rclpy.init(args=args)
    node = MotionSolverNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
