"""Plan and dispatch the startup and repeating pick/place waypoint motions."""

from collections import deque
import math
import time

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from nav_msgs.msg import Path
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty
from trajectory_msgs.msg import JointTrajectory

from .cycle import CartesianPoseABC
from .cycle_node import pose_message
from .moveit_backend import MoveItIKBackend
from .optimizer import JointDistanceOptimizer
from .solver import InverseKinematicsSolver, JointConfiguration, JointLimit


class TrajectoryExecutorNode(Node):
    """Plan collision-aware paths and send them to the demo trajectory player."""

    def __init__(self) -> None:
        super().__init__("trajectory_executor")
        self._group = ReentrantCallbackGroup()
        self._declare_parameters()
        self._joint_names = tuple(self.get_parameter("joint_names").value)
        lower = tuple(self.get_parameter("joint_lower_limits").value)
        upper = tuple(self.get_parameter("joint_upper_limits").value)
        initial = tuple(math.radians(value) for value in self.get_parameter(
            "initial_positions_deg"
        ).value)
        self._current = JointConfiguration.from_iterable(initial)
        limits = {
            name: JointLimit(lo, hi)
            for name, lo, hi in zip(self._joint_names, lower, upper)
        }
        self._ik_client = self.create_client(
            GetPositionIK, "compute_ik", callback_group=self._group
        )
        self._plan_client = self.create_client(
            GetMotionPlan, "plan_kinematic_path", callback_group=self._group
        )
        seeds = self._make_seeds(lower, upper, self._current)
        backend = MoveItIKBackend(
            self._ik_client, self._joint_names, seeds,
            group_name=str(self.get_parameter("group_name").value),
            end_effector_link=str(self.get_parameter(
                "end_effector_link"
            ).value),
            timeout_seconds=float(self.get_parameter(
                "ik_timeout_seconds"
            ).value),
            avoid_collisions=True,
        )
        self._solver = InverseKinematicsSolver(
            self._joint_names, limits, backend
        )
        self._optimizer = JointDistanceOptimizer(
            self.get_parameter("optimizer_weights").value
        )
        self._planning_frame = str(self.get_parameter("planning_frame").value)
        self._group_name = str(self.get_parameter("group_name").value)
        self._queue = deque([self._home_pose()])
        self._busy_until = 0.0
        self._executing_cycle = False
        self._home_reached = False
        self._trajectory_publisher = self.create_publisher(
            JointTrajectory, "planning/joint_trajectory", 1
        )
        self._complete_publisher = self.create_publisher(
            Empty, "planning/cycle_complete", 1
        )
        self._gripper_publisher = self.create_publisher(
            Bool, "gripper/closed", 1
        )
        self.create_subscription(
            JointState, "joint_states", self._joint_state, 10,
            callback_group=self._group,
        )
        self.create_subscription(
            Path, "planning/pick_place_cycle", self._cycle_path, 1,
            callback_group=self._group,
        )
        self.create_timer(0.2, self._advance, callback_group=self._group)

    def _declare_parameters(self) -> None:
        self.declare_parameter("joint_names", [f"joint_{i}" for i in range(1, 7)])
        self.declare_parameter("joint_lower_limits", [0.0] * 6)
        self.declare_parameter("joint_upper_limits", [0.0] * 6)
        self.declare_parameter("initial_positions_deg", [0.0] * 6)
        self.declare_parameter("optimizer_weights", [1.0] * 6)
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("end_effector_link", "gripper_tcp")
        self.declare_parameter("planning_frame", "cell")
        self.declare_parameter("ik_timeout_seconds", 0.1)
        self.declare_parameter(
            "camera_clear_home", [-1.4, 0.0, 1.05, 0.0, 180.0, 0.0]
        )

    @staticmethod
    def _make_seeds(lower, upper, current):
        """Cover alternate elbow/wrist branches for pallet and rear-box poses."""
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

    def _home_pose(self) -> PoseStamped:
        target = CartesianPoseABC.from_sequence(
            self.get_parameter("camera_clear_home").value
        )
        return pose_message(
            self._planning_frame, target, self.get_clock().now().to_msg()
        )

    def _joint_state(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        if all(name in values for name in self._joint_names):
            self._current = JointConfiguration.from_iterable(
                values[name] for name in self._joint_names
            )

    def _cycle_path(self, path: Path) -> None:
        if (
            not self._home_reached
            or len(self._queue) > 0
            or time.monotonic() < self._busy_until
        ):
            return
        # Path pose 2 is contact with the bag and pose 6 is contact with the
        # box. Keep the tool commands in the execution queue so they happen at
        # the correct points instead of being discarded by nav_msgs/Path.
        for index, pose in enumerate(path.poses[1:], start=1):
            self._queue.append(pose)
            if index == 2:
                self._queue.append("grip")
            elif index == 6:
                self._queue.append("release")
        self._executing_cycle = True
        self.get_logger().info("Accepted selected bag cycle")

    def _advance(self) -> None:
        if time.monotonic() < self._busy_until:
            return
        if not self._queue:
            if self._executing_cycle:
                self._complete_publisher.publish(Empty())
                self._executing_cycle = False
            elif not self._home_reached:
                self._home_reached = True
                self.get_logger().info(
                    "Camera-clear home reached; perception cycles enabled"
                )
            return
        if not (
            self._ik_client.service_is_ready()
            and self._plan_client.service_is_ready()
        ):
            return
        target = self._queue.popleft()
        if isinstance(target, str):
            closed = target == "grip"
            self._gripper_publisher.publish(Bool(data=closed))
            self.get_logger().info(
                "Gripper closed on selected bag" if closed
                else "Gripper opened above drop box"
            )
            return
        candidates = self._solver.solve(target)
        goal = self._optimizer.choose(candidates, self._current)
        if goal is None:
            position = target.pose.position
            self.get_logger().error(
                "No collision-free IK for waypoint at "
                f"({position.x:.3f}, {position.y:.3f}, {position.z:.3f})"
            )
            self._queue.clear()
            self._executing_cycle = False
            self._home_reached = False
            self._queue.append(self._home_pose())
            return
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = self._group_name
        motion.allowed_planning_time = 2.0
        motion.num_planning_attempts = 3
        motion.max_velocity_scaling_factor = 0.25
        motion.max_acceleration_scaling_factor = 0.20
        motion.start_state.joint_state.name = list(self._joint_names)
        motion.start_state.joint_state.position = list(self._current.positions)
        constraints = Constraints()
        constraints.joint_constraints = [
            JointConstraint(
                joint_name=name, position=position,
                tolerance_above=0.002, tolerance_below=0.002, weight=1.0,
            )
            for name, position in zip(self._joint_names, goal.positions)
        ]
        motion.goal_constraints = [constraints]
        response = self._plan_client.call(request).motion_plan_response
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("MoveIt failed to plan cycle waypoint")
            self._queue.clear()
            self._executing_cycle = False
            self._home_reached = False
            self._queue.append(self._home_pose())
            return
        trajectory = response.trajectory.joint_trajectory
        if not trajectory.points:
            self._queue.clear()
            return
        self._trajectory_publisher.publish(trajectory)
        duration = trajectory.points[-1].time_from_start
        self.get_logger().info(
            f"Executing planned waypoint with {len(trajectory.points)} trajectory points"
        )
        self._busy_until = time.monotonic() + duration.sec + (
            duration.nanosec / 1e9
        ) + 0.25


def main(args=None) -> None:
    """Run planning services and subscriptions concurrently."""
    rclpy.init(args=args)
    node = TrajectoryExecutorNode()
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
