"""Execute a bag-specific local pick and a canonical two-anchor joint sweep."""

from collections import deque
from dataclasses import replace
import math
import time

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionFK
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from pickcell_interfaces.msg import CycleResult, GripperCommand, GripperResult
from pickcell_interfaces.msg import PickPlaceCycle
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt64
from trajectory_msgs.msg import JointTrajectory

from .cycle import CartesianPoseABC
from .cycle_node import pose_message
from .execution_state import CyclePhase, CycleState
from .fixed_route import build_joint_sweep_trajectory, FixedTransferRoute
from .fixed_route import same_joint_geometry, trajectory_duration_seconds
from .fixed_route import validate_joint_trajectory, validate_lateral_sweep
from .moveit_backend import MoveItIKBackend
from .optimizer import JointDistanceOptimizer
from .pose_tolerance import within_pose_tolerance
from .solver import InverseKinematicsSolver, JointConfiguration, JointLimit


class TrajectoryExecutorNode(Node):
    """Own all arm commands and enforce the two-anchor execution contract."""

    def __init__(self) -> None:
        super().__init__("trajectory_executor")
        self._client_group = ReentrantCallbackGroup()
        self._execution_group = MutuallyExclusiveCallbackGroup()
        self._declare_parameters()
        self._joint_names = tuple(self.get_parameter("joint_names").value)
        self._lower = tuple(float(value) for value in self.get_parameter(
            "joint_lower_limits").value)
        self._upper = tuple(float(value) for value in self.get_parameter(
            "joint_upper_limits").value)
        self._maximum_velocity = tuple(float(value) for value in self.get_parameter(
            "route_max_velocity_rad_s").value)
        self._maximum_acceleration = tuple(float(value) for value in self.get_parameter(
            "route_max_acceleration_rad_s2").value)
        self._maximum_jerk = tuple(float(value) for value in self.get_parameter(
            "route_max_jerk_rad_s3").value)
        vectors = (
            self._lower, self._upper, self._maximum_velocity,
            self._maximum_acceleration, self._maximum_jerk,
        )
        if not self._joint_names or not all(
            len(values) == len(self._joint_names) for values in vectors
        ):
            raise ValueError("all route limits must align with joint_names")
        if any(
            not math.isfinite(value) or value <= 0.0
            for values in vectors[2:] for value in values
        ):
            raise ValueError("route dynamic limits must be finite and positive")
        initial = tuple(math.radians(float(value)) for value in self.get_parameter(
            "initial_positions_deg").value)
        self._current = JointConfiguration.from_iterable(initial)
        self._last_joint_state_monotonic = None
        self._state_timeout = float(
            self.get_parameter("joint_state_timeout_seconds").value
        )
        self._anchor_tolerance = math.radians(float(
            self.get_parameter("route_anchor_tolerance_deg").value
        ))
        self._fixed_geometry_tolerance = float(
            self.get_parameter("fixed_route_joint_tolerance_rad").value
        )
        limits = {
            name: JointLimit(lo, hi)
            for name, lo, hi in zip(self._joint_names, self._lower, self._upper)
        }
        self._planning_frame = str(self.get_parameter("planning_frame").value)
        self._group_name = str(self.get_parameter("group_name").value)
        self._end_effector_link = str(
            self.get_parameter("end_effector_link").value
        )
        self._route = self._route_from_parameters()

        self._ik_client = self.create_client(
            GetPositionIK, "compute_ik", callback_group=self._client_group
        )
        self._plan_client = self.create_client(
            GetMotionPlan, "plan_kinematic_path", callback_group=self._client_group
        )
        self._cartesian_client = self.create_client(
            GetCartesianPath, "compute_cartesian_path",
            callback_group=self._client_group,
        )
        self._fk_client = self.create_client(
            GetPositionFK, "compute_fk", callback_group=self._client_group
        )
        self._state_validity_client = self.create_client(
            GetStateValidity, "check_state_validity",
            callback_group=self._client_group,
        )
        self._ik_backend = MoveItIKBackend(
            self._ik_client,
            self._joint_names,
            self._make_seeds(self._lower, self._upper, self._current),
            group_name=self._group_name,
            end_effector_link=self._end_effector_link,
            timeout_seconds=float(self.get_parameter("ik_timeout_seconds").value),
            avoid_collisions=True,
        )
        self._solver = InverseKinematicsSolver(
            self._joint_names, limits, self._ik_backend
        )
        self._optimizer = JointDistanceOptimizer(
            self.get_parameter("optimizer_weights").value
        )

        self._queue = deque([("planned", self._pick_anchor_pose().pose, None)])
        self._pick_joint_anchor = None
        self._drop_joint_anchor = None
        self._fixed_loaded_trajectory = None
        self._fixed_unloaded_trajectory = None
        self._busy_until = 0.0
        self._awaiting_completion = False
        self._awaiting_trajectory_id = None
        self._pending_arrival = None
        self._trajectory_command_id = 0
        self._gripper_command_id = 0
        self._pending_gripper_command_id = None
        self._grasp_deadline = None
        self._release_deadline = None
        self._active_contract = None
        self._active_cycle_id = None
        self._deferred_contract = None
        self._state = CycleState()
        self._executing_cycle = False
        self._pick_anchor_reached = False
        self._startup_anchor_checked = False
        self._payload_held = False
        self._delivered = False
        self._faulted = False

        self._trajectory_publisher = self.create_publisher(
            JointTrajectory, "planning/joint_trajectory", 1
        )
        self._result_publisher = self.create_publisher(
            CycleResult, "planning/cycle_result", 1
        )
        self._gripper_publisher = self.create_publisher(
            GripperCommand, "gripper/command", 1
        )
        self.create_subscription(
            JointState, "joint_states", self._joint_state, 10,
            callback_group=self._execution_group,
        )
        self.create_subscription(
            PickPlaceCycle, "planning/pick_place_cycle", self._cycle_contract, 1,
            callback_group=self._execution_group,
        )
        self.create_subscription(
            UInt64, "planning/trajectory_complete_id", self._trajectory_complete, 1,
            callback_group=self._execution_group,
        )
        self.create_subscription(
            GripperResult, "gripper/grasp_result", self._grasp_confirmed, 1,
            callback_group=self._execution_group,
        )
        self.create_subscription(
            GripperResult, "gripper/release_result", self._release_confirmed, 1,
            callback_group=self._execution_group,
        )
        self.create_timer(0.05, self._advance, callback_group=self._execution_group)

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
        self.declare_parameter("grasp_confirmation_timeout_seconds", 1.0)
        self.declare_parameter("release_confirmation_timeout_seconds", 1.0)
        self.declare_parameter("joint_state_timeout_seconds", 0.5)
        self.declare_parameter("skip_initial_anchor_if_at_pose", True)
        self.declare_parameter("initial_anchor_position_tolerance_m", 0.01)
        self.declare_parameter("initial_anchor_orientation_tolerance_deg", 2.0)
        self.declare_parameter("route_anchor_tolerance_deg", 0.5)
        self.declare_parameter("fixed_route_joint_tolerance_rad", 0.002)
        self.declare_parameter("route_max_velocity_rad_s", [0.35] * 6)
        self.declare_parameter("route_max_acceleration_rad_s2", [0.50] * 6)
        self.declare_parameter("route_max_jerk_rad_s3", [2.0] * 6)
        self.declare_parameter("route_sample_count", 81)
        self.declare_parameter("local_cartesian_step_m", 0.01)
        self.declare_parameter("pick_anchor", [1.4, 0.0, 1.20, 0.0, 180.0, 0.0])
        self.declare_parameter("drop_anchor", [-1.4, 0.0, 0.72, 0.0, 180.0, 0.0])
        self.declare_parameter("base_yaw_joint", "joint_1")
        self.declare_parameter("base_yaw_direction", "positive")
        self.declare_parameter("minimum_base_yaw_sweep_deg", 150.0)
        self.declare_parameter("maximum_other_joint_excursion_deg", 75.0)
        self.declare_parameter("axial_wrist_joint", "joint_6")
        self.declare_parameter("maximum_axial_wrist_excursion_deg", 190.0)
        self.declare_parameter("maximum_transfer_tcp_height_m", 1.35)

    def _route_from_parameters(self) -> FixedTransferRoute:
        def pose(name: str) -> CartesianPoseABC:
            return CartesianPoseABC.from_sequence(self.get_parameter(name).value)

        return FixedTransferRoute(
            pick_anchor=pose("pick_anchor"),
            drop_anchor=pose("drop_anchor"),
            base_yaw_joint=str(self.get_parameter("base_yaw_joint").value),
            base_yaw_direction=str(
                self.get_parameter("base_yaw_direction").value
            ),
            minimum_base_yaw_sweep_rad=math.radians(float(
                self.get_parameter("minimum_base_yaw_sweep_deg").value
            )),
            maximum_other_joint_excursion_rad=math.radians(float(
                self.get_parameter("maximum_other_joint_excursion_deg").value
            )),
            axial_wrist_joint=str(self.get_parameter("axial_wrist_joint").value),
            maximum_axial_wrist_excursion_rad=math.radians(float(
                self.get_parameter("maximum_axial_wrist_excursion_deg").value
            )),
            maximum_tcp_height_m=float(
                self.get_parameter("maximum_transfer_tcp_height_m").value
            ),
        )

    @staticmethod
    def _make_seeds(lower, upper, current):
        seeds = [current, JointConfiguration.from_iterable(
            (lo + hi) / 2.0 for lo, hi in zip(lower, upper)
        )]
        for index in range(len(current.positions)):
            for fraction in (0.1, 0.9):
                values = list(current.positions)
                values[index] = lower[index] + fraction * (upper[index] - lower[index])
                seeds.append(JointConfiguration.from_iterable(values))
        return tuple(seeds)

    def _pose(self, target: CartesianPoseABC) -> PoseStamped:
        return pose_message(
            self._planning_frame, target, self.get_clock().now().to_msg()
        )

    def _pose_stamped(self, pose: Pose) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = self._planning_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose = pose
        return message

    def _pick_anchor_pose(self) -> PoseStamped:
        return self._pose(self._route.pick_anchor)

    def _drop_anchor_pose(self) -> PoseStamped:
        return self._pose(self._route.drop_anchor)

    def _joint_state(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        if all(
            name in values and math.isfinite(values[name])
            for name in self._joint_names
        ):
            self._current = JointConfiguration.from_iterable(
                values[name] for name in self._joint_names
            )
            self._last_joint_state_monotonic = time.monotonic()

    def _trajectory_complete(self, message: UInt64) -> None:
        if (
            not self._awaiting_completion
            or message.data != self._awaiting_trajectory_id
        ):
            return
        self._awaiting_completion = False
        self._busy_until = 0.0
        if self._pending_arrival is not None:
            try:
                self._state.arrive(self._pending_arrival)
            except ValueError as error:
                self._fail(f"Invalid motion completion transition: {error}")
                return
        self._pending_arrival = None
        self._awaiting_trajectory_id = None

    def _grasp_confirmed(self, message: GripperResult) -> None:
        if not self._matches_gripper_result(message, CyclePhase.WAITING_FOR_GRASP):
            return
        self._grasp_deadline = None
        self._pending_gripper_command_id = None
        if not message.success:
            self._fail(f"Grasp rejected: {message.reason}")
            return
        try:
            self._state.confirm_grasp(True)
        except ValueError as error:
            self._fail(f"Invalid grasp result transition: {error}")
            return
        self._payload_held = True
        contract = self._active_contract
        self._queue.extend((
            ("linear", contract.lift_pose, CyclePhase.RETREATED),
            ("joint_goal", None, CyclePhase.AT_TRANSFER_START),
            ("loaded_route", None, CyclePhase.AT_DROP),
            ("release", None, None),
        ))

    def _release_confirmed(self, message: GripperResult) -> None:
        if not self._matches_gripper_result(message, CyclePhase.RELEASED):
            return
        self._release_deadline = None
        self._pending_gripper_command_id = None
        if not message.success:
            self._fail(f"Release rejected: {message.reason}")
            return
        self._payload_held = False
        self._delivered = True
        self._queue.append(("unloaded_route", None, CyclePhase.IDLE))

    def _matches_gripper_result(
        self, message: GripperResult, phase: CyclePhase,
    ) -> bool:
        return (
            self._state.phase is phase
            and self._active_cycle_id is not None
            and message.cycle_id == self._active_cycle_id
            and message.command_id == self._pending_gripper_command_id
        )

    def _cycle_contract(self, contract: PickPlaceCycle) -> None:
        if self._faulted or self._active_cycle_id is not None or self._deferred_contract:
            return
        if not self._valid_contract(contract):
            self.get_logger().error("Rejected malformed two-anchor cycle contract")
            return
        if not self._pick_anchor_reached:
            self._deferred_contract = contract
            self.get_logger().info(
                f"Deferred cycle {contract.cycle_id} until A_PICK is reached"
            )
            return
        self._start_cycle(contract)

    def _valid_contract(self, contract: PickPlaceCycle) -> bool:
        if contract.header.frame_id != self._planning_frame or contract.cycle_id <= 0:
            return False
        if not self._same_pose(contract.pick_anchor, self._route.pick_anchor):
            return False
        if not self._same_pose(contract.drop_anchor, self._route.drop_anchor):
            return False
        poses = (contract.hover_pose, contract.grasp_pose, contract.lift_pose)
        values = [
            value for pose in poses for value in (
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w,
            )
        ]
        if not all(math.isfinite(value) for value in values):
            return False
        grasp = contract.grasp_pose.position
        hover = contract.hover_pose.position
        lift = contract.lift_pose.position
        if not (
            abs(hover.x - grasp.x) < 1e-6
            and abs(hover.y - grasp.y) < 1e-6
            and hover.z >= grasp.z + 0.05
            and abs(lift.x - grasp.x) < 1e-6
            and abs(lift.y - grasp.y) < 1e-6
            and lift.z >= grasp.z + 0.05
        ):
            return False
        return math.dist(
            (grasp.x, grasp.y, grasp.z),
            (
                self._route.pick_anchor.x,
                self._route.pick_anchor.y,
                self._route.pick_anchor.z,
            ),
        ) >= 0.05

    def _start_cycle(self, contract: PickPlaceCycle) -> None:
        if not self._at_anchor(self._pick_joint_anchor):
            self.get_logger().error("Cannot start cycle away from A_PICK joint anchor")
            return
        self._active_cycle_id = contract.cycle_id
        self._active_contract = contract
        self._state = CycleState()
        self._payload_held = False
        self._delivered = False
        self._queue.extend((
            ("planned", contract.hover_pose, CyclePhase.PICK_APPROACH),
            ("linear", contract.grasp_pose, CyclePhase.AT_PICK),
            ("grip", None, None),
        ))
        self._executing_cycle = True
        self.get_logger().info(f"Started cycle {contract.cycle_id} at A_PICK")

    def _start_deferred_cycle(self) -> bool:
        if self._deferred_contract is None:
            return False
        contract = self._deferred_contract
        self._deferred_contract = None
        self._start_cycle(contract)
        return True

    def _advance(self) -> None:
        if self._faulted:
            return
        now = time.monotonic()
        if (
            self._state.phase is CyclePhase.WAITING_FOR_GRASP
            and self._grasp_deadline is not None and now > self._grasp_deadline
        ):
            self._fail("Grasp confirmation timed out", payload_unknown=True)
            return
        if (
            self._state.phase is CyclePhase.RELEASED
            and self._release_deadline is not None and now > self._release_deadline
        ):
            self._fail("Release confirmation timed out", payload_unknown=True)
            return
        if self._awaiting_completion:
            if now > self._busy_until:
                self._fail("Controller completion watchdog expired")
            return
        if not self._pick_anchor_reached and not self._startup_anchor_checked:
            if not self._state_is_fresh() or not self._fk_client.service_is_ready():
                return
            self._startup_anchor_checked = True
            if self._initial_state_is_pick_anchor():
                self._queue.clear()
                self._pick_joint_anchor = self._current
                self._pick_anchor_reached = True
                self._start_deferred_cycle()
                return
        if not self._queue:
            if self._executing_cycle:
                if (
                    self._state.phase is CyclePhase.IDLE
                    and not self._state.grasp_confirmed
                    and self._delivered
                    and self._at_anchor(self._pick_joint_anchor)
                ):
                    self._publish_result(True, "cycle complete")
                    self._reset_completed_cycle()
                else:
                    self._fail("Cycle ended outside returned-empty A_PICK state")
            elif not self._pick_anchor_reached:
                self._pick_anchor_reached = True
                self.get_logger().info("A_PICK reached; awaiting one selection")
                self._start_deferred_cycle()
            return
        if not self._state_is_fresh():
            self._fail("Joint state is missing or stale")
            return
        kind, target, arrival = self._queue.popleft()
        if kind == "grip":
            self._command_gripper(True)
        elif kind == "release":
            self._command_gripper(False)
        elif kind == "loaded_route":
            self._execute_loaded_route(arrival)
        elif kind == "unloaded_route":
            self._execute_unloaded_route(arrival)
        elif kind == "joint_goal":
            self._execute_joint_goal(self._pick_joint_anchor, arrival)
        elif kind == "linear":
            self._execute_linear(target, arrival)
        else:
            self._execute_planned(target, arrival)

    def _command_gripper(self, close: bool) -> None:
        try:
            if close:
                self._state.request_grasp()
                expected = self._active_contract.grasp_pose
            else:
                self._state.permit_release()
                expected = self._active_contract.drop_anchor
        except ValueError as error:
            self._fail(f"Rejected gripper command: {error}")
            return
        self._gripper_command_id += 1
        self._pending_gripper_command_id = self._gripper_command_id
        command = GripperCommand()
        command.header.frame_id = self._planning_frame
        command.header.stamp = self.get_clock().now().to_msg()
        command.cycle_id = self._active_cycle_id
        command.command_id = self._gripper_command_id
        command.close = close
        command.expected_tcp_pose = expected
        self._gripper_publisher.publish(command)
        timeout_name = (
            "grasp_confirmation_timeout_seconds" if close
            else "release_confirmation_timeout_seconds"
        )
        deadline = time.monotonic() + float(self.get_parameter(timeout_name).value)
        if close:
            self._grasp_deadline = deadline
        else:
            self._release_deadline = deadline

    def _state_is_fresh(self) -> bool:
        return (
            self._last_joint_state_monotonic is not None
            and time.monotonic() - self._last_joint_state_monotonic
            <= self._state_timeout
        )

    def _initial_state_is_pick_anchor(self) -> bool:
        if not bool(self.get_parameter("skip_initial_anchor_if_at_pose").value):
            return False
        pose = self._fk_pose(self._current.positions)
        return pose is not None and within_pose_tolerance(
            pose,
            self._pick_anchor_pose().pose,
            position_tolerance_m=float(self.get_parameter(
                "initial_anchor_position_tolerance_m").value),
            orientation_tolerance_rad=math.radians(float(self.get_parameter(
                "initial_anchor_orientation_tolerance_deg").value)),
        )

    def _execute_planned(self, target: Pose, arrival: CyclePhase | None) -> None:
        if not self._ik_client.service_is_ready() or not self._plan_client.service_is_ready():
            self._queue.appendleft(("planned", target, arrival))
            return
        stamped = self._pose_stamped(target)
        self._ik_backend.set_preferred_seed(self._current)
        goal = self._optimizer.choose(self._solver.solve(stamped), self._current)
        if goal is None:
            self._fail("No collision-free IK for planned target")
            return
        if self._same_pose(target, self._route.pick_anchor):
            self._pick_joint_anchor = goal
        self._execute_joint_goal(goal, arrival)

    def _execute_joint_goal(
        self, goal: JointConfiguration | None, arrival: CyclePhase | None,
    ) -> None:
        if goal is None:
            self._fail("Joint goal is not established")
            return
        if not self._plan_client.service_is_ready():
            self._queue.appendleft(("joint_goal", None, arrival))
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
        motion.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(
                joint_name=name,
                position=position,
                tolerance_above=0.002,
                tolerance_below=0.002,
                weight=1.0,
            )
            for name, position in zip(self._joint_names, goal.positions)
        ])]
        response = self._plan_client.call(request).motion_plan_response
        if (
            response.error_code.val != MoveItErrorCodes.SUCCESS
            or not response.trajectory.joint_trajectory.points
        ):
            self._fail("MoveIt rejected a planned joint leg")
            return
        self._publish_trajectory(response.trajectory.joint_trajectory, arrival)

    def _execute_linear(self, target: Pose, arrival: CyclePhase) -> None:
        if not self._cartesian_client.service_is_ready():
            self._queue.appendleft(("linear", target, arrival))
            return
        request = GetCartesianPath.Request()
        request.header.frame_id = self._planning_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state.joint_state.name = list(self._joint_names)
        request.start_state.joint_state.position = list(self._current.positions)
        request.group_name = self._group_name
        request.link_name = self._end_effector_link
        request.waypoints = [target]
        request.max_step = float(self.get_parameter("local_cartesian_step_m").value)
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        response = self._cartesian_client.call(request)
        if (
            response.error_code.val != MoveItErrorCodes.SUCCESS
            or response.fraction < 0.999999
            or not response.solution.joint_trajectory.points
        ):
            self._fail("MoveIt rejected the complete local Cartesian approach")
            return
        self._publish_trajectory(response.solution.joint_trajectory, arrival)

    def _execute_loaded_route(self, arrival: CyclePhase) -> None:
        try:
            self._state.permit_loaded_transfer()
        except ValueError as error:
            self._fail(f"Rejected loaded transfer: {error}")
            return
        if not self._at_anchor(self._pick_joint_anchor):
            self._fail("Loaded sweep requires exact A_PICK joint anchor")
            return
        if self._drop_joint_anchor is None:
            self._drop_joint_anchor = self._select_drop_branch()
            if self._drop_joint_anchor is None:
                self._fail("No compatible B_DROP IK branch for lateral sweep")
                return
        candidate = self._build_sweep(
            self._pick_joint_anchor, self._drop_joint_anchor, loaded=True
        )
        if candidate is None:
            return
        if self._fixed_loaded_trajectory is None:
            self._fixed_loaded_trajectory = candidate
        elif not same_joint_geometry(
            self._fixed_loaded_trajectory,
            candidate,
            tolerance_rad=self._fixed_geometry_tolerance,
        ):
            self._fail("Loaded sweep changed from canonical joint geometry")
            return
        self._publish_trajectory(candidate, arrival)

    def _execute_unloaded_route(self, arrival: CyclePhase) -> None:
        try:
            self._state.begin_return()
        except ValueError as error:
            self._fail(f"Rejected empty return: {error}")
            return
        if not self._at_anchor(self._drop_joint_anchor):
            self._fail("Empty return requires exact B_DROP joint anchor")
            return
        candidate = self._build_sweep(
            self._drop_joint_anchor, self._pick_joint_anchor, loaded=False
        )
        if candidate is None:
            return
        if self._fixed_unloaded_trajectory is None:
            self._fixed_unloaded_trajectory = candidate
        elif not same_joint_geometry(
            self._fixed_unloaded_trajectory,
            candidate,
            tolerance_rad=self._fixed_geometry_tolerance,
        ):
            self._fail("Empty return changed from canonical joint geometry")
            return
        if self._fixed_loaded_trajectory is not None:
            reverse = JointTrajectory(joint_names=list(self._joint_names))
            reverse.points = list(reversed(self._fixed_loaded_trajectory.points))
            if not same_joint_geometry(
                reverse, candidate, tolerance_rad=self._fixed_geometry_tolerance
            ):
                self._fail("Empty return is not the reverse loaded geometry")
                return
        self._publish_trajectory(candidate, arrival)

    def _select_drop_branch(self) -> JointConfiguration | None:
        base_index = self._joint_names.index(self._route.base_yaw_joint)
        direction = 1.0 if self._route.base_yaw_direction == "positive" else -1.0
        preferred = list(self._pick_joint_anchor.positions)
        preferred[base_index] += direction * math.pi
        preferred[base_index] = min(
            self._upper[base_index], max(self._lower[base_index], preferred[base_index])
        )
        reference = JointConfiguration.from_iterable(preferred)
        self._ik_backend.set_preferred_seed(reference)
        candidates = self._solver.solve(self._drop_anchor_pose())
        compatible = []
        for candidate in candidates:
            delta = candidate.positions[base_index] - self._pick_joint_anchor.positions[base_index]
            if direction * delta < self._route.minimum_base_yaw_sweep_rad:
                continue
            excursions_valid = True
            for index, (left, right) in enumerate(zip(
                self._pick_joint_anchor.positions, candidate.positions
            )):
                if index == base_index:
                    continue
                limit = self._route.maximum_other_joint_excursion_rad
                if self._joint_names[index] == self._route.axial_wrist_joint:
                    limit = self._route.maximum_axial_wrist_excursion_rad
                if abs(right - left) > limit:
                    excursions_valid = False
                    break
            if not excursions_valid:
                continue
            compatible.append(candidate)
        return self._optimizer.choose(tuple(compatible), reference)

    def _build_sweep(
        self,
        start: JointConfiguration,
        goal: JointConfiguration,
        *,
        loaded: bool,
    ) -> JointTrajectory | None:
        trajectory = build_joint_sweep_trajectory(
            start.positions,
            goal.positions,
            self._joint_names,
            maximum_velocity=self._maximum_velocity,
            maximum_acceleration=self._maximum_acceleration,
            maximum_jerk=self._maximum_jerk,
            sample_count=int(self.get_parameter("route_sample_count").value),
        )
        check_route = self._route
        if not loaded:
            check_route = replace(
                self._route,
                pick_anchor=self._route.drop_anchor,
                drop_anchor=self._route.pick_anchor,
                base_yaw_direction=(
                    "negative" if self._route.base_yaw_direction == "positive"
                    else "positive"
                ),
            )
        try:
            validate_joint_trajectory(
                trajectory,
                self._joint_names,
                self._lower,
                self._upper,
                maximum_velocity=self._maximum_velocity,
                maximum_acceleration=self._maximum_acceleration,
                maximum_jerk=self._maximum_jerk,
            )
            validate_lateral_sweep(trajectory, check_route)
        except ValueError as error:
            self._fail(f"Canonical lateral sweep validation failed: {error}")
            return None
        if not self._state_validity_client.service_is_ready():
            self._fail("MoveIt state-validity service is unavailable")
            return None
        peak_height = -math.inf
        for index, point in enumerate(trajectory.points):
            validity = GetStateValidity.Request()
            validity.group_name = self._group_name
            validity.robot_state.joint_state.name = list(self._joint_names)
            validity.robot_state.joint_state.position = list(point.positions)
            if not self._state_validity_client.call(validity).valid:
                self._fail(f"Lateral sweep collision at sample {index}")
                return None
            pose = self._fk_pose(point.positions)
            if pose is None:
                self._fail(f"FK failed for lateral sweep sample {index}")
                return None
            peak_height = max(peak_height, pose.position.z)
        if peak_height > self._route.maximum_tcp_height_m:
            self._fail(
                "Lateral sweep exceeds TCP height envelope: "
                f"{peak_height:.3f} m > {self._route.maximum_tcp_height_m:.3f} m"
            )
            return None
        self.get_logger().info(
            "%s lateral sweep validated: %d samples, peak TCP %.3f m; "
            "joint deltas deg [%s]" % (
                "Loaded" if loaded else "Empty",
                len(trajectory.points),
                peak_height,
                ", ".join(
                    f"{math.degrees(goal_value - start_value):.2f}"
                    for start_value, goal_value in zip(
                        start.positions, goal.positions
                    )
                ),
            )
        )
        return trajectory

    def _fk_pose(self, positions) -> Pose | None:
        if not self._fk_client.service_is_ready():
            return None
        request = GetPositionFK.Request()
        request.header.frame_id = self._planning_frame
        request.fk_link_names = [self._end_effector_link]
        request.robot_state.joint_state.name = list(self._joint_names)
        request.robot_state.joint_state.position = list(positions)
        response = self._fk_client.call(request)
        if (
            response.error_code.val != MoveItErrorCodes.SUCCESS
            or len(response.pose_stamped) != 1
        ):
            return None
        return response.pose_stamped[0].pose

    def _publish_trajectory(
        self, trajectory: JointTrajectory, arrival: CyclePhase | None,
    ) -> None:
        try:
            duration = trajectory_duration_seconds(trajectory)
        except ValueError as error:
            self._fail(f"Controller trajectory rejected: {error}")
            return
        self._trajectory_command_id += 1
        trajectory.header.frame_id = (
            f"pickcell_trajectory/{self._trajectory_command_id}"
        )
        self._trajectory_publisher.publish(trajectory)
        self._awaiting_completion = True
        self._awaiting_trajectory_id = self._trajectory_command_id
        self._pending_arrival = arrival
        self._busy_until = time.monotonic() + max(1.0, duration * 1.5 + 0.5)

    def _at_anchor(self, anchor: JointConfiguration | None) -> bool:
        return anchor is not None and all(
            abs(current - expected) <= self._anchor_tolerance
            for current, expected in zip(self._current.positions, anchor.positions)
        )

    def _same_pose(self, actual: Pose, expected: CartesianPoseABC) -> bool:
        intended = self._pose(expected).pose
        return all(abs(left - right) < 1e-6 for left, right in zip(
            (
                actual.position.x, actual.position.y, actual.position.z,
                actual.orientation.x, actual.orientation.y,
                actual.orientation.z, actual.orientation.w,
            ),
            (
                intended.position.x, intended.position.y, intended.position.z,
                intended.orientation.x, intended.orientation.y,
                intended.orientation.z, intended.orientation.w,
            ),
        ))

    def _publish_result(
        self, success: bool, reason: str, *, phase: str | None = None,
    ) -> None:
        if self._active_cycle_id is None:
            return
        result = CycleResult()
        result.header.frame_id = self._planning_frame
        result.header.stamp = self.get_clock().now().to_msg()
        result.cycle_id = self._active_cycle_id
        result.success = success
        result.delivered = self._delivered
        result.payload_held = self._payload_held
        result.phase = phase or self._state.phase.value
        result.reason = reason
        self._result_publisher.publish(result)

    def _reset_completed_cycle(self) -> None:
        self._executing_cycle = False
        self._active_contract = None
        self._active_cycle_id = None
        self._payload_held = False
        self._delivered = False
        self._state = CycleState()

    def _fail(self, reason: str, *, payload_unknown: bool = False) -> None:
        phase = self._state.phase.value
        self.get_logger().error(
            f"Cycle {self._active_cycle_id} failed in {phase}: {reason}"
        )
        self._queue.clear()
        self._awaiting_completion = False
        self._awaiting_trajectory_id = None
        self._pending_arrival = None
        self._grasp_deadline = None
        self._release_deadline = None
        self._pending_gripper_command_id = None
        if payload_unknown:
            self._payload_held = True
        had_active_cycle = self._active_cycle_id is not None
        self._state.fail()
        self._publish_result(False, reason, phase=phase)
        if self._payload_held:
            self._faulted = True
            self._executing_cycle = False
            return
        self._active_contract = None
        self._active_cycle_id = None
        self._executing_cycle = False
        self._pick_anchor_reached = False
        self._startup_anchor_checked = True
        self._state = CycleState()
        if not had_active_cycle:
            self._faulted = True
            self.get_logger().error(
                "A_PICK recovery failed; executor entered safe stop"
            )
            return
        if self._pick_joint_anchor is not None:
            self._queue.append(("joint_goal", None, None))
        else:
            self._queue.append(("planned", self._pick_anchor_pose().pose, None))


def main(args=None) -> None:
    """Run the serialized planner and mock-controller adapter."""
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
