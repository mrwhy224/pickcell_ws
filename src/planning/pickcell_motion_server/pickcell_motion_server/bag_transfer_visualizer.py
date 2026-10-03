"""Geometrically honest mock gripper and continuous bag visualization."""

import math
import time

from geometry_msgs.msg import PointStamped, Pose
from pickcell_interfaces.msg import ActiveBag, GripperCommand, GripperResult
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .drop_target import DropTarget


INITIAL_BAGS = (
    (1.375, -0.245, 0.230), (1.430, 0.250, 0.230),
    (1.418, -0.238, 0.395), (1.368, 0.242, 0.395),
    (1.365, -0.250, 0.560), (1.425, 0.235, 0.560),
    (1.432, -0.240, 0.725), (1.380, 0.248, 0.725),
    (1.415, 0.005, 0.890),
)


def quaternion_multiply(left, right):
    """Multiply XYZW quaternions."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def rotated_z_axis(quaternion):
    """Return the local +Z direction expressed by an XYZW quaternion."""
    x, y, z, w = quaternion
    return (
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    )


def pose_errors(actual: Pose, expected: Pose) -> tuple[float, float]:
    """Return position and suction-axis errors, ignoring irrelevant tool yaw."""
    position = math.dist(
        (actual.position.x, actual.position.y, actual.position.z),
        (expected.position.x, expected.position.y, expected.position.z),
    )
    actual_axis = rotated_z_axis((
        actual.orientation.x, actual.orientation.y,
        actual.orientation.z, actual.orientation.w,
    ))
    expected_axis = rotated_z_axis((
        expected.orientation.x, expected.orientation.y,
        expected.orientation.z, expected.orientation.w,
    ))
    dot = min(1.0, max(-1.0, sum(
        left * right for left, right in zip(actual_axis, expected_axis)
    )))
    return position, math.acos(dot)


class BagTransferVisualizer(Node):
    """Own simulated contact, attachment identity, and bag marker state."""

    def __init__(self) -> None:
        super().__init__("bag_transfer_visualizer")
        self._positions = list(INITIAL_BAGS)
        self._orientations = [(0.0, 0.0, 0.0, 1.0)] * len(INITIAL_BAGS)
        self._states = ["pallet"] * len(INITIAL_BAGS)
        self._active = None
        self._active_cycle_id = None
        self._active_surface = None
        self._active_grasp_pose = None
        self._held_offset = None
        self._held_orientation = None
        self._last_joint_state_monotonic = None
        self._last_command_id = 0
        self.declare_parameter("planning_frame", "cell")
        self.declare_parameter("drop_frame", "drop_box")
        self.declare_parameter("drop_bag_center_box", [0.0, 0.0, 0.63])
        self.declare_parameter("drop_xy_tolerance_m", 0.04)
        self.declare_parameter("drop_z_tolerance_m", 0.04)
        self.declare_parameter("visual_bag_half_height_m", 0.08)
        self.declare_parameter("bag_mapping_tolerance_m", 0.16)
        self.declare_parameter("grasp_position_tolerance_m", 0.025)
        self.declare_parameter("grasp_axis_tolerance_deg", 8.0)
        self.declare_parameter("contact_xy_tolerance_m", 0.10)
        self.declare_parameter("contact_z_tolerance_m", 0.03)
        self.declare_parameter("joint_state_timeout_seconds", 0.5)
        self._planning_frame = str(self.get_parameter("planning_frame").value)
        self._drop_frame = str(self.get_parameter("drop_frame").value)
        self._drop_center = tuple(float(value) for value in self.get_parameter(
            "drop_bag_center_box").value)
        self._bag_half_height = float(
            self.get_parameter("visual_bag_half_height_m").value
        )
        self._mapping_tolerance = float(
            self.get_parameter("bag_mapping_tolerance_m").value
        )
        self._position_tolerance = float(
            self.get_parameter("grasp_position_tolerance_m").value
        )
        self._axis_tolerance = math.radians(float(
            self.get_parameter("grasp_axis_tolerance_deg").value
        ))
        self._contact_xy_tolerance = float(
            self.get_parameter("contact_xy_tolerance_m").value
        )
        self._contact_z_tolerance = float(
            self.get_parameter("contact_z_tolerance_m").value
        )
        self._state_timeout = float(
            self.get_parameter("joint_state_timeout_seconds").value
        )
        self._drop_target = DropTarget.from_values(
            self._drop_center,
            float(self.get_parameter("drop_xy_tolerance_m").value),
            float(self.get_parameter("drop_z_tolerance_m").value),
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(MarkerArray, "gripper/bag_markers", 1)
        self._grasp_publisher = self.create_publisher(
            GripperResult, "gripper/grasp_result", 1
        )
        self._release_publisher = self.create_publisher(
            GripperResult, "gripper/release_result", 1
        )
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            ActiveBag, "planning/active_bag", self._target, latched
        )
        self.create_subscription(
            GripperCommand, "gripper/command", self._gripper, 1
        )
        self.create_subscription(JointState, "joint_states", self._joint_state, 10)
        self.create_timer(0.1, self._publish)

    def _joint_state(self, message: JointState) -> None:
        if message.name and len(message.name) == len(message.position):
            self._last_joint_state_monotonic = time.monotonic()

    def _target(self, target: ActiveBag) -> None:
        if self._active is not None or target.cycle_id <= 0:
            return
        available = [
            index for index, state in enumerate(self._states) if state == "pallet"
        ]
        if not available:
            self.get_logger().error("No visual pallet bag remains for accepted target")
            return
        intended_center = (
            target.surface_point.x,
            target.surface_point.y,
            target.surface_point.z - self._bag_half_height,
        )
        active = min(
            available,
            key=lambda index: math.dist(self._positions[index], intended_center),
        )
        distance = math.dist(self._positions[active], intended_center)
        if distance > self._mapping_tolerance:
            self.get_logger().error(
                "Cycle %s target has no unambiguous visual bag: nearest error %.3f m"
                % (target.cycle_id, distance)
            )
            return
        self._active = active
        self._active_cycle_id = target.cycle_id
        self._active_surface = (
            target.surface_point.x,
            target.surface_point.y,
            target.surface_point.z,
        )
        self._active_grasp_pose = target.grasp_pose
        self.get_logger().info(
            "Cycle %s locked visual bag %s with mapping error %.3f m" % (
                target.cycle_id, active + 1, distance
            )
        )

    def _gripper(self, command: GripperCommand) -> None:
        if command.command_id <= self._last_command_id:
            return
        self._last_command_id = command.command_id
        if command.close:
            self._close(command)
        else:
            self._open(command)

    def _close(self, command: GripperCommand) -> None:
        reason = self._close_rejection_reason(command)
        if reason:
            self.get_logger().error(f"Rejected grasp: {reason}")
            self._publish_result(self._grasp_publisher, command, False, reason)
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                "gripper_tcp", self._planning_frame, rclpy.time.Time()
            )
        except TransformException as error:
            reason = f"cannot preserve attachment transform: {error}"
            self._publish_result(self._grasp_publisher, command, False, reason)
            return
        center = PointStamped()
        center.header.frame_id = self._planning_frame
        center.point.x, center.point.y, center.point.z = self._positions[self._active]
        relative = do_transform_point(center, transform)
        self._held_offset = (
            relative.point.x, relative.point.y, relative.point.z
        )
        rotation = transform.transform.rotation
        self._held_orientation = (rotation.x, rotation.y, rotation.z, rotation.w)
        self._states[self._active] = "held"
        actual = self._actual_tcp_pose()
        position_error, axis_error = pose_errors(
            actual, command.expected_tcp_pose
        )
        self.get_logger().info(
            "Cycle %s attached visual bag %s: TCP error %.4f m, axis error %.2f deg"
            % (
                command.cycle_id,
                self._active + 1,
                position_error,
                math.degrees(axis_error),
            )
        )
        self._publish_result(self._grasp_publisher, command, True, "contact valid")

    def _close_rejection_reason(self, command: GripperCommand) -> str | None:
        if command.cycle_id != self._active_cycle_id or self._active is None:
            return "command does not own the accepted bag"
        if self._states[self._active] != "pallet":
            return "gripper is not empty or selected bag is unavailable"
        if not self._joint_state_is_fresh():
            return "executed joint state is missing or stale"
        if not self._poses_match(command.expected_tcp_pose, self._active_grasp_pose):
            return "command grasp pose differs from accepted grasp geometry"
        actual = self._actual_tcp_pose()
        if actual is None:
            return "actual TCP transform is unavailable"
        position_error, axis_error = pose_errors(actual, command.expected_tcp_pose)
        if position_error > self._position_tolerance:
            return f"TCP position error {position_error:.3f} m exceeds tolerance"
        if axis_error > self._axis_tolerance:
            return f"TCP approach-axis error {math.degrees(axis_error):.2f} deg"
        dx = actual.position.x - self._active_surface[0]
        dy = actual.position.y - self._active_surface[1]
        dz = actual.position.z - self._active_surface[2]
        if math.hypot(dx, dy) > self._contact_xy_tolerance:
            return "TCP is outside the selected bag contact region"
        if abs(dz) > self._contact_z_tolerance:
            return "TCP has not reached the selected bag surface"
        return None

    def _open(self, command: GripperCommand) -> None:
        if (
            command.cycle_id != self._active_cycle_id
            or self._active is None
            or self._states[self._active] != "held"
        ):
            self._publish_result(
                self._release_publisher, command, False,
                "release command does not own a held bag",
            )
            return
        if not self._joint_state_is_fresh():
            self._publish_result(
                self._release_publisher, command, False,
                "executed joint state is missing or stale",
            )
            return
        actual = self._actual_tcp_pose()
        if actual is None:
            self._publish_result(
                self._release_publisher, command, False,
                "actual TCP transform is unavailable",
            )
            return
        position_error, axis_error = pose_errors(actual, command.expected_tcp_pose)
        if position_error > self._position_tolerance or axis_error > self._axis_tolerance:
            self._publish_result(
                self._release_publisher, command, False,
                "actual TCP is outside the B_DROP release tolerance",
            )
            return
        positions = self._release_positions()
        if positions is None:
            self._publish_result(
                self._release_publisher, command, False,
                "carried bag transform is unavailable",
            )
            return
        if not self._drop_target.accepts(positions[0]):
            bag_in_box = positions[0]
            reason = (
                "carried bag is outside the receiving region at "
                f"({bag_in_box[0]:.3f}, {bag_in_box[1]:.3f}, "
                f"{bag_in_box[2]:.3f})"
            )
            self.get_logger().error(reason)
            self._publish_result(
                self._release_publisher, command, False, reason,
            )
            return
        bag_in_box, bag_in_cell, tcp_rotation = positions
        self._positions[self._active] = bag_in_cell
        self._orientations[self._active] = quaternion_multiply(
            tcp_rotation, self._held_orientation
        )
        self._states[self._active] = "box"
        self.get_logger().info(
            "Cycle %s delivered visual bag %s at box (%.3f, %.3f, %.3f)" % (
                command.cycle_id, self._active + 1, *bag_in_box
            )
        )
        self._publish_result(self._release_publisher, command, True, "released")
        self._active = None
        self._active_cycle_id = None
        self._active_surface = None
        self._active_grasp_pose = None
        self._held_offset = None
        self._held_orientation = None

    def _actual_tcp_pose(self) -> Pose | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._planning_frame, "gripper_tcp", rclpy.time.Time()
            )
        except TransformException:
            return None
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def _release_positions(self):
        point = PointStamped()
        point.header.frame_id = "gripper_tcp"
        point.point.x, point.point.y, point.point.z = self._held_offset
        try:
            box_transform = self._tf_buffer.lookup_transform(
                self._drop_frame, "gripper_tcp", rclpy.time.Time()
            )
            cell_transform = self._tf_buffer.lookup_transform(
                self._planning_frame, "gripper_tcp", rclpy.time.Time()
            )
        except TransformException as error:
            self.get_logger().error(f"Cannot validate release pose: {error}")
            return None
        bag_in_box = do_transform_point(point, box_transform).point
        bag_in_cell = do_transform_point(point, cell_transform).point
        rotation = cell_transform.transform.rotation
        return (
            (bag_in_box.x, bag_in_box.y, bag_in_box.z),
            (bag_in_cell.x, bag_in_cell.y, bag_in_cell.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _joint_state_is_fresh(self) -> bool:
        return (
            self._last_joint_state_monotonic is not None
            and time.monotonic() - self._last_joint_state_monotonic
            <= self._state_timeout
        )

    @staticmethod
    def _poses_match(left: Pose, right: Pose) -> bool:
        return all(abs(a - b) < 1e-6 for a, b in zip(
            (
                left.position.x, left.position.y, left.position.z,
                left.orientation.x, left.orientation.y,
                left.orientation.z, left.orientation.w,
            ),
            (
                right.position.x, right.position.y, right.position.z,
                right.orientation.x, right.orientation.y,
                right.orientation.z, right.orientation.w,
            ),
        ))

    def _publish_result(self, publisher, command, success, reason) -> None:
        result = GripperResult()
        result.header.frame_id = self._planning_frame
        result.header.stamp = self.get_clock().now().to_msg()
        result.cycle_id = command.cycle_id
        result.command_id = command.command_id
        result.success = success
        result.reason = reason
        publisher.publish(result)

    def _publish(self) -> None:
        messages = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for index, (state, position, orientation) in enumerate(zip(
            self._states, self._positions, self._orientations
        )):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = "gripper_tcp" if state == "held" else "cell"
            marker.ns = "chemical_bags"
            marker.id = index
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.mesh_resource = (
                "package://pickcell_description/meshes/chemical_bag.obj"
            )
            marker.mesh_use_embedded_materials = False
            if index == self._active:
                marker.color.r = marker.color.g = marker.color.b = 1.0
            else:
                marker.color.r, marker.color.g, marker.color.b = (0.82, 0.84, 0.78)
            marker.color.a = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            if state == "held":
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
                    self._held_offset
                )
                (
                    marker.pose.orientation.x,
                    marker.pose.orientation.y,
                    marker.pose.orientation.z,
                    marker.pose.orientation.w,
                ) = self._held_orientation
            else:
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
                (
                    marker.pose.orientation.x,
                    marker.pose.orientation.y,
                    marker.pose.orientation.z,
                    marker.pose.orientation.w,
                ) = orientation
            messages.markers.append(marker)
        self._publisher.publish(messages)


def main(args=None) -> None:
    """Run the stateful bag payload visualization and mock gripper."""
    rclpy.init(args=args)
    node = BagTransferVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
