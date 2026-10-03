"""Accept one perception snapshot and publish a two-anchor cycle contract."""

import math

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from pickcell_interfaces.msg import ActiveBag, CycleResult, PickPlaceCycle
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .cycle import CartesianPoseABC, PickPlaceCycleConfig, PickPlaceCyclePlanner
from .cycle_identity import CycleGate


def pose_message(frame_id: str, target: CartesianPoseABC, stamp) -> PoseStamped:
    """Convert one XYZABC cycle target to a ROS pose."""
    yaw = math.radians(target.a_deg)
    pitch = math.radians(target.b_deg)
    roll = math.radians(target.c_deg)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.pose.position.x = target.x
    message.pose.position.y = target.y
    message.pose.position.z = target.z
    message.pose.orientation.x = sr * cp * cy - cr * sp * sy
    message.pose.orientation.y = cr * sp * cy + sr * cp * sy
    message.pose.orientation.z = cr * cp * sy - sr * sp * cy
    message.pose.orientation.w = cr * cp * cy + sr * sp * sy
    return message


class MotionCycleNode(Node):
    """Own selection acceptance and the immutable cycle identity."""

    def __init__(self) -> None:
        super().__init__("motion_cycle")
        self.declare_parameter("planning_frame", "cell")
        self.declare_parameter("selected_point_topic", "perception/selected_bag_point")
        self.declare_parameter("cycle_contract_topic", "planning/pick_place_cycle")
        self.declare_parameter("diagnostic_path_topic", "planning/active_motion_path")
        self.declare_parameter("active_bag_topic", "planning/active_bag")
        self.declare_parameter("cycle_result_topic", "planning/cycle_result")
        self.declare_parameter("task_anchor_topic", "planning/task_anchors")
        self.declare_parameter("pick_anchor", [1.40, 0.0, 1.20, 0.0, 180.0, 0.0])
        self.declare_parameter("drop_anchor", [-1.40, 0.0, 0.72, 0.0, 180.0, 0.0])
        self.declare_parameter("pick_approach_offset_m", 0.20)
        self.declare_parameter("pick_retreat_offset_m", 0.30)

        self._planning_frame = str(self.get_parameter("planning_frame").value)
        config = PickPlaceCycleConfig(
            pick_anchor=self._pose_parameter("pick_anchor"),
            drop_anchor=self._pose_parameter("drop_anchor"),
            pick_approach_offset_m=float(
                self.get_parameter("pick_approach_offset_m").value
            ),
            pick_retreat_offset_m=float(
                self.get_parameter("pick_retreat_offset_m").value
            ),
        )
        self._planner = PickPlaceCyclePlanner(config)
        self._cycle_gate = CycleGate()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._contract_publisher = self.create_publisher(
            PickPlaceCycle,
            str(self.get_parameter("cycle_contract_topic").value),
            latched,
        )
        self._diagnostic_path_publisher = self.create_publisher(
            Path, str(self.get_parameter("diagnostic_path_topic").value), 1
        )
        self._active_bag_publisher = self.create_publisher(
            ActiveBag, str(self.get_parameter("active_bag_topic").value), latched
        )
        self._anchor_publisher = self.create_publisher(
            MarkerArray, str(self.get_parameter("task_anchor_topic").value), latched
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("selected_point_topic").value),
            self._selected_bag,
            1,
        )
        self.create_subscription(
            CycleResult,
            str(self.get_parameter("cycle_result_topic").value),
            self._cycle_result,
            1,
        )
        self._publish_anchors()

    def _pose_parameter(self, name: str) -> CartesianPoseABC:
        return CartesianPoseABC.from_sequence(self.get_parameter(name).value)

    def _selected_bag(self, selected: PointStamped) -> None:
        if self._cycle_gate.active_id is not None:
            return
        source_stamp = selected.header.stamp
        source_frame = selected.header.frame_id
        try:
            transform = self._tf_buffer.lookup_transform(
                self._planning_frame,
                source_frame,
                rclpy.time.Time.from_msg(source_stamp),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Cannot transform selected bag into planning frame: {error}"
            )
            return
        point = do_transform_point(selected, transform)
        source_identity = (
            source_frame, int(source_stamp.sec), int(source_stamp.nanosec)
        )
        cycle_id = self._cycle_gate.begin(source_identity)
        if cycle_id is None:
            return
        accepted_stamp = self.get_clock().now().to_msg()
        bag = CartesianPoseABC(
            point.point.x, point.point.y, point.point.z, 0.0, 180.0, 0.0
        )
        steps = self._planner.build(bag)
        poses = [
            pose_message(self._planning_frame, step.target, accepted_stamp)
            for step in steps if step.target is not None
        ]

        active = ActiveBag()
        active.header.frame_id = self._planning_frame
        active.header.stamp = accepted_stamp
        active.cycle_id = cycle_id
        active.source_stamp = source_stamp
        active.source_frame = source_frame
        active.surface_point = point.point
        active.grasp_pose = poses[2].pose
        self._active_bag_publisher.publish(active)

        contract = PickPlaceCycle()
        contract.header = active.header
        contract.cycle_id = cycle_id
        contract.source_stamp = source_stamp
        contract.source_frame = source_frame
        contract.pick_anchor = poses[0].pose
        contract.hover_pose = poses[1].pose
        contract.grasp_pose = poses[2].pose
        contract.lift_pose = poses[3].pose
        contract.drop_anchor = poses[5].pose
        self._contract_publisher.publish(contract)

        diagnostic = Path()
        diagnostic.header = active.header
        diagnostic.poses = poses
        self._diagnostic_path_publisher.publish(diagnostic)
        self.get_logger().info(
            "Accepted cycle %s from source %s %s.%09d" % (
                cycle_id,
                source_frame,
                source_stamp.sec,
                source_stamp.nanosec,
            )
        )

    def _cycle_result(self, message: CycleResult) -> None:
        recoverable = message.success or (not message.payload_held)
        if not self._cycle_gate.accept_result(
            message.cycle_id, recoverable=recoverable
        ):
            return
        if message.success:
            self.get_logger().info(f"Cycle {message.cycle_id} completed")
        elif recoverable:
            self.get_logger().warning(
                f"Cycle {message.cycle_id} recovered after {message.phase}: "
                f"{message.reason}"
            )
        else:
            self.get_logger().error(
                f"Cycle {message.cycle_id} faulted with payload held: "
                f"{message.reason}"
            )

    def _publish_anchors(self) -> None:
        messages = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for marker_id, (name, pose, color) in enumerate((
            ("A_PICK", self._planner.config.pick_anchor, (0.1, 0.8, 0.2)),
            ("B_DROP", self._planner.config.drop_anchor, (0.1, 0.4, 1.0)),
        )):
            marker = Marker()
            marker.header.frame_id = self._planning_frame
            marker.header.stamp = stamp
            marker.ns = "task_anchors"
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = pose_message(
                self._planning_frame, pose, stamp
            ).pose
            marker.scale.x = marker.scale.y = 0.12
            marker.scale.z = 0.03
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 0.95
            marker.text = name
            messages.markers.append(marker)
        self._anchor_publisher.publish(messages)


def main(args=None) -> None:
    """Run the motion-cycle coordinator."""
    rclpy.init(args=args)
    node = MotionCycleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
