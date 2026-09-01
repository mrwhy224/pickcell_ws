"""Publish a safe motion-cycle waypoint contract for the selected bag."""

import math

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from .cycle import CartesianPoseABC
from .cycle import PickPlaceCycleConfig
from .cycle import PickPlaceCyclePlanner


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
    """Transform selected candidates and publish the complete waypoint order."""

    def __init__(self) -> None:
        super().__init__("motion_cycle")
        self.declare_parameter("planning_frame", "cell")
        self.declare_parameter(
            "selected_point_topic", "perception/selected_bag_point"
        )
        self.declare_parameter("cycle_path_topic", "planning/pick_place_cycle")
        self.declare_parameter(
            "camera_clear_home", [-0.55, 0.0, 1.20, 0.0, 180.0, 0.0]
        )
        self.declare_parameter(
            "transfer_waypoint", [-0.90, 0.0, 1.20, 0.0, 180.0, 0.0]
        )
        self.declare_parameter(
            "box_approach", [-1.40, 0.0, 1.05, 0.0, 180.0, 0.0]
        )
        self.declare_parameter(
            "box_drop", [-1.40, 0.0, 0.72, 0.0, 180.0, 0.0]
        )
        self.declare_parameter("pick_approach_offset_m", 0.20)
        self.declare_parameter("pick_retreat_offset_m", 0.30)

        self._planning_frame = str(
            self.get_parameter("planning_frame").value
        )
        cycle_config = PickPlaceCycleConfig(
            camera_clear_home=self._pose_parameter("camera_clear_home"),
            transfer_waypoint=self._pose_parameter("transfer_waypoint"),
            box_approach=self._pose_parameter("box_approach"),
            box_drop=self._pose_parameter("box_drop"),
            pick_approach_offset_m=float(self.get_parameter(
                "pick_approach_offset_m"
            ).value),
            pick_retreat_offset_m=float(self.get_parameter(
                "pick_retreat_offset_m"
            ).value),
        )
        self._planner = PickPlaceCyclePlanner(cycle_config)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._path_publisher = self.create_publisher(
            Path, str(self.get_parameter("cycle_path_topic").value), 1
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("selected_point_topic").value),
            self._selected_bag,
            1,
        )

    def _pose_parameter(self, name: str) -> CartesianPoseABC:
        return CartesianPoseABC.from_sequence(self.get_parameter(name).value)

    def _selected_bag(self, selected: PointStamped) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._planning_frame,
                selected.header.frame_id,
                rclpy.time.Time.from_msg(selected.header.stamp),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Cannot transform selected bag into planning frame: {error}"
            )
            return
        point = do_transform_point(selected, transform)
        bag = CartesianPoseABC(
            point.point.x, point.point.y, point.point.z,
            0.0, 180.0, 0.0,
        )
        steps = self._planner.build(bag)
        path = Path()
        path.header.frame_id = self._planning_frame
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [
            pose_message(self._planning_frame, step.target, path.header.stamp)
            for step in steps if step.target is not None
        ]
        self._path_publisher.publish(path)
        self.get_logger().info(
            "Published pick/place cycle: dynamic pick, fixed rear-box transfer"
        )


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
