"""Visualize demo bags as payloads that attach to and detach from the TCP."""

import math

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


INITIAL_BAGS = (
    (1.375, -0.245, 0.230), (1.430, 0.250, 0.230),
    (1.418, -0.238, 0.395), (1.368, 0.242, 0.395),
    (1.365, -0.250, 0.560), (1.425, 0.235, 0.560),
    (1.432, -0.240, 0.725), (1.380, 0.248, 0.725),
    (1.415, 0.005, 0.890),
)


class BagTransferVisualizer(Node):
    """Maintain pallet, carried, and dropped states for each demo bag."""

    def __init__(self) -> None:
        super().__init__("bag_transfer_visualizer")
        self._positions = list(INITIAL_BAGS)
        self._states = ["pallet"] * len(INITIAL_BAGS)
        self._active = None
        self._drop_count = 0
        self._publisher = self.create_publisher(
            MarkerArray, "gripper/bag_markers", 1
        )
        self.create_subscription(
            PointStamped, "planning/active_bag_point", self._target, 1
        )
        self.create_subscription(Bool, "gripper/closed", self._gripper, 1)
        self.create_timer(0.1, self._publish)

    def _target(self, point: PointStamped) -> None:
        available = [
            index for index, state in enumerate(self._states)
            if state == "pallet"
        ]
        if not available:
            self._active = None
            return
        self._active = min(
            available,
            key=lambda index: math.dist(
                self._positions[index],
                (point.point.x, point.point.y, point.point.z - 0.08),
            ),
        )

    def _gripper(self, message: Bool) -> None:
        if self._active is None:
            return
        if message.data:
            self._states[self._active] = "held"
            return
        if self._states[self._active] != "held":
            return
        layer, slot = divmod(self._drop_count, 4)
        x = -1.55 + 0.30 * (slot // 2)
        y = -0.25 + 0.50 * (slot % 2)
        self._positions[self._active] = (x, y, 0.63 + 0.16 * layer)
        self._states[self._active] = "box"
        self._drop_count += 1
        self._active = None

    def _publish(self) -> None:
        messages = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for index, (state, position) in enumerate(zip(
            self._states, self._positions
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
            marker.color.r = 0.82
            marker.color.g = 0.84
            marker.color.b = 0.78
            marker.color.a = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            if state == "held":
                marker.pose.position.z = -0.08
            else:
                marker.pose.position.x = position[0]
                marker.pose.position.y = position[1]
                marker.pose.position.z = position[2]
            marker.pose.orientation.w = 1.0
            messages.markers.append(marker)
        self._publisher.publish(messages)


def main(args=None) -> None:
    """Run the stateful bag payload visualization."""
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
