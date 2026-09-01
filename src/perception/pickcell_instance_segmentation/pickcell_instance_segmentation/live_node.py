"""ROS 2 node publishing live candidate segments and the next pick target."""

import time

from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Empty

from .models import CameraModel
from .partition import connected_instance_labels
from .pipeline import process_rgbd_frame
from .ros_cloud import segment_cloud_message
from .selection import describe_segments
from .selection import UpperRightSegmentSelector
from .selection import UpperRightSelectorConfig


def _colorize(labels: np.ndarray, color: np.ndarray) -> np.ndarray:
    """Blend deterministic label colors over a live RGB image."""
    ids = labels.astype(np.uint32)
    palette = np.stack((
        (ids * 37 + 31) % 255,
        (ids * 73 + 67) % 255,
        (ids * 109 + 101) % 255,
    ), axis=2).astype(np.uint8)
    overlay = cv2.addWeighted(color, 0.35, palette, 0.65, 0.0)
    overlay[labels <= 0] = color[labels <= 0]
    boundaries = np.zeros(labels.shape, dtype=bool)
    boundaries[:, 1:] |= labels[:, 1:] != labels[:, :-1]
    boundaries[1:, :] |= labels[1:, :] != labels[:-1, :]
    overlay[boundaries] = (255, 255, 255)
    return overlay


class LiveInstanceSegmentation(Node):
    """Synchronize Isaac/real RGB-D messages and publish debug overlays."""

    def __init__(self) -> None:
        super().__init__("live_instance_segmentation")
        self.declare_parameter(
            "rgb_topic", "sensors/overhead_depth/color/image_raw"
        )
        self.declare_parameter(
            "depth_topic", "sensors/overhead_depth/depth/image_raw"
        )
        self.declare_parameter(
            "camera_info_topic", "sensors/overhead_depth/camera_info"
        )
        self.declare_parameter("patch_overlay_topic", "perception/patch_overlay")
        self.declare_parameter(
            "instance_overlay_topic", "perception/candidate_bag_overlay"
        )
        self.declare_parameter(
            "segmented_cloud_topic", "perception/candidate_bag_cloud"
        )
        self.declare_parameter(
            "selected_cloud_topic", "perception/selected_bag_cloud"
        )
        self.declare_parameter(
            "selected_point_topic", "perception/selected_bag_point"
        )
        self.declare_parameter("maximum_processing_rate_hz", 2.0)
        self.declare_parameter("minimum_candidate_pixels", 100)
        self.declare_parameter("same_height_tolerance_m", 0.02)
        self.declare_parameter("top_depth_percentile", 10.0)
        self.declare_parameter("cycle_complete_topic", "planning/cycle_complete")
        self.declare_parameter("picked_exclusion_radius_m", 0.18)
        self.declare_parameter("synchronization_slop_seconds", 0.05)
        self.declare_parameter("input_is_rectified", True)

        self._bridge = CvBridge()
        self._camera_info = None
        self._last_started = 0.0
        self._pending_pick = None
        self._picked_points = []
        self._patch_publisher = self.create_publisher(
            Image, self.get_parameter("patch_overlay_topic").value, 1
        )
        self._instance_publisher = self.create_publisher(
            Image, self.get_parameter("instance_overlay_topic").value, 1
        )
        self._segmented_cloud_publisher = self.create_publisher(
            PointCloud2,
            self.get_parameter("segmented_cloud_topic").value,
            1,
        )
        self._selected_cloud_publisher = self.create_publisher(
            PointCloud2,
            self.get_parameter("selected_cloud_topic").value,
            1,
        )
        self._selected_point_publisher = self.create_publisher(
            PointStamped,
            self.get_parameter("selected_point_topic").value,
            1,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Empty,
            self.get_parameter("cycle_complete_topic").value,
            self._cycle_complete,
            1,
        )
        rgb = Subscriber(
            self, Image, self.get_parameter("rgb_topic").value,
            qos_profile=qos_profile_sensor_data,
        )
        depth = Subscriber(
            self, Image, self.get_parameter("depth_topic").value,
            qos_profile=qos_profile_sensor_data,
        )
        self._synchronizer = ApproximateTimeSynchronizer(
            [rgb, depth], queue_size=3,
            slop=float(self.get_parameter(
                "synchronization_slop_seconds"
            ).value),
        )
        self._synchronizer.registerCallback(self._on_rgbd)
        self.get_logger().info(
            "Live RGB-D segmentation ready; publishing patch and candidate "
            "bag overlays"
        )

    def _on_camera_info(self, message: CameraInfo) -> None:
        self._camera_info = message

    def _on_rgbd(self, rgb_message: Image, depth_message: Image) -> None:
        rate = float(self.get_parameter("maximum_processing_rate_hz").value)
        now = time.monotonic()
        if rate > 0.0 and now - self._last_started < 1.0 / rate:
            return
        if self._camera_info is None:
            return
        self._last_started = now
        try:
            color = self._bridge.imgmsg_to_cv2(rgb_message, "rgb8")
            depth = self._bridge.imgmsg_to_cv2(depth_message, "passthrough")
            depth = np.asarray(depth)
            if depth.dtype == np.float32:
                depth = np.nan_to_num(
                    depth, nan=0.0, posinf=0.0, neginf=0.0
                )
                depth_scale = 1.0
            elif depth.dtype == np.uint16:
                depth_scale = 1000.0
            else:
                raise ValueError(f"unsupported depth dtype {depth.dtype}")
            info = self._camera_info
            camera = CameraModel(
                width=int(info.width), height=int(info.height),
                fx=float(info.k[0]), fy=float(info.k[4]),
                cx=float(info.k[2]), cy=float(info.k[5]),
                depth_scale=depth_scale,
                distortion_model=info.distortion_model,
                distortion_coefficients=np.asarray(info.d, np.float64),
                frame_id=rgb_message.header.frame_id,
                source_schema="ros_camera_info",
            )
            result = process_rgbd_frame(
                color, depth, camera,
                input_is_rectified=bool(self.get_parameter(
                    "input_is_rectified"
                ).value),
            )
            candidates = connected_instance_labels(
                result.patches, result.graph, result.affinity,
                minimum_pixels=int(self.get_parameter(
                    "minimum_candidate_pixels"
                ).value),
            )
            selector_config = UpperRightSelectorConfig(
                same_height_tolerance_m=float(self.get_parameter(
                    "same_height_tolerance_m"
                ).value),
                top_depth_percentile=float(self.get_parameter(
                    "top_depth_percentile"
                ).value),
            )
            descriptions = describe_segments(
                candidates,
                result.cloud,
                top_depth_percentile=selector_config.top_depth_percentile,
            )
            radius = float(self.get_parameter(
                "picked_exclusion_radius_m"
            ).value)
            descriptions = tuple(
                item for item in descriptions
                if all(np.linalg.norm(
                    np.asarray(item.pick_point_xyz) - previous
                ) > radius for previous in self._picked_points)
            )
            selected = UpperRightSegmentSelector(selector_config).choose(
                descriptions
            )
            self._publish_overlay(
                self._patch_publisher,
                _colorize(result.patches.labels + 1, color),
                rgb_message,
            )
            self._publish_overlay(
                self._instance_publisher,
                self._selected_overlay(candidates, color, selected),
                rgb_message,
            )
            self._segmented_cloud_publisher.publish(segment_cloud_message(
                result.cloud, candidates, rgb_message.header,
            ))
            selected_id = 0 if selected is None else selected.instance_id
            self._selected_cloud_publisher.publish(segment_cloud_message(
                result.cloud, candidates, rgb_message.header,
                selected_instance_id=selected_id,
                color_by_instance=False,
            ))
            if selected is not None:
                self._pending_pick = np.asarray(selected.pick_point_xyz)
                point = PointStamped()
                point.header = rgb_message.header
                point.point.x, point.point.y, point.point.z = (
                    selected.pick_point_xyz
                )
                self._selected_point_publisher.publish(point)
        except (TypeError, ValueError, cv2.error) as error:
            self.get_logger().error(
                f"Live segmentation frame failed: {error}",
                throttle_duration_sec=2.0,
            )

    def _cycle_complete(self, _message: Empty) -> None:
        """Exclude the completed pick location and allow the next candidate."""
        if self._pending_pick is not None:
            self._picked_points.append(self._pending_pick)
            self._pending_pick = None

    @staticmethod
    def _selected_overlay(labels, color, selected) -> np.ndarray:
        """Color all candidates and mark the selected one with a box."""
        overlay = _colorize(labels, color)
        if selected is None:
            return overlay
        minimum_u, minimum_v, maximum_u, maximum_v = (
            selected.bounding_box_uv
        )
        cv2.rectangle(
            overlay,
            (minimum_u, minimum_v),
            (maximum_u, maximum_v),
            (255, 255, 0),
            3,
        )
        cv2.putText(
            overlay,
            f"NEXT {selected.instance_id}",
            (minimum_u, max(15, minimum_v - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 0),
            2,
        )
        return overlay

    def _publish_overlay(
        self, publisher, image: np.ndarray, source: Image
    ) -> None:
        message = self._bridge.cv2_to_imgmsg(image, encoding="rgb8")
        message.header = source.header
        publisher.publish(message)


def main(args=None) -> None:
    """Run the live instance-segmentation ROS node."""
    rclpy.init(args=args)
    node = LiveInstanceSegmentation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
