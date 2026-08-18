"""Capture synchronized RGB-D frames as annotation-ready dataset scenes."""

import json
from pathlib import Path
import threading

from cv_bridge import CvBridge
import cv2
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger


class DatasetCapture(Node):
    """Keep the latest aligned RGB-D pair and save it on explicit request."""

    def __init__(self) -> None:
        super().__init__("dataset_capture")
        self.declare_parameter("system_mode", "capture")
        self.declare_parameter("configuration_hash", "")
        self.declare_parameter(
            "rgb_topic", "sensors/overhead_depth/color/image_raw"
        )
        self.declare_parameter(
            "depth_topic", "sensors/overhead_depth/depth/image_raw"
        )
        self.declare_parameter(
            "camera_info_topic", "sensors/overhead_depth/camera_info"
        )
        self.declare_parameter("output_directory", "datasets/bags_raw")
        self.declare_parameter("synchronization_slop_seconds", 0.05)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_pair = None
        self._camera_info = None
        self._output_directory = Path(
            self.get_parameter("output_directory").value
        ).expanduser().resolve()
        self._output_directory.mkdir(parents=True, exist_ok=True)

        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        rgb_sub = Subscriber(
            self,
            Image,
            self.get_parameter("rgb_topic").value,
            qos_profile=qos_profile_sensor_data,
        )
        depth_sub = Subscriber(
            self,
            Image,
            self.get_parameter("depth_topic").value,
            qos_profile=qos_profile_sensor_data,
        )
        self._synchronizer = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub],
            queue_size=10,
            slop=float(
                self.get_parameter("synchronization_slop_seconds").value
            ),
        )
        self._synchronizer.registerCallback(self._on_rgbd)
        self.create_service(Trigger, "dataset/capture", self._capture)
        self.get_logger().info(
            "Dataset capture ready on dataset/capture; output: "
            f"{self._output_directory}"
        )

    def _on_camera_info(self, message: CameraInfo) -> None:
        with self._lock:
            self._camera_info = message

    def _on_rgbd(self, rgb: Image, depth: Image) -> None:
        with self._lock:
            self._latest_pair = (rgb, depth)

    def _capture(self, _request: Trigger.Request, response: Trigger.Response):
        with self._lock:
            pair = self._latest_pair
            info = self._camera_info
        if pair is None or info is None:
            response.success = False
            response.message = (
                "Waiting for synchronized RGB, depth, and camera info"
            )
            return response

        rgb_message, depth_message = pair
        try:
            rgb = self._bridge.imgmsg_to_cv2(rgb_message, "bgr8")
            depth = self._bridge.imgmsg_to_cv2(depth_message, "passthrough")
            scene_directory = self._next_scene_directory()
            scene_directory.mkdir(parents=True)
            self._write_scene(scene_directory, rgb, depth, info, rgb_message)
        except (OSError, ValueError, cv2.error) as error:
            response.success = False
            response.message = f"Capture failed: {error}"
            return response

        files = sorted(path.name for path in scene_directory.iterdir())
        response.success = True
        response.message = f"{scene_directory} ({', '.join(files)})"
        self.get_logger().info(f"Captured dataset scene: {response.message}")
        return response

    def _next_scene_directory(self) -> Path:
        indices = []
        for path in self._output_directory.glob("scene_*"):
            try:
                indices.append(int(path.name.removeprefix("scene_")))
            except ValueError:
                continue
        return (
            self._output_directory
            / f"scene_{max(indices, default=-1) + 1:06d}"
        )

    def _write_scene(
        self,
        directory: Path,
        rgb: np.ndarray,
        depth: np.ndarray,
        info: CameraInfo,
        rgb_message: Image,
    ) -> None:
        if depth.dtype not in (np.uint16, np.float32):
            raise ValueError(f"Unsupported depth type: {depth.dtype}")
        if rgb.shape[:2] != depth.shape[:2]:
            raise ValueError("RGB and depth resolutions do not match")
        if not cv2.imwrite(str(directory / "color.png"), rgb):
            raise OSError("Could not write color.png")
        source_depth_encoding = depth_message_encoding(depth)
        stored_depth = depth_to_millimetres(depth)
        if not cv2.imwrite(str(directory / "depth.png"), stored_depth):
            raise OSError("Could not write depth.png")
        blank_instances = np.zeros(depth.shape[:2], dtype=np.uint16)
        if not cv2.imwrite(str(directory / "instance.png"), blank_instances):
            raise OSError("Could not write instance.png")

        metadata = {
            "width": info.width,
            "height": info.height,
            "frame_id": rgb_message.header.frame_id,
            "timestamp": {
                "sec": rgb_message.header.stamp.sec,
                "nanosec": rgb_message.header.stamp.nanosec,
            },
            "distortion_model": info.distortion_model,
            "distortion_coefficients": list(info.d),
            "intrinsic_matrix": list(info.k),
            "projection_matrix": list(info.p),
            "source_depth_encoding": source_depth_encoding,
            "stored_depth_encoding": "16UC1",
            "depth_scale": 1000.0,
        }
        (directory / "camera.json").write_text(
            json.dumps(metadata, indent=2) + "\n",
            encoding="utf-8",
        )
        required_files = (
            directory / "color.png",
            directory / "depth.png",
            directory / "instance.png",
            directory / "camera.json",
        )
        incomplete = [
            str(path) for path in required_files
            if not path.is_file() or path.stat().st_size == 0
        ]
        if incomplete:
            raise OSError(f"Dataset files missing or empty: {incomplete}")


def depth_message_encoding(depth: np.ndarray) -> str:
    """Return the ROS-style encoding represented by a converted depth image."""
    if depth.dtype == np.uint16:
        return "16UC1"
    if depth.dtype == np.float32:
        return "32FC1"
    raise ValueError(f"Unsupported depth type: {depth.dtype}")


def depth_to_millimetres(depth: np.ndarray) -> np.ndarray:
    """Convert a ROS depth image to lossless uint16 millimetres for PNG."""
    if depth.dtype == np.uint16:
        return depth
    if depth.dtype != np.float32:
        raise ValueError(f"Unsupported depth type: {depth.dtype}")
    valid = np.isfinite(depth) & (depth > 0.0)
    millimetres = np.zeros(depth.shape, dtype=np.uint16)
    scaled = np.clip(np.rint(depth[valid] * 1000.0), 1, 65535)
    millimetres[valid] = scaled.astype(np.uint16)
    return millimetres


def main(args=None) -> None:
    """Run the dataset capture node."""
    rclpy.init(args=args)
    node = DatasetCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
