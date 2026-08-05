"""Deterministic synthetic ROS messages for unit and integration tests."""

import struct
from collections.abc import Sequence

from geometry_msgs.msg import PoseStamped
from pickcell_interfaces.msg import ModuleStatus, RuntimeMode
from sensor_msgs.msg import PointCloud2, PointField
from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose


def make_module_status(
    module_name: str = "test_server",
    mode: int = RuntimeMode.MOCK,
) -> ModuleStatus:
    """Create a valid status declaration for a synthetic runtime module."""
    message = ModuleStatus()
    message.module_name = module_name
    message.state = ModuleStatus.ACTIVE
    message.mode.value = mode
    message.deployment = "test"
    message.implementation = "pickcell_test_support/SyntheticServer"
    message.backend = "generated_data"
    message.algorithm = "deterministic_fixture"
    message.version = "0.1.0"
    message.use_sim_time = mode != RuntimeMode.REAL
    message.configuration_hash = "test-configuration"
    return message


def make_pose_stamped(
    frame_id: str = "cell",
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> PoseStamped:
    """Create an identity-orientation pose in a canonical frame."""
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.pose.position.x, message.pose.position.y, message.pose.position.z = xyz
    message.pose.orientation.w = 1.0
    return message


def make_detection_3d(
    detection_id: str = "object-001",
    class_id: str = "test_object",
    frame_id: str = "cell",
    xyz: tuple[float, float, float] = (0.5, 0.0, 0.1),
    score: float = 1.0,
) -> Detection3D:
    """Create a deterministic box detection compatible with Humble/Jazzy."""
    detection = Detection3D()
    detection.header.frame_id = frame_id
    detection.id = detection_id
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = class_id
    hypothesis.hypothesis.score = score
    hypothesis.pose.pose.position.x = xyz[0]
    hypothesis.pose.pose.position.y = xyz[1]
    hypothesis.pose.pose.position.z = xyz[2]
    hypothesis.pose.pose.orientation.w = 1.0
    detection.results.append(hypothesis)
    detection.bbox.center.position.x = xyz[0]
    detection.bbox.center.position.y = xyz[1]
    detection.bbox.center.position.z = xyz[2]
    detection.bbox.center.orientation.w = 1.0
    detection.bbox.size.x = 0.05
    detection.bbox.size.y = 0.05
    detection.bbox.size.z = 0.05
    return detection


def make_point_cloud(
    points: Sequence[tuple[float, float, float]],
    frame_id: str = "cell",
) -> PointCloud2:
    """Create an unorganized XYZ float32 point cloud without external tools."""
    message = PointCloud2()
    message.header.frame_id = frame_id
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.is_dense = True
    message.data = b"".join(struct.pack("<fff", *point) for point in points)
    return message
