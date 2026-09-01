"""Tests for publishing compact labeled segment point clouds."""

from std_msgs.msg import Header
import numpy as np

from pickcell_instance_segmentation import CameraModel
from pickcell_instance_segmentation import process_rgbd_frame
from pickcell_instance_segmentation.ros_cloud import segment_cloud_message


def test_segment_cloud_contains_labels_and_can_select_one() -> None:
    """Preserve instance IDs and emit only the requested selected segment."""
    height, width = 4, 5
    color = np.zeros((height, width, 3), dtype=np.uint8)
    color[..., 0] = 200
    depth = np.full((height, width), 1000, dtype=np.uint16)
    camera = CameraModel(
        width=width, height=height,
        fx=100.0, fy=100.0, cx=2.0, cy=1.5,
        depth_scale=1000.0,
        distortion_model="plumb_bob",
        distortion_coefficients=np.zeros(5, dtype=np.float64),
        frame_id="camera", source_schema="test",
    )
    cloud = process_rgbd_frame(color, depth, camera).cloud
    labels = np.zeros((height, width), dtype=np.int32)
    labels[:, :2] = 1
    labels[:, 3:] = 2
    header = Header(frame_id="camera")

    combined = segment_cloud_message(cloud, labels, header)
    selected = segment_cloud_message(
        cloud, labels, header,
        selected_instance_id=2,
        color_by_instance=False,
    )

    assert combined.width == 16
    assert selected.width == 8
    assert combined.point_step == 20
    assert len(combined.data) == combined.row_step
    assert [field.name for field in combined.fields] == [
        "x", "y", "z", "rgb", "instance_id",
    ]
    records = np.frombuffer(selected.data, dtype=[
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
        ("rgb", "<u4"), ("instance_id", "<u4"),
    ])
    assert np.all(records["instance_id"] == 2)
    assert np.all(records["rgb"] == 0xC80000)
