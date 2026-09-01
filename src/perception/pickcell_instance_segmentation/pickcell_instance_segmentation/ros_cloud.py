"""Convert aligned segment arrays to ROS ``PointCloud2`` messages."""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from .models import OrganizedCloud


_POINT_DTYPE = np.dtype([
    ("x", "<f4"),
    ("y", "<f4"),
    ("z", "<f4"),
    ("rgb", "<u4"),
    ("instance_id", "<u4"),
])


def segment_cloud_message(
    cloud: OrganizedCloud,
    labels: np.ndarray,
    header,
    *,
    selected_instance_id: int | None = None,
    color_by_instance: bool = True,
) -> PointCloud2:
    """Create a compact XYZRGB cloud for all or one positive segment label."""
    if labels.shape != cloud.valid.shape:
        raise ValueError("labels and organized cloud dimensions do not match")
    mask = (labels > 0) & cloud.valid
    if selected_instance_id is not None:
        mask &= labels == selected_instance_id

    xyz = cloud.xyz[mask].astype(np.float32, copy=False)
    identifiers = labels[mask].astype(np.uint32, copy=False)
    if color_by_instance:
        red = (identifiers * 37 + 31) % 255
        green = (identifiers * 73 + 67) % 255
        blue = (identifiers * 109 + 101) % 255
        rgb = (red << 16) | (green << 8) | blue
    else:
        colors = cloud.rgb[mask].astype(np.uint32, copy=False)
        rgb = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]

    points = np.empty(xyz.shape[0], dtype=_POINT_DTYPE)
    if xyz.shape[0]:
        points["x"] = xyz[:, 0]
        points["y"] = xyz[:, 1]
        points["z"] = xyz[:, 2]
        points["rgb"] = rgb
        points["instance_id"] = identifiers

    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = int(points.shape[0])
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        PointField(
            name="instance_id", offset=16,
            datatype=PointField.UINT32, count=1,
        ),
    ]
    message.is_bigendian = False
    message.point_step = _POINT_DTYPE.itemsize
    message.row_step = message.point_step * message.width
    message.data = points.tobytes()
    message.is_dense = True
    return message
