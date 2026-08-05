"""Tests for deterministic synthetic message builders."""

import struct

from pickcell_test_support import make_detection_3d, make_point_cloud


def test_detection_has_identity_and_canonical_frame() -> None:
    """Detection fixtures carry stable identity and frame metadata."""
    detection = make_detection_3d()
    assert detection.header.frame_id == "cell"
    assert detection.id == "object-001"
    assert detection.results[0].hypothesis.class_id == "test_object"


def test_point_cloud_layout_and_data() -> None:
    """Point cloud fixtures expose packed XYZ float32 fields."""
    cloud = make_point_cloud([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])
    assert cloud.width == 2
    assert cloud.point_step == 12
    assert cloud.row_step == 24
    assert struct.unpack("<fff", bytes(cloud.data[:12])) == (1.0, 2.0, 3.0)
