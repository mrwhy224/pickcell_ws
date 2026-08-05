"""Unit tests for deterministic mock point-cloud messages."""

import struct

from builtin_interfaces.msg import Time
import pytest

from pickcell_mock_nodes.point_cloud_mock import _triples
from pickcell_mock_nodes.point_cloud_mock import make_point_cloud
from pickcell_mock_nodes.point_cloud_mock import make_point_stamped


def test_point_and_cloud_share_frame_and_stamp() -> None:
    """Path-planning inputs carry aligned metadata."""
    stamp = Time(sec=4, nanosec=5)
    point = make_point_stamped("cell", (1.0, 2.0, 3.0), stamp)
    cloud = make_point_cloud("cell", ((1.0, 2.0, 3.0),), stamp)
    assert point.header.frame_id == cloud.header.frame_id == "cell"
    assert point.header.stamp == cloud.header.stamp
    assert cloud.width == 1
    assert struct.unpack("<fff", bytes(cloud.data)) == (1.0, 2.0, 3.0)


def test_flat_cloud_parameter_is_grouped() -> None:
    """The YAML-friendly flat coordinate parameter becomes XYZ triples."""
    assert _triples([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]) == (
        (1.0, 2.0, 3.0),
        (4.0, 5.0, 6.0),
    )


@pytest.mark.parametrize("values", [[], [1.0, 2.0], [1.0, 2.0, 3.0, 4.0]])
def test_invalid_cloud_parameter_is_rejected(values) -> None:
    """Malformed point arrays fail before a node starts publishing."""
    with pytest.raises(ValueError):
        _triples(values)
