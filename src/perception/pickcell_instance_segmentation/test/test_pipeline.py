"""Tests for the public live-frame processing entry point."""

import numpy as np
import pytest

from pickcell_instance_segmentation import CameraModel
from pickcell_instance_segmentation import connected_instance_labels
from pickcell_instance_segmentation import describe_segments
from pickcell_instance_segmentation import process_rgbd_frame
from pickcell_instance_segmentation import UpperRightSegmentSelector
from pickcell_instance_segmentation import UpperRightSelectorConfig


def camera(width: int, height: int) -> CameraModel:
    """Create distortion-free calibration for a synthetic aligned frame."""
    return CameraModel(
        width=width,
        height=height,
        fx=100.0,
        fy=100.0,
        cx=(width - 1) / 2.0,
        cy=(height - 1) / 2.0,
        depth_scale=1000.0,
        distortion_model="plumb_bob",
        distortion_coefficients=np.zeros(5, dtype=np.float64),
        frame_id="camera_optical_frame",
        source_schema="live_test",
    )


def test_process_rgbd_frame_returns_cloud_and_affinity() -> None:
    """Run an in-memory frame through every currently implemented stage."""
    height, width = 24, 24
    color = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.full((height, width), 1000, dtype=np.uint16)

    result = process_rgbd_frame(color, depth, camera(width, height))

    assert result.cloud.xyz.shape == (height, width, 3)
    assert result.cloud.valid.all()
    assert result.patches.labels.shape == (height, width)
    assert result.affinity.scores.shape == (result.graph.edges.shape[0],)
    assert np.all((result.affinity.scores >= 0.0)
                  & (result.affinity.scores <= 1.0))


def test_process_rgbd_frame_rejects_misaligned_images() -> None:
    """Fail clearly when RGB and depth are not pixel aligned."""
    color = np.zeros((10, 12, 3), dtype=np.uint8)
    depth = np.ones((9, 12), dtype=np.uint16)

    with pytest.raises(ValueError, match="dimensions do not match"):
        process_rgbd_frame(color, depth, camera(12, 10))


def test_boundary_support_fraction_is_bounded() -> None:
    """Ensure live frames cannot violate the affinity input domain."""
    height, width = 24, 24
    color = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.full((height, width), 1000, dtype=np.uint16)
    depth[0, 0] = 1010

    result = process_rgbd_frame(color, depth, camera(width, height))
    column = result.graph.feature_names.index(
        "shared_boundary_fraction_min_patch"
    )
    fractions = result.graph.edge_features[:, column]
    assert np.all((fractions >= 0.0) & (fractions <= 1.0))


def test_connected_instances_align_with_the_input_image() -> None:
    """Convert accepted patch edges into an image-sized component map."""
    height, width = 24, 24
    color = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.full((height, width), 1000, dtype=np.uint16)
    result = process_rgbd_frame(color, depth, camera(width, height))

    labels = connected_instance_labels(
        result.patches, result.graph, result.affinity,
        minimum_pixels=10,
    )

    assert labels.shape == (height, width)
    assert labels.dtype == np.int32
    assert labels.min() >= 0


def test_selector_chooses_the_uppermost_segment() -> None:
    """Prefer the candidate nearest an overhead camera."""
    height, width = 8, 10
    color = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.full((height, width), 1500, dtype=np.uint16)
    depth[1:4, 1:4] = 900
    depth[1:4, 6:9] = 1000
    result = process_rgbd_frame(color, depth, camera(width, height))
    labels = np.zeros((height, width), dtype=np.int32)
    labels[1:4, 1:4] = 1
    labels[1:4, 6:9] = 2

    candidates = describe_segments(labels, result.cloud)
    selected = UpperRightSegmentSelector().choose(candidates)

    assert selected is not None
    assert selected.instance_id == 1


def test_selector_breaks_same_height_tie_to_image_right() -> None:
    """Choose the right candidate when surface heights are equivalent."""
    height, width = 8, 10
    color = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.full((height, width), 1500, dtype=np.uint16)
    depth[1:4, 1:4] = 990
    depth[1:4, 6:9] = 1000
    result = process_rgbd_frame(color, depth, camera(width, height))
    labels = np.zeros((height, width), dtype=np.int32)
    labels[1:4, 1:4] = 1
    labels[1:4, 6:9] = 2
    config = UpperRightSelectorConfig(same_height_tolerance_m=0.02)

    selected = UpperRightSegmentSelector(config).choose_from_labels(
        labels, result.cloud
    )

    assert selected is not None
    assert selected.instance_id == 2


def test_selector_returns_none_without_segments() -> None:
    """Represent an empty scene without inventing a pick target."""
    assert UpperRightSegmentSelector().choose(()) is None
