"""Tests for deterministic organized RGB-D projection."""

import hashlib
import os
from pathlib import Path

import numpy as np
import pytest

from pickcell_instance_segmentation import CameraModel
from pickcell_instance_segmentation import compact_labels_to_organized
from pickcell_instance_segmentation import compact_valid_points
from pickcell_instance_segmentation import load_scene
from pickcell_instance_segmentation import ProjectionError
from pickcell_instance_segmentation import rgbd_to_organized_cloud
from pickcell_instance_segmentation import SceneInput


def camera(
    *, width: int = 3, height: int = 3, distortion: tuple = ()
) -> CameraModel:
    """Return a calibrated camera for analytic projection tests."""
    return CameraModel(
        width=width,
        height=height,
        fx=1.0,
        fy=1.0,
        cx=1.0,
        cy=1.0,
        depth_scale=1000.0,
        distortion_model="plumb_bob",
        distortion_coefficients=np.asarray(distortion, dtype=np.float64),
        frame_id="camera_optical_frame",
        source_schema="test",
    )


def scene(
    *,
    depth: np.ndarray | None = None,
    color: np.ndarray | None = None,
    calibration: CameraModel | None = None,
    ground_truth: np.ndarray | None = None,
) -> SceneInput:
    """Return an in-memory scene without filesystem dependencies."""
    if depth is None:
        depth = np.full((3, 3), 1000, dtype=np.uint16)
    if color is None:
        color = np.arange(27, dtype=np.uint8).reshape(3, 3, 3)
    if calibration is None:
        calibration = camera(width=depth.shape[1], height=depth.shape[0])
    return SceneInput(
        color=color,
        depth=depth,
        camera=calibration,
        ground_truth=ground_truth,
        instance_mapping={},
        semantics={},
        metadata={},
    )


def tree_digest(path: Path) -> dict[str, str]:
    """Return content hashes for every regular file below a scene path."""
    return {
        str(item.relative_to(path)): hashlib.sha256(item.read_bytes()).hexdigest()
        for item in sorted(path.rglob("*"))
        if item.is_file()
    }


def test_constant_depth_plane_has_known_coordinates() -> None:
    """Project a 3-by-3 one-metre plane analytically."""
    cloud = rgbd_to_organized_cloud(scene())
    expected = np.array([
        [[-1.0, -1.0, 1.0], [0.0, -1.0, 1.0], [1.0, -1.0, 1.0]],
        [[-1.0, 0.0, 1.0], [0.0, 0.0, 1.0], [1.0, 0.0, 1.0]],
        [[-1.0, 1.0, 1.0], [0.0, 1.0, 1.0], [1.0, 1.0, 1.0]],
    ], dtype=np.float32)
    np.testing.assert_array_equal(cloud.xyz, expected)
    assert cloud.xyz[1, 1].tolist() == [0.0, 0.0, 1.0]
    assert cloud.xyz[0, 0].tolist() == [-1.0, -1.0, 1.0]


def test_millimetres_are_converted_to_metres() -> None:
    """Divide stored millimetres by units per metre exactly once."""
    depth = np.full((3, 3), 2500, dtype=np.uint16)
    cloud = rgbd_to_organized_cloud(scene(depth=depth))
    assert cloud.xyz[1, 1, 2] == np.float32(2.5)


def test_zero_depth_is_invalid_and_xyz_is_nan() -> None:
    """Keep invalid zero-depth pixels organized and fill XYZ with NaNs."""
    depth = np.full((3, 3), 1000, dtype=np.uint16)
    depth[0, 2] = 0
    cloud = rgbd_to_organized_cloud(scene(depth=depth))
    assert not cloud.valid[0, 2]
    assert np.isnan(cloud.xyz[0, 2]).all()
    assert cloud.valid.sum() == 8


@pytest.mark.parametrize("bad_value", [-1.0, np.nan, np.inf, -np.inf])
def test_negative_or_nonfinite_depth_is_rejected(bad_value: float) -> None:
    """Reject physically invalid or nonfinite floating depth samples."""
    depth = np.ones((3, 3), dtype=np.float32)
    depth[1, 1] = bad_value
    with pytest.raises(ProjectionError, match="negative|nonfinite"):
        rgbd_to_organized_cloud(scene(depth=depth))


def test_rgb_remains_exactly_pixel_aligned() -> None:
    """Preserve each RGB value at its corresponding organized pixel."""
    source = np.arange(27, dtype=np.uint8).reshape(3, 3, 3)
    cloud = rgbd_to_organized_cloud(scene(color=source))
    np.testing.assert_array_equal(cloud.rgb, source)


def test_compact_order_and_instance_correspondence() -> None:
    """Compact valid points and labels in deterministic row-major order."""
    depth = np.array([
        [1000, 0, 1000],
        [0, 1000, 0],
        [1000, 1000, 0],
    ], dtype=np.uint16)
    labels = np.array([
        [0, 99, 2],
        [99, 3, 99],
        [4, 0, 99],
    ], dtype=np.uint16)
    cloud = rgbd_to_organized_cloud(scene(depth=depth))
    compact = compact_valid_points(cloud, labels)
    np.testing.assert_array_equal(
        compact.pixels_vu,
        np.array([[0, 0], [0, 2], [1, 1], [2, 0], [2, 1]]),
    )
    np.testing.assert_array_equal(compact.ground_truth, [0, 2, 3, 4, 0])
    assert compact.ground_truth.dtype == np.uint16
    assert compact.rgb[1].tolist() == cloud.rgb[0, 2].tolist()


def test_compact_to_organized_round_trip_preserves_background() -> None:
    """Restore compact labels to exact pixels and preserve background zero."""
    pixels = np.array([[0, 0], [0, 2], [2, 1]], dtype=np.int64)
    labels = np.array([0, 7, 8], dtype=np.uint16)
    organized = compact_labels_to_organized(labels, pixels, (3, 3))
    expected = np.array([
        [0, -1, 7],
        [-1, -1, -1],
        [-1, 8, -1],
    ], dtype=np.int32)
    np.testing.assert_array_equal(organized, expected)


def test_duplicate_compact_pixels_are_rejected() -> None:
    """Reject ambiguous duplicate inverse-map destinations."""
    pixels = np.array([[0, 0], [0, 0]], dtype=np.int64)
    with pytest.raises(ProjectionError, match="duplicate"):
        compact_labels_to_organized(np.array([1, 2]), pixels, (2, 2))


@pytest.mark.parametrize("pixel", [[-1, 0], [2, 0], [0, -1], [0, 2]])
def test_out_of_bounds_compact_pixels_are_rejected(pixel: list[int]) -> None:
    """Reject negative and upper-bound row or column indices."""
    with pytest.raises(ProjectionError, match="out of bounds"):
        compact_labels_to_organized(
            np.array([1]), np.asarray([pixel]), (2, 2)
        )


def test_label_count_mismatch_is_rejected() -> None:
    """Require one compact label per compact pixel."""
    with pytest.raises(ProjectionError, match="count"):
        compact_labels_to_organized(
            np.array([1, 2]), np.array([[0, 0]]), (2, 2)
        )


def test_mismatched_rgb_and_depth_shapes_are_rejected() -> None:
    """Never resize mismatched RGB and depth inputs silently."""
    depth = np.ones((3, 3), dtype=np.uint16)
    color = np.zeros((2, 3, 3), dtype=np.uint8)
    with pytest.raises(ProjectionError, match="dimensions"):
        rgbd_to_organized_cloud(scene(depth=depth, color=color))


def test_nonzero_distortion_requires_rectification_confirmation() -> None:
    """Reject distorted input unless joint rectification is explicit."""
    distorted = scene(calibration=camera(distortion=(0.1, 0.0, 0.0, 0.0)))
    with pytest.raises(ProjectionError, match="input_is_rectified"):
        rgbd_to_organized_cloud(distorted)
    cloud = rgbd_to_organized_cloud(distorted, input_is_rectified=True)
    assert cloud.valid.all()


def test_repeated_projection_is_deterministic_and_inputs_unchanged() -> None:
    """Produce identical output without mutating caller-owned input arrays."""
    source_depth = np.full((3, 3), 1000, dtype=np.uint16)
    source_color = np.arange(27, dtype=np.uint8).reshape(3, 3, 3)
    original_depth = source_depth.copy()
    original_color = source_color.copy()
    source_scene = scene(depth=source_depth, color=source_color)
    first = rgbd_to_organized_cloud(source_scene)
    second = rgbd_to_organized_cloud(source_scene)
    np.testing.assert_array_equal(first.xyz, second.xyz)
    np.testing.assert_array_equal(first.rgb, second.rgb)
    np.testing.assert_array_equal(source_depth, original_depth)
    np.testing.assert_array_equal(source_color, original_color)
    assert not first.xyz.flags.writeable
    assert not first.rgb.flags.writeable
    assert not first.valid.flags.writeable


def test_empty_valid_point_result_has_stable_shapes() -> None:
    """Represent an all-invalid image as an empty compact cloud."""
    depth = np.zeros((3, 3), dtype=np.uint16)
    labels = np.zeros((3, 3), dtype=np.uint16)
    compact = compact_valid_points(
        rgbd_to_organized_cloud(scene(depth=depth)), labels
    )
    assert compact.xyz.shape == (0, 3)
    assert compact.rgb.shape == (0, 3)
    assert compact.pixels_vu.shape == (0, 2)
    assert compact.ground_truth.shape == (0,)


def test_real_scene_projection_contract() -> None:
    """Project a configured real scene without altering any dataset bytes."""
    configured = os.environ.get("PICKCELL_TEST_SCENE")
    if not configured:
        pytest.skip("PICKCELL_TEST_SCENE is not set; real projection not run")
    path = Path(configured)
    before = tree_digest(path)
    source_scene = load_scene(path, require_ground_truth=True)
    cloud = rgbd_to_organized_cloud(source_scene)
    compact = compact_valid_points(cloud, source_scene.ground_truth)
    accepted_valid = source_scene.depth != 0

    assert cloud.xyz.shape == (*source_scene.depth.shape, 3)
    assert cloud.rgb.shape[:2] == source_scene.depth.shape
    assert np.count_nonzero(cloud.valid) == np.count_nonzero(accepted_valid)
    assert np.isfinite(cloud.xyz[cloud.valid]).all()
    assert np.isnan(cloud.xyz[~cloud.valid]).all()
    assert compact.xyz.shape[0] == np.count_nonzero(accepted_valid)
    np.testing.assert_array_equal(
        compact.ground_truth,
        source_scene.ground_truth[compact.pixels_vu[:, 0],
                                  compact.pixels_vu[:, 1]],
    )
    instance_pixels = source_scene.ground_truth != 0
    assert np.all(cloud.valid[instance_pixels])
    assert tree_digest(path) == before
    print(
        "real projection counts: "
        f"valid={np.count_nonzero(cloud.valid)}, "
        f"invalid={cloud.valid.size - np.count_nonzero(cloud.valid)}"
    )
