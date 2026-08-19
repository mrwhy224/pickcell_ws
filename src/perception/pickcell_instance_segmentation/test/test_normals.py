"""Tests for organized depth edges, normals, and normal variation."""

import hashlib
import os
from pathlib import Path
import time

import numpy as np
import pytest

from pickcell_instance_segmentation import compute_depth_edges
from pickcell_instance_segmentation import compute_normal_variation
from pickcell_instance_segmentation import compute_organized_geometry
from pickcell_instance_segmentation import estimate_organized_normals
from pickcell_instance_segmentation import load_scene
from pickcell_instance_segmentation import NormalEstimationConfig
from pickcell_instance_segmentation import OrganizedCloud
from pickcell_instance_segmentation import rgbd_to_organized_cloud


def cloud_from_depth(
    depth: np.ndarray, *, spacing: float = 0.01
) -> OrganizedCloud:
    """Construct a simple organized optical-frame cloud from metre depths."""
    depth = np.asarray(depth, dtype=np.float32)
    height, width = depth.shape
    rows, columns = np.indices(depth.shape, dtype=np.float32)
    valid = depth > 0.0
    xyz = np.stack((columns * spacing, rows * spacing, depth), axis=2)
    xyz[~valid] = np.nan
    return OrganizedCloud(
        xyz=xyz.astype(np.float32),
        rgb=np.zeros((height, width, 3), dtype=np.uint8),
        valid=valid,
        width=width,
        height=height,
        frame_id="camera_optical_frame",
        coordinate_convention="opencv_optical_frame:+x_right,+y_down,+z_forward",
    )


def tilted_plane(
    height: int = 5, width: int = 5, slope: float = 0.5
) -> OrganizedCloud:
    """Construct points on z = 1 + slope*x with known normal."""
    rows, columns = np.indices((height, width), dtype=np.float32)
    x = columns * np.float32(0.01)
    y = rows * np.float32(0.01)
    z = 1.0 + np.float32(slope) * x
    xyz = np.stack((x, y, z), axis=2).astype(np.float32)
    return OrganizedCloud(
        xyz=xyz,
        rgb=np.zeros_like(xyz, dtype=np.uint8),
        valid=np.ones((height, width), dtype=bool),
        width=width,
        height=height,
        frame_id=None,
        coordinate_convention="opencv_optical_frame:+x_right,+y_down,+z_forward",
    )


def tree_digest(path: Path) -> dict[str, str]:
    """Return hashes of all regular scene files."""
    return {
        str(item.relative_to(path)): hashlib.sha256(item.read_bytes()).hexdigest()
        for item in sorted(path.rglob("*"))
        if item.is_file()
    }


def test_fronto_parallel_plane_has_camera_facing_unit_normals() -> None:
    """Orient valid normals on a fronto-parallel plane toward the camera."""
    geometry = compute_organized_geometry(
        cloud_from_depth(np.ones((5, 5))), NormalEstimationConfig()
    )
    assert geometry.normal_valid[1:-1, 1:-1].all()
    expected = np.array([0.0, 0.0, -1.0], dtype=np.float32)
    actual = geometry.normals[geometry.normal_valid]
    np.testing.assert_allclose(actual, np.broadcast_to(expected, actual.shape))
    lengths = np.linalg.norm(geometry.normals[geometry.normal_valid], axis=1)
    np.testing.assert_allclose(lengths, 1.0, atol=1e-6)


def test_tilted_plane_has_analytic_normal() -> None:
    """Recover the known camera-facing normal of a tilted plane."""
    slope = 0.5
    normals, valid = estimate_organized_normals(
        tilted_plane(slope=slope), NormalEstimationConfig()
    )
    expected = np.array([slope, 0.0, -1.0], dtype=np.float32)
    expected /= np.linalg.norm(expected)
    actual = normals[valid]
    np.testing.assert_allclose(
        actual, np.broadcast_to(expected, actual.shape), atol=1e-5
    )


def test_depth_step_marks_both_sides_and_blocks_normals() -> None:
    """Detect a depth step and prevent opposite tangents from crossing it."""
    depth = np.ones((5, 6), dtype=np.float32)
    depth[:, 3:] = 1.2
    cloud = cloud_from_depth(depth)
    config = NormalEstimationConfig(
        absolute_depth_jump_m=0.02, relative_depth_jump=0.0
    )
    edges = compute_depth_edges(cloud, config)
    assert edges[:, 2:4].all()
    assert not edges[:, :2].any()
    normals, valid = estimate_organized_normals(cloud, config)
    assert not valid[:, 2:4].any()
    assert np.isnan(normals[:, 2:4]).all()


def test_invalid_center_and_neighbor_hole_block_normals() -> None:
    """Keep a depth hole and tangents crossing its boundary invalid."""
    depth = np.ones((5, 5), dtype=np.float32)
    depth[2, 2] = 0.0
    cloud = cloud_from_depth(depth)
    normals, valid = estimate_organized_normals(
        cloud, NormalEstimationConfig()
    )
    assert not valid[2, 2]
    assert not valid[2, 1]
    assert not valid[2, 3]
    assert not valid[1, 2]
    assert not valid[3, 2]
    assert np.isnan(normals[~valid]).all()


def test_degenerate_and_short_baselines_are_invalid() -> None:
    """Reject coincident points and tangents below the configured baseline."""
    depth = np.ones((5, 5), dtype=np.float32)
    degenerate = cloud_from_depth(depth, spacing=0.0)
    _, degenerate_valid = estimate_organized_normals(
        degenerate, NormalEstimationConfig()
    )
    assert not degenerate_valid.any()
    short = cloud_from_depth(depth, spacing=1e-5)
    _, short_valid = estimate_organized_normals(
        short, NormalEstimationConfig(minimum_baseline_m=1e-3)
    )
    assert not short_valid.any()


def test_border_policy_and_radius() -> None:
    """Invalidate borders lacking opposite neighbors at the chosen radius."""
    _, valid = estimate_organized_normals(
        cloud_from_depth(np.ones((7, 7))),
        NormalEstimationConfig(pixel_radius=2),
    )
    assert not valid[:2].any()
    assert not valid[-2:].any()
    assert not valid[:, :2].any()
    assert not valid[:, -2:].any()
    assert valid[2:-2, 2:-2].all()


@pytest.mark.parametrize("shape", [(1, 5), (5, 1), (2, 2), (1, 1)])
def test_undersized_clouds_return_shaped_invalid_outputs(shape: tuple) -> None:
    """Handle one-dimensional and undersized organized clouds safely."""
    geometry = compute_organized_geometry(
        cloud_from_depth(np.ones(shape)), NormalEstimationConfig()
    )
    assert geometry.normals.shape == (*shape, 3)
    assert geometry.normal_valid.shape == shape
    assert not geometry.normal_valid.any()
    assert np.isnan(geometry.normals).all()
    assert np.isnan(geometry.normal_variation).all()


def test_empty_valid_depth_cloud_is_safe() -> None:
    """Return no normals or variation for an entirely invalid cloud."""
    geometry = compute_organized_geometry(
        cloud_from_depth(np.zeros((5, 5))), NormalEstimationConfig()
    )
    assert not geometry.normal_valid.any()
    assert not geometry.depth_edges.any()
    assert np.isnan(geometry.normals).all()
    assert np.isnan(geometry.normal_variation).all()


def test_repeated_results_are_deterministic_and_input_unchanged() -> None:
    """Compute identical geometry without mutating cloud arrays."""
    cloud = tilted_plane()
    xyz_before = cloud.xyz.copy()
    valid_before = cloud.valid.copy()
    first = compute_organized_geometry(cloud, NormalEstimationConfig())
    second = compute_organized_geometry(cloud, NormalEstimationConfig())
    np.testing.assert_array_equal(first.normals, second.normals)
    np.testing.assert_array_equal(first.normal_valid, second.normal_valid)
    np.testing.assert_array_equal(first.depth_edges, second.depth_edges)
    np.testing.assert_array_equal(first.normal_variation, second.normal_variation)
    np.testing.assert_array_equal(cloud.xyz, xyz_before)
    np.testing.assert_array_equal(cloud.valid, valid_before)


def test_normal_variation_is_zero_on_plane() -> None:
    """Produce near-zero local normal variation on a perfect plane."""
    geometry = compute_organized_geometry(
        cloud_from_depth(np.ones((5, 5))), NormalEstimationConfig()
    )
    finite = np.isfinite(geometry.normal_variation)
    assert finite.any()
    np.testing.assert_allclose(geometry.normal_variation[finite], 0.0, atol=1e-6)


def test_variation_increases_at_angular_transition() -> None:
    """Measure greater disagreement where neighboring normals change angle."""
    normals = np.zeros((3, 4, 3), dtype=np.float32)
    normals[:, :2, 2] = -1.0
    normals[:, 2:, 0] = np.float32(2 ** -0.5)
    normals[:, 2:, 2] = np.float32(-(2 ** -0.5))
    valid = np.ones((3, 4), dtype=bool)
    edges = np.zeros((3, 4), dtype=bool)
    variation = compute_normal_variation(
        normals, valid, edges, NormalEstimationConfig()
    )
    assert variation[1, 1] > variation[1, 0]
    assert variation[1, 2] > variation[1, 3]


def test_variation_never_compares_across_depth_edge() -> None:
    """Exclude all comparisons involving marked depth-edge pixels."""
    normals = np.zeros((1, 2, 3), dtype=np.float32)
    normals[0, 0] = [0.0, 0.0, -1.0]
    normals[0, 1] = [1.0, 0.0, 0.0]
    valid = np.ones((1, 2), dtype=bool)
    no_edges = compute_normal_variation(
        normals, valid, np.zeros((1, 2), dtype=bool),
        NormalEstimationConfig(),
    )
    assert np.isfinite(no_edges).all()
    with_edges = compute_normal_variation(
        normals, valid, np.ones((1, 2), dtype=bool),
        NormalEstimationConfig(),
    )
    assert np.isnan(with_edges).all()


def test_orientation_can_preserve_cross_product_sign() -> None:
    """Allow deterministic positive-Z cross-product orientation when disabled."""
    cloud = cloud_from_depth(np.ones((5, 5)))
    toward, valid = estimate_organized_normals(
        cloud, NormalEstimationConfig(orient_toward_camera=True)
    )
    raw, raw_valid = estimate_organized_normals(
        cloud, NormalEstimationConfig(orient_toward_camera=False)
    )
    assert np.all(toward[valid, 2] < 0.0)
    assert np.all(raw[raw_valid, 2] > 0.0)


def test_diagonal_depth_edges_are_configuration_controlled() -> None:
    """Inspect diagonal transitions only when explicitly enabled."""
    depth = np.array([[1.0, 0.0], [0.0, 1.2]], dtype=np.float32)
    cloud = cloud_from_depth(depth)
    cardinal = compute_depth_edges(
        cloud, NormalEstimationConfig(use_diagonal_neighbors=False)
    )
    diagonal = compute_depth_edges(
        cloud, NormalEstimationConfig(use_diagonal_neighbors=True)
    )
    assert not cardinal.any()
    assert diagonal[0, 0] and diagonal[1, 1]


@pytest.mark.parametrize(
    "updates,exception",
    [
        ({"pixel_radius": 0}, ValueError),
        ({"pixel_radius": True}, TypeError),
        ({"pixel_radius": 1.5}, TypeError),
        ({"absolute_depth_jump_m": 0.0}, ValueError),
        ({"absolute_depth_jump_m": np.inf}, ValueError),
        ({"absolute_depth_jump_m": True}, TypeError),
        ({"relative_depth_jump": -0.1}, ValueError),
        ({"relative_depth_jump": np.nan}, ValueError),
        ({"minimum_baseline_m": 0.0}, ValueError),
        ({"orient_toward_camera": 1}, TypeError),
        ({"use_diagonal_neighbors": 0}, TypeError),
    ],
)
def test_invalid_configuration_values(updates: dict, exception: type) -> None:
    """Reject invalid configuration values and ambiguous boolean numerics."""
    with pytest.raises(exception):
        NormalEstimationConfig(**updates)


def test_output_contract_shapes_dtypes_nans_and_read_only() -> None:
    """Return exact immutable array contracts with NaN invalid values."""
    geometry = compute_organized_geometry(
        cloud_from_depth(np.ones((4, 6))), NormalEstimationConfig()
    )
    assert geometry.normals.shape == (4, 6, 3)
    assert geometry.normals.dtype == np.float32
    assert geometry.normal_valid.shape == (4, 6)
    assert geometry.normal_valid.dtype == np.bool_
    assert geometry.depth_edges.shape == (4, 6)
    assert geometry.depth_edges.dtype == np.bool_
    assert geometry.normal_variation.shape == (4, 6)
    assert geometry.normal_variation.dtype == np.float32
    assert np.isnan(geometry.normals[~geometry.normal_valid]).all()
    assert np.isnan(
        geometry.normal_variation[~np.isfinite(geometry.normal_variation)]
    ).all()
    assert not geometry.normals.flags.writeable
    assert not geometry.normal_valid.flags.writeable
    assert not geometry.depth_edges.flags.writeable
    assert not geometry.normal_variation.flags.writeable


def test_real_scene_geometry_contract() -> None:
    """Compute real-scene geometry deterministically without modifying files."""
    configured = os.environ.get("PICKCELL_TEST_SCENE")
    if not configured:
        pytest.skip("PICKCELL_TEST_SCENE is not set; real geometry not run")
    path = Path(configured)
    before = tree_digest(path)
    cloud = rgbd_to_organized_cloud(load_scene(path))
    config = NormalEstimationConfig()
    start = time.perf_counter()
    first = compute_organized_geometry(cloud, config)
    elapsed = time.perf_counter() - start
    second = compute_organized_geometry(cloud, config)

    shape = (cloud.height, cloud.width)
    assert first.normals.shape == (*shape, 3)
    assert first.normal_valid.shape == shape
    assert first.depth_edges.shape == shape
    assert first.normal_variation.shape == shape
    lengths = np.linalg.norm(first.normals[first.normal_valid], axis=1)
    assert np.isfinite(first.normals[first.normal_valid]).all()
    np.testing.assert_allclose(lengths, 1.0, atol=1e-5)
    assert np.isnan(first.normals[~first.normal_valid]).all()
    assert not np.any(first.normal_valid & ~cloud.valid)
    np.testing.assert_array_equal(first.normals, second.normals)
    np.testing.assert_array_equal(first.normal_valid, second.normal_valid)
    np.testing.assert_array_equal(first.depth_edges, second.depth_edges)
    np.testing.assert_array_equal(first.normal_variation, second.normal_variation)
    assert tree_digest(path) == before

    valid_xyz = int(np.count_nonzero(cloud.valid))
    valid_normals = int(np.count_nonzero(first.normal_valid))
    ratio = valid_normals / valid_xyz if valid_xyz else 0.0
    print({
        "valid_xyz": valid_xyz,
        "valid_normals": valid_normals,
        "valid_normal_ratio": ratio,
        "depth_edges": int(np.count_nonzero(first.depth_edges)),
        "finite_normal_variation": int(
            np.count_nonzero(np.isfinite(first.normal_variation))
        ),
        "execution_seconds": elapsed,
    })
