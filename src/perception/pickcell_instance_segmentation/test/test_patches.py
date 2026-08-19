"""Tests for conservative deterministic surface-patch oversegmentation."""

import hashlib
import os
from pathlib import Path
import time

import numpy as np
import pytest

from pickcell_instance_segmentation import compute_organized_geometry
from pickcell_instance_segmentation import load_scene
from pickcell_instance_segmentation import NormalEstimationConfig
from pickcell_instance_segmentation import OrganizedCloud
from pickcell_instance_segmentation import OrganizedGeometry
from pickcell_instance_segmentation import PatchSegmentationConfig
from pickcell_instance_segmentation import PatchSegmentationError
from pickcell_instance_segmentation import rgbd_to_organized_cloud
from pickcell_instance_segmentation import segment_surface_patches


def cloud_from_valid(
    valid: np.ndarray,
    *,
    depth: np.ndarray | None = None,
    rgb: np.ndarray | None = None,
    spacing: float = 0.01,
) -> OrganizedCloud:
    """Construct an organized planar cloud for patch tests."""
    valid = np.asarray(valid, dtype=bool)
    height, width = valid.shape
    if depth is None:
        depth = np.ones(valid.shape, dtype=np.float32)
    else:
        depth = np.asarray(depth, dtype=np.float32)
    rows, columns = np.indices(valid.shape, dtype=np.float32)
    xyz = np.stack((columns * spacing, rows * spacing, depth), axis=2)
    xyz[~valid] = np.nan
    if rgb is None:
        rgb = np.zeros((height, width, 3), dtype=np.uint8)
    return OrganizedCloud(
        xyz=xyz.astype(np.float32),
        rgb=rgb,
        valid=valid,
        width=width,
        height=height,
        frame_id=None,
        coordinate_convention="opencv_optical_frame:+x_right,+y_down,+z_forward",
    )


def geometry_for(
    cloud: OrganizedCloud,
    *,
    normals: np.ndarray | None = None,
    normal_valid: np.ndarray | None = None,
    depth_edges: np.ndarray | None = None,
) -> OrganizedGeometry:
    """Construct controlled Phase 3 geometry for patch compatibility tests."""
    shape = cloud.valid.shape
    if normals is None:
        normals = np.zeros((*shape, 3), dtype=np.float32)
        normals[..., 2] = -1.0
    if normal_valid is None:
        normal_valid = cloud.valid.copy()
    normals = np.asarray(normals, dtype=np.float32).copy()
    normals[~normal_valid] = np.nan
    if depth_edges is None:
        depth_edges = np.zeros(shape, dtype=bool)
    return OrganizedGeometry(
        normals=normals,
        normal_valid=normal_valid,
        depth_edges=depth_edges,
        normal_variation=np.zeros(shape, dtype=np.float32),
    )


def segment(
    cloud: OrganizedCloud,
    *,
    geometry: OrganizedGeometry | None = None,
    **config,
):
    """Segment a test cloud with controlled geometry and configuration."""
    if geometry is None:
        geometry = geometry_for(cloud)
    return segment_surface_patches(
        cloud, geometry, PatchSegmentationConfig(**config)
    )


def tree_digest(path: Path) -> dict[str, str]:
    """Return content hashes for all real-scene files."""
    return {
        str(item.relative_to(path)): hashlib.sha256(item.read_bytes()).hexdigest()
        for item in sorted(path.rglob("*"))
        if item.is_file()
    }


def test_flat_plane_is_deliberately_split_by_tiles() -> None:
    """Never merge a compatible plane across fixed tile boundaries."""
    result = segment(
        cloud_from_valid(np.ones((5, 5), dtype=bool)), tile_size_pixels=2
    )
    assert result.count == 9
    np.testing.assert_array_equal(result.pixel_counts, [4, 4, 2, 4, 4, 2, 2, 2, 1])


def test_patch_tile_metadata_matches_source_tiles() -> None:
    """Expose immutable deterministic tile identity without changing labels."""
    result = segment(
        cloud_from_valid(np.ones((3, 5), dtype=bool)), tile_size_pixels=2
    )
    np.testing.assert_array_equal(
        result.tile_indices,
        [[0, 0], [0, 1], [0, 2], [1, 0], [1, 1], [1, 2]],
    )
    assert result.tile_indices.dtype == np.int32
    assert result.source_tile_size_pixels == 2
    assert not result.tile_indices.flags.writeable


def test_ids_are_contiguous_deterministic_and_cover_validity() -> None:
    """Assign every valid point one contiguous row-major deterministic ID."""
    valid = np.array([[1, 0, 1], [1, 1, 1]], dtype=bool)
    cloud = cloud_from_valid(valid)
    first = segment(cloud, tile_size_pixels=3)
    second = segment(cloud, tile_size_pixels=3)
    assert np.all(first.labels[valid] >= 0)
    assert np.all(first.labels[~valid] == -1)
    np.testing.assert_array_equal(np.unique(first.labels[valid]), np.arange(first.count))
    np.testing.assert_array_equal(first.labels, second.labels)


def test_depth_step_and_phase3_edge_split_inside_tile() -> None:
    """Keep incompatible depth surfaces separate without dropping edge pixels."""
    depth = np.ones((3, 4), dtype=np.float32)
    depth[:, 2:] = 1.2
    cloud = cloud_from_valid(np.ones_like(depth, dtype=bool), depth=depth)
    normal_config = NormalEstimationConfig(
        absolute_depth_jump_m=0.02, relative_depth_jump=0.0
    )
    geometry = compute_organized_geometry(cloud, normal_config)
    result = segment(cloud, geometry=geometry, tile_size_pixels=8)
    assert np.all(result.labels >= 0)
    assert not np.any(result.labels[:, 1] == result.labels[:, 2])


def test_large_point_jump_splits_but_compatible_points_connect() -> None:
    """Apply configured 3D separation while retaining safe neighbors."""
    cloud = cloud_from_valid(np.ones((1, 4), dtype=bool))
    xyz = cloud.xyz.copy()
    xyz[0, 2:, 0] += 0.2
    jumped = OrganizedCloud(
        xyz, cloud.rgb, cloud.valid, cloud.width, cloud.height,
        cloud.frame_id, cloud.coordinate_convention,
    )
    result = segment(
        jumped, geometry=geometry_for(jumped), tile_size_pixels=8,
        absolute_point_jump_m=0.03, relative_point_jump=0.0,
    )
    assert result.labels[0, 0] == result.labels[0, 1]
    assert result.labels[0, 1] != result.labels[0, 2]
    assert result.labels[0, 2] == result.labels[0, 3]


def test_different_tiles_never_merge() -> None:
    """Keep compatible adjacent pixels separate across tile boundaries."""
    result = segment(
        cloud_from_valid(np.ones((1, 4), dtype=bool)), tile_size_pixels=2
    )
    assert result.labels[0, 0] == result.labels[0, 1]
    assert result.labels[0, 1] != result.labels[0, 2]


def test_normal_angle_threshold_splits_angular_surface() -> None:
    """Split valid normal pairs exceeding the unsigned angular threshold."""
    cloud = cloud_from_valid(np.ones((1, 4), dtype=bool))
    normals = np.zeros((1, 4, 3), dtype=np.float32)
    normals[:, :2, 2] = -1.0
    normals[:, 2:, 0] = 1.0
    result = segment(
        cloud, geometry=geometry_for(cloud, normals=normals),
        tile_size_pixels=8, maximum_normal_angle_deg=20.0,
    )
    assert result.labels[0, 1] != result.labels[0, 2]


def test_invalid_normals_are_optional_or_required() -> None:
    """Label invalid-normal pixels while optionally blocking their connections."""
    cloud = cloud_from_valid(np.ones((1, 3), dtype=bool))
    normal_valid = np.array([[1, 0, 1]], dtype=bool)
    geometry = geometry_for(cloud, normal_valid=normal_valid)
    allowed = segment(
        cloud, geometry=geometry, tile_size_pixels=8,
        require_valid_normals=False,
    )
    required = segment(
        cloud, geometry=geometry, tile_size_pixels=8,
        require_valid_normals=True,
    )
    assert allowed.count == 1
    assert required.count == 3
    assert np.all(required.labels >= 0)


def test_optional_rgb_threshold_splits_and_disabled_rgb_does_not() -> None:
    """Use RGB only when its optional distance threshold is configured."""
    rgb = np.zeros((1, 4, 3), dtype=np.uint8)
    rgb[:, 2:] = 255
    cloud = cloud_from_valid(np.ones((1, 4), dtype=bool), rgb=rgb)
    disabled = segment(cloud, tile_size_pixels=8)
    enabled = segment(
        cloud, tile_size_pixels=8, maximum_color_distance=10.0
    )
    assert disabled.count == 1
    assert enabled.labels[0, 1] != enabled.labels[0, 2]


def test_four_and_eight_connectivity_differ_for_diagonal_pixels() -> None:
    """Connect diagonal-only valid pixels exclusively under eight-connectivity."""
    valid = np.array([[1, 0], [0, 1]], dtype=bool)
    cloud = cloud_from_valid(valid)
    four = segment(cloud, tile_size_pixels=4, connectivity=4)
    eight = segment(cloud, tile_size_pixels=4, connectivity=8)
    assert four.count == 2
    assert eight.count == 1


def test_invalid_hole_is_not_filled() -> None:
    """Leave invalid holes at label minus one without discarding valid points."""
    valid = np.ones((3, 3), dtype=bool)
    valid[1, 1] = False
    result = segment(cloud_from_valid(valid), tile_size_pixels=4)
    assert result.labels[1, 1] == -1
    assert np.count_nonzero(result.labels >= 0) == 8


def test_all_invalid_and_single_pixel_clouds() -> None:
    """Handle empty and singleton valid inputs without special-case failures."""
    empty = segment(cloud_from_valid(np.zeros((2, 3), dtype=bool)))
    assert empty.count == 0
    assert np.all(empty.labels == -1)
    assert empty.centroids_xyz.shape == (0, 3)
    single = segment(cloud_from_valid(np.array([[True]])))
    assert single.count == 1
    assert single.labels[0, 0] == 0
    assert single.pixel_counts.tolist() == [1]


def test_small_image_and_partial_border_tiles() -> None:
    """Support images smaller than a tile and partial tiles at borders."""
    small = segment(
        cloud_from_valid(np.ones((2, 3), dtype=bool)), tile_size_pixels=12
    )
    assert small.count == 1
    partial = segment(
        cloud_from_valid(np.ones((3, 5), dtype=bool)), tile_size_pixels=2
    )
    assert partial.count == 6
    assert int(partial.pixel_counts.sum()) == 15


def test_exact_statistics_bounds_and_normal_means() -> None:
    """Compute exact aggregates and inclusive-exclusive bounding boxes."""
    valid = np.array([[1, 1], [1, 0]], dtype=bool)
    rgb = np.array([
        [[0, 10, 20], [30, 40, 50]],
        [[60, 70, 80], [0, 0, 0]],
    ], dtype=np.uint8)
    cloud = cloud_from_valid(valid, rgb=rgb, spacing=1.0)
    normal_valid = np.array([[1, 1], [0, 0]], dtype=bool)
    geometry = geometry_for(cloud, normal_valid=normal_valid)
    result = segment(cloud, geometry=geometry, tile_size_pixels=4,
                     absolute_point_jump_m=2.0)
    assert result.count == 1
    assert result.pixel_counts.tolist() == [3]
    np.testing.assert_allclose(result.centroids_xyz[0], [1 / 3, 1 / 3, 1])
    np.testing.assert_allclose(result.mean_rgb[0], [30, 40, 50])
    np.testing.assert_allclose(result.mean_normals[0], [0, 0, -1])
    assert result.normal_valid_fraction[0] == pytest.approx(2 / 3)
    np.testing.assert_array_equal(result.bounding_boxes_vuvu[0], [0, 0, 2, 2])
    assert np.linalg.norm(result.mean_normals[0]) == pytest.approx(1.0)


def test_mean_normal_is_nan_without_valid_contributions() -> None:
    """Report NaN mean normal when a patch has no valid normal samples."""
    cloud = cloud_from_valid(np.ones((2, 2), dtype=bool))
    geometry = geometry_for(cloud, normal_valid=np.zeros((2, 2), dtype=bool))
    result = segment(cloud, geometry=geometry, tile_size_pixels=4)
    assert np.isnan(result.mean_normals).all()
    assert np.all(result.normal_valid_fraction == 0.0)


def test_outputs_are_read_only_and_inputs_unchanged() -> None:
    """Protect output ownership and leave all Phase 2/3 arrays unchanged."""
    cloud = cloud_from_valid(np.ones((3, 3), dtype=bool))
    geometry = geometry_for(cloud)
    xyz_before = cloud.xyz.copy()
    normals_before = geometry.normals.copy()
    result = segment(cloud, geometry=geometry, tile_size_pixels=4)
    for array in (
        result.labels, result.pixel_counts, result.centroids_xyz,
        result.mean_rgb, result.mean_normals, result.normal_valid_fraction,
        result.bounding_boxes_vuvu,
    ):
        assert not array.flags.writeable
    np.testing.assert_array_equal(cloud.xyz, xyz_before)
    np.testing.assert_array_equal(geometry.normals, normals_before)


def test_repeated_execution_is_fully_deterministic() -> None:
    """Return identical labels and statistics on repeated execution."""
    cloud = cloud_from_valid(np.ones((5, 7), dtype=bool))
    geometry = geometry_for(cloud)
    first = segment(cloud, geometry=geometry, tile_size_pixels=3)
    second = segment(cloud, geometry=geometry, tile_size_pixels=3)
    assert first.count == second.count
    for name in (
        "labels", "pixel_counts", "centroids_xyz", "mean_rgb",
        "mean_normals", "normal_valid_fraction", "bounding_boxes_vuvu",
    ):
        np.testing.assert_array_equal(getattr(first, name), getattr(second, name))


@pytest.mark.parametrize(
    "updates,exception",
    [
        ({"tile_size_pixels": 1}, ValueError),
        ({"tile_size_pixels": True}, TypeError),
        ({"connectivity": 6}, ValueError),
        ({"connectivity": True}, ValueError),
        ({"maximum_normal_angle_deg": 0.0}, ValueError),
        ({"maximum_normal_angle_deg": 181.0}, ValueError),
        ({"maximum_normal_angle_deg": np.nan}, ValueError),
        ({"absolute_point_jump_m": 0.0}, ValueError),
        ({"relative_point_jump": -1.0}, ValueError),
        ({"maximum_color_distance": 0.0}, ValueError),
        ({"maximum_color_distance": False}, TypeError),
        ({"require_valid_normals": 1}, TypeError),
    ],
)
def test_invalid_configuration_is_rejected(updates: dict, exception: type) -> None:
    """Reject invalid thresholds, choices, and ambiguous boolean numerics."""
    with pytest.raises(exception):
        PatchSegmentationConfig(**updates)


def test_cloud_and_geometry_shape_mismatch_is_rejected() -> None:
    """Require all Phase 2 and Phase 3 arrays to share organized dimensions."""
    cloud = cloud_from_valid(np.ones((2, 2), dtype=bool))
    wrong_cloud = cloud_from_valid(np.ones((3, 3), dtype=bool))
    with pytest.raises(PatchSegmentationError, match="contract"):
        segment_surface_patches(
            cloud, geometry_for(wrong_cloud), PatchSegmentationConfig()
        )


def test_real_scene_patch_diagnostic() -> None:
    """Diagnose real patches without modifying or using labels in segmentation."""
    configured = os.environ.get("PICKCELL_TEST_SCENE")
    if not configured:
        pytest.skip("PICKCELL_TEST_SCENE is not set; real patch diagnostic not run")
    path = Path(configured)
    before = tree_digest(path)
    scene = load_scene(path)
    cloud = rgbd_to_organized_cloud(scene)
    normal_config = NormalEstimationConfig()
    geometry = compute_organized_geometry(cloud, normal_config)
    config = PatchSegmentationConfig()
    start = time.perf_counter()
    result = segment_surface_patches(cloud, geometry, config)
    elapsed = time.perf_counter() - start
    repeated = segment_surface_patches(cloud, geometry, config)

    assert np.all(result.labels[cloud.valid] >= 0)
    assert np.all(result.labels[~cloud.valid] == -1)
    np.testing.assert_array_equal(
        np.unique(result.labels[cloud.valid]), np.arange(result.count)
    )
    np.testing.assert_array_equal(result.labels, repeated.labels)
    assert tree_digest(path) == before

    sizes = result.pixel_counts
    diagnostic = {
        "valid_points": int(np.count_nonzero(cloud.valid)),
        "patch_count": result.count,
        "minimum_patch_size": int(sizes.min()) if sizes.size else 0,
        "median_patch_size": float(np.median(sizes)) if sizes.size else 0.0,
        "mean_patch_size": float(np.mean(sizes)) if sizes.size else 0.0,
        "maximum_patch_size": int(sizes.max(initial=0)),
        "singleton_patches": int(np.count_nonzero(sizes == 1)),
        "runtime_seconds": elapsed,
        "configuration": config,
    }
    if scene.ground_truth is not None:
        majority_ids = []
        purities = []
        nonzero_impurities = []
        mixed_nonzero = 0
        background_only = 0
        for patch_id in range(result.count):
            ids, counts = np.unique(
                scene.ground_truth[result.labels == patch_id], return_counts=True
            )
            majority_ids.append(int(ids[np.argmax(counts)]))
            purities.append(float(counts.max() / counts.sum()))
            nonzero_counts = counts[ids != 0]
            if nonzero_counts.size == 0:
                background_only += 1
            else:
                impurity = 1.0 - float(nonzero_counts.max() / nonzero_counts.sum())
                nonzero_impurities.append(impurity)
                mixed_nonzero += int(np.count_nonzero(nonzero_counts) > 1)
        diagnostic.update({
            "majority_instance_ids": majority_ids,
            "mean_patch_purity": float(np.mean(purities)),
            "mixed_nonzero_instance_patches": mixed_nonzero,
            "maximum_nonzero_impurity": max(nonzero_impurities, default=0.0),
            "mean_nonzero_impurity": (
                float(np.mean(nonzero_impurities)) if nonzero_impurities else 0.0
            ),
            "background_only_patches": background_only,
        })
    print(diagnostic)
