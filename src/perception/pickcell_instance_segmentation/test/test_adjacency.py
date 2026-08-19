"""Tests for deterministic patch contact adjacency and raw features."""

import hashlib
import os
from pathlib import Path
import time

import numpy as np
import pytest

from pickcell_instance_segmentation import build_patch_graph
from pickcell_instance_segmentation import compute_organized_geometry
from pickcell_instance_segmentation import load_scene
from pickcell_instance_segmentation import NormalEstimationConfig
from pickcell_instance_segmentation import OrganizedCloud
from pickcell_instance_segmentation import OrganizedGeometry
from pickcell_instance_segmentation import PATCH_GRAPH_FEATURE_NAMES
from pickcell_instance_segmentation import PatchGraphConfig
from pickcell_instance_segmentation import PatchGraphError
from pickcell_instance_segmentation import PatchSegmentationConfig
from pickcell_instance_segmentation import PatchSet
from pickcell_instance_segmentation import rgbd_to_organized_cloud
from pickcell_instance_segmentation import segment_surface_patches


def cloud_for_labels(
    labels: np.ndarray,
    *,
    xyz: np.ndarray | None = None,
    rgb: np.ndarray | None = None,
) -> OrganizedCloud:
    """Construct a valid cloud wherever a synthetic patch label is present."""
    labels = np.asarray(labels, dtype=np.int32)
    height, width = labels.shape
    valid = labels >= 0
    if xyz is None:
        rows, columns = np.indices(labels.shape, dtype=np.float32)
        xyz = np.stack((columns * 0.01, rows * 0.01,
                        np.ones(labels.shape, np.float32)), axis=2)
    xyz = np.asarray(xyz, dtype=np.float32).copy()
    xyz[~valid] = np.nan
    if rgb is None:
        rgb = np.zeros((height, width, 3), dtype=np.uint8)
    return OrganizedCloud(
        xyz, rgb, valid, width, height, None,
        "opencv_optical_frame:+x_right,+y_down,+z_forward",
    )


def geometry_for(
    cloud: OrganizedCloud,
    *,
    normals: np.ndarray | None = None,
    normal_valid: np.ndarray | None = None,
    depth_edges: np.ndarray | None = None,
) -> OrganizedGeometry:
    """Construct controlled organized geometry for feature tests."""
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
        normals, normal_valid, depth_edges,
        np.zeros(shape, dtype=np.float32),
    )


def patches_for_labels(
    labels: np.ndarray,
    cloud: OrganizedCloud,
    geometry: OrganizedGeometry,
    *,
    tile_size: int = 12,
) -> PatchSet:
    """Build exact synthetic PatchSet statistics from prescribed labels."""
    labels = np.asarray(labels, dtype=np.int32)
    count = int(labels.max(initial=-1)) + 1
    pixel_counts = np.zeros(count, dtype=np.int64)
    centroids = np.empty((count, 3), dtype=np.float32)
    mean_rgb = np.empty((count, 3), dtype=np.float32)
    mean_normals = np.full((count, 3), np.nan, dtype=np.float32)
    fractions = np.zeros(count, dtype=np.float32)
    boxes = np.empty((count, 4), dtype=np.int32)
    tiles = np.empty((count, 2), dtype=np.int32)
    for patch_id in range(count):
        mask = labels == patch_id
        rows, columns = np.nonzero(mask)
        pixel_counts[patch_id] = mask.sum()
        centroids[patch_id] = cloud.xyz[mask].mean(axis=0)
        mean_rgb[patch_id] = cloud.rgb[mask].mean(axis=0)
        normal_mask = mask & geometry.normal_valid
        fractions[patch_id] = normal_mask.sum() / mask.sum()
        if normal_mask.any():
            value = geometry.normals[normal_mask].sum(axis=0)
            mean_normals[patch_id] = value / np.linalg.norm(value)
        boxes[patch_id] = [rows.min(), columns.min(), rows.max() + 1,
                           columns.max() + 1]
        tiles[patch_id] = [rows[0] // tile_size, columns[0] // tile_size]
        assert np.all(rows // tile_size == tiles[patch_id, 0])
        assert np.all(columns // tile_size == tiles[patch_id, 1])
    return PatchSet(
        labels, count, pixel_counts, centroids, mean_rgb, mean_normals,
        fractions, boxes, tiles, tile_size,
    )


def graph_for(
    labels: np.ndarray,
    *,
    cloud: OrganizedCloud | None = None,
    geometry: OrganizedGeometry | None = None,
    tile_size: int = 12,
    **config,
):
    """Build a graph from prescribed in-memory labels."""
    labels = np.asarray(labels, dtype=np.int32)
    if cloud is None:
        cloud = cloud_for_labels(labels)
    if geometry is None:
        geometry = geometry_for(cloud)
    patches = patches_for_labels(
        labels, cloud, geometry, tile_size=tile_size
    )
    return build_patch_graph(cloud, geometry, patches, PatchGraphConfig(**config))


def feature(graph, name: str) -> np.ndarray:
    """Return one named stable graph feature column."""
    return graph.edge_features[:, graph.feature_names.index(name)]


def tree_digest(path: Path) -> dict[str, str]:
    """Hash every regular file in a diagnostic scene."""
    return {
        str(item.relative_to(path)): hashlib.sha256(item.read_bytes()).hexdigest()
        for item in sorted(path.rglob("*")) if item.is_file()
    }


def test_orthogonal_contacts_aggregate_into_one_canonical_edge() -> None:
    """Aggregate repeated physical boundary contacts without duplicates."""
    graph = graph_for([[0, 1], [0, 1]])
    np.testing.assert_array_equal(graph.edges, [[0, 1]])
    assert graph.shared_boundary_counts.tolist() == [2]
    assert graph.diagonal_contact_counts.tolist() == [0]


def test_edges_are_canonical_lexicographic_unique_and_not_self() -> None:
    """Sort canonical unique edges independently of label encounter direction."""
    graph = graph_for([[2, 1, 0]])
    np.testing.assert_array_equal(graph.edges, [[0, 1], [1, 2]])
    assert np.all(graph.edges[:, 0] < graph.edges[:, 1])
    assert np.unique(graph.edges, axis=0).shape == graph.edges.shape


def test_invalid_gap_creates_no_edge() -> None:
    """Never invent adjacency across invalid pixels."""
    graph = graph_for([[0, -1, 1]])
    assert graph.edges.shape == (0, 2)
    np.testing.assert_array_equal(graph.node_degrees, [0, 0])


def test_depth_edge_does_not_remove_contact_adjacency() -> None:
    """Keep candidate graph edges across detected object boundaries."""
    labels = np.array([[0, 1]], dtype=np.int32)
    cloud = cloud_for_labels(labels)
    geometry = geometry_for(cloud, depth_edges=np.ones((1, 2), dtype=bool))
    graph = graph_for(labels, cloud=cloud, geometry=geometry)
    np.testing.assert_array_equal(graph.edges, [[0, 1]])
    assert feature(graph, "depth_edge_endpoint_fraction")[0] == 1.0


def test_cross_tile_and_same_tile_features() -> None:
    """Expose whether contacting patches originate from different source tiles."""
    cross = graph_for([[0, 0, 1, 1]], tile_size=2)
    same = graph_for([[0, 1]], tile_size=2)
    assert feature(cross, "cross_tile_boundary_fraction")[0] == 1.0
    assert feature(same, "cross_tile_boundary_fraction")[0] == 0.0


def test_boundary_fraction_centroids_depth_and_size_ratio() -> None:
    """Calculate normalized boundary and patch-level geometry exactly."""
    labels = np.array([[0, 1], [0, 1]], dtype=np.int32)
    cloud = cloud_for_labels(labels)
    graph = graph_for(labels, cloud=cloud)
    assert feature(graph, "shared_boundary_count")[0] == 2.0
    assert feature(graph, "shared_boundary_fraction_min_patch")[0] == 1.0
    assert feature(graph, "centroid_distance_m")[0] == pytest.approx(0.01)
    assert feature(graph, "centroid_depth_difference_m")[0] == 0.0
    assert feature(graph, "patch_size_ratio")[0] == 1.0


def test_exact_boundary_xyz_depth_normal_and_rgb_features() -> None:
    """Preserve raw metric, angular, and RGB units on one boundary pair."""
    labels = np.array([[0, 1]], dtype=np.int32)
    xyz = np.array([[[0.0, 0.0, 1.0], [0.03, 0.0, 1.1]]], np.float32)
    rgb = np.array([[[0, 0, 0], [3, 4, 0]]], dtype=np.uint8)
    cloud = cloud_for_labels(labels, xyz=xyz, rgb=rgb)
    normals = np.array([[[0, 0, -1], [1, 0, 0]]], dtype=np.float32)
    geometry = geometry_for(
        cloud, normals=normals, depth_edges=np.array([[1, 0]], dtype=bool)
    )
    graph = graph_for(labels, cloud=cloud, geometry=geometry)
    distance = np.linalg.norm(xyz[0, 0] - xyz[0, 1])
    for name in ("mean_boundary_point_distance_m",
                 "max_boundary_point_distance_m"):
        assert feature(graph, name)[0] == pytest.approx(distance)
    for name in ("mean_boundary_depth_difference_m",
                 "max_boundary_depth_difference_m"):
        assert feature(graph, name)[0] == pytest.approx(0.1)
    assert feature(graph, "depth_edge_endpoint_fraction")[0] == 1.0
    assert feature(graph, "normal_pair_valid_fraction")[0] == 1.0
    assert feature(graph, "mean_unsigned_normal_angle_deg")[0] == 90.0
    assert feature(graph, "max_unsigned_normal_angle_deg")[0] == 90.0
    assert feature(graph, "mean_boundary_rgb_distance")[0] == 5.0
    assert feature(graph, "max_boundary_rgb_distance")[0] == 5.0
    assert feature(graph, "mean_patch_rgb_distance")[0] == 5.0


def test_missing_normals_use_finite_configured_values() -> None:
    """Keep explicit normal missingness without NaN feature values."""
    labels = np.array([[0, 1]], dtype=np.int32)
    cloud = cloud_for_labels(labels)
    geometry = geometry_for(cloud, normal_valid=np.zeros((1, 2), dtype=bool))
    graph = graph_for(
        labels, cloud=cloud, geometry=geometry, missing_feature_value=-7.0
    )
    assert feature(graph, "normal_pair_valid_fraction")[0] == 0.0
    assert feature(graph, "mean_unsigned_normal_angle_deg")[0] == -7.0
    assert feature(graph, "max_unsigned_normal_angle_deg")[0] == -7.0
    assert np.isfinite(graph.edge_features).all()


def test_diagonal_contact_is_optional_and_counted_separately() -> None:
    """Distinguish diagonal contact count from orthogonal shared boundary."""
    diagonal_labels = np.array([[0, -1], [-1, 1]], dtype=np.int32)
    disabled = graph_for(diagonal_labels)
    enabled = graph_for(
        diagonal_labels, include_diagonal_contacts=True,
        missing_feature_value=-7.0,
    )
    assert disabled.edges.shape == (0, 2)
    np.testing.assert_array_equal(enabled.edges, [[0, 1]])
    assert enabled.shared_boundary_counts.tolist() == [0]
    assert enabled.diagonal_contact_counts.tolist() == [1]
    assert feature(enabled, "mean_boundary_point_distance_m")[0] == -7.0
    assert feature(enabled, "normal_pair_valid_fraction")[0] == 0.0

    mixed = graph_for([[0, 1], [0, 1]], include_diagonal_contacts=True)
    assert mixed.shared_boundary_counts.tolist() == [2]
    assert mixed.diagonal_contact_counts.tolist() == [2]


def test_node_degrees_include_isolated_patches() -> None:
    """Count incident unique edges while preserving isolated nodes."""
    graph = graph_for([[0, 1, 2], [-1, -1, -1], [3, -1, -1]])
    np.testing.assert_array_equal(graph.edges, [[0, 1], [1, 2]])
    np.testing.assert_array_equal(graph.node_degrees, [1, 2, 1, 0])


def test_empty_graphs_with_one_or_zero_patches() -> None:
    """Return stable empty edge arrays for singleton and empty patch sets."""
    one = graph_for([[0]])
    empty = graph_for([[-1]])
    assert one.edges.shape == empty.edges.shape == (0, 2)
    assert one.edge_features.shape == empty.edge_features.shape == (
        0, len(PATCH_GRAPH_FEATURE_NAMES)
    )
    assert one.node_degrees.tolist() == [0]
    assert empty.node_degrees.shape == (0,)


def test_shape_and_invalid_patch_ids_are_rejected() -> None:
    """Reject mismatched organized shapes and invalid label contracts."""
    labels = np.array([[0, 1]], dtype=np.int32)
    cloud = cloud_for_labels(labels)
    geometry = geometry_for(cloud)
    patches = patches_for_labels(labels, cloud, geometry)
    wrong_cloud = cloud_for_labels([[0], [1]])
    with pytest.raises(PatchGraphError, match="contract"):
        build_patch_graph(
            wrong_cloud, geometry, patches, PatchGraphConfig()
        )
    bad_labels = patches.labels.copy()
    bad_labels[0, 1] = 3
    bad = PatchSet(
        bad_labels, patches.count, patches.pixel_counts,
        patches.centroids_xyz, patches.mean_rgb, patches.mean_normals,
        patches.normal_valid_fraction, patches.bounding_boxes_vuvu,
        patches.tile_indices, patches.source_tile_size_pixels,
    )
    with pytest.raises(PatchGraphError, match="patch IDs"):
        build_patch_graph(cloud, geometry, bad, PatchGraphConfig())


def test_stable_names_dtypes_shapes_and_read_only_outputs() -> None:
    """Expose one immutable stable feature order and exact graph contracts."""
    graph = graph_for([[0, 1]])
    assert graph.feature_names == PATCH_GRAPH_FEATURE_NAMES
    assert len(graph.feature_names) == 18
    assert graph.edges.dtype == np.int32 and graph.edges.shape == (1, 2)
    assert graph.edge_features.dtype == np.float32
    assert graph.edge_features.shape == (1, 18)
    assert np.issubdtype(graph.shared_boundary_counts.dtype, np.integer)
    assert np.issubdtype(graph.diagonal_contact_counts.dtype, np.integer)
    assert np.issubdtype(graph.node_degrees.dtype, np.integer)
    for array in (
        graph.edges, graph.edge_features, graph.shared_boundary_counts,
        graph.diagonal_contact_counts, graph.node_degrees,
    ):
        assert not array.flags.writeable


def test_inputs_unchanged_and_repeated_graph_is_deterministic() -> None:
    """Never mutate inputs and reproduce every graph output exactly."""
    labels = np.array([[0, 1], [0, 1]], dtype=np.int32)
    cloud = cloud_for_labels(labels)
    geometry = geometry_for(cloud)
    patches = patches_for_labels(labels, cloud, geometry)
    xyz_before = cloud.xyz.copy()
    labels_before = patches.labels.copy()
    first = build_patch_graph(cloud, geometry, patches, PatchGraphConfig())
    second = build_patch_graph(cloud, geometry, patches, PatchGraphConfig())
    for name in (
        "edges", "edge_features", "shared_boundary_counts",
        "diagonal_contact_counts", "node_degrees",
    ):
        np.testing.assert_array_equal(getattr(first, name), getattr(second, name))
    np.testing.assert_array_equal(cloud.xyz, xyz_before)
    np.testing.assert_array_equal(patches.labels, labels_before)


@pytest.mark.parametrize(
    "updates,exception",
    [
        ({"include_diagonal_contacts": 1}, TypeError),
        ({"missing_feature_value": True}, TypeError),
        ({"missing_feature_value": np.nan}, ValueError),
        ({"missing_feature_value": np.inf}, ValueError),
    ],
)
def test_invalid_graph_configuration(updates: dict, exception: type) -> None:
    """Reject ambiguous flags and nonfinite missing-feature values."""
    with pytest.raises(exception):
        PatchGraphConfig(**updates)


def test_real_scene_graph_diagnostic() -> None:
    """Diagnose a real graph without using labels during construction."""
    configured = os.environ.get("PICKCELL_TEST_SCENE")
    if not configured:
        pytest.skip("PICKCELL_TEST_SCENE is not set; real graph diagnostic not run")
    path = Path(configured)
    before = tree_digest(path)
    scene = load_scene(path)
    cloud = rgbd_to_organized_cloud(scene)
    geometry = compute_organized_geometry(cloud, NormalEstimationConfig())
    patches = segment_surface_patches(
        cloud, geometry, PatchSegmentationConfig()
    )
    config = PatchGraphConfig()
    start = time.perf_counter()
    graph = build_patch_graph(cloud, geometry, patches, config)
    elapsed = time.perf_counter() - start
    repeated = build_patch_graph(cloud, geometry, patches, config)

    assert np.all(graph.edges >= 0) and np.all(graph.edges < patches.count)
    assert np.all(graph.edges[:, 0] < graph.edges[:, 1])
    assert np.unique(graph.edges, axis=0).shape == graph.edges.shape
    assert np.isfinite(graph.edge_features).all()
    np.testing.assert_array_equal(graph.edges, repeated.edges)
    np.testing.assert_array_equal(graph.edge_features, repeated.edge_features)
    assert tree_digest(path) == before

    degrees = graph.node_degrees
    cross_tile = feature(graph, "cross_tile_boundary_fraction")
    depth_edge = feature(graph, "depth_edge_endpoint_fraction")
    diagnostic = {
        "patch_count": patches.count,
        "edge_count": len(graph.edges),
        "isolated_patches": int(np.count_nonzero(degrees == 0)),
        "degree_minimum": int(degrees.min()) if degrees.size else 0,
        "degree_median": float(np.median(degrees)) if degrees.size else 0.0,
        "degree_mean": float(np.mean(degrees)) if degrees.size else 0.0,
        "degree_maximum": int(degrees.max()) if degrees.size else 0,
        "cross_tile_edge_percentage": (
            float(np.mean(cross_tile > 0) * 100) if cross_tile.size else 0.0
        ),
        "depth_edge_touch_percentage": (
            float(np.mean(depth_edge > 0) * 100) if depth_edge.size else 0.0
        ),
        "runtime_seconds": elapsed,
        "features": {},
    }
    for index, name in enumerate(graph.feature_names):
        values = graph.edge_features[:, index]
        diagnostic["features"][name] = {
            "min": float(values.min()) if values.size else 0.0,
            "median": float(np.median(values)) if values.size else 0.0,
            "mean": float(values.mean()) if values.size else 0.0,
            "max": float(values.max()) if values.size else 0.0,
        }
    if scene.ground_truth is not None:
        majority = np.full(patches.count, -1, dtype=np.int32)
        ambiguous = np.zeros(patches.count, dtype=bool)
        for patch_id in range(patches.count):
            values = scene.ground_truth[patches.labels == patch_id]
            nonzero = values[values != 0]
            if nonzero.size:
                ids, counts = np.unique(nonzero, return_counts=True)
                majority[patch_id] = ids[np.argmax(counts)]
                ambiguous[patch_id] = np.count_nonzero(
                    counts == counts.max()
                ) > 1
        edge_a = graph.edges[:, 0]
        edge_b = graph.edges[:, 1]
        usable = (
            (majority[edge_a] > 0) & (majority[edge_b] > 0)
            & ~ambiguous[edge_a] & ~ambiguous[edge_b]
        )
        same = usable & (majority[edge_a] == majority[edge_b])
        different = usable & (majority[edge_a] != majority[edge_b])
        other = ~usable
        diagnostic["label_edges"] = {
            "same_instance": int(np.count_nonzero(same)),
            "different_nonzero_instance": int(np.count_nonzero(different)),
            "background_or_ambiguous": int(np.count_nonzero(other)),
            "same_feature_distributions": graph.edge_features[same].tolist(),
            "different_feature_distributions": graph.edge_features[different].tolist(),
        }
    print(diagnostic)
