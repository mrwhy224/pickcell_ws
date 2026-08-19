"""Deterministic undirected patch adjacency and raw edge features."""

import numpy as np

from .models import OrganizedCloud
from .models import OrganizedGeometry
from .models import PatchGraph
from .models import PatchGraphConfig
from .models import PatchSet


PATCH_GRAPH_FEATURE_NAMES = (
    "shared_boundary_count",
    "diagonal_contact_count",
    "shared_boundary_fraction_min_patch",
    "centroid_distance_m",
    "centroid_depth_difference_m",
    "patch_size_ratio",
    "mean_boundary_point_distance_m",
    "max_boundary_point_distance_m",
    "mean_boundary_depth_difference_m",
    "max_boundary_depth_difference_m",
    "depth_edge_endpoint_fraction",
    "normal_pair_valid_fraction",
    "mean_unsigned_normal_angle_deg",
    "max_unsigned_normal_angle_deg",
    "mean_boundary_rgb_distance",
    "max_boundary_rgb_distance",
    "mean_patch_rgb_distance",
    "cross_tile_boundary_fraction",
)


class PatchGraphError(ValueError):
    """Indicate invalid inputs to patch graph construction."""


def _validate_inputs(
    cloud: OrganizedCloud, geometry: OrganizedGeometry, patches: PatchSet
) -> tuple[int, int]:
    """Validate aligned cloud, geometry, patch labels, and tile metadata."""
    shape = (cloud.height, cloud.width)
    contracts = (
        (cloud.xyz, (*shape, 3), np.float32, "cloud XYZ"),
        (cloud.rgb, (*shape, 3), np.uint8, "cloud RGB"),
        (cloud.valid, shape, np.bool_, "cloud valid"),
        (geometry.normals, (*shape, 3), np.float32, "normals"),
        (geometry.normal_valid, shape, np.bool_, "normal valid"),
        (geometry.depth_edges, shape, np.bool_, "depth edges"),
        (patches.labels, shape, np.int32, "patch labels"),
    )
    for array, expected_shape, dtype, name in contracts:
        if array.shape != expected_shape or array.dtype != dtype:
            raise PatchGraphError(f"{name} contract is invalid")
    count = patches.count
    patch_contracts = (
        (patches.pixel_counts, (count,), "pixel_counts"),
        (patches.centroids_xyz, (count, 3), "centroids_xyz"),
        (patches.mean_rgb, (count, 3), "mean_rgb"),
        (patches.tile_indices, (count, 2), "tile_indices"),
    )
    for array, expected_shape, name in patch_contracts:
        if array.shape != expected_shape:
            raise PatchGraphError(f"patch {name} shape is invalid")
    labels = patches.labels
    if np.any(labels[~cloud.valid] != -1):
        raise PatchGraphError("invalid XYZ pixels must have patch label -1")
    if np.any(labels[cloud.valid] < 0) or np.any(labels[cloud.valid] >= count):
        raise PatchGraphError("valid pixels contain invalid patch IDs")
    present = np.unique(labels[labels >= 0])
    if not np.array_equal(present, np.arange(count, dtype=present.dtype)):
        raise PatchGraphError("patch IDs must be contiguous")
    if patches.source_tile_size_pixels < 2:
        raise PatchGraphError("source tile size is invalid")
    if patches.tile_indices.dtype != np.int32:
        raise PatchGraphError("tile_indices must use int32")
    if count:
        rows, columns = np.nonzero(labels >= 0)
        expected_tiles = np.stack((
            rows // patches.source_tile_size_pixels,
            columns // patches.source_tile_size_pixels,
        ), axis=1)
        if np.any(patches.tile_indices[labels[labels >= 0]] != expected_tiles):
            raise PatchGraphError("patch tile metadata disagrees with labels")
    return shape


def _contact_records(
    labels: np.ndarray, directions: tuple[tuple[int, int], ...], count: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Extract canonical contacting patch pairs and endpoint coordinates."""
    height, width = labels.shape
    keys_parts = []
    va_parts = []
    ua_parts = []
    vb_parts = []
    ub_parts = []
    for delta_v, delta_u in directions:
        rows_a = slice(0, height - delta_v)
        rows_b = slice(delta_v, height)
        if delta_u >= 0:
            cols_a = slice(0, width - delta_u)
            cols_b = slice(delta_u, width)
        else:
            cols_a = slice(-delta_u, width)
            cols_b = slice(0, width + delta_u)
        first = labels[rows_a, cols_a]
        second = labels[rows_b, cols_b]
        contact = (first >= 0) & (second >= 0) & (first != second)
        local_v, local_u = np.nonzero(contact)
        va = local_v + (rows_a.start or 0)
        ua = local_u + (cols_a.start or 0)
        vb = va + delta_v
        ub = ua + delta_u
        first_ids = labels[va, ua]
        second_ids = labels[vb, ub]
        lower = np.minimum(first_ids, second_ids).astype(np.int64)
        upper = np.maximum(first_ids, second_ids).astype(np.int64)
        keys_parts.append(lower * count + upper)
        va_parts.append(va)
        ua_parts.append(ua)
        vb_parts.append(vb)
        ub_parts.append(ub)
    if not keys_parts:
        empty = np.empty(0, dtype=np.int64)
        return empty, empty, empty, empty, empty
    return tuple(np.concatenate(parts) for parts in (
        keys_parts, va_parts, ua_parts, vb_parts, ub_parts
    ))


def _aggregate_counts(keys: np.ndarray, edge_keys: np.ndarray) -> np.ndarray:
    """Count contacts per lexicographically ordered graph edge."""
    if keys.size == 0:
        return np.zeros(edge_keys.size, dtype=np.int64)
    indices = np.searchsorted(edge_keys, keys)
    return np.bincount(indices, minlength=edge_keys.size).astype(np.int64)


def _orthogonal_statistics(
    cloud: OrganizedCloud,
    geometry: OrganizedGeometry,
    keys: np.ndarray,
    coordinates: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
    edge_keys: np.ndarray,
    missing: float,
) -> tuple[np.ndarray, ...]:
    """Aggregate raw orthogonal boundary measurements for each edge."""
    edge_count = edge_keys.size
    shared = _aggregate_counts(keys, edge_keys)
    if keys.size == 0:
        missing_values = tuple(
            np.full(edge_count, missing, dtype=np.float32) for _ in range(10)
        )
        values = list(missing_values)
        values[5] = np.zeros(edge_count, dtype=np.float32)
        return (shared,) + tuple(values)
    va, ua, vb, ub = coordinates
    edge_indices = np.searchsorted(edge_keys, keys)
    points_a = cloud.xyz[va, ua]
    points_b = cloud.xyz[vb, ub]
    point_distance = np.linalg.norm(points_a - points_b, axis=1)
    depth_difference = np.abs(points_a[:, 2] - points_b[:, 2])
    edge_endpoint = geometry.depth_edges[va, ua] | geometry.depth_edges[vb, ub]
    normal_pair_valid = (
        geometry.normal_valid[va, ua] & geometry.normal_valid[vb, ub]
    )
    dots = np.abs(np.sum(
        geometry.normals[va, ua] * geometry.normals[vb, ub], axis=1
    ))
    angles = np.degrees(np.arccos(np.clip(dots, 0.0, 1.0)))
    rgb_distance = np.linalg.norm(
        cloud.rgb[va, ua].astype(np.float32)
        - cloud.rgb[vb, ub].astype(np.float32), axis=1
    )

    def mean_and_max(values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        sums = np.bincount(
            edge_indices, weights=values, minlength=edge_count
        )
        means = (sums / shared).astype(np.float32)
        maxima = np.full(edge_count, -np.inf, dtype=np.float64)
        np.maximum.at(maxima, edge_indices, values)
        return means, maxima.astype(np.float32)

    point_mean, point_max = mean_and_max(point_distance)
    depth_mean, depth_max = mean_and_max(depth_difference)
    rgb_mean, rgb_max = mean_and_max(rgb_distance)
    depth_edge_fraction = (
        np.bincount(edge_indices, weights=edge_endpoint, minlength=edge_count)
        / shared
    ).astype(np.float32)
    normal_counts = np.bincount(
        edge_indices, weights=normal_pair_valid, minlength=edge_count
    )
    normal_fraction = (normal_counts / shared).astype(np.float32)
    angle_mean = np.full(edge_count, missing, dtype=np.float32)
    angle_max = np.full(edge_count, missing, dtype=np.float32)
    if np.any(normal_pair_valid):
        valid_edges = edge_indices[normal_pair_valid]
        valid_angles = angles[normal_pair_valid]
        angle_sums = np.bincount(
            valid_edges, weights=valid_angles, minlength=edge_count
        )
        usable = normal_counts > 0
        angle_mean[usable] = (angle_sums[usable] / normal_counts[usable])
        maxima = np.full(edge_count, -np.inf, dtype=np.float64)
        np.maximum.at(maxima, valid_edges, valid_angles)
        angle_max[usable] = maxima[usable]
    return (
        shared, point_mean, point_max, depth_mean, depth_max,
        depth_edge_fraction, normal_fraction, angle_mean, angle_max,
        rgb_mean, rgb_max,
    )


def build_patch_graph(
    cloud: OrganizedCloud,
    geometry: OrganizedGeometry,
    patches: PatchSet,
    config: PatchGraphConfig,
) -> PatchGraph:
    """Build contact adjacency and stable raw features without affinity logic."""
    _validate_inputs(cloud, geometry, patches)
    count = patches.count
    orthogonal = _contact_records(patches.labels, ((0, 1), (1, 0)), count)
    if config.include_diagonal_contacts:
        diagonal = _contact_records(patches.labels, ((1, 1), (1, -1)), count)
    else:
        empty = np.empty(0, dtype=np.int64)
        diagonal = (empty, empty, empty, empty, empty)
    edge_keys = np.unique(np.concatenate((orthogonal[0], diagonal[0])))
    edge_count = edge_keys.size
    if edge_count:
        edges = np.stack((edge_keys // count, edge_keys % count), axis=1).astype(
            np.int32
        )
    else:
        edges = np.empty((0, 2), dtype=np.int32)
    diagonal_counts = _aggregate_counts(diagonal[0], edge_keys)
    orth_stats = _orthogonal_statistics(
        cloud, geometry, orthogonal[0], orthogonal[1:], edge_keys,
        float(config.missing_feature_value),
    )
    shared = orth_stats[0]
    missing = np.float32(config.missing_feature_value)
    features = np.full(
        (edge_count, len(PATCH_GRAPH_FEATURE_NAMES)), missing, dtype=np.float32
    )
    if edge_count:
        first_ids, second_ids = edges[:, 0], edges[:, 1]
        sizes_a = patches.pixel_counts[first_ids].astype(np.float32)
        sizes_b = patches.pixel_counts[second_ids].astype(np.float32)
        centroid_delta = (
            patches.centroids_xyz[first_ids] - patches.centroids_xyz[second_ids]
        )
        tile_difference = np.any(
            patches.tile_indices[first_ids] != patches.tile_indices[second_ids],
            axis=1,
        )
        features[:, 0] = shared
        features[:, 1] = diagonal_counts
        features[:, 2] = shared / np.minimum(sizes_a, sizes_b)
        features[:, 3] = np.linalg.norm(centroid_delta, axis=1)
        features[:, 4] = np.abs(centroid_delta[:, 2])
        features[:, 5] = np.minimum(sizes_a, sizes_b) / np.maximum(sizes_a, sizes_b)
        features[:, 6:16] = np.stack(orth_stats[1:], axis=1)
        features[:, 16] = np.linalg.norm(
            patches.mean_rgb[first_ids] - patches.mean_rgb[second_ids], axis=1
        )
        features[:, 17] = tile_difference.astype(np.float32)
    degrees = np.zeros(count, dtype=np.int64)
    if edge_count:
        np.add.at(degrees, edges[:, 0], 1)
        np.add.at(degrees, edges[:, 1], 1)
    if not np.all(np.isfinite(features)):
        raise PatchGraphError("edge feature calculation produced nonfinite values")
    return PatchGraph(
        edges=edges,
        edge_features=features,
        feature_names=PATCH_GRAPH_FEATURE_NAMES,
        shared_boundary_counts=shared,
        diagonal_contact_counts=diagonal_counts,
        node_degrees=degrees,
    )
