"""Conservative deterministic organized surface-patch oversegmentation."""

import numpy as np

from .models import OrganizedCloud
from .models import OrganizedGeometry
from .models import PatchSegmentationConfig
from .models import PatchSet


class PatchSegmentationError(ValueError):
    """Indicate invalid or incompatible patch-segmentation inputs."""


def _validate_inputs(
    cloud: OrganizedCloud, geometry: OrganizedGeometry
) -> tuple[int, int]:
    """Validate matching Phase 2 and Phase 3 organized contracts."""
    shape = (cloud.height, cloud.width)
    if cloud.xyz.shape != (*shape, 3) or cloud.xyz.dtype != np.float32:
        raise PatchSegmentationError("cloud XYZ contract is invalid")
    if cloud.rgb.shape != (*shape, 3) or cloud.rgb.dtype != np.uint8:
        raise PatchSegmentationError("cloud RGB contract is invalid")
    if cloud.valid.shape != shape or cloud.valid.dtype != np.bool_:
        raise PatchSegmentationError("cloud validity contract is invalid")
    contracts = (
        (geometry.normals, (*shape, 3), np.float32, "normals"),
        (geometry.normal_valid, shape, np.bool_, "normal_valid"),
        (geometry.depth_edges, shape, np.bool_, "depth_edges"),
        (geometry.normal_variation, shape, np.float32, "normal_variation"),
    )
    for array, expected_shape, dtype, name in contracts:
        if array.shape != expected_shape or array.dtype != dtype:
            raise PatchSegmentationError(f"geometry {name} contract is invalid")
    if np.any(~np.isfinite(cloud.xyz[cloud.valid])):
        raise PatchSegmentationError("valid XYZ points must be finite")
    if np.any(~np.isfinite(geometry.normals[geometry.normal_valid])):
        raise PatchSegmentationError("valid normals must be finite")
    return shape


def _pair_slices(
    shape: tuple[int, int], delta_v: int, delta_u: int
) -> tuple[tuple[slice, slice], tuple[slice, slice]]:
    """Return aligned slices for one positive-discovery neighbor direction."""
    height, width = shape
    if abs(delta_v) >= height or abs(delta_u) >= width:
        empty = (slice(0, 0), slice(0, 0))
        return empty, empty
    rows_a = slice(0, height - delta_v)
    rows_b = slice(delta_v, height)
    if delta_u >= 0:
        cols_a = slice(0, width - delta_u)
        cols_b = slice(delta_u, width)
    else:
        cols_a = slice(-delta_u, width)
        cols_b = slice(0, width + delta_u)
    return (rows_a, cols_a), (rows_b, cols_b)


def _compatible_pairs(
    cloud: OrganizedCloud,
    geometry: OrganizedGeometry,
    config: PatchSegmentationConfig,
    first: tuple[slice, slice],
    second: tuple[slice, slice],
) -> np.ndarray:
    """Compute conservative local compatibility for aligned pixel pairs."""
    valid_a = cloud.valid[first]
    valid_b = cloud.valid[second]
    compatible = valid_a & valid_b
    points_a = cloud.xyz[first]
    points_b = cloud.xyz[second]
    distance = np.linalg.norm(points_a - points_b, axis=2)
    depth_minimum = np.minimum(points_a[..., 2], points_b[..., 2])
    limit = np.maximum(
        np.float32(config.absolute_point_jump_m),
        np.float32(config.relative_point_jump) * depth_minimum,
    )
    compatible &= np.isfinite(distance) & (distance <= limit)

    # Phase 3 represents discontinuities per pixel rather than per edge. The
    # conservative policy excludes connections involving either edge pixel.
    compatible &= ~geometry.depth_edges[first] & ~geometry.depth_edges[second]
    normals_a_valid = geometry.normal_valid[first]
    normals_b_valid = geometry.normal_valid[second]
    both_normals = normals_a_valid & normals_b_valid
    if config.require_valid_normals:
        compatible &= both_normals
    normals_a = geometry.normals[first]
    normals_b = geometry.normals[second]
    cosine = np.abs(np.sum(normals_a * normals_b, axis=2))
    cosine = np.clip(cosine, 0.0, 1.0)
    cosine_limit = np.cos(np.deg2rad(config.maximum_normal_angle_deg))
    normal_compatible = cosine >= cosine_limit
    compatible &= ~both_normals | normal_compatible

    if config.maximum_color_distance is not None:
        colors_a = cloud.rgb[first].astype(np.float32)
        colors_b = cloud.rgb[second].astype(np.float32)
        color_distance = np.linalg.norm(colors_a - colors_b, axis=2)
        compatible &= color_distance <= config.maximum_color_distance
    return compatible


def _union_components(
    cloud: OrganizedCloud,
    geometry: OrganizedGeometry,
    config: PatchSegmentationConfig,
) -> np.ndarray:
    """Union accepted local edges without ever crossing a tile boundary."""
    height, width = cloud.valid.shape
    size = height * width
    parent = np.arange(size, dtype=np.int64)

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = int(parent[index])
        return index

    def union(first_index: int, second_index: int) -> None:
        first_root = find(first_index)
        second_root = find(second_index)
        if first_root == second_root:
            return
        if first_root < second_root:
            parent[second_root] = first_root
        else:
            parent[first_root] = second_root

    directions = [(0, 1), (1, 0)]
    if config.connectivity == 8:
        directions.extend(((1, 1), (1, -1)))
    tile = config.tile_size_pixels
    shape = (height, width)
    for delta_v, delta_u in directions:
        first, second = _pair_slices(shape, delta_v, delta_u)
        compatible = _compatible_pairs(cloud, geometry, config, first, second)
        rows_a, columns_a = np.nonzero(compatible)
        rows_a += first[0].start or 0
        columns_a += first[1].start or 0
        rows_b = rows_a + delta_v
        columns_b = columns_a + delta_u
        same_tile = (
            (rows_a // tile == rows_b // tile)
            & (columns_a // tile == columns_b // tile)
        )
        indices_a = rows_a[same_tile] * width + columns_a[same_tile]
        indices_b = rows_b[same_tile] * width + columns_b[same_tile]
        for first_index, second_index in zip(indices_a, indices_b):
            union(int(first_index), int(second_index))

    roots = np.full(size, -1, dtype=np.int64)
    valid_indices = np.flatnonzero(cloud.valid.reshape(-1))
    for index in valid_indices:
        roots[index] = find(int(index))
    return roots.reshape(height, width)


def _contiguous_labels(roots: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Assign component IDs by deterministic row-major first discovery."""
    labels = np.full(roots.shape, -1, dtype=np.int32)
    valid_indices = np.flatnonzero(valid.reshape(-1))
    if valid_indices.size == 0:
        return labels
    valid_roots = roots.reshape(-1)[valid_indices]
    unique_roots, first_indices = np.unique(valid_roots, return_index=True)
    discovery_order = unique_roots[np.argsort(first_indices)]
    root_to_id = np.full(roots.size, -1, dtype=np.int32)
    root_to_id[discovery_order] = np.arange(
        discovery_order.size, dtype=np.int32
    )
    labels.reshape(-1)[valid_indices] = root_to_id[valid_roots]
    return labels


def _patch_statistics(
    cloud: OrganizedCloud,
    geometry: OrganizedGeometry,
    labels: np.ndarray,
) -> tuple[np.ndarray, ...]:
    """Aggregate exact per-patch geometry and inclusive-exclusive bounds."""
    count = int(labels.max(initial=-1)) + 1
    if count == 0:
        return (
            np.empty(0, dtype=np.int64),
            np.empty((0, 3), dtype=np.float32),
            np.empty((0, 3), dtype=np.float32),
            np.empty((0, 3), dtype=np.float32),
            np.empty(0, dtype=np.float32),
            np.empty((0, 4), dtype=np.int32),
        )
    valid = labels >= 0
    patch_ids = labels[valid]
    pixel_counts = np.bincount(patch_ids, minlength=count).astype(np.int64)
    centroids = np.zeros((count, 3), dtype=np.float64)
    colors = np.zeros((count, 3), dtype=np.float64)
    np.add.at(centroids, patch_ids, cloud.xyz[valid])
    np.add.at(colors, patch_ids, cloud.rgb[valid])
    centroids = (centroids / pixel_counts[:, None]).astype(np.float32)
    colors = (colors / pixel_counts[:, None]).astype(np.float32)

    normal_mask = valid & geometry.normal_valid
    normal_ids = labels[normal_mask]
    normal_counts = np.bincount(normal_ids, minlength=count).astype(np.int64)
    normal_sums = np.zeros((count, 3), dtype=np.float64)
    np.add.at(normal_sums, normal_ids, geometry.normals[normal_mask])
    lengths = np.linalg.norm(normal_sums, axis=1)
    mean_normals = np.full((count, 3), np.nan, dtype=np.float32)
    usable = (normal_counts > 0) & np.isfinite(lengths) & (lengths > 0.0)
    mean_normals[usable] = (
        normal_sums[usable] / lengths[usable, None]
    ).astype(np.float32)
    fractions = (normal_counts / pixel_counts).astype(np.float32)

    rows, columns = np.nonzero(valid)
    minimum_rows = np.full(count, cloud.height, dtype=np.int32)
    minimum_columns = np.full(count, cloud.width, dtype=np.int32)
    maximum_rows = np.full(count, -1, dtype=np.int32)
    maximum_columns = np.full(count, -1, dtype=np.int32)
    np.minimum.at(minimum_rows, patch_ids, rows)
    np.minimum.at(minimum_columns, patch_ids, columns)
    np.maximum.at(maximum_rows, patch_ids, rows)
    np.maximum.at(maximum_columns, patch_ids, columns)
    boxes = np.stack((
        minimum_rows,
        minimum_columns,
        maximum_rows + 1,
        maximum_columns + 1,
    ), axis=1).astype(np.int32)
    return pixel_counts, centroids, colors, mean_normals, fractions, boxes


def segment_surface_patches(
    cloud: OrganizedCloud,
    geometry: OrganizedGeometry,
    config: PatchSegmentationConfig,
) -> PatchSet:
    """Oversegment valid organized surfaces into conservative tile patches."""
    _validate_inputs(cloud, geometry)
    roots = _union_components(cloud, geometry, config)
    labels = _contiguous_labels(roots, cloud.valid)
    statistics = _patch_statistics(cloud, geometry, labels)
    count = int(labels.max(initial=-1)) + 1
    if count:
        tile_indices = (
            statistics[5][:, :2] // config.tile_size_pixels
        ).astype(np.int32)
    else:
        tile_indices = np.empty((0, 2), dtype=np.int32)
    return PatchSet(
        labels=labels,
        count=count,
        pixel_counts=statistics[0],
        centroids_xyz=statistics[1],
        mean_rgb=statistics[2],
        mean_normals=statistics[3],
        normal_valid_fraction=statistics[4],
        bounding_boxes_vuvu=statistics[5],
        tile_indices=tile_indices,
        source_tile_size_pixels=config.tile_size_pixels,
    )
