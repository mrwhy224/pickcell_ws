"""Vectorized organized depth-edge and surface-normal estimation."""

import numpy as np

from .models import NormalEstimationConfig
from .models import OrganizedCloud
from .models import OrganizedGeometry


class NormalEstimationError(ValueError):
    """Indicate an invalid organized-cloud geometry input."""


def _validate_cloud(cloud: OrganizedCloud) -> tuple[int, int]:
    """Validate the Phase 2 organized-cloud contract."""
    shape = (cloud.height, cloud.width)
    if cloud.xyz.shape != (*shape, 3) or cloud.xyz.dtype != np.float32:
        raise NormalEstimationError("XYZ must have shape H x W x 3, float32")
    if cloud.valid.shape != shape or cloud.valid.dtype != np.bool_:
        raise NormalEstimationError("valid must have shape H x W, boolean")
    if np.any(~np.isfinite(cloud.xyz[cloud.valid])):
        raise NormalEstimationError("valid XYZ coordinates must be finite")
    if np.any(~np.isnan(cloud.xyz[~cloud.valid])):
        raise NormalEstimationError("invalid XYZ coordinates must be NaN")
    if np.any(cloud.xyz[..., 2][cloud.valid] <= 0.0):
        raise NormalEstimationError("valid depth must be positive")
    return shape


def _pair_slices(
    shape: tuple[int, int], delta_v: int, delta_u: int
) -> tuple[tuple[slice, slice], tuple[slice, slice]]:
    """Return aligned slices for an image-space neighbor direction."""
    height, width = shape
    if abs(delta_v) >= height or abs(delta_u) >= width:
        empty = (slice(0, 0), slice(0, 0))
        return empty, empty
    if delta_v >= 0:
        rows_a, rows_b = slice(0, height - delta_v), slice(delta_v, height)
    else:
        rows_a, rows_b = slice(-delta_v, height), slice(0, height + delta_v)
    if delta_u >= 0:
        cols_a, cols_b = slice(0, width - delta_u), slice(delta_u, width)
    else:
        cols_a, cols_b = slice(-delta_u, width), slice(0, width + delta_u)
    return (rows_a, cols_a), (rows_b, cols_b)


def _compatible_depth(
    depth_a: np.ndarray,
    depth_b: np.ndarray,
    valid_a: np.ndarray,
    valid_b: np.ndarray,
    config: NormalEstimationConfig,
) -> np.ndarray:
    """Apply the configured absolute-relative depth compatibility rule."""
    difference = np.abs(depth_a - depth_b)
    limit = np.maximum(
        np.float32(config.absolute_depth_jump_m),
        np.float32(config.relative_depth_jump) * np.minimum(depth_a, depth_b),
    )
    return valid_a & valid_b & (difference <= limit)


def compute_depth_edges(
    cloud: OrganizedCloud, config: NormalEstimationConfig
) -> np.ndarray:
    """Mark pixels participating in incompatible valid-depth transitions."""
    shape = _validate_cloud(cloud)
    depth = cloud.xyz[..., 2]
    edges = np.zeros(shape, dtype=bool)
    directions = [(0, 1), (1, 0)]
    if config.use_diagonal_neighbors:
        directions.extend(((1, 1), (1, -1)))
    for delta_v, delta_u in directions:
        first, second = _pair_slices(shape, delta_v, delta_u)
        both_valid = cloud.valid[first] & cloud.valid[second]
        compatible = _compatible_depth(
            depth[first], depth[second], cloud.valid[first], cloud.valid[second],
            config,
        )
        incompatible = both_valid & ~compatible
        edges[first] |= incompatible
        edges[second] |= incompatible
    return edges


def _tangent_path_validity(
    cloud: OrganizedCloud, config: NormalEstimationConfig
) -> tuple[np.ndarray, np.ndarray]:
    """Return center masks whose horizontal and vertical paths are safe."""
    height, width = cloud.valid.shape
    radius = config.pixel_radius
    inner_height = max(height - 2 * radius, 0)
    inner_width = max(width - 2 * radius, 0)
    horizontal_ok = np.ones((inner_height, inner_width), dtype=bool)
    vertical_ok = np.ones((inner_height, inner_width), dtype=bool)
    depth = cloud.xyz[..., 2]
    center_rows = slice(radius, height - radius)
    center_cols = slice(radius, width - radius)
    for offset in range(-radius, radius):
        left = (center_rows, slice(radius + offset, width - radius + offset))
        right = (
            center_rows,
            slice(radius + offset + 1, width - radius + offset + 1),
        )
        horizontal_ok &= _compatible_depth(
            depth[left], depth[right], cloud.valid[left], cloud.valid[right],
            config,
        )
        upper = (slice(radius + offset, height - radius + offset), center_cols)
        lower = (
            slice(radius + offset + 1, height - radius + offset + 1),
            center_cols,
        )
        vertical_ok &= _compatible_depth(
            depth[upper], depth[lower], cloud.valid[upper], cloud.valid[lower],
            config,
        )
    return horizontal_ok, vertical_ok


def estimate_organized_normals(
    cloud: OrganizedCloud, config: NormalEstimationConfig
) -> tuple[np.ndarray, np.ndarray]:
    """Estimate opposite-neighbor cross-product normals without interpolation."""
    height, width = _validate_cloud(cloud)
    normals = np.full((height, width, 3), np.nan, dtype=np.float32)
    normal_valid = np.zeros((height, width), dtype=bool)
    radius = config.pixel_radius
    if height <= 2 * radius or width <= 2 * radius:
        return normals, normal_valid

    rows = slice(radius, height - radius)
    cols = slice(radius, width - radius)
    horizontal = (
        cloud.xyz[rows, 2 * radius:width]
        - cloud.xyz[rows, 0:width - 2 * radius]
    )
    vertical = (
        cloud.xyz[2 * radius:height, cols]
        - cloud.xyz[0:height - 2 * radius, cols]
    )
    horizontal_ok, vertical_ok = _tangent_path_validity(cloud, config)
    horizontal_length = np.linalg.norm(horizontal, axis=2)
    vertical_length = np.linalg.norm(vertical, axis=2)
    cross = np.cross(horizontal, vertical)
    cross_length = np.linalg.norm(cross, axis=2)
    valid = (
        cloud.valid[rows, cols]
        & horizontal_ok
        & vertical_ok
        & np.isfinite(horizontal_length)
        & np.isfinite(vertical_length)
        & np.isfinite(cross_length)
        & (horizontal_length > config.minimum_baseline_m)
        & (vertical_length > config.minimum_baseline_m)
        & (cross_length > np.finfo(np.float32).eps)
    )
    inner_normals = np.full(cross.shape, np.nan, dtype=np.float32)
    inner_normals[valid] = cross[valid] / cross_length[valid, None]
    if config.orient_toward_camera and np.any(valid):
        points = cloud.xyz[rows, cols]
        flip = valid & (np.sum(inner_normals * (-points), axis=2) < 0.0)
        inner_normals[flip] *= -1.0
    normals[rows, cols] = inner_normals
    normal_valid[rows, cols] = valid
    return normals, normal_valid


def compute_normal_variation(
    normals: np.ndarray,
    normal_valid: np.ndarray,
    depth_edges: np.ndarray,
    config: NormalEstimationConfig,
) -> np.ndarray:
    """Average angular normal disagreement over safe image-space neighbors."""
    if normals.ndim != 3 or normals.shape[2] != 3:
        raise NormalEstimationError("normals must have shape H x W x 3")
    shape = normals.shape[:2]
    if normals.dtype != np.float32:
        raise NormalEstimationError("normals must have dtype float32")
    if normal_valid.shape != shape or normal_valid.dtype != np.bool_:
        raise NormalEstimationError("normal_valid contract is invalid")
    if depth_edges.shape != shape or depth_edges.dtype != np.bool_:
        raise NormalEstimationError("depth_edges contract is invalid")
    if np.any(~np.isfinite(normals[normal_valid])):
        raise NormalEstimationError("valid normals must be finite")

    total = np.zeros(shape, dtype=np.float32)
    count = np.zeros(shape, dtype=np.int32)
    radius = config.pixel_radius
    directions = [(0, radius), (radius, 0)]
    if config.use_diagonal_neighbors:
        directions.extend(((radius, radius), (radius, -radius)))
    for delta_v, delta_u in directions:
        first, second = _pair_slices(shape, delta_v, delta_u)
        comparable = (
            normal_valid[first]
            & normal_valid[second]
            & ~depth_edges[first]
            & ~depth_edges[second]
        )
        dot = np.sum(normals[first] * normals[second], axis=2)
        disagreement = 1.0 - np.clip(dot, -1.0, 1.0)
        values = np.where(comparable, disagreement, 0.0).astype(np.float32)
        total[first] += values
        total[second] += values
        count[first] += comparable
        count[second] += comparable
    variation = np.full(shape, np.nan, dtype=np.float32)
    usable = normal_valid & (count > 0)
    variation[usable] = total[usable] / count[usable]
    return variation


def compute_organized_geometry(
    cloud: OrganizedCloud, config: NormalEstimationConfig
) -> OrganizedGeometry:
    """Compute deterministic depth edges, normals, and normal variation."""
    depth_edges = compute_depth_edges(cloud, config)
    normals, normal_valid = estimate_organized_normals(cloud, config)
    variation = compute_normal_variation(
        normals, normal_valid, depth_edges, config
    )
    return OrganizedGeometry(
        normals=normals,
        normal_valid=normal_valid,
        depth_edges=depth_edges,
        normal_variation=variation,
    )
