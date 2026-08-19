"""Deterministic RGB-D projection preserving pixel correspondence."""

import math

import numpy as np

from .models import CompactCloud
from .models import OrganizedCloud
from .models import SceneInput


OPENCV_OPTICAL_FRAME = "opencv_optical_frame:+x_right,+y_down,+z_forward"


class ProjectionError(ValueError):
    """Indicate invalid or unsupported RGB-D projection input."""


def _validate_scene_arrays(scene: SceneInput) -> tuple[int, int]:
    """Validate shapes, types, camera resolution, and depth values."""
    color = scene.color
    depth = scene.depth
    if color.ndim != 3 or color.shape[2] != 3 or color.dtype != np.uint8:
        raise ProjectionError("RGB must have shape H x W x 3 and dtype uint8")
    if depth.ndim != 2:
        raise ProjectionError("depth must have shape H x W")
    if depth.shape != color.shape[:2]:
        raise ProjectionError("RGB and depth dimensions do not match")
    height, width = depth.shape
    if (scene.camera.height, scene.camera.width) != (height, width):
        raise ProjectionError("camera resolution does not match RGB-D images")
    if depth.dtype not in (np.uint16, np.float32):
        raise ProjectionError("depth must have dtype uint16 or float32")
    if np.issubdtype(depth.dtype, np.floating):
        if not np.all(np.isfinite(depth)):
            raise ProjectionError("floating depth contains nonfinite values")
    if np.any(depth < 0):
        raise ProjectionError("depth contains negative values")
    camera = scene.camera
    for value, name in (
        (camera.fx, "fx"),
        (camera.fy, "fy"),
        (camera.depth_scale, "depth_scale"),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ProjectionError(f"{name} must be positive and finite")
    if not math.isfinite(camera.cx) or not math.isfinite(camera.cy):
        raise ProjectionError("principal point must be finite")
    return height, width


def rgbd_to_organized_cloud(
    scene: SceneInput, *, input_is_rectified: bool = False
) -> OrganizedCloud:
    """
    Project RGB-D into an organized cloud in the OpenCV optical frame.

    Distorted inputs are rejected unless the caller explicitly confirms that
    RGB and depth are already jointly rectified. This function never performs
    undistortion, resizing, smoothing, completion, or frame transformation.
    """
    height, width = _validate_scene_arrays(scene)
    distortion = scene.camera.distortion_coefficients
    has_distortion = distortion.size > 0 and np.any(distortion != 0.0)
    if has_distortion and not input_is_rectified:
        raise ProjectionError(
            "nonzero distortion requires input_is_rectified=True for jointly "
            "rectified RGB and depth"
        )

    valid = scene.depth != 0
    z = scene.depth.astype(np.float32) / np.float32(scene.camera.depth_scale)
    rows, columns = np.indices((height, width), dtype=np.float32)
    x = (columns - np.float32(scene.camera.cx)) * z / np.float32(scene.camera.fx)
    y = (rows - np.float32(scene.camera.cy)) * z / np.float32(scene.camera.fy)
    xyz = np.stack((x, y, z), axis=-1).astype(np.float32, copy=False)
    xyz[~valid] = np.nan
    if not np.all(np.isfinite(xyz[valid])):
        raise ProjectionError("valid depth produced nonfinite XYZ coordinates")
    return OrganizedCloud(
        xyz=xyz,
        rgb=scene.color,
        valid=valid,
        width=width,
        height=height,
        frame_id=scene.camera.frame_id,
        coordinate_convention=OPENCV_OPTICAL_FRAME,
    )


def compact_valid_points(
    cloud: OrganizedCloud, ground_truth: np.ndarray | None = None
) -> CompactCloud:
    """Return valid points in deterministic row-major pixel order."""
    expected_xyz = (cloud.height, cloud.width, 3)
    expected_image = (cloud.height, cloud.width)
    if cloud.xyz.shape != expected_xyz or cloud.xyz.dtype != np.float32:
        raise ProjectionError("organized XYZ contract is invalid")
    if cloud.rgb.shape != expected_xyz or cloud.rgb.dtype != np.uint8:
        raise ProjectionError("organized RGB contract is invalid")
    if cloud.valid.shape != expected_image or cloud.valid.dtype != np.bool_:
        raise ProjectionError("organized validity-mask contract is invalid")
    if ground_truth is not None:
        if ground_truth.shape != expected_image:
            raise ProjectionError("ground-truth dimensions do not match cloud")
        if ground_truth.dtype != np.uint16:
            raise ProjectionError("ground truth must have dtype uint16")

    pixels_vu = np.argwhere(cloud.valid).astype(np.int64, copy=False)
    xyz = cloud.xyz[cloud.valid].reshape(-1, 3)
    rgb = cloud.rgb[cloud.valid].reshape(-1, 3)
    labels = None if ground_truth is None else ground_truth[cloud.valid]
    return CompactCloud(
        xyz=xyz,
        rgb=rgb,
        pixels_vu=pixels_vu,
        ground_truth=labels,
        organized_shape=expected_image,
    )


def compact_labels_to_organized(
    labels: np.ndarray,
    pixels_vu: np.ndarray,
    organized_shape: tuple[int, int],
    *,
    invalid_label: int = -1,
) -> np.ndarray:
    """Map compact integer labels back to their exact organized pixels."""
    labels_array = np.asarray(labels)
    pixels = np.asarray(pixels_vu)
    if labels_array.ndim != 1:
        raise ProjectionError("labels must be one-dimensional")
    if pixels.ndim != 2 or pixels.shape[1] != 2:
        raise ProjectionError("pixels_vu must have shape N x 2")
    if len(labels_array) != len(pixels):
        raise ProjectionError("label count does not match pixel count")
    if not np.issubdtype(labels_array.dtype, np.integer):
        raise ProjectionError("labels must use an integer dtype")
    if not np.issubdtype(pixels.dtype, np.integer):
        raise ProjectionError("pixel coordinates must use an integer dtype")
    if len(organized_shape) != 2:
        raise ProjectionError("organized_shape must contain height and width")
    height, width = organized_shape
    if not isinstance(height, int) or not isinstance(width, int):
        raise ProjectionError("organized dimensions must be integers")
    if height < 0 or width < 0:
        raise ProjectionError("organized dimensions cannot be negative")
    if pixels.size:
        rows, columns = pixels[:, 0], pixels[:, 1]
        if np.any(rows < 0) or np.any(rows >= height):
            raise ProjectionError("pixel row is out of bounds")
        if np.any(columns < 0) or np.any(columns >= width):
            raise ProjectionError("pixel column is out of bounds")
        linear = rows.astype(np.int64) * width + columns.astype(np.int64)
        if np.unique(linear).size != linear.size:
            raise ProjectionError("duplicate pixel indices are not allowed")
    limits = np.iinfo(np.int32)
    if labels_array.size and (
        np.any(labels_array < limits.min) or np.any(labels_array > limits.max)
    ):
        raise ProjectionError("labels exceed int32 range")
    if invalid_label < limits.min or invalid_label > limits.max:
        raise ProjectionError("invalid_label exceeds int32 range")
    result = np.full((height, width), invalid_label, dtype=np.int32)
    if pixels.size:
        result[pixels[:, 0], pixels[:, 1]] = labels_array.astype(np.int32)
    return result
