"""Read and validate RGB-D scene folders without modifying them."""

import json
import math
from pathlib import Path
from typing import Mapping

import cv2
import numpy as np

from .models import CameraModel
from .models import SceneInput
from .models import SceneValidationReport
from .models import ValidationIssue


class DatasetValidationError(ValueError):
    """Indicate that a scene or camera contract is invalid."""


def _positive_finite(value: object, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise DatasetValidationError(f"{name} must be numeric") from error
    if not math.isfinite(result) or result <= 0.0:
        raise DatasetValidationError(f"{name} must be positive and finite")
    return result


def _dimension(value: object, name: str) -> int:
    if isinstance(value, bool):
        raise DatasetValidationError(f"{name} must be a positive integer")
    try:
        result = int(value)
    except (TypeError, ValueError) as error:
        raise DatasetValidationError(
            f"{name} must be a positive integer"
        ) from error
    if result <= 0 or float(value) != result:
        raise DatasetValidationError(f"{name} must be a positive integer")
    return result


def _number_list(value: object, name: str, lengths: tuple[int, ...]) -> list:
    if not isinstance(value, (list, tuple)) or len(value) not in lengths:
        expected = " or ".join(str(length) for length in lengths)
        raise DatasetValidationError(f"{name} must contain {expected} values")
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError) as error:
        raise DatasetValidationError(f"{name} must be numeric") from error
    if not all(math.isfinite(item) for item in result):
        raise DatasetValidationError(f"{name} must contain finite values")
    return result


def _depth_scale(data: Mapping[str, object], fallback: float | None) -> float:
    value = data.get("depth_scale", fallback)
    if value is None:
        raise DatasetValidationError(
            "depth scale is missing; expected stored depth units per metre"
        )
    return _positive_finite(value, "depth_scale")


def _camera_model(
    *,
    width: int,
    height: int,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    depth_scale: float,
    distortion_model: object,
    distortion: object,
    frame_id: object,
    schema: str,
) -> CameraModel:
    for value, name in ((cx, "cx"), (cy, "cy")):
        if not math.isfinite(float(value)):
            raise DatasetValidationError(f"{name} must be finite")
    coefficients = _number_list(distortion or [], "distortion coefficients", (
        0, 4, 5, 8, 12, 14,
    ))
    return CameraModel(
        width=width,
        height=height,
        fx=_positive_finite(fx, "fx"),
        fy=_positive_finite(fy, "fy"),
        cx=float(cx),
        cy=float(cy),
        depth_scale=depth_scale,
        distortion_model=str(distortion_model or ""),
        distortion_coefficients=np.asarray(coefficients, dtype=np.float64),
        frame_id=None if frame_id is None else str(frame_id),
        source_schema=schema,
    )


def parse_capture_camera_json(data: Mapping[str, object]) -> CameraModel:
    """Parse camera metadata emitted by the ROS dataset capture node."""
    width = _dimension(data.get("width"), "width")
    height = _dimension(data.get("height"), "height")
    intrinsic = data.get("intrinsic_matrix")
    projection = data.get("projection_matrix")
    if intrinsic is not None:
        matrix = _number_list(intrinsic, "intrinsic_matrix", (9,))
        fx, fy, cx, cy = matrix[0], matrix[4], matrix[2], matrix[5]
    elif projection is not None:
        matrix = _number_list(projection, "projection_matrix", (12,))
        fx, fy, cx, cy = matrix[0], matrix[5], matrix[2], matrix[6]
    else:
        raise DatasetValidationError(
            "capture schema requires intrinsic_matrix or projection_matrix"
        )
    return _camera_model(
        width=width,
        height=height,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        depth_scale=_depth_scale(data, None),
        distortion_model=data.get("distortion_model", ""),
        distortion=data.get("distortion_coefficients", []),
        frame_id=data.get("frame_id"),
        schema="ros_capture",
    )


def intrinsics_from_symmetric_projection(
    projection: object, width: int, height: int
) -> tuple[float, float, float, float]:
    """
    Convert a supported row-major symmetric OpenGL projection matrix.

    The supported convention has ``p00 = 2*fx/width`` and
    ``p11 = 2*fy/height``. Its offset and skew terms must be zero, which makes
    the principal point exactly the image centre. Perspective depth terms are
    deliberately not interpreted here.
    """
    values = _number_list(projection, "cameraProjection", (16,))
    if not all(abs(values[index]) <= 1e-9 for index in (1, 2, 4, 6, 8, 9)):
        raise DatasetValidationError(
            "Isaac projection is offset, skewed, or uses an unsupported "
            "matrix convention"
        )
    fx = _positive_finite(values[0] * width / 2.0, "derived fx")
    fy = _positive_finite(values[5] * height / 2.0, "derived fy")
    return fx, fy, width / 2.0, height / 2.0


def parse_isaac_camera_json(
    data: Mapping[str, object], *, depth_scale: float | None = None
) -> CameraModel:
    """Parse the explicitly supported Isaac Replicator camera schema."""
    resolution = data.get("renderProductResolution")
    if resolution is None:
        resolution = data.get("render_resolution")
    values = _number_list(resolution, "renderProductResolution", (2,))
    width = _dimension(values[0], "render width")
    height = _dimension(values[1], "render height")

    explicit_keys = ("fx", "fy", "cx", "cy")
    present = tuple(key in data for key in explicit_keys)
    if any(present) and not all(present):
        raise DatasetValidationError(
            "Isaac OpenCV intrinsics must provide fx, fy, cx, and cy together"
        )
    if all(present):
        raw_fx = float(data["fx"])
        raw_fy = float(data["fy"])
        if raw_fx > 0.0 and raw_fy > 0.0:
            fx, fy = raw_fx, raw_fy
            cx, cy = float(data["cx"]), float(data["cy"])
        elif raw_fx == 0.0 and raw_fy == 0.0:
            fx, fy, cx, cy = intrinsics_from_symmetric_projection(
                data.get("cameraProjection"), width, height
            )
            if float(data["cx"]) not in (0.0, cx) or float(data["cy"]) not in (
                0.0, cy
            ):
                raise DatasetValidationError(
                    "zero focal values conflict with explicit principal point"
                )
        else:
            raise DatasetValidationError(
                "Isaac focal values must both be positive or both be zero"
            )
    else:
        fx, fy, cx, cy = intrinsics_from_symmetric_projection(
            data.get("cameraProjection"), width, height
        )

    model = str(data.get("cameraModel", "pinhole")).lower()
    if model not in ("pinhole", "perspective"):
        raise DatasetValidationError(f"unsupported Isaac camera model: {model}")
    return _camera_model(
        width=width,
        height=height,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        depth_scale=_depth_scale(data, depth_scale),
        distortion_model=data.get("distortion_model", ""),
        distortion=data.get("distortion_coefficients", []),
        frame_id=data.get("frame_id"),
        schema="isaac_replicator",
    )


def parse_camera_json(
    data: Mapping[str, object], *, depth_scale: float | None = None
) -> CameraModel:
    """Select an unambiguous camera-schema adapter and parse the data."""
    capture = "width" in data and (
        "intrinsic_matrix" in data or "projection_matrix" in data
    )
    isaac = (
        "renderProductResolution" in data or "render_resolution" in data
    )
    if capture == isaac:
        kind = "ambiguous" if capture else "unsupported"
        raise DatasetValidationError(f"{kind} camera JSON schema")
    if capture:
        return parse_capture_camera_json(data)
    return parse_isaac_camera_json(data, depth_scale=depth_scale)


def _read_json(path: Path) -> Mapping[str, object]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise DatasetValidationError(f"malformed JSON in {path.name}: {error}")
    if not isinstance(value, dict):
        raise DatasetValidationError(f"{path.name} must contain a JSON object")
    return value


def _integer_mapping(data: Mapping[str, object], name: str) -> dict:
    result = {}
    for key, value in data.items():
        try:
            integer_key = int(key)
        except (TypeError, ValueError) as error:
            raise DatasetValidationError(
                f"{name} contains a non-integer key: {key!r}"
            ) from error
        if str(integer_key) != str(key):
            raise DatasetValidationError(
                f"{name} contains a non-canonical integer key: {key!r}"
            )
        result[integer_key] = value
    return result


def _metadata_depth_scale(
    camera_data: Mapping[str, object], scene_data: Mapping[str, object]
) -> float | None:
    candidates = []
    if "depth_scale" in camera_data:
        candidates.append(
            _positive_finite(camera_data["depth_scale"], "depth_scale")
        )
    if "depth_scale" in scene_data:
        candidates.append(
            _positive_finite(scene_data["depth_scale"], "depth_scale")
        )
    unit = scene_data.get("depth_unit")
    if unit is not None:
        normalized = str(unit).lower()
        millimetre_units = (
            "millimetre", "millimetres", "millimeter", "millimeters", "mm"
        )
        if normalized in millimetre_units:
            candidates.append(1000.0)
        elif normalized in ("metre", "metres", "meter", "meters", "m"):
            candidates.append(1.0)
        else:
            raise DatasetValidationError(f"unsupported depth unit: {unit}")
    conflicts = candidates and any(
        not math.isclose(item, candidates[0]) for item in candidates[1:]
    )
    if conflicts:
        raise DatasetValidationError("conflicting depth-scale metadata")
    return candidates[0] if candidates else None


def _load_scene_data(scene_dir: Path) -> SceneInput:
    required = ("color.png", "depth.png", "camera.json")
    missing = [name for name in required if not (scene_dir / name).is_file()]
    if missing:
        raise DatasetValidationError(f"missing required files: {', '.join(missing)}")

    color_raw = cv2.imread(str(scene_dir / "color.png"), cv2.IMREAD_UNCHANGED)
    depth = cv2.imread(str(scene_dir / "depth.png"), cv2.IMREAD_UNCHANGED)
    if color_raw is None or depth is None:
        raise DatasetValidationError("OpenCV could not decode a required image")
    if color_raw.dtype != np.uint8:
        raise DatasetValidationError("color image must use uint8 samples")
    if color_raw.ndim != 3 or color_raw.shape[2] not in (3, 4):
        channels = 1 if color_raw.ndim == 2 else color_raw.shape[-1]
        raise DatasetValidationError(f"unsupported color channel count: {channels}")
    conversion = (
        cv2.COLOR_BGR2RGB
        if color_raw.shape[2] == 3
        else cv2.COLOR_BGRA2RGB
    )
    color = cv2.cvtColor(color_raw, conversion)
    if depth.ndim != 2 or depth.dtype not in (np.uint16, np.float32):
        raise DatasetValidationError(
            f"unsupported depth encoding: shape={depth.shape}, dtype={depth.dtype}"
        )

    scene_path = scene_dir / "scene.json"
    scene_data = _read_json(scene_path) if scene_path.is_file() else {}
    camera_data = _read_json(scene_dir / "camera.json")
    encodings = [
        str(value)
        for value in (
            camera_data.get("stored_depth_encoding"),
            scene_data.get("depth_encoding"),
        )
        if value is not None
    ]
    if encodings and any(value != encodings[0] for value in encodings[1:]):
        raise DatasetValidationError("conflicting depth encodings")
    expected_encoding = "16UC1" if depth.dtype == np.uint16 else "32FC1"
    if encodings and encodings[0] != expected_encoding:
        raise DatasetValidationError(
            f"unsupported depth encoding: {encodings[0]} for {depth.dtype}"
        )
    scale = _metadata_depth_scale(camera_data, scene_data)
    camera = parse_camera_json(camera_data, depth_scale=scale)

    instance_path = scene_dir / "instance.png"
    ground_truth = None
    if instance_path.is_file():
        ground_truth = cv2.imread(str(instance_path), cv2.IMREAD_UNCHANGED)
        if ground_truth is None:
            raise DatasetValidationError("OpenCV could not decode instance.png")
        if ground_truth.ndim != 2 or ground_truth.dtype != np.uint16:
            raise DatasetValidationError("instance.png must be single-channel uint16")

    mapping_path = scene_dir / "instance_mapping.json"
    mapping = {}
    if mapping_path.is_file():
        raw_mapping = _integer_mapping(_read_json(mapping_path), "instance mapping")
        if not all(isinstance(value, str) for value in raw_mapping.values()):
            raise DatasetValidationError("instance mapping values must be strings")
        mapping = raw_mapping
    semantics_path = scene_dir / "semantics.json"
    semantics = {}
    if semantics_path.is_file():
        semantics = _integer_mapping(_read_json(semantics_path), "semantics")
    return SceneInput(
        color=color,
        depth=depth,
        camera=camera,
        ground_truth=ground_truth,
        instance_mapping=mapping,
        semantics=semantics,
        metadata=scene_data,
    )


def _issues(scene: SceneInput, require_ground_truth: bool) -> list[ValidationIssue]:
    result = []
    shape = scene.color.shape[:2]
    if scene.depth.shape != shape:
        result.append(ValidationIssue(
            "error", "image_shape_mismatch", "RGB and depth dimensions differ"
        ))
    if (scene.camera.height, scene.camera.width) != shape:
        result.append(ValidationIssue(
            "error",
            "camera_shape_mismatch",
            "camera resolution differs from RGB image",
        ))
    if scene.ground_truth is None:
        severity = "error" if require_ground_truth else "info"
        result.append(ValidationIssue(
            severity, "ground_truth_missing", "instance.png is unavailable"
        ))
        return result
    if scene.ground_truth.shape != shape:
        result.append(ValidationIssue(
            "error",
            "instance_shape_mismatch",
            "instance and RGB dimensions differ",
        ))
        return result
    visible = set(int(item) for item in np.unique(scene.ground_truth) if item != 0)
    zero_is_valid = scene.metadata.get("zero_depth_is_valid") is True
    invalid_instance = (
        np.zeros(scene.depth.shape, dtype=bool)
        if zero_is_valid
        else (scene.ground_truth != 0) & (scene.depth == 0)
    )
    if np.any(invalid_instance):
        result.append(ValidationIssue(
            "error",
            "instance_on_invalid_depth",
            "instance pixels contain zero depth",
        ))
    if scene.instance_mapping:
        missing = sorted(visible - set(scene.instance_mapping))
        if missing:
            result.append(ValidationIssue(
                "error",
                "instance_mapping_missing_id",
                f"visible IDs missing from mapping: {missing}",
            ))
        unused = sorted(set(scene.instance_mapping) - visible - {0})
        if unused:
            result.append(ValidationIssue(
                "warning",
                "instance_mapping_unused_id",
                f"mapping IDs are not visible: {unused}",
            ))
    if not scene.semantics:
        result.append(ValidationIssue(
            "warning",
            "semantics_missing",
            "optional semantics.json is unavailable",
        ))
    bag_count = scene.metadata.get("bag_count")
    if isinstance(bag_count, int) and bag_count != len(visible):
        result.append(ValidationIssue(
            "info",
            "object_count_differs_from_visible",
            f"scene object count {bag_count} differs from "
            f"{len(visible)} visible instances",
        ))
    return result


def validate_scene(
    scene_dir: Path, *, require_ground_truth: bool = False
) -> SceneValidationReport:
    """Validate a scene and return fatal, warning, and informational findings."""
    path = Path(scene_dir)
    try:
        if not path.is_dir():
            raise DatasetValidationError(f"scene directory does not exist: {path}")
        scene = _load_scene_data(path)
        issues = _issues(scene, require_ground_truth)
        shape = scene.color.shape[:2]
        visible = () if scene.ground_truth is None else tuple(
            int(item) for item in np.unique(scene.ground_truth) if item != 0
        )
    except DatasetValidationError as error:
        issues = [ValidationIssue("error", "scene_invalid", str(error))]
        shape = None
        visible = ()
    return SceneValidationReport(
        valid=not any(issue.severity == "error" for issue in issues),
        issues=tuple(issues),
        image_shape=shape,
        visible_instance_ids=visible,
    )


def load_scene(
    scene_dir: Path, *, require_ground_truth: bool = False
) -> SceneInput:
    """Load a scene after enforcing all fatal dataset validation rules."""
    path = Path(scene_dir)
    scene = _load_scene_data(path)
    errors = [
        issue for issue in _issues(scene, require_ground_truth)
        if issue.severity == "error"
    ]
    if errors:
        detail = "; ".join(f"{issue.code}: {issue.message}" for issue in errors)
        raise DatasetValidationError(detail)
    return scene
