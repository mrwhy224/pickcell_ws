"""Immutable data contracts for scene loading and validation."""

from dataclasses import dataclass
import math
import numbers
from types import MappingProxyType
from typing import Mapping

import numpy as np


def immutable_array(value: np.ndarray) -> np.ndarray:
    """Return an owned, read-only NumPy array."""
    result = np.array(value, copy=True)
    result.setflags(write=False)
    return result


def immutable_mapping(value: Mapping) -> Mapping:
    """Return a shallow, read-only copy of a mapping."""
    return MappingProxyType(dict(value))


@dataclass(frozen=True)
class CameraModel:
    """Pinhole camera calibration with depth units per metre."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float
    distortion_model: str
    distortion_coefficients: np.ndarray
    frame_id: str | None
    source_schema: str

    def __post_init__(self) -> None:
        """Protect array-valued calibration data from mutation."""
        object.__setattr__(
            self,
            "distortion_coefficients",
            immutable_array(self.distortion_coefficients),
        )


@dataclass(frozen=True)
class SceneInput:
    """Validated RGB-D scene and optional annotation data."""

    color: np.ndarray
    depth: np.ndarray
    camera: CameraModel
    ground_truth: np.ndarray | None
    instance_mapping: Mapping[int, str]
    semantics: Mapping[int, object]
    metadata: Mapping[str, object]

    def __post_init__(self) -> None:
        """Make loaded arrays and top-level mappings read-only."""
        object.__setattr__(self, "color", immutable_array(self.color))
        object.__setattr__(self, "depth", immutable_array(self.depth))
        if self.ground_truth is not None:
            object.__setattr__(
                self, "ground_truth", immutable_array(self.ground_truth)
            )
        object.__setattr__(
            self, "instance_mapping", immutable_mapping(self.instance_mapping)
        )
        object.__setattr__(self, "semantics", immutable_mapping(self.semantics))
        object.__setattr__(self, "metadata", immutable_mapping(self.metadata))


@dataclass(frozen=True)
class ValidationIssue:
    """One machine-readable scene validation finding."""

    severity: str
    code: str
    message: str


@dataclass(frozen=True)
class SceneValidationReport:
    """Complete validation outcome for a scene directory."""

    valid: bool
    issues: tuple[ValidationIssue, ...]
    image_shape: tuple[int, int] | None
    visible_instance_ids: tuple[int, ...]


@dataclass(frozen=True)
class OrganizedCloud:
    """
    Represent a pixel-aligned XYZRGB cloud in the OpenCV optical frame.

    Positive X points image-right, positive Y points image-down, and positive
    Z points forward from the camera.
    """

    xyz: np.ndarray
    rgb: np.ndarray
    valid: np.ndarray
    width: int
    height: int
    frame_id: str | None
    coordinate_convention: str

    def __post_init__(self) -> None:
        """Own and protect all organized cloud arrays."""
        object.__setattr__(self, "xyz", immutable_array(self.xyz))
        object.__setattr__(self, "rgb", immutable_array(self.rgb))
        object.__setattr__(self, "valid", immutable_array(self.valid))


@dataclass(frozen=True)
class CompactCloud:
    """Deterministic row-major view of valid organized cloud points."""

    xyz: np.ndarray
    rgb: np.ndarray
    pixels_vu: np.ndarray
    ground_truth: np.ndarray | None
    organized_shape: tuple[int, int]

    def __post_init__(self) -> None:
        """Own and protect compact arrays and optional labels."""
        object.__setattr__(self, "xyz", immutable_array(self.xyz))
        object.__setattr__(self, "rgb", immutable_array(self.rgb))
        object.__setattr__(self, "pixels_vu", immutable_array(self.pixels_vu))
        if self.ground_truth is not None:
            object.__setattr__(
                self, "ground_truth", immutable_array(self.ground_truth)
            )


@dataclass(frozen=True)
class NormalEstimationConfig:
    """Configuration for organized depth and normal geometry estimation."""

    pixel_radius: int = 1
    absolute_depth_jump_m: float = 0.02
    relative_depth_jump: float = 0.02
    minimum_baseline_m: float = 1e-4
    orient_toward_camera: bool = True
    use_diagonal_neighbors: bool = False

    def __post_init__(self) -> None:
        """Reject ambiguous types and physically invalid thresholds."""
        if isinstance(self.pixel_radius, bool) or not isinstance(
            self.pixel_radius, numbers.Integral
        ):
            raise TypeError("pixel_radius must be an integer")
        if self.pixel_radius < 1:
            raise ValueError("pixel_radius must be at least 1")
        thresholds = (
            (self.absolute_depth_jump_m, "absolute_depth_jump_m", False),
            (self.relative_depth_jump, "relative_depth_jump", True),
            (self.minimum_baseline_m, "minimum_baseline_m", False),
        )
        for value, name, allow_zero in thresholds:
            if isinstance(value, bool) or not isinstance(value, numbers.Real):
                raise TypeError(f"{name} must be a real number")
            valid_sign = value >= 0.0 if allow_zero else value > 0.0
            if not math.isfinite(float(value)) or not valid_sign:
                condition = "nonnegative" if allow_zero else "positive"
                raise ValueError(f"{name} must be finite and {condition}")
        for value, name in (
            (self.orient_toward_camera, "orient_toward_camera"),
            (self.use_diagonal_neighbors, "use_diagonal_neighbors"),
        ):
            if not isinstance(value, bool):
                raise TypeError(f"{name} must be boolean")


@dataclass(frozen=True)
class OrganizedGeometry:
    """Pixel-aligned normals, depth edges, and normal variation."""

    normals: np.ndarray
    normal_valid: np.ndarray
    depth_edges: np.ndarray
    normal_variation: np.ndarray

    def __post_init__(self) -> None:
        """Own and protect all organized geometry arrays."""
        object.__setattr__(self, "normals", immutable_array(self.normals))
        object.__setattr__(
            self, "normal_valid", immutable_array(self.normal_valid)
        )
        object.__setattr__(self, "depth_edges", immutable_array(self.depth_edges))
        object.__setattr__(
            self, "normal_variation", immutable_array(self.normal_variation)
        )


@dataclass(frozen=True)
class PatchSegmentationConfig:
    """Conservative tile-bounded surface-patch configuration."""

    tile_size_pixels: int = 12
    connectivity: int = 4
    maximum_normal_angle_deg: float = 20.0
    absolute_point_jump_m: float = 0.03
    relative_point_jump: float = 0.02
    maximum_color_distance: float | None = None
    require_valid_normals: bool = False

    def __post_init__(self) -> None:
        """Validate patch geometry thresholds and strict field types."""
        if isinstance(self.tile_size_pixels, bool) or not isinstance(
            self.tile_size_pixels, numbers.Integral
        ):
            raise TypeError("tile_size_pixels must be an integer")
        if self.tile_size_pixels < 2:
            raise ValueError("tile_size_pixels must be at least 2")
        if isinstance(self.connectivity, bool) or self.connectivity not in (4, 8):
            raise ValueError("connectivity must be exactly 4 or 8")
        values = (
            (self.maximum_normal_angle_deg, "maximum_normal_angle_deg", 180.0),
            (self.absolute_point_jump_m, "absolute_point_jump_m", None),
        )
        for value, name, upper in values:
            if isinstance(value, bool) or not isinstance(value, numbers.Real):
                raise TypeError(f"{name} must be a real number")
            if not math.isfinite(float(value)) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
            if upper is not None and value > upper:
                raise ValueError(f"{name} must not exceed {upper}")
        relative = self.relative_point_jump
        if isinstance(relative, bool) or not isinstance(relative, numbers.Real):
            raise TypeError("relative_point_jump must be a real number")
        if not math.isfinite(float(relative)) or relative < 0.0:
            raise ValueError("relative_point_jump must be finite and nonnegative")
        color = self.maximum_color_distance
        if color is not None:
            if isinstance(color, bool) or not isinstance(color, numbers.Real):
                raise TypeError("maximum_color_distance must be a real number")
            if not math.isfinite(float(color)) or color <= 0.0:
                raise ValueError(
                    "maximum_color_distance must be positive and finite"
                )
        if not isinstance(self.require_valid_normals, bool):
            raise TypeError("require_valid_normals must be boolean")


@dataclass(frozen=True)
class PatchSet:
    """
    Store deterministic organized surface patches and aggregate statistics.

    Bounding boxes use ``[v_min, u_min, v_max_exclusive, u_max_exclusive]``.
    """

    labels: np.ndarray
    count: int
    pixel_counts: np.ndarray
    centroids_xyz: np.ndarray
    mean_rgb: np.ndarray
    mean_normals: np.ndarray
    normal_valid_fraction: np.ndarray
    bounding_boxes_vuvu: np.ndarray
    tile_indices: np.ndarray
    source_tile_size_pixels: int

    def __post_init__(self) -> None:
        """Own and protect all organized patch arrays."""
        for name in (
            "labels",
            "pixel_counts",
            "centroids_xyz",
            "mean_rgb",
            "mean_normals",
            "normal_valid_fraction",
            "bounding_boxes_vuvu",
            "tile_indices",
        ):
            object.__setattr__(self, name, immutable_array(getattr(self, name)))


@dataclass(frozen=True)
class PatchGraphConfig:
    """Configuration for deterministic patch-contact graph construction."""

    include_diagonal_contacts: bool = False
    missing_feature_value: float = 0.0

    def __post_init__(self) -> None:
        """Validate strict graph configuration types and values."""
        if not isinstance(self.include_diagonal_contacts, bool):
            raise TypeError("include_diagonal_contacts must be boolean")
        value = self.missing_feature_value
        if isinstance(value, bool) or not isinstance(value, numbers.Real):
            raise TypeError("missing_feature_value must be a real number")
        if not math.isfinite(float(value)):
            raise ValueError("missing_feature_value must be finite")


@dataclass(frozen=True)
class PatchGraph:
    """Immutable undirected patch graph with stable raw edge features."""

    edges: np.ndarray
    edge_features: np.ndarray
    feature_names: tuple[str, ...]
    shared_boundary_counts: np.ndarray
    diagonal_contact_counts: np.ndarray
    node_degrees: np.ndarray

    def __post_init__(self) -> None:
        """Own and protect graph arrays."""
        for name in (
            "edges",
            "edge_features",
            "shared_boundary_counts",
            "diagonal_contact_counts",
            "node_degrees",
        ):
            object.__setattr__(self, name, immutable_array(getattr(self, name)))


@dataclass(frozen=True)
class EdgeLabelConfig:
    """Configuration for assigning class-agnostic graph-edge targets."""

    background_instance_id: int = 0
    minimum_patch_purity: float = 0.98
    minimum_patch_pixels: int = 1

    def __post_init__(self) -> None:
        """Validate strict label configuration types and ranges."""
        background = self.background_instance_id
        if isinstance(background, bool) or not isinstance(
            background, numbers.Integral
        ):
            raise TypeError("background_instance_id must be an integer")
        if background < 0:
            raise ValueError("background_instance_id must be nonnegative")
        purity = self.minimum_patch_purity
        if isinstance(purity, bool) or not isinstance(purity, numbers.Real):
            raise TypeError("minimum_patch_purity must be a real number")
        if not math.isfinite(float(purity)) or not 0.0 < purity <= 1.0:
            raise ValueError("minimum_patch_purity must be finite and in (0, 1]")
        pixels = self.minimum_patch_pixels
        if isinstance(pixels, bool) or not isinstance(pixels, numbers.Integral):
            raise TypeError("minimum_patch_pixels must be an integer")
        if pixels < 1:
            raise ValueError("minimum_patch_pixels must be positive")


@dataclass(frozen=True)
class SceneSplitConfig:
    """Configuration for deterministic whole-scene dataset splitting."""

    train_fraction: float = 0.70
    validation_fraction: float = 0.15
    test_fraction: float = 0.15
    seed: int = 42

    def __post_init__(self) -> None:
        """Validate fractions with a one-part-per-billion sum tolerance."""
        fractions = (
            self.train_fraction,
            self.validation_fraction,
            self.test_fraction,
        )
        for value in fractions:
            if isinstance(value, bool) or not isinstance(value, numbers.Real):
                raise TypeError("split fractions must be real numbers")
            if not math.isfinite(float(value)) or value < 0.0:
                raise ValueError("split fractions must be finite and nonnegative")
        if not math.isclose(sum(fractions), 1.0, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("split fractions must sum to 1 within 1e-9")
        if isinstance(self.seed, bool) or not isinstance(
            self.seed, numbers.Integral
        ):
            raise TypeError("seed must be an integer")


@dataclass(frozen=True)
class PatchGroundTruth:
    """Immutable majority-instance assignment for every surface patch."""

    majority_instance_ids: np.ndarray
    purity: np.ndarray
    eligible: np.ndarray
    pixel_counts: np.ndarray

    def __post_init__(self) -> None:
        """Own and protect patch ground-truth arrays."""
        for name in (
            "majority_instance_ids",
            "purity",
            "eligible",
            "pixel_counts",
        ):
            object.__setattr__(self, name, immutable_array(getattr(self, name)))


@dataclass(frozen=True)
class SceneEdgeExamples:
    """Selected supervised graph edges from one scene."""

    features: np.ndarray
    targets: np.ndarray
    graph_edge_indices: np.ndarray
    patch_pairs: np.ndarray
    feature_names: tuple[str, ...]
    scene_id: str
    positive_count: int
    negative_count: int
    ignored_count: int
    object_object_negative_count: int
    object_background_negative_count: int
    background_background_ignored_count: int
    ambiguous_ignored_count: int

    def __post_init__(self) -> None:
        """Own arrays and normalize feature names to an immutable tuple."""
        for name in (
            "features",
            "targets",
            "graph_edge_indices",
            "patch_pairs",
        ):
            object.__setattr__(self, name, immutable_array(getattr(self, name)))
        object.__setattr__(self, "feature_names", tuple(self.feature_names))


@dataclass(frozen=True)
class EdgeTrainingDataset:
    """Immutable concatenation of labelled edges from whole scenes."""

    features: np.ndarray
    targets: np.ndarray
    scene_indices: np.ndarray
    graph_edge_indices: np.ndarray
    patch_pairs: np.ndarray
    feature_names: tuple[str, ...]
    scene_ids: tuple[str, ...]
    split_name: str

    def __post_init__(self) -> None:
        """Own arrays and normalize sequence metadata to tuples."""
        for name in (
            "features",
            "targets",
            "scene_indices",
            "graph_edge_indices",
            "patch_pairs",
        ):
            object.__setattr__(self, name, immutable_array(getattr(self, name)))
        object.__setattr__(self, "feature_names", tuple(self.feature_names))
        object.__setattr__(self, "scene_ids", tuple(self.scene_ids))


@dataclass(frozen=True)
class PatchAffinityConfig:
    """Thresholds for deterministic, explainable patch-edge affinity."""

    maximum_mean_boundary_point_distance_m: float = 0.025
    maximum_boundary_depth_difference_m: float = 0.025
    maximum_centroid_depth_difference_m: float = 0.08
    maximum_unsigned_normal_angle_deg: float = 35.0
    maximum_depth_edge_endpoint_fraction: float = 0.50
    minimum_shared_boundary_fraction: float = 0.03
    minimum_normal_pair_valid_fraction: float = 0.20
    normal_evidence_required: bool = False
    maximum_mean_boundary_rgb_distance: float | None = None
    decision_threshold: float = 0.50

    def __post_init__(self) -> None:
        """Reject ambiguous field types and out-of-domain thresholds."""
        nonnegative = (
            "maximum_mean_boundary_point_distance_m",
            "maximum_boundary_depth_difference_m",
            "maximum_centroid_depth_difference_m",
        )
        fractions = (
            "maximum_depth_edge_endpoint_fraction",
            "minimum_shared_boundary_fraction",
            "minimum_normal_pair_valid_fraction",
            "decision_threshold",
        )
        for name in nonnegative:
            _validate_finite_range(getattr(self, name), name, 0.0, None)
        _validate_finite_range(
            self.maximum_unsigned_normal_angle_deg,
            "maximum_unsigned_normal_angle_deg", 0.0, 180.0,
        )
        for name in fractions:
            _validate_finite_range(getattr(self, name), name, 0.0, 1.0)
        color = self.maximum_mean_boundary_rgb_distance
        if color is not None:
            _validate_finite_range(
                color, "maximum_mean_boundary_rgb_distance", 0.0, None
            )
        if not isinstance(self.normal_evidence_required, bool):
            raise TypeError("normal_evidence_required must be boolean")


def _validate_finite_range(
    value: float, name: str, minimum: float, maximum: float | None
) -> None:
    """Validate a nonboolean real against an inclusive finite range."""
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise TypeError(f"{name} must be a real number")
    numeric = float(value)
    if not math.isfinite(numeric) or numeric < minimum:
        raise ValueError(f"{name} is outside its valid range")
    if maximum is not None and numeric > maximum:
        raise ValueError(f"{name} is outside its valid range")


@dataclass(frozen=True)
class EdgeAffinity:
    """Immutable affinity scores and hard-gate decisions in graph-edge order."""

    scores: np.ndarray
    same_object: np.ndarray
    hard_rejected: np.ndarray
    rejection_codes: np.ndarray

    def __post_init__(self) -> None:
        """Validate, own, and protect all affinity result arrays."""
        arrays = (
            (self.scores, np.float32, "scores"),
            (self.same_object, np.bool_, "same_object"),
            (self.hard_rejected, np.bool_, "hard_rejected"),
            (self.rejection_codes, np.int32, "rejection_codes"),
        )
        for value, dtype, name in arrays:
            if not isinstance(value, np.ndarray):
                raise TypeError(f"{name} must be a NumPy array")
            if value.ndim != 1 or value.dtype != dtype:
                raise ValueError(f"{name} must be a one-dimensional {dtype} array")
        count = self.scores.shape[0]
        if any(value.shape != (count,) for value, _, _ in arrays):
            raise ValueError("affinity result arrays must have equal lengths")
        if not np.isfinite(self.scores).all() or np.any(
            (self.scores < 0.0) | (self.scores > 1.0)
        ):
            raise ValueError("scores must be finite and in [0, 1]")
        if np.any(self.rejection_codes < 0):
            raise ValueError("rejection_codes must be nonnegative")
        if np.any(self.hard_rejected != (self.rejection_codes != 0)):
            raise ValueError("hard_rejected and rejection_codes disagree")
        if np.any(self.hard_rejected & (self.scores != 0.0)):
            raise ValueError("hard-rejected edges must have score zero")
        for name in (
            "scores", "same_object", "hard_rejected", "rejection_codes"
        ):
            object.__setattr__(self, name, immutable_array(getattr(self, name)))


@dataclass(frozen=True)
class RGBDProcessingResult:
    """Complete reusable output of the RGB-D patch-affinity pipeline."""

    scene: SceneInput
    cloud: OrganizedCloud
    geometry: OrganizedGeometry
    patches: PatchSet
    graph: PatchGraph
    affinity: EdgeAffinity


@dataclass(frozen=True)
class AffinityMetrics:
    """Immutable binary edge-affinity evaluation statistics."""

    evaluated_edge_count: int
    positive_target_count: int
    negative_target_count: int
    true_positives: int
    true_negatives: int
    false_positives: int
    false_negatives: int
    precision: float
    recall: float
    f1_score: float
    specificity: float
    balanced_accuracy: float

    def __post_init__(self) -> None:
        """Validate count identities and finite unit-interval metrics."""
        count_names = (
            "evaluated_edge_count", "positive_target_count",
            "negative_target_count", "true_positives", "true_negatives",
            "false_positives", "false_negatives",
        )
        for name in count_names:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, numbers.Integral):
                raise TypeError(f"{name} must be an integer")
            if value < 0:
                raise ValueError(f"{name} must be nonnegative")
        if self.positive_target_count != self.true_positives + self.false_negatives:
            raise ValueError("positive target count is inconsistent")
        if self.negative_target_count != self.true_negatives + self.false_positives:
            raise ValueError("negative target count is inconsistent")
        if self.evaluated_edge_count != (
            self.positive_target_count + self.negative_target_count
        ):
            raise ValueError("evaluated edge count is inconsistent")
        for name in (
            "precision", "recall", "f1_score", "specificity",
            "balanced_accuracy",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, numbers.Real):
                raise TypeError(f"{name} must be a real number")
            if not math.isfinite(float(value)) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be finite and in [0, 1]")

        def ratio(numerator: int, denominator: int) -> float:
            return float(numerator / denominator) if denominator else 0.0

        expected = (
            ratio(self.true_positives,
                  self.true_positives + self.false_positives),
            ratio(self.true_positives,
                  self.true_positives + self.false_negatives),
            ratio(2 * self.true_positives,
                  2 * self.true_positives + self.false_positives
                  + self.false_negatives),
            ratio(self.true_negatives,
                  self.true_negatives + self.false_positives),
        )
        actual = (self.precision, self.recall, self.f1_score, self.specificity)
        if any(not math.isclose(float(value), expected_value,
                                rel_tol=0.0, abs_tol=1e-15)
               for value, expected_value in zip(actual, expected)):
            raise ValueError("reported metrics disagree with confusion counts")
        expected_balanced = (expected[1] + expected[3]) / 2.0
        if not math.isclose(float(self.balanced_accuracy), expected_balanced,
                            rel_tol=0.0, abs_tol=1e-15):
            raise ValueError("balanced accuracy disagrees with recall and specificity")
