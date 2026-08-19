"""Deterministic and explainable patch-edge affinity prediction."""

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np

from .adjacency import PATCH_GRAPH_FEATURE_NAMES
from .models import AffinityMetrics
from .models import EdgeAffinity
from .models import PatchAffinityConfig
from .models import PatchGraph
from .models import SceneEdgeExamples


REJECTION_NONE = 0
REJECTION_BOUNDARY_POINT_DISTANCE = 1
REJECTION_BOUNDARY_DEPTH_DIFFERENCE = 2
REJECTION_CENTROID_DEPTH_DIFFERENCE = 3
REJECTION_DEPTH_EDGE_EVIDENCE = 4
REJECTION_NORMAL_ANGLE = 5
REJECTION_INSUFFICIENT_BOUNDARY_SUPPORT = 6
REJECTION_INSUFFICIENT_NORMAL_EVIDENCE = 7
REJECTION_RGB_DISTANCE = 8

AFFINITY_REJECTION_CODE_NAMES = (
    "none",
    "excessive_boundary_point_distance",
    "excessive_boundary_depth_difference",
    "excessive_centroid_depth_difference",
    "excessive_depth_edge_evidence",
    "excessive_normal_angle_evidence",
    "insufficient_boundary_support",
    "insufficient_valid_normal_evidence",
    "excessive_rgb_difference",
)

# Gates are applied in this public order; the first failing gate supplies the
# sole rejection code. Diagonal-only contacts reach the boundary-support gate
# without interpreting their Phase 5 missing orthogonal measurements.
AFFINITY_REJECTION_PRECEDENCE = (
    REJECTION_BOUNDARY_POINT_DISTANCE,
    REJECTION_BOUNDARY_DEPTH_DIFFERENCE,
    REJECTION_CENTROID_DEPTH_DIFFERENCE,
    REJECTION_DEPTH_EDGE_EVIDENCE,
    REJECTION_NORMAL_ANGLE,
    REJECTION_INSUFFICIENT_BOUNDARY_SUPPORT,
    REJECTION_INSUFFICIENT_NORMAL_EVIDENCE,
    REJECTION_RGB_DISTANCE,
)

_REQUIRED_FEATURES = (
    "shared_boundary_count",
    "diagonal_contact_count",
    "shared_boundary_fraction_min_patch",
    "centroid_depth_difference_m",
    "mean_boundary_point_distance_m",
    "max_boundary_depth_difference_m",
    "mean_boundary_depth_difference_m",
    "depth_edge_endpoint_fraction",
    "normal_pair_valid_fraction",
    "mean_unsigned_normal_angle_deg",
    "mean_boundary_rgb_distance",
)


class PatchAffinityError(ValueError):
    """Indicate an invalid graph or affinity evaluation contract."""


@runtime_checkable
class PatchAffinityPredictor(Protocol):
    """Structural interface for interchangeable patch-affinity predictors."""

    def predict(self, graph: PatchGraph) -> EdgeAffinity:
        """Predict scores and decisions in the supplied graph-edge order."""
        ...


@dataclass(frozen=True)
class DeterministicPatchAffinityPredictor:
    """Reusable deterministic predictor holding validated configuration."""

    config: PatchAffinityConfig = PatchAffinityConfig()

    def __post_init__(self) -> None:
        """Require the package affinity configuration contract."""
        if not isinstance(self.config, PatchAffinityConfig):
            raise TypeError("config must be a PatchAffinityConfig")

    def predict(self, graph: PatchGraph) -> EdgeAffinity:
        """Delegate to the deterministic functional predictor."""
        return predict_patch_affinity(graph, self.config)


def _feature_columns(graph: PatchGraph) -> dict[str, np.ndarray]:
    """Validate graph structure and resolve required features by public name."""
    if graph.edges.ndim != 2 or graph.edges.shape[1:] != (2,):
        raise PatchAffinityError("graph edges must have shape (E, 2)")
    edge_count = graph.edges.shape[0]
    names = tuple(graph.feature_names)
    if graph.edge_features.shape != (edge_count, len(names)):
        raise PatchAffinityError("graph feature dimensions are inconsistent")
    if len(names) != len(set(names)):
        raise PatchAffinityError("graph feature names must be unique")
    if set(names) != set(PATCH_GRAPH_FEATURE_NAMES):
        raise PatchAffinityError("graph features do not match the Phase 5 schema")
    missing = [name for name in _REQUIRED_FEATURES if names.count(name) != 1]
    if missing:
        raise PatchAffinityError(f"required graph features are missing: {missing}")
    if not np.isfinite(graph.edge_features).all():
        raise PatchAffinityError("graph edge features must be finite")
    columns = {
        name: graph.edge_features[:, names.index(name)]
        for name in _REQUIRED_FEATURES
    }
    orthogonal = columns["shared_boundary_count"] > 0.0
    for name in ("shared_boundary_count", "diagonal_contact_count"):
        values = columns[name]
        if np.any(values < 0.0) or np.any(values != np.floor(values)):
            raise PatchAffinityError(f"{name} must contain nonnegative counts")
    for name in (
        "centroid_depth_difference_m", "mean_boundary_point_distance_m",
        "max_boundary_depth_difference_m",
        "mean_boundary_depth_difference_m", "mean_boundary_rgb_distance",
    ):
        available = np.ones(edge_count, dtype=bool)
        if name != "centroid_depth_difference_m":
            available = orthogonal
        if np.any(columns[name][available] < 0.0):
            raise PatchAffinityError(f"{name} must be nonnegative")
    for name, available in (
        ("shared_boundary_fraction_min_patch",
         np.ones(edge_count, dtype=bool)),
        ("depth_edge_endpoint_fraction", orthogonal),
        ("normal_pair_valid_fraction", orthogonal),
    ):
        values = columns[name][available]
        if np.any((values < 0.0) | (values > 1.0)):
            raise PatchAffinityError(f"{name} must be in [0, 1]")
    angle = columns["mean_unsigned_normal_angle_deg"]
    normal_available = orthogonal & (columns["normal_pair_valid_fraction"] > 0.0)
    if np.any(normal_available & ((angle < 0.0) | (angle > 180.0))):
        raise PatchAffinityError("available normal angles must be in [0, 180]")
    return columns


def _compatibility(value: np.ndarray, maximum: float) -> np.ndarray:
    """Map nonnegative discontinuity to monotonic unit compatibility."""
    if maximum == 0.0:
        return (value == 0.0).astype(np.float64)
    return np.clip(1.0 - value.astype(np.float64) / maximum, 0.0, 1.0)


def _set_first_rejection(
    codes: np.ndarray, condition: np.ndarray, code: int
) -> None:
    """Apply one hard gate only where no higher-precedence gate applied."""
    codes[(codes == REJECTION_NONE) & condition] = code


def predict_patch_affinity(
    graph: PatchGraph, config: PatchAffinityConfig
) -> EdgeAffinity:
    """
    Predict affinity using ordered hard gates and an equal-weight evidence mean.

    Each available component increases from zero (at its incompatibility
    threshold) to one (best agreement). Boundary support increases with shared
    fraction; every discontinuity component decreases as evidence grows.
    """
    if not isinstance(config, PatchAffinityConfig):
        raise TypeError("config must be a PatchAffinityConfig")
    columns = _feature_columns(graph)
    edge_count = graph.edges.shape[0]
    codes = np.zeros(edge_count, dtype=np.int32)
    orthogonal = columns["shared_boundary_count"] > 0.0
    normal_fraction = columns["normal_pair_valid_fraction"]
    sufficient_normals = (
        orthogonal
        & (normal_fraction > 0.0)
        & (normal_fraction >= config.minimum_normal_pair_valid_fraction)
    )
    diagonal_only = (
        (columns["shared_boundary_count"] == 0.0)
        & (columns["diagonal_contact_count"] > 0.0)
    )
    _set_first_rejection(
        codes,
        orthogonal & (
            columns["mean_boundary_point_distance_m"]
            > config.maximum_mean_boundary_point_distance_m
        ),
        REJECTION_BOUNDARY_POINT_DISTANCE,
    )
    _set_first_rejection(
        codes,
        orthogonal & (
            columns["max_boundary_depth_difference_m"]
            > config.maximum_boundary_depth_difference_m
        ),
        REJECTION_BOUNDARY_DEPTH_DIFFERENCE,
    )
    _set_first_rejection(
        codes,
        columns["centroid_depth_difference_m"]
        > config.maximum_centroid_depth_difference_m,
        REJECTION_CENTROID_DEPTH_DIFFERENCE,
    )
    _set_first_rejection(
        codes,
        orthogonal & (
            columns["depth_edge_endpoint_fraction"]
            > config.maximum_depth_edge_endpoint_fraction
        ),
        REJECTION_DEPTH_EDGE_EVIDENCE,
    )
    _set_first_rejection(
        codes,
        sufficient_normals
        & (columns["mean_unsigned_normal_angle_deg"]
           > config.maximum_unsigned_normal_angle_deg),
        REJECTION_NORMAL_ANGLE,
    )
    _set_first_rejection(
        codes,
        diagonal_only
        | (columns["shared_boundary_fraction_min_patch"]
           < config.minimum_shared_boundary_fraction),
        REJECTION_INSUFFICIENT_BOUNDARY_SUPPORT,
    )
    if config.normal_evidence_required:
        _set_first_rejection(
            codes, ~sufficient_normals,
            REJECTION_INSUFFICIENT_NORMAL_EVIDENCE,
        )
    if config.maximum_mean_boundary_rgb_distance is not None:
        _set_first_rejection(
            codes,
            orthogonal & (
                columns["mean_boundary_rgb_distance"]
                > config.maximum_mean_boundary_rgb_distance
            ),
            REJECTION_RGB_DISTANCE,
        )

    components = [
        np.clip(
            columns["shared_boundary_fraction_min_patch"].astype(np.float64)
            / max(config.minimum_shared_boundary_fraction, np.finfo(float).eps),
            0.0, 1.0,
        ),
        _compatibility(
            columns["mean_boundary_point_distance_m"],
            config.maximum_mean_boundary_point_distance_m,
        ),
        _compatibility(
            columns["mean_boundary_depth_difference_m"],
            config.maximum_boundary_depth_difference_m,
        ),
        _compatibility(
            columns["centroid_depth_difference_m"],
            config.maximum_centroid_depth_difference_m,
        ),
        _compatibility(
            columns["depth_edge_endpoint_fraction"],
            config.maximum_depth_edge_endpoint_fraction,
        ),
    ]
    totals = np.sum(components, axis=0, dtype=np.float64)
    weights = np.full(edge_count, len(components), dtype=np.float64)
    if np.any(sufficient_normals):
        normal_component = _compatibility(
            columns["mean_unsigned_normal_angle_deg"],
            config.maximum_unsigned_normal_angle_deg,
        )
        totals[sufficient_normals] += normal_component[sufficient_normals]
        weights[sufficient_normals] += 1.0
    if config.maximum_mean_boundary_rgb_distance is not None:
        totals += _compatibility(
            columns["mean_boundary_rgb_distance"],
            config.maximum_mean_boundary_rgb_distance,
        )
        weights += 1.0
    scores = np.divide(
        totals, weights, out=np.zeros_like(totals), where=weights > 0.0
    )
    hard_rejected = codes != REJECTION_NONE
    scores[hard_rejected] = 0.0
    scores = np.clip(scores, 0.0, 1.0).astype(np.float32)
    same_object = (~hard_rejected) & (scores >= config.decision_threshold)
    return EdgeAffinity(scores, same_object, hard_rejected, codes)


def _safe_ratio(numerator: int, denominator: int) -> float:
    """Return zero for an undefined binary metric denominator."""
    return float(numerator / denominator) if denominator else 0.0


def evaluate_patch_affinity(
    affinity: EdgeAffinity, examples: SceneEdgeExamples
) -> AffinityMetrics:
    """Evaluate predictions only at Phase 6 eligible graph-edge indices."""
    edge_count = affinity.scores.shape[0]
    contracts = (
        (affinity.same_object, np.bool_, "same_object"),
        (affinity.hard_rejected, np.bool_, "hard_rejected"),
        (affinity.rejection_codes, np.int32, "rejection_codes"),
    )
    if affinity.scores.shape != (edge_count,) or affinity.scores.dtype != np.float32:
        raise PatchAffinityError("affinity scores contract is invalid")
    for array, dtype, name in contracts:
        if array.shape != (edge_count,) or array.dtype != dtype:
            raise PatchAffinityError(f"affinity {name} contract is invalid")
    if not np.isfinite(affinity.scores).all() or np.any(
        (affinity.scores < 0.0) | (affinity.scores > 1.0)
    ):
        raise PatchAffinityError("affinity scores must be finite and in [0, 1]")
    count = examples.targets.shape[0]
    if examples.targets.dtype != np.uint8 or np.any(examples.targets > 1):
        raise PatchAffinityError("example targets must be uint8 zero or one")
    indices = examples.graph_edge_indices
    if indices.shape != (count,) or not np.issubdtype(indices.dtype, np.integer):
        raise PatchAffinityError("example graph edge indices are invalid")
    if count and (np.any(indices < 0) or np.any(indices >= edge_count)):
        raise PatchAffinityError("example graph edge index is out of bounds")
    if np.unique(indices).size != count:
        raise PatchAffinityError("example graph edge indices must be unique")
    predicted = affinity.same_object[indices]
    positive = examples.targets == 1
    negative = ~positive
    true_positives = int(np.count_nonzero(predicted & positive))
    true_negatives = int(np.count_nonzero(~predicted & negative))
    false_positives = int(np.count_nonzero(predicted & negative))
    false_negatives = int(np.count_nonzero(~predicted & positive))
    precision = _safe_ratio(true_positives, true_positives + false_positives)
    recall = _safe_ratio(true_positives, true_positives + false_negatives)
    specificity = _safe_ratio(true_negatives, true_negatives + false_positives)
    f1_score = _safe_ratio(2 * true_positives, 2 * true_positives
                           + false_positives + false_negatives)
    return AffinityMetrics(
        count,
        int(np.count_nonzero(positive)),
        int(np.count_nonzero(negative)),
        true_positives,
        true_negatives,
        false_positives,
        false_negatives,
        precision,
        recall,
        f1_score,
        specificity,
        (recall + specificity) / 2.0,
    )
