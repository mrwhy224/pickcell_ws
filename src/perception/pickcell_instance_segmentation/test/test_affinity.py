"""Tests for deterministic explainable patch-edge affinity prediction."""

import os
from pathlib import Path

import numpy as np
import pytest

from pickcell_instance_segmentation import AFFINITY_REJECTION_CODE_NAMES
from pickcell_instance_segmentation import DeterministicPatchAffinityPredictor
from pickcell_instance_segmentation import EdgeAffinity
from pickcell_instance_segmentation import evaluate_patch_affinity
from pickcell_instance_segmentation import PATCH_GRAPH_FEATURE_NAMES
from pickcell_instance_segmentation import PatchAffinityConfig
from pickcell_instance_segmentation import PatchAffinityError
from pickcell_instance_segmentation import PatchAffinityPredictor
from pickcell_instance_segmentation import PatchGraph
from pickcell_instance_segmentation import predict_patch_affinity
from pickcell_instance_segmentation import REJECTION_BOUNDARY_DEPTH_DIFFERENCE
from pickcell_instance_segmentation import REJECTION_BOUNDARY_POINT_DISTANCE
from pickcell_instance_segmentation import REJECTION_CENTROID_DEPTH_DIFFERENCE
from pickcell_instance_segmentation import REJECTION_DEPTH_EDGE_EVIDENCE
from pickcell_instance_segmentation import REJECTION_INSUFFICIENT_BOUNDARY_SUPPORT
from pickcell_instance_segmentation import REJECTION_INSUFFICIENT_NORMAL_EVIDENCE
from pickcell_instance_segmentation import REJECTION_NONE
from pickcell_instance_segmentation import REJECTION_NORMAL_ANGLE
from pickcell_instance_segmentation import REJECTION_RGB_DISTANCE
from pickcell_instance_segmentation import SceneEdgeExamples


def graph_with(**updates: float) -> PatchGraph:
    """Construct one or more valid edges with controlled named features."""
    edge_count = max((np.asarray(value).size for value in updates.values()), default=1)
    defaults = {
        "shared_boundary_count": 10.0,
        "diagonal_contact_count": 0.0,
        "shared_boundary_fraction_min_patch": 0.5,
        "centroid_distance_m": 0.01,
        "centroid_depth_difference_m": 0.005,
        "patch_size_ratio": 1.0,
        "mean_boundary_point_distance_m": 0.005,
        "max_boundary_point_distance_m": 0.005,
        "mean_boundary_depth_difference_m": 0.003,
        "max_boundary_depth_difference_m": 0.005,
        "depth_edge_endpoint_fraction": 0.0,
        "normal_pair_valid_fraction": 1.0,
        "mean_unsigned_normal_angle_deg": 0.0,
        "max_unsigned_normal_angle_deg": 0.0,
        "mean_boundary_rgb_distance": 0.0,
        "max_boundary_rgb_distance": 0.0,
        "mean_patch_rgb_distance": 0.0,
        "cross_tile_boundary_fraction": 0.0,
    }
    features = np.empty(
        (edge_count, len(PATCH_GRAPH_FEATURE_NAMES)), dtype=np.float32
    )
    for column, name in enumerate(PATCH_GRAPH_FEATURE_NAMES):
        value = updates.get(name, defaults[name])
        features[:, column] = np.broadcast_to(value, (edge_count,))
    edges = np.column_stack((
        np.arange(edge_count, dtype=np.int32),
        np.arange(1, edge_count + 1, dtype=np.int32),
    ))
    return PatchGraph(
        edges, features, PATCH_GRAPH_FEATURE_NAMES,
        features[:, 0].astype(np.int64),
        features[:, 1].astype(np.int64),
        np.zeros(edge_count + 1, dtype=np.int64),
    )


def examples_for(indices, targets) -> SceneEdgeExamples:
    """Create eligible Phase 6 examples selecting original graph edges."""
    indices = np.asarray(indices, dtype=np.int64)
    targets = np.asarray(targets, dtype=np.uint8)
    count = targets.size
    return SceneEdgeExamples(
        np.zeros((count, len(PATCH_GRAPH_FEATURE_NAMES)), np.float32),
        targets, indices, np.zeros((count, 2), np.int32),
        PATCH_GRAPH_FEATURE_NAMES, "scene", int(targets.sum()),
        int(count - targets.sum()), 0, 0, 0, 0, 0,
    )


def test_coplanar_neighbor_has_high_affinity() -> None:
    """Give strongly compatible coplanar evidence a high positive score."""
    result = predict_patch_affinity(graph_with(), PatchAffinityConfig())
    assert result.rejection_codes.tolist() == [REJECTION_NONE]
    assert result.scores[0] > 0.8
    assert result.same_object.tolist() == [True]


@pytest.mark.parametrize(
    "feature,value,expected",
    [
        ("mean_boundary_point_distance_m", 0.026,
         REJECTION_BOUNDARY_POINT_DISTANCE),
        ("max_boundary_depth_difference_m", 0.026,
         REJECTION_BOUNDARY_DEPTH_DIFFERENCE),
        ("centroid_depth_difference_m", 0.081,
         REJECTION_CENTROID_DEPTH_DIFFERENCE),
        ("depth_edge_endpoint_fraction", 0.51,
         REJECTION_DEPTH_EDGE_EVIDENCE),
        ("mean_unsigned_normal_angle_deg", 36.0, REJECTION_NORMAL_ANGLE),
        ("shared_boundary_fraction_min_patch", 0.02,
         REJECTION_INSUFFICIENT_BOUNDARY_SUPPORT),
    ],
)
def test_each_hard_geometric_gate(feature, value, expected) -> None:
    """Reject each independently excessive boundary cue with its stable code."""
    result = predict_patch_affinity(
        graph_with(**{feature: value}), PatchAffinityConfig()
    )
    assert result.hard_rejected.tolist() == [True]
    assert result.rejection_codes.tolist() == [expected]
    assert result.scores.tolist() == [0.0]


def test_normal_missingness_is_neutral_or_required() -> None:
    """Exclude unavailable normals unless configuration explicitly requires them."""
    graph = graph_with(
        normal_pair_valid_fraction=0.0,
        mean_unsigned_normal_angle_deg=0.0,
    )
    neutral = predict_patch_affinity(graph, PatchAffinityConfig())
    neutral_at_zero = predict_patch_affinity(
        graph, PatchAffinityConfig(minimum_normal_pair_valid_fraction=0.0)
    )
    required = predict_patch_affinity(
        graph, PatchAffinityConfig(normal_evidence_required=True)
    )
    assert not neutral.hard_rejected[0]
    assert not neutral_at_zero.hard_rejected[0]
    assert neutral.scores.tobytes() == neutral_at_zero.scores.tobytes()
    assert required.rejection_codes[0] == REJECTION_INSUFFICIENT_NORMAL_EVIDENCE


def test_rgb_is_disabled_by_default_and_optional_gate_rejects() -> None:
    """Ignore raw RGB distance unless its explicit optional gate is enabled."""
    graph = graph_with(mean_boundary_rgb_distance=200.0)
    default = predict_patch_affinity(graph, PatchAffinityConfig())
    enabled = predict_patch_affinity(
        graph, PatchAffinityConfig(maximum_mean_boundary_rgb_distance=50.0)
    )
    assert not default.hard_rejected[0]
    assert enabled.rejection_codes[0] == REJECTION_RGB_DISTANCE


def test_diagonal_only_contact_is_conservatively_rejected() -> None:
    """Never treat Phase 5 missing orthogonal values as perfect continuity."""
    graph = graph_with(
        shared_boundary_count=0.0,
        diagonal_contact_count=1.0,
        shared_boundary_fraction_min_patch=0.0,
        mean_boundary_point_distance_m=0.0,
        max_boundary_depth_difference_m=0.0,
    )
    result = predict_patch_affinity(
        graph, PatchAffinityConfig(minimum_shared_boundary_fraction=0.0)
    )
    assert result.rejection_codes[0] == REJECTION_INSUFFICIENT_BOUNDARY_SUPPORT


def test_diagonal_only_contact_ignores_phase_five_missing_sentinel() -> None:
    """Do not interpret unavailable orthogonal measurements as observations."""
    graph = graph_with(
        shared_boundary_count=0.0,
        diagonal_contact_count=1.0,
        shared_boundary_fraction_min_patch=0.0,
        mean_boundary_point_distance_m=-1.0,
        max_boundary_depth_difference_m=-1.0,
        mean_boundary_depth_difference_m=-1.0,
        depth_edge_endpoint_fraction=-1.0,
        normal_pair_valid_fraction=-1.0,
        mean_unsigned_normal_angle_deg=-1.0,
        mean_boundary_rgb_distance=-1.0,
    )
    result = predict_patch_affinity(graph, PatchAffinityConfig())
    assert result.rejection_codes[0] == REJECTION_INSUFFICIENT_BOUNDARY_SUPPORT


@pytest.mark.parametrize(
    "feature,better,worse",
    [
        ("mean_boundary_point_distance_m", 0.002, 0.015),
        ("mean_boundary_depth_difference_m", 0.002, 0.015),
        ("mean_unsigned_normal_angle_deg", 2.0, 25.0),
    ],
)
def test_discontinuity_score_monotonicity(feature, better, worse) -> None:
    """Ensure increasing a surviving discontinuity never raises its score."""
    graph = graph_with(**{feature: np.array([better, worse])})
    scores = predict_patch_affinity(graph, PatchAffinityConfig()).scores
    assert scores[0] >= scores[1]


def test_boundary_support_score_is_monotonic() -> None:
    """Ensure stronger valid shared-boundary support never reduces affinity."""
    graph = graph_with(
        shared_boundary_fraction_min_patch=np.array([0.03, 0.02])
    )
    result = predict_patch_affinity(
        graph, PatchAffinityConfig(minimum_shared_boundary_fraction=0.0)
    )
    assert result.scores[0] >= result.scores[1]


def test_exact_thresholds_pass_and_precedence_is_stable() -> None:
    """Keep inclusive thresholds and select the first documented failed gate."""
    exact = graph_with(
        mean_boundary_point_distance_m=0.025,
        max_boundary_depth_difference_m=0.025,
        centroid_depth_difference_m=0.08,
        depth_edge_endpoint_fraction=0.50,
        mean_unsigned_normal_angle_deg=35.0,
        shared_boundary_fraction_min_patch=0.03,
    )
    assert not predict_patch_affinity(exact, PatchAffinityConfig()).hard_rejected[0]
    multiple = graph_with(
        mean_boundary_point_distance_m=1.0,
        max_boundary_depth_difference_m=1.0,
        depth_edge_endpoint_fraction=1.0,
    )
    result = predict_patch_affinity(multiple, PatchAffinityConfig())
    assert result.rejection_codes[0] == REJECTION_BOUNDARY_POINT_DISTANCE
    assert AFFINITY_REJECTION_CODE_NAMES[result.rejection_codes[0]].startswith(
        "excessive_boundary_point"
    )


def test_empty_graph_and_output_contracts() -> None:
    """Preserve exact types, shapes, finite ranges, ownership, and emptiness."""
    graph = graph_with(
        shared_boundary_count=np.empty(0),
        diagonal_contact_count=np.empty(0),
    )
    result = predict_patch_affinity(graph, PatchAffinityConfig())
    assert result.scores.shape == result.same_object.shape == (0,)
    expected = (np.float32, np.bool_, np.bool_, np.int32)
    for array, dtype in zip(vars(result).values(), expected):
        assert array.dtype == dtype
        assert array.flags.owndata and not array.flags.writeable
    assert np.isfinite(result.scores).all()


def test_result_model_rejects_invalid_array_contracts() -> None:
    """Enforce result contracts even for independently supplied predictors."""
    with pytest.raises(ValueError, match="float32"):
        EdgeAffinity(
            np.array([0.5], np.float64), np.array([True]),
            np.array([False]), np.array([0], np.int32),
        )
    with pytest.raises(ValueError, match="disagree"):
        EdgeAffinity(
            np.array([0.0], np.float32), np.array([False]),
            np.array([False]), np.array([1], np.int32),
        )


def test_invalid_configurations_and_graphs_are_rejected() -> None:
    """Reject invalid numeric types, domains, missing names, and feature values."""
    invalid = (
        {"maximum_mean_boundary_point_distance_m": True},
        {"maximum_boundary_depth_difference_m": -1.0},
        {"maximum_unsigned_normal_angle_deg": 181.0},
        {"maximum_depth_edge_endpoint_fraction": np.nan},
        {"minimum_shared_boundary_fraction": 1.1},
        {"normal_evidence_required": 1},
        {"decision_threshold": -0.1},
    )
    for update in invalid:
        with pytest.raises((TypeError, ValueError)):
            PatchAffinityConfig(**update)
    graph = graph_with()
    malformed = PatchGraph(
        graph.edges, graph.edge_features[:, :-1], graph.feature_names,
        graph.shared_boundary_counts, graph.diagonal_contact_counts,
        graph.node_degrees,
    )
    with pytest.raises(PatchAffinityError, match="dimensions"):
        predict_patch_affinity(malformed, PatchAffinityConfig())
    bad_values = graph_with(depth_edge_endpoint_fraction=2.0)
    with pytest.raises(PatchAffinityError, match=r"\[0, 1\]"):
        predict_patch_affinity(bad_values, PatchAffinityConfig())


def test_repeated_results_are_byte_identical_and_predictor_matches_protocol() -> None:
    """Provide deterministic functional and reusable protocol-compatible APIs."""
    graph = graph_with(
        mean_boundary_point_distance_m=np.array([0.002, 0.012])
    )
    predictor = DeterministicPatchAffinityPredictor(PatchAffinityConfig())
    assert isinstance(predictor, PatchAffinityPredictor)
    first = predictor.predict(graph)
    second = predict_patch_affinity(graph, predictor.config)
    for name in vars(first):
        assert getattr(first, name).tobytes() == getattr(second, name).tobytes()


def test_offline_metrics_and_ignored_edges() -> None:
    """Evaluate only selected Phase 6 edges with deterministic zero handling."""
    affinity = EdgeAffinity(
        np.array([0.9, 0.8, 0.1, 0.2, 0.7], np.float32),
        np.array([1, 1, 0, 0, 1], bool),
        np.zeros(5, bool),
        np.zeros(5, np.int32),
    )
    examples = examples_for([0, 1, 2, 3], [1, 0, 1, 0])
    metrics = evaluate_patch_affinity(affinity, examples)
    assert metrics.evaluated_edge_count == 4
    assert (metrics.true_positives, metrics.true_negatives) == (1, 1)
    assert (metrics.false_positives, metrics.false_negatives) == (1, 1)
    assert metrics.precision == metrics.recall == metrics.f1_score == 0.5
    assert metrics.specificity == metrics.balanced_accuracy == 0.5
    empty = evaluate_patch_affinity(affinity, examples_for([], []))
    assert empty.precision == empty.recall == empty.balanced_accuracy == 0.0


def test_ground_truth_never_changes_prediction() -> None:
    """Keep Phase 6 targets entirely outside the predictor input and output."""
    graph = graph_with()
    before = predict_patch_affinity(graph, PatchAffinityConfig())
    evaluate_patch_affinity(before, examples_for([0], [0]))
    evaluate_patch_affinity(before, examples_for([0], [1]))
    after = predict_patch_affinity(graph, PatchAffinityConfig())
    assert before.scores.tobytes() == after.scores.tobytes()
    assert before.same_object.tobytes() == after.same_object.tobytes()


@pytest.mark.skipif(
    "PICKCELL_DATASET_ROOT" not in os.environ,
    reason="PICKCELL_DATASET_ROOT is not set; affinity diagnostic skipped",
)
def test_real_dataset_affinity_diagnostic() -> None:
    """Reserve the opt-in real-dataset diagnostic for configured environments."""
    root = Path(os.environ["PICKCELL_DATASET_ROOT"]).resolve(strict=True)
    assert root.is_dir()
    pytest.skip(
        "dataset root is configured; run the full read-only diagnostic explicitly"
    )
