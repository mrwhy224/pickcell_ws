"""Tests for class-agnostic supervised patch-edge datasets."""

import hashlib
import json
import os
from pathlib import Path

import numpy as np
import pytest

from pickcell_instance_segmentation import aggregate_scene_examples
from pickcell_instance_segmentation import assign_patch_ground_truth
from pickcell_instance_segmentation import build_patch_graph
from pickcell_instance_segmentation import build_scene_edge_examples
from pickcell_instance_segmentation import compute_organized_geometry
from pickcell_instance_segmentation import EdgeDatasetError
from pickcell_instance_segmentation import EdgeDatasetSerializationError
from pickcell_instance_segmentation import EdgeLabelConfig
from pickcell_instance_segmentation import load_edge_training_dataset
from pickcell_instance_segmentation import load_scene
from pickcell_instance_segmentation import NormalEstimationConfig
from pickcell_instance_segmentation import PATCH_GRAPH_FEATURE_NAMES
from pickcell_instance_segmentation import PatchGraph
from pickcell_instance_segmentation import PatchGraphConfig
from pickcell_instance_segmentation import PatchSegmentationConfig
from pickcell_instance_segmentation import PatchSet
from pickcell_instance_segmentation import rgbd_to_organized_cloud
from pickcell_instance_segmentation import save_edge_training_dataset
from pickcell_instance_segmentation import SceneEdgeExamples
from pickcell_instance_segmentation import SceneSplitConfig
from pickcell_instance_segmentation import segment_surface_patches
from pickcell_instance_segmentation import split_scene_ids


def patches_for(labels: np.ndarray) -> PatchSet:
    """Build a contract-complete PatchSet from explicit organized labels."""
    labels = np.asarray(labels, dtype=np.int32)
    count = int(labels.max(initial=-1)) + 1
    counts = np.bincount(labels[labels >= 0], minlength=count).astype(np.int64)
    centers = np.zeros((count, 3), dtype=np.float32)
    colors = np.zeros((count, 3), dtype=np.float32)
    normals = np.full((count, 3), np.nan, dtype=np.float32)
    fractions = np.zeros(count, dtype=np.float32)
    boxes = np.zeros((count, 4), dtype=np.int32)
    tiles = np.zeros((count, 2), dtype=np.int32)
    for patch_id in range(count):
        rows, columns = np.nonzero(labels == patch_id)
        boxes[patch_id] = [rows.min(), columns.min(), rows.max() + 1,
                           columns.max() + 1]
    return PatchSet(
        labels, count, counts, centers, colors, normals, fractions, boxes,
        tiles, 12,
    )


def graph_for(edges: list[list[int]], patch_count: int) -> PatchGraph:
    """Build a graph with uniquely identifiable stable feature rows."""
    edge_array = np.asarray(edges, dtype=np.int32).reshape((-1, 2))
    feature_count = len(PATCH_GRAPH_FEATURE_NAMES)
    features = np.arange(
        edge_array.shape[0] * feature_count, dtype=np.float32
    ).reshape((edge_array.shape[0], feature_count))
    return PatchGraph(
        edge_array,
        features,
        PATCH_GRAPH_FEATURE_NAMES,
        np.ones(edge_array.shape[0], dtype=np.int64),
        np.zeros(edge_array.shape[0], dtype=np.int64),
        np.zeros(patch_count, dtype=np.int64),
    )


def example_for(scene_id: str, targets: list[int]) -> SceneEdgeExamples:
    """Build a compact valid per-scene example for aggregation tests."""
    values = np.asarray(targets, dtype=np.uint8)
    count = values.size
    features = np.full(
        (count, len(PATCH_GRAPH_FEATURE_NAMES)),
        float(len(scene_id)), dtype=np.float32,
    )
    pairs = np.column_stack((np.arange(count), np.arange(count) + 1)).astype(
        np.int32
    ) if count else np.empty((0, 2), dtype=np.int32)
    return SceneEdgeExamples(
        features, values, np.arange(count, dtype=np.int64), pairs,
        PATCH_GRAPH_FEATURE_NAMES, scene_id,
        int(np.count_nonzero(values == 1)),
        int(np.count_nonzero(values == 0)), 0, 0, 0, 0, 0,
    )


def tree_digest(path: Path) -> dict[str, str]:
    """Hash regular files below a read-only diagnostic root."""
    return {
        str(item.relative_to(path)): hashlib.sha256(item.read_bytes()).hexdigest()
        for item in sorted(path.rglob("*")) if item.is_file()
    }


def test_patch_majority_purity_ambiguity_tie_and_minimum_size() -> None:
    """Assign pure and mixed patches with deterministic smallest-ID ties."""
    patches = patches_for([[0, 0, 1, 1], [0, 0, 1, 1]])
    truth = np.array([[42, 42, 9, 7], [42, 42, 7, 9]], dtype=np.uint16)
    result = assign_patch_ground_truth(
        patches, truth, EdgeLabelConfig(minimum_patch_purity=0.75)
    )
    np.testing.assert_array_equal(result.majority_instance_ids, [42, 7])
    np.testing.assert_allclose(result.purity, [1.0, 0.5])
    np.testing.assert_array_equal(result.eligible, [True, False])
    size_result = assign_patch_ground_truth(
        patches, truth, EdgeLabelConfig(
            minimum_patch_purity=0.5, minimum_patch_pixels=5
        )
    )
    assert not size_result.eligible.any()


def test_target_policy_and_exact_diagnostics() -> None:
    """Apply positive, negative, background, and ambiguous edge policies."""
    patches = patches_for([[0, 1, 2, 3, 4, 5]])
    truth = np.array([[10, 10, 77, 0, 0, 8]], dtype=np.uint16)
    graph = graph_for([[0, 1], [1, 2], [2, 3], [3, 4], [4, 5]], 6)
    examples = build_scene_edge_examples(
        graph, patches, truth, "scene-a", EdgeLabelConfig()
    )
    np.testing.assert_array_equal(examples.targets, [1, 0, 0, 0])
    np.testing.assert_array_equal(examples.graph_edge_indices, [0, 1, 2, 4])
    np.testing.assert_array_equal(examples.patch_pairs, graph.edges[[0, 1, 2, 4]])
    np.testing.assert_array_equal(
        examples.features, graph.edge_features[[0, 1, 2, 4]]
    )
    assert examples.positive_count == 1
    assert examples.negative_count == 3
    assert examples.ignored_count == 1
    assert examples.object_object_negative_count == 1
    assert examples.object_background_negative_count == 2
    assert examples.background_background_ignored_count == 1
    assert examples.ambiguous_ignored_count == 0


def test_ambiguous_edge_is_ignored_and_empty_classes_are_allowed() -> None:
    """Ignore an impure endpoint while allowing empty selected datasets."""
    patches = patches_for([[0, 0, 1, 1]])
    truth = np.array([[5, 6, 8, 8]], dtype=np.uint16)
    examples = build_scene_edge_examples(
        graph_for([[0, 1]], 2), patches, truth, "mixed",
        EdgeLabelConfig(minimum_patch_purity=0.75),
    )
    assert examples.features.shape == (0, len(PATCH_GRAPH_FEATURE_NAMES))
    assert examples.positive_count == examples.negative_count == 0
    assert examples.ambiguous_ignored_count == examples.ignored_count == 1


def test_arbitrary_ids_are_scene_local_and_never_feature_columns() -> None:
    """Use IDs only for within-scene targets, never numeric feature data."""
    patches = patches_for([[0, 1]])
    graph = graph_for([[0, 1]], 2)
    same = build_scene_edge_examples(
        graph, patches, np.array([[101, 101]], np.uint16), "one",
        EdgeLabelConfig(),
    )
    different = build_scene_edge_examples(
        graph, patches, np.array([[101, 909]], np.uint16), "two",
        EdgeLabelConfig(),
    )
    assert same.targets.tolist() == [1]
    assert different.targets.tolist() == [0]
    np.testing.assert_array_equal(same.features, graph.edge_features)
    np.testing.assert_array_equal(different.features, graph.edge_features)


def test_input_validation_and_config_validation() -> None:
    """Reject invalid ground truth, feature schemas, scene IDs, and configs."""
    patches = patches_for([[0, 1]])
    with pytest.raises(EdgeDatasetError, match="shape"):
        assign_patch_ground_truth(patches, np.zeros((2, 2), np.uint16), EdgeLabelConfig())
    with pytest.raises(EdgeDatasetError, match="integer"):
        assign_patch_ground_truth(patches, np.zeros((1, 2), np.float32), EdgeLabelConfig())
    graph = graph_for([[0, 1]], 2)
    bad_graph = PatchGraph(
        graph.edges, graph.edge_features, ("wrong",),
        graph.shared_boundary_counts, graph.diagonal_contact_counts,
        graph.node_degrees,
    )
    with pytest.raises(EdgeDatasetError, match="feature order"):
        build_scene_edge_examples(
            bad_graph, patches, np.array([[1, 1]], np.uint16), "scene",
            EdgeLabelConfig(),
        )
    with pytest.raises(EdgeDatasetError, match="scene_id"):
        build_scene_edge_examples(
            graph, patches, np.array([[1, 1]], np.uint16), " ",
            EdgeLabelConfig(),
        )
    for kwargs in (
        {"background_instance_id": True}, {"background_instance_id": -1},
        {"minimum_patch_purity": 0.0}, {"minimum_patch_purity": np.inf},
        {"minimum_patch_pixels": True}, {"minimum_patch_pixels": 0},
    ):
        with pytest.raises((TypeError, ValueError)):
            EdgeLabelConfig(**kwargs)


def test_empty_graph_and_background_only_graph() -> None:
    """Represent empty and wholly ignored graphs without shape loss."""
    patches = patches_for([[0, 1]])
    empty = build_scene_edge_examples(
        graph_for([], 2), patches, np.array([[1, 2]], np.uint16), "empty",
        EdgeLabelConfig(),
    )
    assert empty.patch_pairs.shape == (0, 2)
    ignored = build_scene_edge_examples(
        graph_for([[0, 1]], 2), patches, np.zeros((1, 2), np.uint16),
        "background", EdgeLabelConfig(),
    )
    assert ignored.targets.size == 0
    assert ignored.background_background_ignored_count == 1


def test_scene_split_is_deterministic_disjoint_and_handles_small_counts() -> None:
    """Split sorted whole scenes without overlap, duplication, or edge splitting."""
    ids = ["d", "b", "a", "c"]
    first = split_scene_ids(ids, SceneSplitConfig())
    second = split_scene_ids(reversed(ids), SceneSplitConfig())
    assert first == second
    flattened = sum((first[name] for name in ("train", "validation", "test")), ())
    assert len(flattened) == len(set(flattened)) == len(ids)
    assert set(flattened) == set(ids)
    small = split_scene_ids(["only"], SceneSplitConfig())
    assert sum(map(len, small.values())) == 1
    assert sum(not values for values in small.values()) == 2
    with pytest.raises(EdgeDatasetError, match="unique"):
        split_scene_ids(["same", "same"], SceneSplitConfig())
    with pytest.raises(ValueError, match="sum"):
        SceneSplitConfig(0.5, 0.3, 0.3)
    with pytest.raises(TypeError, match="seed"):
        SceneSplitConfig(seed=True)


def test_aggregation_preserves_scene_and_edge_order() -> None:
    """Concatenate scenes deterministically and retain numeric provenance."""
    first = example_for("alpha", [1, 0])
    second = example_for("beta-long", [0])
    dataset = aggregate_scene_examples([first, second], "train")
    assert dataset.scene_ids == ("alpha", "beta-long")
    np.testing.assert_array_equal(dataset.targets, [1, 0, 0])
    np.testing.assert_array_equal(dataset.scene_indices, [0, 0, 1])
    np.testing.assert_array_equal(dataset.features[:2], first.features)
    np.testing.assert_array_equal(dataset.features[2:], second.features)
    assert dataset.feature_names == PATCH_GRAPH_FEATURE_NAMES


def test_aggregation_rejects_schema_and_preserves_empty_shape() -> None:
    """Reject incompatible feature ordering and preserve safe empty arrays."""
    empty = aggregate_scene_examples([], "test")
    assert empty.features.shape == (0, len(PATCH_GRAPH_FEATURE_NAMES))
    assert empty.patch_pairs.shape == (0, 2)
    assert empty.scene_ids == ()
    source = example_for("bad", [1])
    incompatible = SceneEdgeExamples(
        source.features, source.targets, source.graph_edge_indices,
        source.patch_pairs, tuple(reversed(source.feature_names)), source.scene_id,
        1, 0, 0, 0, 0, 0, 0,
    )
    with pytest.raises(EdgeDatasetError, match="schema"):
        aggregate_scene_examples([incompatible], "train")


def test_save_load_round_trip_and_allow_pickle_false(tmp_path: Path) -> None:
    """Round-trip numeric-only NPZ arrays and explicit JSON metadata."""
    dataset = aggregate_scene_examples([example_for("scene", [1, 0])], "train")
    output = tmp_path / "artifact"
    metadata = {
        "split_manifest": {"train": ["scene"], "validation": [], "test": []},
        "configuration": {"minimum_patch_purity": 0.98},
        "ignored_edge_diagnostics": {"ambiguous": 3},
    }
    save_edge_training_dataset(dataset, output, metadata)
    loaded = load_edge_training_dataset(output)
    assert loaded.scene_ids == dataset.scene_ids
    assert loaded.feature_names == PATCH_GRAPH_FEATURE_NAMES
    for name in (
        "features", "targets", "scene_indices", "graph_edge_indices",
        "patch_pairs",
    ):
        np.testing.assert_array_equal(getattr(loaded, name), getattr(dataset, name))
    with np.load(output / "arrays.npz", allow_pickle=False) as archive:
        assert all(archive[name].dtype != object for name in archive.files)
    document = json.loads((output / "metadata.json").read_text("utf-8"))
    assert document["schema_version"] == 1
    assert document["feature_names"] == list(PATCH_GRAPH_FEATURE_NAMES)
    assert document["metadata"] == metadata


def test_refuse_overwrite_and_cleanup_temporary_files(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Refuse implicit replacement and clean sibling temporary files on failure."""
    dataset = aggregate_scene_examples([example_for("scene", [1])], "train")
    output = tmp_path / "artifact"
    save_edge_training_dataset(dataset, output, {})
    with pytest.raises(FileExistsError):
        save_edge_training_dataset(dataset, output, {})
    failed = tmp_path / "failed"

    def fail_save(*args, **kwargs):
        raise RuntimeError("synthetic failure")

    monkeypatch.setattr(np, "savez", fail_save)
    with pytest.raises(RuntimeError, match="synthetic"):
        save_edge_training_dataset(dataset, failed, {})
    assert not list(failed.glob("*.tmp"))
    assert not list(failed.glob(".*.tmp"))


def test_corrupted_schema_and_arrays_are_rejected(tmp_path: Path) -> None:
    """Reject incompatible metadata schema and inconsistent persisted arrays."""
    dataset = aggregate_scene_examples([example_for("scene", [1])], "train")
    output = tmp_path / "artifact"
    save_edge_training_dataset(dataset, output, {})
    document_path = output / "metadata.json"
    document = json.loads(document_path.read_text("utf-8"))
    document["schema_version"] = 999
    document_path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(EdgeDatasetSerializationError, match="version"):
        load_edge_training_dataset(output)


def test_outputs_are_read_only_and_inputs_unchanged() -> None:
    """Own result arrays without mutating ground truth, patches, or graph."""
    patches = patches_for([[0, 1]])
    graph = graph_for([[0, 1]], 2)
    truth = np.array([[5, 5]], dtype=np.uint16)
    truth_before = truth.copy()
    graph_before = graph.edge_features.copy()
    patch_before = patches.labels.copy()
    assigned = assign_patch_ground_truth(patches, truth, EdgeLabelConfig())
    examples = build_scene_edge_examples(
        graph, patches, truth, "scene", EdgeLabelConfig()
    )
    aggregate = aggregate_scene_examples([examples], "train")
    for model in (assigned, examples, aggregate):
        for value in vars(model).values():
            if isinstance(value, np.ndarray):
                assert not value.flags.writeable
    np.testing.assert_array_equal(truth, truth_before)
    np.testing.assert_array_equal(graph.edge_features, graph_before)
    np.testing.assert_array_equal(patches.labels, patch_before)


@pytest.mark.skipif(
    "PICKCELL_DATASET_ROOT" not in os.environ,
    reason="PICKCELL_DATASET_ROOT is not set; real-dataset diagnostic skipped",
)
def test_real_dataset_edge_examples_are_read_only(tmp_path: Path) -> None:
    """Run Phases 1-6 on direct real scenes without altering source files."""
    root = Path(os.environ["PICKCELL_DATASET_ROOT"]).resolve(strict=True)
    before = tree_digest(root)
    scenes = []
    root_prefix = str(root) + os.sep
    for candidate in sorted(root.iterdir()):
        if not candidate.is_dir() or candidate.is_symlink():
            continue
        resolved = candidate.resolve(strict=True)
        if not str(resolved).startswith(root_prefix):
            continue
        if all((resolved / name).is_file() for name in (
            "color.png", "depth.png", "camera.json", "instance.png"
        )):
            scenes.append(resolved)
    if not scenes:
        pytest.skip("no direct child scenes with required ground truth files")
    split_manifest = split_scene_ids(
        [scene.name for scene in scenes], SceneSplitConfig()
    )
    examples_by_id = {}
    diagnostics = {}
    for scene_path in scenes:
        scene = load_scene(scene_path, require_ground_truth=True)
        cloud = rgbd_to_organized_cloud(scene)
        geometry = compute_organized_geometry(cloud, NormalEstimationConfig())
        patches = segment_surface_patches(
            cloud, geometry, PatchSegmentationConfig()
        )
        graph = build_patch_graph(
            cloud, geometry, patches, PatchGraphConfig()
        )
        examples = build_scene_edge_examples(
            graph, patches, scene.ground_truth, scene_path.name,
            EdgeLabelConfig(),
        )
        examples_by_id[scene_path.name] = examples
    for split_name, identifiers in split_manifest.items():
        selected = [examples_by_id[value] for value in identifiers]
        dataset = aggregate_scene_examples(selected, split_name)
        output = tmp_path / split_name
        totals = {
            "object_object_negatives": sum(
                item.object_object_negative_count for item in selected
            ),
            "object_background_negatives": sum(
                item.object_background_negative_count for item in selected
            ),
            "background_background_ignored": sum(
                item.background_background_ignored_count for item in selected
            ),
            "ambiguous_ignored": sum(
                item.ambiguous_ignored_count for item in selected
            ),
        }
        save_edge_training_dataset(
            dataset, output,
            {"split_manifest": dict(split_manifest), "diagnostics": totals},
        )
        assert np.isfinite(dataset.features).all()
        positives = int(np.count_nonzero(dataset.targets == 1))
        negatives = int(np.count_nonzero(dataset.targets == 0))
        diagnostics[split_name] = {
            "scenes": len(identifiers), "positive": positives,
            "negative": negatives,
            "positive_negative_ratio": positives / negatives if negatives else None,
            **totals,
        }
    assert tree_digest(root) == before
    print(f"real dataset Phase 6 diagnostics: {diagnostics}")
