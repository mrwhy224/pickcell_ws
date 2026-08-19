"""Class-agnostic supervised examples for deterministic patch-graph edges."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
from types import MappingProxyType
from typing import Mapping, Sequence

import numpy as np

from .adjacency import PATCH_GRAPH_FEATURE_NAMES
from .models import EdgeLabelConfig
from .models import EdgeTrainingDataset
from .models import PatchGraph
from .models import PatchGroundTruth
from .models import PatchSet
from .models import SceneEdgeExamples
from .models import SceneSplitConfig


EDGE_DATASET_SCHEMA_NAME = "pickcell_edge_training_dataset"
EDGE_DATASET_SCHEMA_VERSION = 1
_ARRAY_FILE = "arrays.npz"
_METADATA_FILE = "metadata.json"


class EdgeDatasetError(ValueError):
    """Indicate an invalid edge-labelling or aggregation contract."""


class EdgeDatasetSerializationError(EdgeDatasetError):
    """Indicate invalid or incompatible serialized edge data."""


def _validate_patch_labels(patches: PatchSet) -> None:
    """Validate the Phase 4 label and count contract needed for labelling."""
    labels = patches.labels
    if labels.ndim != 2 or labels.dtype != np.int32:
        raise EdgeDatasetError("patch labels must be a two-dimensional int32 array")
    if isinstance(patches.count, bool) or patches.count < 0:
        raise EdgeDatasetError("patch count must be a nonnegative integer")
    if patches.pixel_counts.shape != (patches.count,):
        raise EdgeDatasetError("patch pixel_counts shape is invalid")
    present = np.unique(labels[labels >= 0])
    expected = np.arange(patches.count, dtype=np.int32)
    if not np.array_equal(present, expected):
        raise EdgeDatasetError("patch IDs must be contiguous")
    if np.any(labels < -1) or np.any(labels >= patches.count):
        raise EdgeDatasetError("patch labels contain an invalid patch ID")
    counts = np.bincount(labels[labels >= 0], minlength=patches.count)
    if not np.array_equal(counts, patches.pixel_counts):
        raise EdgeDatasetError("patch pixel counts disagree with labels")


def _validate_ground_truth(
    patches: PatchSet, ground_truth: np.ndarray
) -> np.ndarray:
    """Validate and safely represent integer per-pixel instance IDs."""
    if not isinstance(ground_truth, np.ndarray):
        raise TypeError("ground_truth must be a NumPy array")
    if ground_truth.shape != patches.labels.shape:
        raise EdgeDatasetError("ground-truth shape must match patch labels")
    if not np.issubdtype(ground_truth.dtype, np.integer) or np.issubdtype(
        ground_truth.dtype, np.bool_
    ):
        raise EdgeDatasetError("ground truth must contain integer instance IDs")
    if np.issubdtype(ground_truth.dtype, np.unsignedinteger):
        if ground_truth.size and int(ground_truth.max()) > np.iinfo(np.int64).max:
            raise EdgeDatasetError("ground-truth IDs exceed the safe int64 range")
    elif ground_truth.size and int(ground_truth.min()) < 0:
        raise EdgeDatasetError("ground-truth instance IDs must be nonnegative")
    return ground_truth.astype(np.int64, copy=False)


def assign_patch_ground_truth(
    patches: PatchSet,
    ground_truth: np.ndarray,
    config: EdgeLabelConfig,
) -> PatchGroundTruth:
    """Assign deterministic majority instance IDs without changing patches."""
    if not isinstance(config, EdgeLabelConfig):
        raise TypeError("config must be an EdgeLabelConfig")
    _validate_patch_labels(patches)
    labels = _validate_ground_truth(patches, ground_truth)
    majority = np.zeros(patches.count, dtype=np.int64)
    purity = np.zeros(patches.count, dtype=np.float32)
    counts = np.asarray(patches.pixel_counts, dtype=np.int64)
    for patch_id in range(patches.count):
        values, frequencies = np.unique(
            labels[patches.labels == patch_id], return_counts=True
        )
        winner = int(np.argmax(frequencies))
        majority[patch_id] = values[winner]
        purity[patch_id] = frequencies[winner] / counts[patch_id]
    eligible = (
        (counts >= config.minimum_patch_pixels)
        & (purity >= config.minimum_patch_purity)
    )
    return PatchGroundTruth(majority, purity, eligible, counts)


def _validate_graph(graph: PatchGraph, patch_count: int) -> None:
    """Validate graph shapes, schema, values, and referenced patch IDs."""
    if tuple(graph.feature_names) != PATCH_GRAPH_FEATURE_NAMES:
        raise EdgeDatasetError("graph feature order does not match Phase 5")
    edge_count = graph.edges.shape[0]
    if graph.edges.shape != (edge_count, 2) or graph.edges.dtype != np.int32:
        raise EdgeDatasetError("graph edges must have shape (E, 2) and int32")
    expected_features = (edge_count, len(PATCH_GRAPH_FEATURE_NAMES))
    if graph.edge_features.shape != expected_features:
        raise EdgeDatasetError("graph edge feature shape is invalid")
    if graph.edge_features.dtype != np.float32:
        raise EdgeDatasetError("graph edge features must use float32")
    if not np.isfinite(graph.edge_features).all():
        raise EdgeDatasetError("graph edge features must be finite")
    if edge_count and (
        np.any(graph.edges < 0) or np.any(graph.edges >= patch_count)
    ):
        raise EdgeDatasetError("graph contains an invalid patch ID")


def build_scene_edge_examples(
    graph: PatchGraph,
    patches: PatchSet,
    ground_truth: np.ndarray,
    scene_id: str,
    config: EdgeLabelConfig,
) -> SceneEdgeExamples:
    """Select labelled edges in their original deterministic graph order."""
    if not isinstance(scene_id, str) or not scene_id.strip():
        raise EdgeDatasetError("scene_id must be a nonempty string")
    patch_truth = assign_patch_ground_truth(patches, ground_truth, config)
    _validate_graph(graph, patches.count)
    edges = graph.edges
    if edges.shape[0]:
        first = edges[:, 0]
        second = edges[:, 1]
        eligible = patch_truth.eligible[first] & patch_truth.eligible[second]
        first_id = patch_truth.majority_instance_ids[first]
        second_id = patch_truth.majority_instance_ids[second]
    else:
        eligible = np.empty(0, dtype=bool)
        first_id = np.empty(0, dtype=np.int64)
        second_id = np.empty(0, dtype=np.int64)
    background = config.background_instance_id
    first_background = first_id == background
    second_background = second_id == background
    both_background = eligible & first_background & second_background
    positive = eligible & (first_id == second_id) & ~first_background
    negative = eligible & ~both_background & ~positive
    selected = positive | negative
    object_object_negative = negative & ~first_background & ~second_background
    object_background_negative = negative & (first_background != second_background)
    ambiguous = ~eligible
    selected_indices = np.flatnonzero(selected).astype(np.int64)
    targets = positive[selected].astype(np.uint8)
    return SceneEdgeExamples(
        graph.edge_features[selected].astype(np.float32, copy=True),
        targets,
        selected_indices,
        edges[selected].astype(np.int32, copy=True),
        PATCH_GRAPH_FEATURE_NAMES,
        scene_id,
        int(np.count_nonzero(positive)),
        int(np.count_nonzero(negative)),
        int(edges.shape[0] - np.count_nonzero(selected)),
        int(np.count_nonzero(object_object_negative)),
        int(np.count_nonzero(object_background_negative)),
        int(np.count_nonzero(both_background)),
        int(np.count_nonzero(ambiguous)),
    )


def split_scene_ids(
    scene_ids: Sequence[str], config: SceneSplitConfig
) -> Mapping[str, tuple[str, ...]]:
    """
    Split sorted scene IDs by seeded permutation and largest remainders.

    Whole scenes are assigned once. With few scenes, zero-fraction or
    lower-allocation splits remain empty rather than duplicating a scene.
    """
    if not isinstance(config, SceneSplitConfig):
        raise TypeError("config must be a SceneSplitConfig")
    identifiers = list(scene_ids)
    if any(not isinstance(value, str) or not value.strip() for value in identifiers):
        raise EdgeDatasetError("scene IDs must be nonempty strings")
    if len(set(identifiers)) != len(identifiers):
        raise EdgeDatasetError("scene IDs must be unique")
    ordered = sorted(identifiers)
    generator = np.random.default_rng(config.seed)
    permuted = [ordered[index] for index in generator.permutation(len(ordered))]
    fractions = np.array(
        [config.train_fraction, config.validation_fraction, config.test_fraction],
        dtype=np.float64,
    )
    raw_counts = fractions * len(permuted)
    counts = np.floor(raw_counts).astype(np.int64)
    remaining = len(permuted) - int(counts.sum())
    priority = sorted(
        range(3), key=lambda index: (-(raw_counts[index] - counts[index]), index)
    )
    for index in priority[:remaining]:
        counts[index] += 1
    names = ("train", "validation", "test")
    result = {}
    start = 0
    for name, count in zip(names, counts):
        end = start + int(count)
        result[name] = tuple(permuted[start:end])
        start = end
    return MappingProxyType(result)


def _validate_examples(example: SceneEdgeExamples) -> None:
    """Validate a per-scene model before aggregation."""
    if tuple(example.feature_names) != PATCH_GRAPH_FEATURE_NAMES:
        raise EdgeDatasetError("incompatible scene feature schema")
    count = example.targets.shape[0]
    if example.features.shape != (count, len(PATCH_GRAPH_FEATURE_NAMES)):
        raise EdgeDatasetError("scene feature shape is invalid")
    if example.features.dtype != np.float32 or not np.isfinite(
        example.features
    ).all():
        raise EdgeDatasetError("scene features must be finite float32")
    if example.targets.dtype != np.uint8 or np.any(example.targets > 1):
        raise EdgeDatasetError("scene targets must be uint8 zero or one")
    if example.graph_edge_indices.shape != (count,):
        raise EdgeDatasetError("scene graph edge index shape is invalid")
    if example.patch_pairs.shape != (count, 2) or example.patch_pairs.dtype != np.int32:
        raise EdgeDatasetError("scene patch pair contract is invalid")
    if not isinstance(example.scene_id, str) or not example.scene_id.strip():
        raise EdgeDatasetError("scene_id must be a nonempty string")


def aggregate_scene_examples(
    examples: Sequence[SceneEdgeExamples], split_name: str
) -> EdgeTrainingDataset:
    """Concatenate whole-scene examples without changing edge order."""
    if not isinstance(split_name, str) or not split_name.strip():
        raise EdgeDatasetError("split_name must be a nonempty string")
    items = list(examples)
    for item in items:
        _validate_examples(item)
    scene_ids = tuple(item.scene_id for item in items)
    if len(set(scene_ids)) != len(scene_ids):
        raise EdgeDatasetError("aggregated scene IDs must be unique")
    feature_count = len(PATCH_GRAPH_FEATURE_NAMES)
    if items:
        features = np.concatenate([item.features for item in items], axis=0)
        targets = np.concatenate([item.targets for item in items])
        graph_indices = np.concatenate(
            [item.graph_edge_indices for item in items]
        ).astype(np.int64, copy=False)
        patch_pairs = np.concatenate([item.patch_pairs for item in items], axis=0)
        scene_indices = np.concatenate([
            np.full(item.targets.shape[0], index, dtype=np.int32)
            for index, item in enumerate(items)
        ])
    else:
        features = np.empty((0, feature_count), dtype=np.float32)
        targets = np.empty(0, dtype=np.uint8)
        scene_indices = np.empty(0, dtype=np.int32)
        graph_indices = np.empty(0, dtype=np.int64)
        patch_pairs = np.empty((0, 2), dtype=np.int32)
    return EdgeTrainingDataset(
        features,
        targets,
        scene_indices,
        graph_indices,
        patch_pairs,
        PATCH_GRAPH_FEATURE_NAMES,
        scene_ids,
        split_name,
    )


def _validate_training_dataset(dataset: EdgeTrainingDataset) -> None:
    """Validate all persisted dataset array and schema relationships."""
    if tuple(dataset.feature_names) != PATCH_GRAPH_FEATURE_NAMES:
        raise EdgeDatasetSerializationError("feature schema is incompatible")
    count = dataset.targets.shape[0]
    if dataset.features.shape != (count, len(PATCH_GRAPH_FEATURE_NAMES)):
        raise EdgeDatasetSerializationError("feature array shape is invalid")
    if dataset.features.dtype != np.float32 or not np.isfinite(
        dataset.features
    ).all():
        raise EdgeDatasetSerializationError("features must be finite float32")
    contracts = (
        (dataset.targets, (count,), np.uint8, "targets"),
        (dataset.scene_indices, (count,), np.int32, "scene_indices"),
        (dataset.graph_edge_indices, (count,), np.int64, "graph_edge_indices"),
        (dataset.patch_pairs, (count, 2), np.int32, "patch_pairs"),
    )
    for array, shape, dtype, name in contracts:
        if array.shape != shape or array.dtype != dtype:
            raise EdgeDatasetSerializationError(f"{name} contract is invalid")
    if np.any(dataset.targets > 1):
        raise EdgeDatasetSerializationError("targets must contain only zero or one")
    if count and (
        np.any(dataset.scene_indices < 0)
        or np.any(dataset.scene_indices >= len(dataset.scene_ids))
    ):
        raise EdgeDatasetSerializationError("scene index is out of bounds")
    if len(set(dataset.scene_ids)) != len(dataset.scene_ids):
        raise EdgeDatasetSerializationError("scene IDs must be unique")
    if not isinstance(dataset.split_name, str) or not dataset.split_name.strip():
        raise EdgeDatasetSerializationError("split name is invalid")


def save_edge_training_dataset(
    dataset: EdgeTrainingDataset,
    output_directory: Path,
    metadata: Mapping[str, object],
    *,
    overwrite: bool = False,
) -> None:
    """Atomically save numeric arrays and JSON schema to an explicit directory."""
    _validate_training_dataset(dataset)
    if not isinstance(metadata, Mapping):
        raise TypeError("metadata must be a mapping")
    directory = Path(output_directory)
    if directory.exists() and not directory.is_dir():
        raise FileExistsError(f"output path is not a directory: {directory}")
    if directory.exists() and any(directory.iterdir()) and not overwrite:
        raise FileExistsError(f"output directory is not empty: {directory}")
    document = {
        "schema_name": EDGE_DATASET_SCHEMA_NAME,
        "schema_version": EDGE_DATASET_SCHEMA_VERSION,
        "split_name": dataset.split_name,
        "scene_ids": list(dataset.scene_ids),
        "feature_names": list(dataset.feature_names),
        "sample_count": int(dataset.targets.size),
        "positive_count": int(np.count_nonzero(dataset.targets == 1)),
        "negative_count": int(np.count_nonzero(dataset.targets == 0)),
        "metadata": dict(metadata),
    }
    try:
        encoded = json.dumps(document, indent=2, sort_keys=True) + "\n"
    except (TypeError, ValueError) as error:
        raise EdgeDatasetSerializationError("metadata must be JSON serializable") from error
    directory.mkdir(parents=True, exist_ok=True)
    temporary_paths = []
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", dir=directory, prefix=".arrays-", suffix=".tmp", delete=False
        ) as stream:
            array_temp = Path(stream.name)
            temporary_paths.append(array_temp)
            np.savez(
                stream,
                features=dataset.features,
                targets=dataset.targets,
                scene_indices=dataset.scene_indices,
                graph_edge_indices=dataset.graph_edge_indices,
                patch_pairs=dataset.patch_pairs,
            )
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=directory,
            prefix=".metadata-", suffix=".tmp", delete=False
        ) as stream:
            metadata_temp = Path(stream.name)
            temporary_paths.append(metadata_temp)
            stream.write(encoded)
        array_temp.replace(directory / _ARRAY_FILE)
        temporary_paths.remove(array_temp)
        metadata_temp.replace(directory / _METADATA_FILE)
        temporary_paths.remove(metadata_temp)
    finally:
        for path in temporary_paths:
            path.unlink(missing_ok=True)


def load_edge_training_dataset(output_directory: Path) -> EdgeTrainingDataset:
    """Load and validate a dependency-free Phase 6 dataset artifact."""
    directory = Path(output_directory)
    try:
        document = json.loads((directory / _METADATA_FILE).read_text("utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise EdgeDatasetSerializationError("metadata JSON is missing or malformed") from error
    if document.get("schema_name") != EDGE_DATASET_SCHEMA_NAME:
        raise EdgeDatasetSerializationError("dataset schema name is incompatible")
    if document.get("schema_version") != EDGE_DATASET_SCHEMA_VERSION:
        raise EdgeDatasetSerializationError("dataset schema version is incompatible")
    if tuple(document.get("feature_names", ())) != PATCH_GRAPH_FEATURE_NAMES:
        raise EdgeDatasetSerializationError("serialized feature schema is incompatible")
    scene_ids = document.get("scene_ids")
    if not isinstance(scene_ids, list) or not all(
        isinstance(value, str) for value in scene_ids
    ):
        raise EdgeDatasetSerializationError("serialized scene IDs are invalid")
    try:
        with np.load(directory / _ARRAY_FILE, allow_pickle=False) as archive:
            required = {
                "features", "targets", "scene_indices",
                "graph_edge_indices", "patch_pairs",
            }
            if set(archive.files) != required:
                raise EdgeDatasetSerializationError("serialized array keys are invalid")
            arrays = {name: np.array(archive[name], copy=True) for name in required}
    except (OSError, ValueError) as error:
        raise EdgeDatasetSerializationError("NPZ arrays are missing or malformed") from error
    dataset = EdgeTrainingDataset(
        arrays["features"],
        arrays["targets"],
        arrays["scene_indices"],
        arrays["graph_edge_indices"],
        arrays["patch_pairs"],
        PATCH_GRAPH_FEATURE_NAMES,
        tuple(scene_ids),
        document.get("split_name", ""),
    )
    _validate_training_dataset(dataset)
    if document.get("sample_count") != dataset.targets.size:
        raise EdgeDatasetSerializationError("serialized sample count is inconsistent")
    if document.get("positive_count") != int(np.count_nonzero(dataset.targets == 1)):
        raise EdgeDatasetSerializationError("serialized positive count is inconsistent")
    if document.get("negative_count") != int(np.count_nonzero(dataset.targets == 0)):
        raise EdgeDatasetSerializationError("serialized negative count is inconsistent")
    return dataset
