"""Convert accepted patch affinities into connected candidate instances."""

import numpy as np

from .models import EdgeAffinity
from .models import PatchGraph
from .models import PatchSet


def connected_instance_labels(
    patches: PatchSet,
    graph: PatchGraph,
    affinity: EdgeAffinity,
    *,
    minimum_pixels: int = 1,
) -> np.ndarray:
    """Return an HxW int32 map of affinity-connected candidate regions."""
    if minimum_pixels < 1:
        raise ValueError("minimum_pixels must be positive")
    edge_count = graph.edges.shape[0]
    if affinity.same_object.shape != (edge_count,):
        raise ValueError("affinity and graph edge counts do not match")

    parents = np.arange(patches.count, dtype=np.int32)

    def find(item: int) -> int:
        while parents[item] != item:
            parents[item] = parents[parents[item]]
            item = int(parents[item])
        return item

    for first, second in graph.edges[affinity.same_object]:
        root_a = find(int(first))
        root_b = find(int(second))
        if root_a != root_b:
            parents[max(root_a, root_b)] = min(root_a, root_b)

    roots = np.array([find(item) for item in range(patches.count)], np.int32)
    component_sizes = np.zeros(patches.count, dtype=np.int64)
    np.add.at(component_sizes, roots, patches.pixel_counts)
    kept_roots = np.flatnonzero(component_sizes >= minimum_pixels)
    root_to_instance = np.zeros(patches.count, dtype=np.int32)
    root_to_instance[kept_roots] = np.arange(
        1, kept_roots.size + 1, dtype=np.int32
    )
    output = np.zeros(patches.labels.shape, dtype=np.int32)
    valid = patches.labels >= 0
    output[valid] = root_to_instance[roots[patches.labels[valid]]]
    return output
