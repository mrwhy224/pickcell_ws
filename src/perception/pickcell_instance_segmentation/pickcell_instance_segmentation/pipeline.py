"""Public in-memory and saved-scene entry points for RGB-D processing."""

from pathlib import Path
from typing import Mapping

import numpy as np

from .adjacency import build_patch_graph
from .affinity import DeterministicPatchAffinityPredictor
from .affinity import PatchAffinityPredictor
from .dataset import load_scene
from .models import CameraModel
from .models import NormalEstimationConfig
from .models import PatchAffinityConfig
from .models import PatchGraphConfig
from .models import PatchSegmentationConfig
from .models import RGBDProcessingResult
from .models import SceneInput
from .normals import compute_organized_geometry
from .patches import segment_surface_patches
from .projection import rgbd_to_organized_cloud


def process_rgbd_frame(
    color: np.ndarray,
    depth: np.ndarray,
    camera: CameraModel,
    *,
    input_is_rectified: bool = False,
    normal_config: NormalEstimationConfig | None = None,
    patch_config: PatchSegmentationConfig | None = None,
    graph_config: PatchGraphConfig | None = None,
    affinity_config: PatchAffinityConfig | None = None,
    predictor: PatchAffinityPredictor | None = None,
    metadata: Mapping[str, object] | None = None,
) -> RGBDProcessingResult:
    """
    Process one aligned RGB-D frame through the patch-affinity pipeline.

    ``camera.depth_scale`` specifies stored depth units per metre. Ground truth
    is deliberately absent because this is the entry point for live inference.
    The result contains all currently implemented intermediate products; final
    bag instances require a downstream graph-partitioning/classification stage.
    """
    if not isinstance(camera, CameraModel):
        raise TypeError("camera must be a CameraModel")
    if predictor is not None and affinity_config is not None:
        raise ValueError("pass predictor or affinity_config, not both")

    scene = SceneInput(
        color=color,
        depth=depth,
        camera=camera,
        ground_truth=None,
        instance_mapping={},
        semantics={},
        metadata={} if metadata is None else metadata,
    )
    cloud = rgbd_to_organized_cloud(
        scene, input_is_rectified=input_is_rectified
    )
    geometry = compute_organized_geometry(
        cloud, normal_config or NormalEstimationConfig()
    )
    patches = segment_surface_patches(
        cloud, geometry, patch_config or PatchSegmentationConfig()
    )
    graph = build_patch_graph(
        cloud, geometry, patches, graph_config or PatchGraphConfig()
    )
    selected_predictor = predictor or DeterministicPatchAffinityPredictor(
        affinity_config or PatchAffinityConfig()
    )
    affinity = selected_predictor.predict(graph)
    return RGBDProcessingResult(
        scene=scene,
        cloud=cloud,
        geometry=geometry,
        patches=patches,
        graph=graph,
        affinity=affinity,
    )


def process_scene_directory(
    scene_directory: Path,
    **kwargs,
) -> RGBDProcessingResult:
    """Load a saved scene and process it with the live-frame pipeline."""
    scene = load_scene(Path(scene_directory), require_ground_truth=False)
    return process_rgbd_frame(
        scene.color,
        scene.depth,
        scene.camera,
        metadata=scene.metadata,
        **kwargs,
    )
