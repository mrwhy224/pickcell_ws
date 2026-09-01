"""Dataset contracts for isolated point-cloud instance segmentation."""

from .dataset import DatasetValidationError
from .dataset import load_scene
from .dataset import parse_camera_json
from .dataset import parse_capture_camera_json
from .dataset import parse_isaac_camera_json
from .dataset import validate_scene
from .models import CameraModel
from .models import CompactCloud
from .models import OrganizedCloud
from .models import OrganizedGeometry
from .models import NormalEstimationConfig
from .models import PatchSegmentationConfig
from .models import PatchGraph
from .models import PatchGraphConfig
from .models import PatchSet
from .models import EdgeLabelConfig
from .models import EdgeTrainingDataset
from .models import PatchGroundTruth
from .models import SceneEdgeExamples
from .models import SceneSplitConfig
from .models import AffinityMetrics
from .models import EdgeAffinity
from .models import PatchAffinityConfig
from .models import SceneInput
from .models import SceneValidationReport
from .models import RGBDProcessingResult
from .models import ValidationIssue
from .projection import compact_labels_to_organized
from .projection import compact_valid_points
from .projection import ProjectionError
from .projection import rgbd_to_organized_cloud
from .normals import compute_depth_edges
from .normals import compute_normal_variation
from .normals import compute_organized_geometry
from .normals import estimate_organized_normals
from .normals import NormalEstimationError
from .patches import PatchSegmentationError
from .patches import segment_surface_patches
from .adjacency import build_patch_graph
from .adjacency import PATCH_GRAPH_FEATURE_NAMES
from .adjacency import PatchGraphError
from .edge_dataset import aggregate_scene_examples
from .edge_dataset import assign_patch_ground_truth
from .edge_dataset import build_scene_edge_examples
from .edge_dataset import EdgeDatasetError
from .edge_dataset import EdgeDatasetSerializationError
from .edge_dataset import load_edge_training_dataset
from .edge_dataset import save_edge_training_dataset
from .edge_dataset import split_scene_ids
from .affinity import AFFINITY_REJECTION_CODE_NAMES
from .affinity import AFFINITY_REJECTION_PRECEDENCE
from .affinity import DeterministicPatchAffinityPredictor
from .affinity import evaluate_patch_affinity
from .affinity import PatchAffinityError
from .affinity import PatchAffinityPredictor
from .affinity import predict_patch_affinity
from .affinity import REJECTION_BOUNDARY_DEPTH_DIFFERENCE
from .affinity import REJECTION_BOUNDARY_POINT_DISTANCE
from .affinity import REJECTION_CENTROID_DEPTH_DIFFERENCE
from .affinity import REJECTION_DEPTH_EDGE_EVIDENCE
from .affinity import REJECTION_INSUFFICIENT_BOUNDARY_SUPPORT
from .affinity import REJECTION_INSUFFICIENT_NORMAL_EVIDENCE
from .affinity import REJECTION_NONE
from .affinity import REJECTION_NORMAL_ANGLE
from .affinity import REJECTION_RGB_DISTANCE
from .pipeline import process_rgbd_frame
from .pipeline import process_scene_directory
from .partition import connected_instance_labels
from .selection import describe_segments
from .selection import SegmentCandidate
from .selection import UpperRightSegmentSelector
from .selection import UpperRightSelectorConfig

__all__ = [
    "CameraModel",
    "AffinityMetrics",
    "AFFINITY_REJECTION_CODE_NAMES",
    "AFFINITY_REJECTION_PRECEDENCE",
    "CompactCloud",
    "DatasetValidationError",
    "EdgeDatasetError",
    "EdgeDatasetSerializationError",
    "EdgeLabelConfig",
    "EdgeAffinity",
    "EdgeTrainingDataset",
    "OrganizedCloud",
    "OrganizedGeometry",
    "NormalEstimationConfig",
    "NormalEstimationError",
    "PatchSegmentationConfig",
    "PatchGraph",
    "PatchGraphConfig",
    "PatchGraphError",
    "PatchAffinityConfig",
    "PatchAffinityError",
    "PatchAffinityPredictor",
    "PATCH_GRAPH_FEATURE_NAMES",
    "PatchSegmentationError",
    "PatchSet",
    "PatchGroundTruth",
    "ProjectionError",
    "DeterministicPatchAffinityPredictor",
    "SceneInput",
    "SceneEdgeExamples",
    "SceneSplitConfig",
    "SceneValidationReport",
    "RGBDProcessingResult",
    "ValidationIssue",
    "compact_labels_to_organized",
    "aggregate_scene_examples",
    "assign_patch_ground_truth",
    "compact_valid_points",
    "build_patch_graph",
    "build_scene_edge_examples",
    "compute_depth_edges",
    "compute_normal_variation",
    "compute_organized_geometry",
    "estimate_organized_normals",
    "evaluate_patch_affinity",
    "load_scene",
    "load_edge_training_dataset",
    "parse_camera_json",
    "parse_capture_camera_json",
    "parse_isaac_camera_json",
    "rgbd_to_organized_cloud",
    "predict_patch_affinity",
    "process_rgbd_frame",
    "process_scene_directory",
    "connected_instance_labels",
    "describe_segments",
    "SegmentCandidate",
    "UpperRightSegmentSelector",
    "UpperRightSelectorConfig",
    "REJECTION_BOUNDARY_DEPTH_DIFFERENCE",
    "REJECTION_BOUNDARY_POINT_DISTANCE",
    "REJECTION_CENTROID_DEPTH_DIFFERENCE",
    "REJECTION_DEPTH_EDGE_EVIDENCE",
    "REJECTION_INSUFFICIENT_BOUNDARY_SUPPORT",
    "REJECTION_INSUFFICIENT_NORMAL_EVIDENCE",
    "REJECTION_NONE",
    "REJECTION_NORMAL_ANGLE",
    "REJECTION_RGB_DISTANCE",
    "save_edge_training_dataset",
    "segment_surface_patches",
    "split_scene_ids",
    "validate_scene",
]
