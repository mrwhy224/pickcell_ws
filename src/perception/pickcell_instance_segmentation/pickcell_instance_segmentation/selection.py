"""Extract geometric segments and choose the next overhead pick target."""

from dataclasses import dataclass
import math

import numpy as np

from .models import OrganizedCloud


@dataclass(frozen=True)
class SegmentCandidate:
    """Compact measurements for one labeled point-cloud segment."""

    instance_id: int
    pixel_count: int
    centroid_xyz: tuple[float, float, float]
    centroid_uv: tuple[float, float]
    top_depth_m: float
    bounding_box_uv: tuple[int, int, int, int]


@dataclass(frozen=True)
class UpperRightSelectorConfig:
    """Selection settings for a camera looking down on a bag stack."""

    same_height_tolerance_m: float = 0.02
    top_depth_percentile: float = 10.0

    def __post_init__(self) -> None:
        """Reject thresholds that would make target ordering ambiguous."""
        if not math.isfinite(self.same_height_tolerance_m):
            raise ValueError("same_height_tolerance_m must be finite")
        if self.same_height_tolerance_m < 0.0:
            raise ValueError("same_height_tolerance_m must be nonnegative")
        if not math.isfinite(self.top_depth_percentile):
            raise ValueError("top_depth_percentile must be finite")
        if not 0.0 <= self.top_depth_percentile <= 100.0:
            raise ValueError("top_depth_percentile must be in [0, 100]")


def describe_segments(
    labels: np.ndarray,
    cloud: OrganizedCloud,
    *,
    top_depth_percentile: float = 10.0,
) -> tuple[SegmentCandidate, ...]:
    """Measure every positive label using its aligned organized cloud points."""
    if labels.shape != cloud.valid.shape:
        raise ValueError("labels and organized cloud dimensions do not match")
    if not np.issubdtype(labels.dtype, np.integer):
        raise TypeError("labels must contain integers")
    if not math.isfinite(top_depth_percentile):
        raise ValueError("top_depth_percentile must be finite")
    if not 0.0 <= top_depth_percentile <= 100.0:
        raise ValueError("top_depth_percentile must be in [0, 100]")

    candidates = []
    valid_labels = labels[cloud.valid]
    for instance_id in np.unique(valid_labels):
        identifier = int(instance_id)
        if identifier <= 0:
            continue
        mask = (labels == identifier) & cloud.valid
        pixels_vu = np.argwhere(mask)
        xyz = cloud.xyz[mask]
        if xyz.size == 0:
            continue
        minimum_v, minimum_u = pixels_vu.min(axis=0)
        maximum_v, maximum_u = pixels_vu.max(axis=0)
        centroid = np.mean(xyz, axis=0, dtype=np.float64)
        candidates.append(SegmentCandidate(
            instance_id=identifier,
            pixel_count=int(xyz.shape[0]),
            centroid_xyz=tuple(float(value) for value in centroid),
            centroid_uv=(
                float(np.mean(pixels_vu[:, 0])),
                float(np.mean(pixels_vu[:, 1])),
            ),
            top_depth_m=float(np.percentile(
                xyz[:, 2], top_depth_percentile
            )),
            bounding_box_uv=(
                int(minimum_u), int(minimum_v),
                int(maximum_u), int(maximum_v),
            ),
        ))
    return tuple(candidates)


class UpperRightSegmentSelector:
    """Choose the uppermost segment, preferring image-right at equal height."""

    def __init__(
        self,
        config: UpperRightSelectorConfig | None = None,
    ) -> None:
        self.config = config or UpperRightSelectorConfig()

    def choose(
        self,
        candidates: tuple[SegmentCandidate, ...],
    ) -> SegmentCandidate | None:
        """Return a deterministic next target or ``None`` for no candidates."""
        if not candidates:
            return None
        closest_depth = min(item.top_depth_m for item in candidates)
        same_height = [
            item for item in candidates
            if item.top_depth_m
            <= closest_depth + self.config.same_height_tolerance_m
        ]
        return min(
            same_height,
            key=lambda item: (-item.centroid_uv[1], item.instance_id),
        )

    def choose_from_labels(
        self,
        labels: np.ndarray,
        cloud: OrganizedCloud,
    ) -> SegmentCandidate | None:
        """Measure a label image and choose its next pick candidate."""
        candidates = describe_segments(
            labels,
            cloud,
            top_depth_percentile=self.config.top_depth_percentile,
        )
        return self.choose(candidates)
