"""Quality-based selection of inverse-kinematics candidates."""

from __future__ import annotations

from collections.abc import Iterable
import math

from .solver import JointConfiguration


class JointDistanceOptimizer:
    """Choose the candidate requiring the least weighted joint displacement."""

    def __init__(self, weights: Iterable[float]) -> None:
        """Store finite, nonnegative per-joint movement weights."""
        self._weights = tuple(float(weight) for weight in weights)
        if not self._weights:
            raise ValueError("at least one optimizer weight is required")
        if any(
            not math.isfinite(weight) or weight < 0.0
            for weight in self._weights
        ):
            raise ValueError(
                "optimizer weights must be finite and nonnegative"
            )
        if not any(self._weights):
            raise ValueError("at least one optimizer weight must be positive")

    def choose(
        self,
        candidates: Iterable[JointConfiguration],
        current: JointConfiguration,
    ) -> JointConfiguration | None:
        """Return the lowest-cost candidate or None when none are available."""
        if len(current.positions) != len(self._weights):
            raise ValueError(
                "current configuration has an unexpected joint count"
            )
        normalized = tuple(candidates)
        if any(
            len(item.positions) != len(self._weights)
            for item in normalized
        ):
            raise ValueError("candidate has an unexpected joint count")
        return min(
            normalized,
            key=lambda item: self.cost(item, current),
            default=None,
        )

    def cost(
        self,
        candidate: JointConfiguration,
        current: JointConfiguration,
    ) -> float:
        """Calculate weighted squared joint displacement."""
        return sum(
            weight * (target - start) ** 2
            for weight, target, start in zip(
                self._weights, candidate.positions, current.positions
            )
        )
