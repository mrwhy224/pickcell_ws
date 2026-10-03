"""Stable box-reference drop target validation."""

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class DropTarget:
    """Bag-centre target expressed once in a stable, named box frame."""

    center: tuple[float, float, float]
    xy_tolerance_m: float
    z_tolerance_m: float

    @classmethod
    def from_values(
        cls,
        center: Iterable[float],
        xy_tolerance_m: float,
        z_tolerance_m: float,
    ) -> "DropTarget":
        """Construct a finite target without retaining mutable cycle state."""
        center = tuple(float(value) for value in center)
        if len(center) != 3 or not all(math.isfinite(value) for value in center):
            raise ValueError("drop bag centre must contain three finite values")
        if (
            not math.isfinite(xy_tolerance_m)
            or not math.isfinite(z_tolerance_m)
            or xy_tolerance_m <= 0.0
            or z_tolerance_m <= 0.0
        ):
            raise ValueError("drop tolerances must be finite and positive")
        return cls(center, float(xy_tolerance_m), float(z_tolerance_m))

    def accepts(self, actual_cell: Iterable[float]) -> bool:
        """Check one freshly transformed bag centre against the fixed target."""
        actual = tuple(float(value) for value in actual_cell)
        if len(actual) != 3 or not all(math.isfinite(value) for value in actual):
            return False
        return (
            math.hypot(
                actual[0] - self.center[0],
                actual[1] - self.center[1],
            ) <= self.xy_tolerance_m
            and abs(actual[2] - self.center[2]) <= self.z_tolerance_m
        )
