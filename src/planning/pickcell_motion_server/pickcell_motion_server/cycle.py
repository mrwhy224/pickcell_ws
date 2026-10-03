"""Two-anchor pick/place cycle topology with bag-specific local poses."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Sequence


@dataclass(frozen=True)
class CartesianPoseABC:
    """TCP pose in metres and KUKA-style ABC degrees."""

    x: float
    y: float
    z: float
    a_deg: float
    b_deg: float
    c_deg: float

    @classmethod
    def from_sequence(cls, values: Sequence[float]) -> "CartesianPoseABC":
        """Construct one validated XYZABC pose from configuration data."""
        if len(values) != 6:
            raise ValueError("an XYZABC pose must contain six values")
        normalized = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in normalized):
            raise ValueError("XYZABC pose values must be finite")
        return cls(*normalized)

    def translated(self, *, dz: float) -> "CartesianPoseABC":
        """Return the same tool orientation at a vertical offset."""
        return CartesianPoseABC(
            self.x, self.y, self.z + float(dz),
            self.a_deg, self.b_deg, self.c_deg,
        )


class MotionType(Enum):
    """How an industrial controller should realize a cycle step."""

    PLANNED = "planned"
    LINEAR = "linear"
    FIXED = "fixed"
    GRIP = "grip"
    RELEASE = "release"


@dataclass(frozen=True)
class CycleStep:
    """One named motion or tool action in the pick/place cycle."""

    name: str
    motion_type: MotionType
    target: CartesianPoseABC | None
    reusable: bool


@dataclass(frozen=True)
class PickPlaceCycleConfig:
    """The two persistent anchors and local-pick clearance offsets."""

    pick_anchor: CartesianPoseABC
    drop_anchor: CartesianPoseABC
    pick_approach_offset_m: float = 0.20
    pick_retreat_offset_m: float = 0.30

    def __post_init__(self) -> None:
        """Validate clearance offsets and distinct opposite-side anchors."""
        for value, name in (
            (self.pick_approach_offset_m, "pick_approach_offset_m"),
            (self.pick_retreat_offset_m, "pick_retreat_offset_m"),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if math.dist(
            (self.pick_anchor.x, self.pick_anchor.y, self.pick_anchor.z),
            (self.drop_anchor.x, self.drop_anchor.y, self.drop_anchor.z),
        ) < 0.25:
            raise ValueError("pick and drop anchors must be distinct")
        if self.pick_anchor.x * self.drop_anchor.x >= 0.0:
            raise ValueError("pick and drop anchors must be on opposite robot sides")


class PickPlaceCyclePlanner:
    """Build a dynamic pick followed by a repeatable taught transfer cycle."""

    def __init__(self, config: PickPlaceCycleConfig) -> None:
        self.config = config

    def build(self, selected_bag: CartesianPoseABC) -> tuple[CycleStep, ...]:
        """Create the local pick loop and the fixed two-anchor transfer."""
        approach = selected_bag.translated(
            dz=self.config.pick_approach_offset_m
        )
        retreat = selected_bag.translated(
            dz=self.config.pick_retreat_offset_m
        )
        return (
            CycleStep(
                "establish_pick_anchor", MotionType.FIXED,
                self.config.pick_anchor, True,
            ),
            CycleStep("plan_to_pick_approach", MotionType.PLANNED, approach, False),
            CycleStep("descend_to_bag", MotionType.LINEAR, selected_bag, False),
            CycleStep("close_gripper", MotionType.GRIP, None, False),
            CycleStep("lift_bag", MotionType.LINEAR, retreat, False),
            CycleStep(
                "return_to_pick_anchor", MotionType.FIXED,
                self.config.pick_anchor, True,
            ),
            CycleStep(
                "fixed_loaded_to_drop_anchor", MotionType.FIXED,
                self.config.drop_anchor, True,
            ),
            CycleStep("open_gripper", MotionType.RELEASE, None, True),
            CycleStep(
                "fixed_return_to_pick_anchor", MotionType.FIXED,
                self.config.pick_anchor, True,
            ),
        )
