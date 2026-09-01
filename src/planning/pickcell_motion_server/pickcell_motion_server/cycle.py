"""Industrial-style pick/place cycle topology and taught safe waypoints."""

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
    """Safe points and offsets for the pallet-to-rear-box operation."""

    camera_clear_home: CartesianPoseABC
    transfer_waypoint: CartesianPoseABC
    box_approach: CartesianPoseABC
    box_drop: CartesianPoseABC
    pick_approach_offset_m: float = 0.20
    pick_retreat_offset_m: float = 0.30

    def __post_init__(self) -> None:
        """Validate clearance offsets and rear-side waypoint placement."""
        for value, name in (
            (self.pick_approach_offset_m, "pick_approach_offset_m"),
            (self.pick_retreat_offset_m, "pick_retreat_offset_m"),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.camera_clear_home.x >= 0.0:
            raise ValueError("camera_clear_home must stay behind the robot")
        if self.transfer_waypoint.x >= 0.0:
            raise ValueError("transfer_waypoint must stay behind the robot")
        if self.box_approach.z <= self.box_drop.z:
            raise ValueError("box_approach must be above box_drop")
        if not (
            math.isclose(self.camera_clear_home.x, self.box_approach.x)
            and math.isclose(self.camera_clear_home.y, self.box_approach.y)
        ):
            raise ValueError(
                "camera_clear_home must be directly above the box approach"
            )


class PickPlaceCyclePlanner:
    """Build a dynamic pick followed by a repeatable taught transfer cycle."""

    def __init__(self, config: PickPlaceCycleConfig) -> None:
        self.config = config

    def build(self, selected_bag: CartesianPoseABC) -> tuple[CycleStep, ...]:
        """Create the ordered cycle without prescribing intermediate paths."""
        approach = selected_bag.translated(
            dz=self.config.pick_approach_offset_m
        )
        retreat = selected_bag.translated(
            dz=self.config.pick_retreat_offset_m
        )
        return (
            CycleStep(
                "camera_clear_home", MotionType.FIXED,
                self.config.camera_clear_home, True,
            ),
            CycleStep("plan_to_pick_approach", MotionType.PLANNED, approach, False),
            CycleStep("descend_to_bag", MotionType.LINEAR, selected_bag, False),
            CycleStep("close_gripper", MotionType.GRIP, None, False),
            CycleStep("lift_bag", MotionType.LINEAR, retreat, False),
            CycleStep(
                "plan_to_transfer_waypoint", MotionType.PLANNED,
                self.config.transfer_waypoint, False,
            ),
            CycleStep(
                "fixed_transfer_to_box", MotionType.FIXED,
                self.config.box_approach, True,
            ),
            CycleStep(
                "fixed_descend_to_box", MotionType.FIXED,
                self.config.box_drop, True,
            ),
            CycleStep("open_gripper", MotionType.RELEASE, None, True),
            CycleStep(
                "fixed_retreat_from_box", MotionType.FIXED,
                self.config.box_approach, True,
            ),
            CycleStep(
                "fixed_return_via_transfer", MotionType.FIXED,
                self.config.transfer_waypoint, True,
            ),
            CycleStep(
                "fixed_return_to_box_home", MotionType.FIXED,
                self.config.camera_clear_home, True,
            ),
        )
