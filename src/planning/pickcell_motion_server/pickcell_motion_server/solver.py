"""Inverse-kinematics solution enumeration for motion planning."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
import math
from typing import Protocol

from geometry_msgs.msg import PoseStamped


@dataclass(frozen=True)
class JointLimit:
    """Allowed position interval for one revolute joint, in radians."""

    lower: float
    upper: float

    def __post_init__(self) -> None:
        """Validate the interval after dataclass initialization."""
        if not math.isfinite(self.lower) or not math.isfinite(self.upper):
            raise ValueError("joint limits must be finite")
        if self.lower > self.upper:
            raise ValueError("joint limit lower bound exceeds upper bound")

    def contains(self, position: float, tolerance: float = 0.0) -> bool:
        """Return whether a joint position is inside this interval."""
        return self.lower - tolerance <= position <= self.upper + tolerance


@dataclass(frozen=True)
class JointConfiguration:
    """One complete robot joint configuration, in radians."""

    positions: tuple[float, ...]

    @classmethod
    def from_iterable(cls, positions: Iterable[float]) -> JointConfiguration:
        """Create an immutable configuration from backend output."""
        return cls(tuple(float(position) for position in positions))


class InverseKinematicsBackend(Protocol):
    """Robot-specific backend capable of sampling IK candidates."""

    def sample(self, target: PoseStamped) -> Iterable[Iterable[float]]:
        """Yield joint solutions discovered for the requested TCP pose."""
        ...


class InverseKinematicsSolver:
    """Return unique, joint-limit-valid candidates for a TCP pose."""

    def __init__(
        self,
        joint_names: Iterable[str],
        joint_limits: Mapping[str, JointLimit],
        backend: InverseKinematicsBackend,
        *,
        duplicate_tolerance: float = 1.0e-6,
        limit_tolerance: float = 1.0e-9,
    ) -> None:
        """Configure joint order, limits, backend, and numeric tolerances."""
        self.joint_names = tuple(joint_names)
        if not self.joint_names:
            raise ValueError("at least one joint is required")
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint names must be unique")
        missing_limits = set(self.joint_names) - set(joint_limits)
        if missing_limits:
            names = ", ".join(sorted(missing_limits))
            raise ValueError(f"missing limits for joints: {names}")
        if (
            not math.isfinite(duplicate_tolerance)
            or duplicate_tolerance < 0.0
        ):
            raise ValueError(
                "duplicate tolerance must be finite and nonnegative"
            )
        if not math.isfinite(limit_tolerance) or limit_tolerance < 0.0:
            raise ValueError("limit tolerance must be finite and nonnegative")

        self._limits = tuple(joint_limits[name] for name in self.joint_names)
        self._backend = backend
        self._duplicate_tolerance = duplicate_tolerance
        self._limit_tolerance = limit_tolerance

    def solve(self, target: PoseStamped) -> tuple[JointConfiguration, ...]:
        """Find unique finite IK candidates that satisfy joint limits."""
        self._validate_target(target)
        solutions: list[JointConfiguration] = []
        for raw_positions in self._backend.sample(target):
            candidate = JointConfiguration.from_iterable(raw_positions)
            if not self._is_valid(candidate):
                continue
            if any(self._equivalent(candidate, known) for known in solutions):
                continue
            solutions.append(candidate)
        return tuple(solutions)

    def as_joint_map(
        self, configuration: JointConfiguration,
    ) -> dict[str, float]:
        """Associate a complete solution with its configured joint names."""
        if len(configuration.positions) != len(self.joint_names):
            raise ValueError("configuration has an unexpected joint count")
        return dict(zip(self.joint_names, configuration.positions))

    def _is_valid(self, configuration: JointConfiguration) -> bool:
        if len(configuration.positions) != len(self.joint_names):
            return False
        return all(
            limit.contains(position, self._limit_tolerance)
            for position, limit in zip(configuration.positions, self._limits)
        )

    def _equivalent(
        self,
        left: JointConfiguration,
        right: JointConfiguration,
    ) -> bool:
        return all(
            abs(left_position - right_position) <= self._duplicate_tolerance
            for left_position, right_position in zip(
                left.positions, right.positions
            )
        )

    @staticmethod
    def _validate_target(target: PoseStamped) -> None:
        values = (
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        )
        if not target.header.frame_id:
            raise ValueError("target pose must have a frame_id")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("target pose must contain only finite values")
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
        if quaternion_norm <= 1.0e-12:
            raise ValueError("target orientation quaternion must not be zero")
