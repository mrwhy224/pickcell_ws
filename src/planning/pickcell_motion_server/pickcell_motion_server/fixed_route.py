"""Explicit, repeatable Cartesian transfer-route definitions and checks."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .cycle import CartesianPoseABC


@dataclass(frozen=True)
class FixedTransferRoute:
    """Two anchors joined by one canonical, primarily base-yaw joint sweep."""

    pick_anchor: CartesianPoseABC
    drop_anchor: CartesianPoseABC
    base_yaw_joint: str
    base_yaw_direction: str
    minimum_base_yaw_sweep_rad: float
    maximum_other_joint_excursion_rad: float
    axial_wrist_joint: str
    maximum_axial_wrist_excursion_rad: float
    maximum_tcp_height_m: float

    def __post_init__(self) -> None:
        """Reject an underspecified or internally inconsistent route."""
        if not self.base_yaw_joint:
            raise ValueError("fixed transfer route needs the base-yaw joint name")
        if self.base_yaw_direction not in ("positive", "negative"):
            raise ValueError("base-yaw direction must be positive or negative")
        for value, name in (
            (self.minimum_base_yaw_sweep_rad, "minimum base-yaw sweep"),
            (self.maximum_other_joint_excursion_rad, "other-joint excursion"),
            (self.maximum_axial_wrist_excursion_rad, "axial-wrist excursion"),
            (self.maximum_tcp_height_m, "TCP height envelope"),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        all_poses = (self.pick_anchor, self.drop_anchor)
        if any(not math.isfinite(value) for pose in all_poses for value in (
            pose.x, pose.y, pose.z, pose.a_deg, pose.b_deg, pose.c_deg,
        )):
            raise ValueError("fixed transfer route contains non-finite values")
        if self.pick_anchor == self.drop_anchor:
            raise ValueError("fixed transfer anchors must be distinct")

    @property
    def loaded_poses(self) -> tuple[CartesianPoseABC, ...]:
        """Return the two persistent anchors in loaded order."""
        return (self.pick_anchor, self.drop_anchor)

    @property
    def unloaded_poses(self) -> tuple[CartesianPoseABC, ...]:
        """Return the exact reverse anchor order."""
        return tuple(reversed(self.loaded_poses))


def build_joint_sweep_trajectory(
    start: Iterable[float],
    goal: Iterable[float],
    joint_names: Iterable[str],
    *,
    maximum_velocity: Iterable[float],
    maximum_acceleration: Iterable[float],
    maximum_jerk: Iterable[float],
    sample_count: int = 81,
) -> JointTrajectory:
    """Build a smooth minimum-jerk interpolation between anchor branches."""
    start_values = tuple(float(value) for value in start)
    goal_values = tuple(float(value) for value in goal)
    names = tuple(joint_names)
    velocity = tuple(maximum_velocity)
    acceleration = tuple(maximum_acceleration)
    jerk = tuple(maximum_jerk)
    if sample_count < 5:
        raise ValueError("joint sweep needs at least five samples")
    if not names or not all(len(values) == len(names) for values in (
        start_values, goal_values, velocity, acceleration, jerk,
    )):
        raise ValueError("joint sweep inputs must align with joint_names")
    deltas = tuple(right - left for left, right in zip(start_values, goal_values))
    duration = max(
        0.5,
        *(
            max(
                abs(delta) * 1.875 / max_v,
                math.sqrt(abs(delta) * 5.8 / max_a),
                (abs(delta) * 60.0 / max_j) ** (1.0 / 3.0),
            )
            for delta, max_v, max_a, max_j in zip(
                deltas, velocity, acceleration, jerk
            )
        ),
    ) * 1.25
    trajectory = JointTrajectory(joint_names=list(names))
    for index in range(sample_count):
        u = index / (sample_count - 1)
        blend = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
        blend_d = (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / duration
        blend_dd = (60.0 * u - 180.0 * u**2 + 120.0 * u**3) / duration**2
        point = JointTrajectoryPoint()
        point.positions = [left + delta * blend for left, delta in zip(
            start_values, deltas
        )]
        point.velocities = [delta * blend_d for delta in deltas]
        point.accelerations = [delta * blend_dd for delta in deltas]
        nanoseconds = round(u * duration * 1_000_000_000)
        point.time_from_start.sec = nanoseconds // 1_000_000_000
        point.time_from_start.nanosec = nanoseconds % 1_000_000_000
        trajectory.points.append(point)
    return trajectory


def validate_lateral_sweep(
    trajectory: JointTrajectory,
    route: FixedTransferRoute,
) -> None:
    """Require the canonical geometry to be a monotonic base-yaw sweep."""
    try:
        base_index = tuple(trajectory.joint_names).index(route.base_yaw_joint)
    except ValueError as error:
        raise ValueError("configured base-yaw joint is absent from route") from error
    if len(trajectory.points) < 2:
        raise ValueError("lateral sweep needs at least two samples")
    base_values = [point.positions[base_index] for point in trajectory.points]
    signed_steps = [right - left for left, right in zip(base_values, base_values[1:])]
    direction = 1.0 if route.base_yaw_direction == "positive" else -1.0
    if any(direction * step < -1e-9 for step in signed_steps):
        raise ValueError("base-yaw sweep reverses configured direction")
    if direction * (base_values[-1] - base_values[0]) < route.minimum_base_yaw_sweep_rad:
        raise ValueError("base-yaw sweep is too small for opposite-side transfer")
    start = trajectory.points[0].positions
    goal = trajectory.points[-1].positions
    for index, (left, right) in enumerate(zip(start, goal)):
        if index == base_index:
            continue
        limit = route.maximum_other_joint_excursion_rad
        if trajectory.joint_names[index] == route.axial_wrist_joint:
            limit = route.maximum_axial_wrist_excursion_rad
        if abs(right - left) > limit:
            raise ValueError("non-base joint excursion exceeds lateral envelope")


def trajectory_duration_seconds(trajectory: JointTrajectory) -> float:
    """Return a trajectory duration after checking its time ordering."""
    if not trajectory.points:
        raise ValueError("trajectory has no points")
    times = [
        point.time_from_start.sec + point.time_from_start.nanosec / 1e9
        for point in trajectory.points
    ]
    if not all(math.isfinite(value) and value >= 0.0 for value in times):
        raise ValueError("trajectory contains an invalid timestamp")
    if any(right <= left for left, right in zip(times, times[1:])):
        raise ValueError("trajectory timestamps must strictly increase")
    return times[-1]


def validate_joint_trajectory(
    trajectory: JointTrajectory,
    joint_names: Iterable[str],
    lower_limits: Iterable[float],
    upper_limits: Iterable[float],
    *,
    maximum_velocity: Iterable[float],
    maximum_acceleration: Iterable[float],
    maximum_jerk: Iterable[float],
) -> None:
    """Validate the full sampled route before it can be cached or executed."""
    names = tuple(joint_names)
    lower = tuple(lower_limits)
    upper = tuple(upper_limits)
    velocity = tuple(maximum_velocity)
    acceleration = tuple(maximum_acceleration)
    jerk = tuple(maximum_jerk)
    if not names or not all(len(values) == len(names) for values in (
        lower, upper, velocity, acceleration, jerk,
    )):
        raise ValueError("route limits must align with joint_names")
    if tuple(trajectory.joint_names) != names:
        raise ValueError("route trajectory joint order does not match configuration")
    duration = trajectory_duration_seconds(trajectory)
    if duration <= 0.0:
        raise ValueError("route trajectory duration must be positive")
    previous_positions = None
    previous_velocity = None
    previous_acceleration = None
    previous_time = None
    for point in trajectory.points:
        positions = tuple(point.positions)
        if len(positions) != len(names) or not all(math.isfinite(value) for value in positions):
            raise ValueError("route point has incomplete or non-finite positions")
        if any(value < lo or value > hi for value, lo, hi in zip(positions, lower, upper)):
            raise ValueError("route point exceeds a joint limit")
        current_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
        if previous_positions is not None:
            interval = current_time - previous_time
            derived_velocity = tuple(
                (right - left) / interval
                for left, right in zip(previous_positions, positions)
            )
            if any(abs(value) > limit for value, limit in zip(derived_velocity, velocity)):
                raise ValueError("route exceeds a configured joint velocity limit")
            if previous_velocity is not None:
                derived_acceleration = tuple(
                    (right - left) / interval
                    for left, right in zip(previous_velocity, derived_velocity)
                )
                if any(
                    abs(value) > limit
                    for value, limit in zip(
                        derived_acceleration, acceleration
                    )
                ):
                    raise ValueError("route exceeds a configured joint acceleration limit")
                if previous_acceleration is not None:
                    derived_jerk = tuple(
                        (right - left) / interval
                        for left, right in zip(previous_acceleration, derived_acceleration)
                    )
                    if any(abs(value) > limit for value, limit in zip(derived_jerk, jerk)):
                        raise ValueError("route exceeds a configured joint jerk limit")
                previous_acceleration = derived_acceleration
            previous_velocity = derived_velocity
        previous_positions = positions
        previous_time = current_time


def same_joint_geometry(
    expected: JointTrajectory,
    actual: JointTrajectory,
    *,
    tolerance_rad: float,
) -> bool:
    """
    Return whether two sampled fixed routes have the same joint geometry.

    Timing is intentionally excluded: a controller may retime a trajectory,
    but that must not turn the transfer into a different spatial route.
    """
    if not math.isfinite(tolerance_rad) or tolerance_rad <= 0.0:
        raise ValueError("fixed-route geometry tolerance must be positive")
    if tuple(expected.joint_names) != tuple(actual.joint_names):
        return False
    if not expected.points or not actual.points:
        return False
    sample_count = max(len(expected.points), len(actual.points), 25)

    def resample(trajectory: JointTrajectory) -> tuple[tuple[float, ...], ...]:
        points = trajectory.points
        if len(points) == 1:
            return (tuple(points[0].positions),) * sample_count
        result = []
        for sample in range(sample_count):
            progress = sample / (sample_count - 1)
            scaled = progress * (len(points) - 1)
            left_index = min(int(scaled), len(points) - 2)
            fraction = scaled - left_index
            left = points[left_index].positions
            right = points[left_index + 1].positions
            result.append(tuple(
                a + (b - a) * fraction for a, b in zip(left, right)
            ))
        return tuple(result)

    return all(
        len(reference) == len(candidate)
        and all(abs(a - b) <= tolerance_rad for a, b in zip(reference, candidate))
        for reference, candidate in zip(resample(expected), resample(actual))
    )
