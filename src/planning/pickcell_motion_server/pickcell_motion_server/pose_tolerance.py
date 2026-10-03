"""Pose-comparison helpers used for safe startup-state verification."""

import math


def within_pose_tolerance(
    actual,
    expected,
    *,
    position_tolerance_m: float,
    orientation_tolerance_rad: float,
) -> bool:
    """Return whether two geometry messages are within finite pose limits."""
    values = (
        actual.position.x, actual.position.y, actual.position.z,
        actual.orientation.x, actual.orientation.y, actual.orientation.z,
        actual.orientation.w, expected.position.x, expected.position.y,
        expected.position.z, expected.orientation.x, expected.orientation.y,
        expected.orientation.z, expected.orientation.w, position_tolerance_m,
        orientation_tolerance_rad,
    )
    if (
        not all(math.isfinite(value) for value in values)
        or position_tolerance_m <= 0.0
        or orientation_tolerance_rad <= 0.0
    ):
        return False
    position_error = math.dist(
        (actual.position.x, actual.position.y, actual.position.z),
        (expected.position.x, expected.position.y, expected.position.z),
    )
    dot = abs(
        actual.orientation.x * expected.orientation.x
        + actual.orientation.y * expected.orientation.y
        + actual.orientation.z * expected.orientation.z
        + actual.orientation.w * expected.orientation.w
    )
    orientation_error = 2.0 * math.acos(min(1.0, max(-1.0, dot)))
    return (
        position_error <= position_tolerance_m
        and orientation_error <= orientation_tolerance_rad
    )
