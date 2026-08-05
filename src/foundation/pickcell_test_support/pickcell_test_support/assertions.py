"""Assertions shared by subsystem conformance tests."""

import math
from collections.abc import Iterable

from geometry_msgs.msg import PoseStamped
from pickcell_interfaces.msg import ErrorCode, ModuleStatus, RuntimeMode
from std_msgs.msg import Header


_KNOWN_ERROR_CODES = {
    ErrorCode.SUCCESS,
    ErrorCode.INVALID_REQUEST,
    ErrorCode.NO_INPUT_DATA,
    ErrorCode.NO_OBJECTS_FOUND,
    ErrorCode.NO_GRASP_FOUND,
    ErrorCode.PLANNING_FAILED,
    ErrorCode.SCENE_CHANGED,
    ErrorCode.SAFETY_REJECTED,
    ErrorCode.CONTROLLER_REJECTED,
    ErrorCode.CANCELED,
    ErrorCode.TIMEOUT,
    ErrorCode.INTERNAL_ERROR,
    ErrorCode.UNSUPPORTED,
    ErrorCode.NOT_READY,
}

_KNOWN_RUNTIME_MODES = {
    RuntimeMode.MOCK,
    RuntimeMode.SIM,
    RuntimeMode.REAL,
}


def assert_valid_header(header: Header, allowed_frames: Iterable[str] = ()) -> None:
    """Assert that a public message has a frame and a nonnegative stamp."""
    assert header.frame_id, "frame_id must not be empty"
    assert header.stamp.sec >= 0, "timestamp seconds must be nonnegative"
    assert 0 <= header.stamp.nanosec < 1_000_000_000
    allowed = set(allowed_frames)
    if allowed:
        assert header.frame_id in allowed, (
            f"unexpected frame '{header.frame_id}'; expected one of {allowed}"
        )


def assert_pose_stamped(
    pose: PoseStamped,
    allowed_frames: Iterable[str] = (),
) -> None:
    """Assert frame metadata and a normalized, finite quaternion."""
    assert_valid_header(pose.header, allowed_frames)
    values = (
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    )
    assert all(math.isfinite(value) for value in values)
    orientation = pose.pose.orientation
    norm = math.sqrt(
        orientation.x**2
        + orientation.y**2
        + orientation.z**2
        + orientation.w**2
    )
    assert math.isclose(norm, 1.0, abs_tol=1e-6), (
        f"orientation quaternion must be normalized; norm={norm}"
    )


def assert_error_code(error: ErrorCode, expected: int | None = None) -> None:
    """Assert that a result uses a documented machine-readable error code."""
    assert error.code in _KNOWN_ERROR_CODES, f"unknown error code {error.code}"
    if expected is not None:
        assert error.code == expected
    if error.code != ErrorCode.SUCCESS:
        assert error.message, "failure results must include a diagnostic message"


def assert_runtime_mode(mode: RuntimeMode) -> None:
    """Assert that a running module declares an explicit environment mode."""
    assert mode.value in _KNOWN_RUNTIME_MODES, (
        "runtime mode must be MOCK, SIM, or REAL"
    )


def assert_module_status(status: ModuleStatus) -> None:
    """Assert the common status fields required from every runtime module."""
    assert status.module_name, "module_name must not be empty"
    assert status.implementation, "implementation must not be empty"
    assert status.version, "version must not be empty"
    assert status.configuration_hash, "configuration_hash must not be empty"
    assert_runtime_mode(status.mode)
