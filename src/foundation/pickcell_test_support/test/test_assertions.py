"""Tests for shared contract assertions."""

import pytest

from pickcell_interfaces.msg import ErrorCode
from pickcell_test_support import assert_error_code, assert_pose_stamped
from pickcell_test_support import make_pose_stamped


def test_valid_pose_passes() -> None:
    """A canonical, normalized fixture should satisfy the contract."""
    assert_pose_stamped(make_pose_stamped(), {"cell"})


def test_empty_frame_is_rejected() -> None:
    """Public poses without a frame must be rejected."""
    with pytest.raises(AssertionError):
        assert_pose_stamped(make_pose_stamped(frame_id=""))


def test_failure_needs_message() -> None:
    """Machine errors also require human-readable diagnostic context."""
    with pytest.raises(AssertionError):
        assert_error_code(ErrorCode(code=ErrorCode.TIMEOUT))


def test_known_error_with_message_passes() -> None:
    """A documented failure with context should satisfy the contract."""
    error = ErrorCode(code=ErrorCode.TIMEOUT, message="input timed out")
    assert_error_code(error, ErrorCode.TIMEOUT)
