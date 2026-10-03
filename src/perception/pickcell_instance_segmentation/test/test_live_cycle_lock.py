"""Regression tests for source-correlated perception cycle ownership."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time
import numpy as np
from pickcell_interfaces.msg import ActiveBag, CycleResult
from std_msgs.msg import Header

from pickcell_instance_segmentation.live_node import LiveInstanceSegmentation


class Logger:
    """Minimal logger accepted by callbacks under test."""

    def warning(self, *_args, **_kwargs) -> None:
        """Accept warning calls without a ROS node."""

    def error(self, *_args, **_kwargs) -> None:
        """Accept error calls without a ROS node."""


def node_with_lock(active_cycle=None, terminal_ids=()):
    """Create a callback-only node with one frame-identified selection."""
    node = LiveInstanceSegmentation.__new__(LiveInstanceSegmentation)
    node._pending_pick = np.array([1.0, 2.0, 3.0])
    node._picked_points = []
    node._cycle_active = True
    node._active_cycle_id = active_cycle
    node._terminal_cycle_ids = set(terminal_ids)
    header = Header(
        frame_id="camera", stamp=Time(sec=12, nanosec=34)
    )
    node._locked_snapshot = SimpleNamespace(
        selected_cloud=SimpleNamespace(header=header)
    )
    node.get_logger = lambda: Logger()
    return node


def acceptance(cycle_id=42, sec=12, nanosec=34):
    """Build an acceptance that names a source RGB-D frame."""
    message = ActiveBag()
    message.cycle_id = cycle_id
    message.source_frame = "camera"
    message.source_stamp = Time(sec=sec, nanosec=nanosec)
    return message


def result(cycle_id=42, *, success=True, delivered=True, held=False):
    """Build one explicit terminal outcome."""
    message = CycleResult()
    message.cycle_id = cycle_id
    message.success = success
    message.delivered = delivered
    message.payload_held = held
    return message


def test_only_matching_snapshot_acceptance_binds_cycle():
    """An acceptance for another frame cannot claim the pending selection."""
    node = node_with_lock()

    node._cycle_accepted(acceptance(sec=11))
    assert node._active_cycle_id is None
    node._cycle_accepted(acceptance())
    assert node._active_cycle_id == 42


def test_unaccepted_newer_terminal_cannot_release_lock():
    """Numerical recency is not evidence that a result owns this selection."""
    node = node_with_lock()

    node._cycle_result(result(99))

    assert node._cycle_active
    assert node._pending_pick is not None
    assert node._picked_points == []


def test_success_excludes_exact_pick_and_allows_next_cycle():
    """A matching delivered result finalizes the accepted snapshot."""
    node = node_with_lock(active_cycle=42)

    node._cycle_result(result())

    assert not node._cycle_active
    assert node._active_cycle_id is None
    assert node._pending_pick is None
    assert len(node._picked_points) == 1


def test_recovered_pregrasp_failure_unlocks_without_excluding_bag():
    """A known-empty recovery may retry the same bag on a later cycle."""
    node = node_with_lock(active_cycle=42)

    node._cycle_result(result(success=False, delivered=False, held=False))

    assert not node._cycle_active
    assert node._picked_points == []


def test_unknown_held_payload_keeps_selection_locked():
    """A fault with possible payload cannot silently select another bag."""
    node = node_with_lock(active_cycle=42)

    node._cycle_result(result(success=False, delivered=False, held=True))

    assert node._cycle_active
    assert node._active_cycle_id == 42
