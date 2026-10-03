"""Small helpers for correlating one cycle across independent ROS topics."""

from dataclasses import dataclass
import secrets
from typing import Hashable


@dataclass
class CycleGate:
    """Permit one active coordinator cycle and reject stale completion events."""

    active_id: int | None = None
    counter: int = 0
    session_prefix: int = 0
    active_source: Hashable | None = None
    last_finished_source: Hashable | None = None

    def __post_init__(self) -> None:
        """Give each process a non-time-based namespace for cycle IDs."""
        if self.session_prefix == 0:
            self.session_prefix = secrets.randbits(30) + 1

    def begin(self, source: Hashable | None = None) -> int | None:
        """Lock a new source and return a session-scoped positive cycle ID."""
        if self.active_id is not None or source == self.last_finished_source:
            return None
        self.counter += 1
        self.active_id = (self.session_prefix << 31) | self.counter
        self.active_source = source
        return self.active_id

    def accept_result(self, result_id: int, *, recoverable: bool) -> bool:
        """Accept a matching result and clear only completed/recovered work."""
        if self.active_id is None or result_id != self.active_id:
            return False
        if recoverable:
            self.last_finished_source = self.active_source
            self.active_id = None
            self.active_source = None
        return True
