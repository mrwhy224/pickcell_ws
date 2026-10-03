"""Regression tests for explicit session-scoped cycle ownership."""

from pickcell_motion_server.cycle_identity import CycleGate


def test_gate_rejects_repeated_pick_and_unknown_result() -> None:
    """Only a matching recovered or completed cycle releases the gate."""
    gate = CycleGate(session_prefix=7)
    first = gate.begin(("camera", 1, 2))

    assert first == (7 << 31) | 1
    assert gate.begin() is None
    assert not gate.accept_result(first + 1, recoverable=True)
    assert gate.active_id == first
    assert gate.accept_result(first, recoverable=False)
    assert gate.active_id == first
    assert gate.accept_result(first, recoverable=True)
    assert gate.active_id is None
    assert gate.begin(("camera", 1, 2)) is None
    assert gate.begin(("camera", 1, 3)) is not None


def test_cycle_ids_do_not_depend_on_ros_time_or_restart_clock() -> None:
    """A process session namespace and counter create unique positive IDs."""
    gate = CycleGate(session_prefix=11)
    completed = []
    for _ in range(10):
        cycle_id = gate.begin(("camera", 2, _))
        completed.append(cycle_id)
        assert gate.accept_result(cycle_id, recoverable=True)

    assert len(set(completed)) == 10
    assert completed == sorted(completed)
