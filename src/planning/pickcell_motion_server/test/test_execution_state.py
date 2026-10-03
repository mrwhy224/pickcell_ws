"""Tests for grasp/release and loaded-transfer phase guards."""

import pytest

from pickcell_motion_server.execution_state import CyclePhase
from pickcell_motion_server.execution_state import CycleState


def reached_pick() -> CycleState:
    """Return a state that has completed approach and descent."""
    result = CycleState()
    result.arrive(CyclePhase.PICK_APPROACH)
    result.arrive(CyclePhase.AT_PICK)
    return result


def reached_transfer_start() -> CycleState:
    """Return a state with a confirmed grasp at the common P1 anchor."""
    result = reached_pick()
    result.request_grasp()
    result.confirm_grasp(True)
    result.arrive(CyclePhase.RETREATED)
    result.arrive(CyclePhase.AT_TRANSFER_START)
    return result


def test_unconfirmed_grasp_cannot_start_loaded_transfer():
    """A close command alone never authorizes the pallet-to-box corridor."""
    state = reached_pick()
    state.request_grasp()

    with pytest.raises(ValueError, match="confirmed grasp"):
        state.permit_loaded_transfer()


def test_failed_grasp_is_terminal_and_never_opens_gripper():
    """A negative result transitions directly to a safe stop."""
    state = reached_pick()
    state.request_grasp()
    state.confirm_grasp(False)

    assert state.phase is CyclePhase.FAILED
    assert state.grasp_confirmed is False
    with pytest.raises(ValueError, match="release"):
        state.permit_release()


def test_loaded_transfer_requires_the_transfer_start_anchor():
    """A confirmed grasp away from A_PICK is insufficient for transfer."""
    state = reached_pick()
    state.request_grasp()
    state.confirm_grasp(True)

    with pytest.raises(ValueError, match="A_PICK"):
        state.permit_loaded_transfer()


def test_release_is_permitted_only_after_validated_drop_arrival():
    """Opening while high over the pallet or corridor is rejected."""
    state = reached_transfer_start()
    state.permit_loaded_transfer()
    state.arrive(CyclePhase.AT_DROP)
    state.permit_release()

    assert state.phase is CyclePhase.RELEASED
    assert state.grasp_confirmed is False


def test_four_serial_cycles_all_release_before_the_next_pick() -> None:
    """Model the required state order for four independent selected bags."""
    for _ in range(4):
        state = reached_transfer_start()
        state.permit_loaded_transfer()
        state.arrive(CyclePhase.AT_DROP)
        state.permit_release()
        state.begin_return()
        state.arrive(CyclePhase.IDLE)

        assert state.phase is CyclePhase.IDLE
        assert not state.grasp_confirmed
