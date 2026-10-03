"""Guarded phase transitions for one pick, transfer, and release cycle."""

from dataclasses import dataclass
from enum import Enum


class CyclePhase(Enum):
    """States in which a command is permitted to change the cell."""

    IDLE = "idle"
    PICK_APPROACH = "pick_approach"
    AT_PICK = "at_pick"
    WAITING_FOR_GRASP = "waiting_for_grasp"
    GRASP_CONFIRMED = "grasp_confirmed"
    RETREATED = "retreated"
    AT_TRANSFER_START = "at_transfer_start"
    LOADED_TRANSFER = "loaded_transfer"
    AT_DROP = "at_drop"
    RELEASED = "released"
    RETURN_TRANSFER = "return_transfer"
    FAILED = "failed"


@dataclass
class CycleState:
    """Track phase and grasp confidence without accepting implicit transitions."""

    phase: CyclePhase = CyclePhase.IDLE
    grasp_confirmed: bool = False

    def arrive(self, phase: CyclePhase) -> None:
        """Record a completed motion only when it follows the legal phase."""
        legal_predecessors = {
            CyclePhase.PICK_APPROACH: {CyclePhase.IDLE},
            CyclePhase.AT_PICK: {CyclePhase.PICK_APPROACH},
            CyclePhase.RETREATED: {CyclePhase.GRASP_CONFIRMED},
            CyclePhase.AT_TRANSFER_START: {CyclePhase.RETREATED},
            CyclePhase.LOADED_TRANSFER: {CyclePhase.AT_TRANSFER_START},
            CyclePhase.AT_DROP: {CyclePhase.LOADED_TRANSFER},
            CyclePhase.IDLE: {CyclePhase.RETURN_TRANSFER},
        }
        if self.phase not in legal_predecessors.get(phase, set()):
            raise ValueError(f"cannot arrive at {phase.value} from {self.phase.value}")
        self.phase = phase

    def request_grasp(self) -> None:
        """Permit closing only at the completed pick pose."""
        if self.phase is not CyclePhase.AT_PICK:
            raise ValueError("grasp command is allowed only at the pick pose")
        self.phase = CyclePhase.WAITING_FOR_GRASP
        self.grasp_confirmed = False

    def confirm_grasp(self, confirmed: bool) -> None:
        """Accept one explicit gripper result while that result is awaited."""
        if self.phase is not CyclePhase.WAITING_FOR_GRASP:
            raise ValueError("unexpected grasp result")
        if not confirmed:
            self.fail()
            return
        self.grasp_confirmed = True
        self.phase = CyclePhase.GRASP_CONFIRMED

    def permit_loaded_transfer(self) -> None:
        """Require both grasp feedback and the common transfer-start anchor."""
        if (
            self.phase is not CyclePhase.AT_TRANSFER_START
            or not self.grasp_confirmed
        ):
            raise ValueError("loaded transfer requires confirmed grasp at A_PICK")
        self.phase = CyclePhase.LOADED_TRANSFER

    def permit_release(self) -> None:
        """Permit opening only at the completed, validated drop pose."""
        if self.phase is not CyclePhase.AT_DROP or not self.grasp_confirmed:
            raise ValueError("release is allowed only at the validated drop pose")
        self.grasp_confirmed = False
        self.phase = CyclePhase.RELEASED

    def begin_return(self) -> None:
        """Begin the empty reverse sweep only after confirmed release."""
        if self.phase is not CyclePhase.RELEASED or self.grasp_confirmed:
            raise ValueError("empty return requires a confirmed release")
        self.phase = CyclePhase.RETURN_TRANSFER

    def fail(self) -> None:
        """Enter a terminal safe-stop state without opening the gripper."""
        self.phase = CyclePhase.FAILED
