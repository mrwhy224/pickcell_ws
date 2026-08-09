"""MoveIt service adapter for inverse-kinematics enumeration."""

from __future__ import annotations

from collections.abc import Iterable
from copy import deepcopy
import math
from typing import Protocol

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK

from .solver import JointConfiguration


class GetPositionIKClient(Protocol):
    """Minimal synchronous client contract used by the MoveIt adapter."""

    def call(
        self, request: GetPositionIK.Request,
    ) -> GetPositionIK.Response:
        """Call MoveIt's compute_ik service."""
        ...


class MoveItIKBackend:
    """Query MoveIt from multiple seeds to discover distinct IK branches."""

    def __init__(
        self,
        client: GetPositionIKClient,
        joint_names: Iterable[str],
        seed_configurations: Iterable[JointConfiguration],
        *,
        group_name: str,
        end_effector_link: str,
        timeout_seconds: float = 0.05,
        avoid_collisions: bool = True,
    ) -> None:
        """Configure the MoveIt group, TCP link, seeds, and request policy."""
        self._client = client
        self._joint_names = tuple(joint_names)
        self._seeds = tuple(seed_configurations)
        self._group_name = group_name
        self._end_effector_link = end_effector_link
        self._avoid_collisions = avoid_collisions
        if not self._joint_names:
            raise ValueError("MoveIt IK requires at least one joint")
        if not self._seeds:
            raise ValueError("MoveIt IK requires at least one seed")
        if any(
            len(seed.positions) != len(self._joint_names)
            for seed in self._seeds
        ):
            raise ValueError("MoveIt IK seed has an unexpected joint count")
        if not group_name or not end_effector_link:
            raise ValueError("MoveIt group and end-effector link are required")
        if not math.isfinite(timeout_seconds) or timeout_seconds <= 0.0:
            raise ValueError("MoveIt IK timeout must be finite and positive")
        nanoseconds = round(timeout_seconds * 1_000_000_000)
        self._timeout = Duration(
            sec=nanoseconds // 1_000_000_000,
            nanosec=nanoseconds % 1_000_000_000,
        )

    def sample(
        self, target: PoseStamped,
    ) -> Iterable[Iterable[float]]:
        """Yield successful MoveIt solutions found from configured seeds."""
        for seed in self._seeds:
            response = self._client.call(self._request(target, seed))
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            positions_by_name = dict(zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            ))
            if not all(
                name in positions_by_name for name in self._joint_names
            ):
                continue
            yield tuple(positions_by_name[name] for name in self._joint_names)

    def _request(
        self,
        target: PoseStamped,
        seed: JointConfiguration,
    ) -> GetPositionIK.Request:
        request = GetPositionIK.Request()
        request.ik_request.group_name = self._group_name
        request.ik_request.ik_link_name = self._end_effector_link
        request.ik_request.pose_stamped = deepcopy(target)
        request.ik_request.avoid_collisions = self._avoid_collisions
        request.ik_request.timeout = self._timeout
        request.ik_request.robot_state.joint_state.name = list(
            self._joint_names
        )
        request.ik_request.robot_state.joint_state.position = list(
            seed.positions
        )
        return request
