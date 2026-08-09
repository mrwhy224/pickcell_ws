"""Unit tests for the MoveIt inverse-kinematics adapter."""

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
import pytest

from pickcell_motion_server.moveit_backend import MoveItIKBackend
from pickcell_motion_server.solver import JointConfiguration


class FakeClient:
    """Record requests and return configured MoveIt responses."""

    def __init__(self, responses):
        self.responses = iter(responses)
        self.requests = []

    def call(self, request):
        """Record one request and return the next response."""
        self.requests.append(request)
        return next(self.responses)


def response(code, names=(), positions=()):
    """Build a compact GetPositionIK response fixture."""
    result = GetPositionIK.Response()
    result.error_code.val = code
    result.solution.joint_state.name = list(names)
    result.solution.joint_state.position = list(positions)
    return result


def test_backend_queries_each_seed_and_returns_successes_in_joint_order():
    """Failed calls are skipped and response joint ordering is normalized."""
    client = FakeClient([
        response(MoveItErrorCodes.NO_IK_SOLUTION),
        response(
            MoveItErrorCodes.SUCCESS,
            ("joint_2", "joint_1"),
            (0.2, 0.1),
        ),
    ])
    backend = MoveItIKBackend(
        client,
        ("joint_1", "joint_2"),
        (JointConfiguration((0.0, 0.0)), JointConfiguration((1.0, 1.0))),
        group_name="manipulator",
        end_effector_link="gripper_tcp",
    )
    target = PoseStamped()
    target.header.frame_id = "cell"
    target.pose.orientation.w = 1.0

    assert tuple(backend.sample(target)) == ((0.1, 0.2),)
    assert len(client.requests) == 2
    request = client.requests[0].ik_request
    assert request.group_name == "manipulator"
    assert request.ik_link_name == "gripper_tcp"
    assert request.avoid_collisions is True
    assert request.robot_state.joint_state.name == ["joint_1", "joint_2"]


@pytest.mark.parametrize("timeout", [float("nan"), float("inf"), 0.0])
def test_backend_rejects_invalid_timeout(timeout):
    """Reject a MoveIt request timeout unless it is finite and positive."""
    with pytest.raises(ValueError, match="finite and positive"):
        MoveItIKBackend(
            FakeClient([]),
            ("joint_1",),
            (JointConfiguration((0.0,)),),
            group_name="manipulator",
            end_effector_link="gripper_tcp",
            timeout_seconds=timeout,
        )
