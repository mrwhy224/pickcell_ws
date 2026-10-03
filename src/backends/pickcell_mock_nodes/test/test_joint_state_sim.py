"""Tests for the acknowledged demo trajectory interpreter."""

from pickcell_mock_nodes.joint_state_sim import JointStateSimulationNode
from trajectory_msgs.msg import JointTrajectory


def test_trajectory_command_id_requires_executor_header() -> None:
    """A stale generic completion cannot acknowledge a newer trajectory."""
    trajectory = JointTrajectory()
    trajectory.header.frame_id = "pickcell_trajectory/42"

    assert JointStateSimulationNode._command_id(trajectory) == 42
    trajectory.header.frame_id = "arbitrary_frame"
    assert JointStateSimulationNode._command_id(trajectory) is None
