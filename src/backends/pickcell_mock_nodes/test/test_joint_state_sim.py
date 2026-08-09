"""Tests for applying selected configurations in simulation."""

from sensor_msgs.msg import JointState

from pickcell_mock_nodes.joint_state_sim import positions_in_order


def test_positions_are_normalized_to_configured_joint_order() -> None:
    """Selected configurations may arrive in any name order."""
    message = JointState()
    message.name = ["joint_2", "joint_1"]
    message.position = [0.2, 0.1]

    assert positions_in_order(
        message, ("joint_1", "joint_2")
    ) == (0.1, 0.2)


def test_incomplete_configuration_is_rejected() -> None:
    """A partial state must not overwrite the simulated robot."""
    message = JointState()
    message.name = ["joint_1"]
    message.position = [0.1]

    assert positions_in_order(message, ("joint_1", "joint_2")) is None
