"""Smoke tests for the generated public interface surface."""

from pickcell_interfaces import action, msg, srv


def test_recommended_interface_types_are_importable() -> None:
    """All roadmap contract types must be generated and importable."""
    message_names = (
        "ObjectReference",
        "RuntimeMode",
        "GraspCandidate",
        "GraspCandidateArray",
        "MotionPlanMetadata",
        "SafetyState",
        "ModuleStatus",
        "SystemStatus",
        "TaskState",
    )
    action_names = (
        "DetectObjects",
        "PlanGrasps",
        "PlanMotion",
        "ExecuteTrajectory",
        "ExecuteTask",
    )
    service_names = (
        "InjectDetections",
        "GetCapabilities",
        "GetModuleStatus",
        "ValidateTrajectory",
        "ClearScene",
        "GetSystemProfile",
    )
    assert all(hasattr(msg, name) for name in message_names)
    assert all(hasattr(action, name) for name in action_names)
    assert all(hasattr(srv, name) for name in service_names)
