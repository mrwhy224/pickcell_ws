# PickCell MoveIt configuration

This package defines the `manipulator` planning group from `robot_base` to
`gripper_tcp`, KDL inverse kinematics, KUKA velocity limits, and an OMPL
planning pipeline.

Joint position limits and geometry continue to come from the maintained KUKA
URDF. This package only supplies MoveIt-specific semantic and planning data.
