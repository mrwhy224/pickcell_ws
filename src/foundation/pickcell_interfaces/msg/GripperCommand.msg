# Cycle-scoped mock gripper command with the TCP pose that must be verified.
std_msgs/Header header
int64 cycle_id
uint64 command_id
bool close
geometry_msgs/Pose expected_tcp_pose
