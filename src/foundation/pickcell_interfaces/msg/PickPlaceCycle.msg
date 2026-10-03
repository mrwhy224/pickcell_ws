# Complete cycle contract. All poses use header.frame_id.
std_msgs/Header header
int64 cycle_id
builtin_interfaces/Time source_stamp
string source_frame
geometry_msgs/Pose pick_anchor
geometry_msgs/Pose hover_pose
geometry_msgs/Pose grasp_pose
geometry_msgs/Pose lift_pose
geometry_msgs/Pose drop_anchor
