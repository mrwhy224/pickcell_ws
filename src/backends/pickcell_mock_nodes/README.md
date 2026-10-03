# Mock point publisher

This node publishes the configured point cloud and target pose. It also
broadcasts that pose in TF as the configured `target_frame_id`, relative to
`frame_id`.
It has no separate configuration or launch file.

`joint_state_sim` publishes the configured initial arm state and subscribes to
`planning/joint_trajectory`. It interpolates only that demo trajectory topic,
publishes `planning/trajectory_complete_id` with the command ID carried in the
trajectory header at its final sample, and updates
simulated `joint_states`; `robot_state_publisher` then derives robot TF. The
target pose and target TF are never modified by this node.

Its publication timer uses wall time so the configured initial pose remains
available in RViz before an external simulator starts publishing `/clock`.
Joint-state message timestamps still use the system-wide ROS clock.

All values come from:

```text
pickcell_config/config/application.yaml
```

Run it through the application launch:

```bash
ros2 launch pickcell_bringup system.launch.py
```
