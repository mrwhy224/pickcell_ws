# Mock point publisher

This node publishes the configured point cloud and target pose. It also
broadcasts that pose in TF as the configured `target_frame_id`, relative to
`frame_id`.
It has no separate configuration or launch file.

`joint_state_sim` publishes the configured initial arm state and subscribes to
`planning/selected_joint_configuration`. A received solution changes only the
simulated `joint_states`; `robot_state_publisher` then derives the robot TF.
The target pose and target TF are never modified by this node.

All values come from:

```text
pickcell_config/config/application.yaml
```

Run it through the application launch:

```bash
ros2 launch pickcell_bringup system.launch.py
```
