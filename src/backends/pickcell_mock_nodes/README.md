# Mock point publisher

This node publishes the configured point cloud, final point, and target point.
It has no separate configuration or launch file.

All values come from:

```text
pickcell_config/config/application.yaml
```

Run it through the application launch:

```bash
ros2 launch pickcell_bringup system.launch.py
```
