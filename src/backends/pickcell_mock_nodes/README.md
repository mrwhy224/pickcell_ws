# PickCell mock point cloud

`point_cloud_mock` publishes a deterministic fixture for developing path planning before perception or hardware exists.

Topics are relative to the node namespace:

- `point_cloud` — unorganized XYZ `sensor_msgs/PointCloud2`.
- `final_point` — the current/final `geometry_msgs/PointStamped`.
- `target_point` — the requested goal `geometry_msgs/PointStamped`.
- `status` — active `pickcell_interfaces/ModuleStatus`.

All three data messages use the same timestamp and frame. The default frame is `cell`, the final point is `(0.45, 0.0, 0.10)`, and the target is `(0.80, 0.0, 0.20)`.

Run it with:

```bash
ros2 launch pickcell_mock_nodes point_cloud_mock.launch.py
```

Override the experiment without editing code:

```bash
ros2 launch pickcell_mock_nodes point_cloud_mock.launch.py \
  frame_id:=cell \
  final_point:="[0.45, 0.0, 0.10]" \
  target_point:="[0.75, 0.20, 0.25]"
```

The point cloud is intentionally simple and repeatable. It is a planning fixture, not a perception result.
