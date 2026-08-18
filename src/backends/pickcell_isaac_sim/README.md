# PickCell Isaac Sim bridge

This package implements the simulator-owned part of the PickCell sensor
contract. `pickcell_bringup/system.launch.py` starts the configured Isaac Sim
installation, imports the expanded URDF, and shuts Isaac down with the launch.

## Automatic startup

The default application profile uses the detected installation at
`/home/mahdi/isaacsim` in headless mode. Override it without editing the shared
configuration when needed:

```bash
export PICKCELL_ISAAC_SIM_ROOT=/path/to/isaacsim
ros2 launch pickcell_bringup system.launch.py
```

Isaac may take roughly 15 to 30 seconds to import the robot and initialize RTX
rendering. The point-cloud topic appears after that initialization completes.

## Scene setup performed by the runner

1. Import the expanded `pickcell.urdf.xacro` into a Z-up Isaac Sim stage. Keep
   the cell origin at `/World/PickCell` and preserve the bag mesh visuals.
2. Create an RTX camera at `/World/PickCell/overhead_depth_camera` using the
   values in `config/overhead_depth_camera.yaml`. The camera is fixed at
   `(1.4, 0.0, 2.0) m` and looks vertically down at the pallet centre.
   Add a neutral dome light so the RGB render is usable in headless mode.
3. Enable `isaacsim.ros2.bridge` and create ROS 2 camera publishers for RGB,
   depth, camera info, and point cloud. Configure the namespace, topics, frame ID,
   resolution, clipping range, field of view, and tick rate from the YAML.
4. Publish `/clock`. Do not publish camera TF from Isaac: the workspace's
   `robot_state_publisher` already owns the fixed
   `overhead_depth_camera_optical_frame`, and a second publisher would create
   conflicting transforms.
5. The automatic profile sets `system.use_sim_time: true` because this process
   owns `/clock`. Set it false if the simulator is disabled and RViz is run
   without another simulation-clock publisher.

The point cloud writer must use distance-to-image-plane depth and camera
intrinsics. It publishes `sensor_msgs/PointCloud2` directly on:

```text
/pickcell/sensors/overhead_depth/points
```

RViz is preconfigured with three independent display switches:

- `Robot - KUKA KR 180 R2900-2`
- `Pallet and bags`
- `Overhead depth point cloud`

Turn off `Pallet and bags` while leaving the point-cloud display enabled to
inspect only the simulated camera observation. The robot display remains
independent.

## ROS checks

```bash
ros2 topic hz /pickcell/sensors/overhead_depth/points
ros2 topic hz /pickcell/sensors/overhead_depth/color/image_raw
ros2 topic echo --once /pickcell/sensors/overhead_depth/camera_info
ros2 topic echo --once /pickcell/sensors/overhead_depth/points
ros2 run tf2_ros tf2_echo cell overhead_depth_camera_optical_frame
```

The point-cloud header must use `overhead_depth_camera_optical_frame`, and all
message timestamps must follow Isaac Sim's `/clock`.
