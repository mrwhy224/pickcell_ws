# PickCell

The current workspace demonstrates one simple data flow:

```text
mock point publisher -> planning target listener
```

The same launch publishes the cell and KUKA KR 16 R2010-2 frame tree using
`pickcell_description`. Fixed frames are published on `/tf_static`; the six
robot joints are initialized to zero and published on `/tf` for simulation.

The exact robot kinematics and meshes come from the released ROS 2
`kuka_cybertech_support` package. Install it once with:

```bash
sudo apt update
sudo apt install ros-humble-kuka-cybertech-support
```

The robot mode, selected model, mounting transform and future gripper TCP
transform are in `system.robot` inside the application YAML. The TCP
intentionally coincides with
KUKA `tool0` until a gripper is selected; it is not a guessed 100 mm offset.

## Change the target

Edit only this file:

[`src/foundation/pickcell_config/config/application.yaml`](src/foundation/pickcell_config/config/application.yaml)

The active value is the `target_point` entry under:

```yaml
nodes:
  point_cloud_mock:
    parameters:
      target_point: [x, y, z]
```

The same file says that the point publisher runs in `mock` mode and the planning
listener runs in `sim` mode.

## Build and run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to pickcell_bringup
source install/setup.bash
ros2 launch pickcell_bringup system.launch.py
```

The publisher and listener will both print the value currently stored in that
entry. There are no coordinate defaults in their source code or launch files.
After the initial symlink build, configuration edits only require restarting the
launch; they do not require another build.

[`deep-research-report.md`](deep-research-report.md) is a future architecture
reference; it does not describe files that must exist today.
