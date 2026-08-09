# PickCell

The current workspace demonstrates one simple data flow:

```text
mock point publisher -> target pose and TF
```

The same launch publishes the cell and KUKA KR 16 R2010-2 frame tree using
`pickcell_description`. Fixed frames are published on `/tf_static`; the six
robot joints are initialized to zero and published on `/tf` for simulation.

The exact robot kinematics and meshes come from the released ROS 2
`kuka_cybertech_support` package. Install it once with:

```bash
sudo apt update
sudo apt install ros-humble-kuka-cybertech-support ros-humble-moveit
```

The robot mode, selected model, mounting transform and future gripper TCP
transform are in `system.robot` inside the application YAML. The TCP
intentionally coincides with
KUKA `tool0` until a gripper is selected; it is not a guessed 100 mm offset.
MoveIt loads the `manipulator` group from `robot_base` through `gripper_tcp`,
provides collision-aware inverse kinematics through `compute_ik`, and starts
with KDL and OMPL configuration from `pickcell_moveit_config`.
The simulated starting pose is stored in `initial_joint_positions_deg` in the
same section, using degrees for readability.

RViz starts automatically with the system launch. Its configured displays use:

- KUKA mesh: **Robot - KUKA KR 16 R2010-2**
- planning target: a **red pose arrow** showing position and orientation
- mock point cloud: **cyan spheres**

TF axes are available as `TF frames (optional)` but disabled initially so they
do not hide the robot. RViz can be disabled with `system.visualization.enabled`
in the application YAML.

## Change the target

Edit only this file:

[`src/foundation/pickcell_config/config/application.yaml`](src/foundation/pickcell_config/config/application.yaml)

The active value is the `target_pose` entry under:

```yaml
nodes:
  point_cloud_mock:
    parameters:
      target_pose: [x, y, z, a, b, c]
```

XYZ is expressed in metres. ABC is expressed in degrees using KUKA's Z-Y-X
convention: A rotates about Z, B about Y, and C about X. This is the only
planning target; there is no separate `final_position` topic.

The same file says that the point publisher runs in `mock` mode. The motion
solver is a library boundary and is not launched as a listener node.

## Build and run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to pickcell_bringup
source install/setup.bash
ros2 launch pickcell_bringup system.launch.py
```

The publisher prints the value currently stored in that entry. The motion
solver reads its TF, samples collision-aware IK candidates, selects the least
weighted joint displacement, and applies that configuration to the simulated
joint-state display. It does not plan a path or command a controller.
After the initial symlink build, configuration edits only require restarting the
launch; they do not require another build.

[`deep-research-report.md`](deep-research-report.md) is a future architecture
reference; it does not describe files that must exist today.
