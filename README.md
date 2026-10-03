# PickCell

The current workspace demonstrates one simple data flow:

```text
mock point publisher -> target pose and TF
```

The same launch publishes the cell and KUKA KR 180 R2900-2 frame tree using
`pickcell_description`. Fixed frames are published on `/tf_static`; the six
robot joints are initialized to the configured valid bent pose and published
on `/tf` for simulation.

The released ROS 2 packages do not currently contain an exact KR 180 model.
The maintained KR 240 R2900-2 QUANTEC model is therefore used as a temporary
same-family, same-reach geometry proxy. Install its support package once with:

```bash
sudo apt update
sudo apt install ros-humble-kuka-quantec-support ros-humble-moveit
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

- KUKA mesh: **Robot - KUKA KR 180 R2900-2** (KR 240 R2900-2 proxy geometry)
- load support: a **wall-free 0.8 m deep x 1.2 m wide x 0.15 m high wooden
  pallet**, with its nearest edge 1 m in front of the robot base
- pallet load: **nine filled chemical bags** arranged in a repeatable,
  randomized five-layer stack
- simulated sensor: a fixed **640 x 480 overhead RGB-D camera** at
  `(1.4, 0.0, 2.0) m`, looking straight down at the pallet
- task anchors: **A_PICK** above the pallet and **B_DROP** above the box

RViz keeps the robot, pallet load, and overhead point cloud in separate display
groups. Each can be shown or hidden without changing the other two.

Isaac Sim camera and ROS 2 bridge setup is documented in
`src/backends/pickcell_isaac_sim/README.md`.

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

The point publisher remains a deterministic mock fixture; it does not command
the arm.

## Build and run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to pickcell_bringup
source install/setup.bash
ros2 launch pickcell_bringup system.launch.py
```

The active motion-cycle executor uses collision-aware IK for bag-dependent
legs, then follows a separately defined fixed Cartesian transfer corridor. It
publishes a demo trajectory to the mock joint-state player; it does not command
an Isaac articulation or a physical controller.
After the initial symlink build, configuration edits only require restarting the
launch; they do not require another build.

## Capture a bag dataset scene

The RGB and depth images come from the same simulated optical camera and are
therefore pixel-aligned. To save the latest synchronized pair, calibration,
and an empty instance-mask template, run:

```bash
ros2 service call /pickcell/dataset/capture std_srvs/srv/Trigger '{}'
```

Scenes are written under `/home/mahdi/pickcell_ws/datasets/bags_raw` by
default. Annotate each
`instance.png` so that 0 is background and 1, 2, ... identify the visible bags.
Depth is stored losslessly as a 16-bit PNG in millimetres; `camera.json` records
the original ROS encoding and a `depth_scale` of 1000.
The capture node creates the dataset root when it starts and logs its absolute
path. A numbered `scene_*` directory is created only after the service reports
`success: true`.

## Generate a synthetic dataset automatically

The dedicated headless launch randomizes a physically plausible bottom-up bag
stack, varies the bag count and poses, and writes one readable `scene_*` folder
per sample. Each folder contains `color.png`, 16-bit millimetre `depth.png`,
raw-ID `instance.png`, `pointcloud.ply`, and JSON camera, label, and pose
metadata. The KUKA links are hidden only during dataset generation so the arm
cannot occlude the bags; normal simulation and ROS camera output still include
the robot. In `instance.png`, 0 is background and the visible bags use compact
IDs 1, 2, ...:

```bash
ros2 launch pickcell_bringup generate_dataset.launch.py \
  sample_count:=10000 \
  output_directory:=/home/mahdi/pickcell_ws/datasets/bags_run_001 \
  seed:=42 min_bags:=1 max_bags:=9
```

An existing dataset is never overwritten. If the requested directory contains
files, the generator automatically creates a numbered sibling such as
`bags_run_001_001`. Generation progress is printed every 100 samples. Temporary
NumPy arrays are converted and removed before the process exits automatically.

[`deep-research-report.md`](deep-research-report.md) is a future architecture
reference; it does not describe files that must exist today.

## Process one RGB-D frame

`process_rgbd_frame` is the reusable inference entry point for tests and live
camera callbacks. It accepts an aligned BGR/RGB-compatible `uint8` color array,
the aligned depth array, and camera calibration:

```python
from pickcell_instance_segmentation import CameraModel
from pickcell_instance_segmentation import process_rgbd_frame

camera = CameraModel(
    width=depth.shape[1], height=depth.shape[0],
    fx=fx, fy=fy, cx=cx, cy=cy,
    depth_scale=1000.0,  # uint16 depth is in millimetres
    distortion_model="plumb_bob",
    distortion_coefficients=distortion_coefficients,
    frame_id="camera_optical_frame",
    source_schema="live_camera",
)
result = process_rgbd_frame(
    color, depth, camera,
    input_is_rectified=True,
)

xyz = result.cloud.xyz       # H x W x 3 organized point cloud, metres
valid = result.cloud.valid   # H x W valid-depth mask
patches = result.patches
graph = result.graph
affinity = result.affinity
```

Use `process_scene_directory(path, input_is_rectified=True)` to run the same
pipeline on a captured or synthetic `scene_*` directory. The live ROS node also
turns accepted patch affinities into candidate instances and publishes both a
colored candidate cloud and the selected next-pick cloud.

The reusable `UpperRightSegmentSelector` chooses the segment with the smallest
overhead-camera depth (the upper/closest surface). Candidates within the
configured height tolerance are treated as level and the image-right candidate
wins. This selection is geometric; downstream grasp planning should still
validate the candidate and transform its published centroid into the robot
planning frame.
Once a candidate starts a motion cycle, the live node keeps the selected
overlay and selected cloud from that accepted RGB-D snapshot. Later camera
frames may change candidate ordering, but the active highlight, selected
cloud, and published target point remain tied to the same locked selection
until the matching positive cycle result releases it.

## Pick/place cell cycle

The receiving box is mirrored behind the robot at `x=-1.4 m`, opposite the bag
pallet at `x=+1.4 m`. The motion-cycle coordinator consumes one locked selected
bag and publishes an explicit named contract. The executor moves from A_PICK
to that bag's temporary hover, executes a Cartesian contact descent, attaches
only after geometric confirmation, lifts back to exact A_PICK, and performs a
validated base-yaw-dominant lateral sweep to B_DROP. The empty return uses the
same geometry in reverse. See
`src/planning/pickcell_motion_server/README.md` and
[`docs/motion-route-report.md`](docs/motion-route-report.md) for its direction,
safety boundaries, and physical-system limitations.
The launch demonstration collision-plans and animates the complete arm cycle
with a simple rigid vacuum gripper. A cycle ID holds one selected candidate
until its matching successful release and unloaded return; source timestamps
remain sensor timestamps, and failed or late events cannot claim another bag.
