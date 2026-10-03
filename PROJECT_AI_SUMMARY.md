# PickCell Project Summary

Updated: 2026-10-03

## Purpose and status

PickCell is a ROS 2 Humble simulation workspace for selecting chemical bags
from overhead Isaac RGB-D data and animating a guarded KUKA pick/place cycle.
It is an integration and visualization system, not a physical robot controller.
The active KUKA geometry is the maintained KR 240 R2900-2 model used as a
same-family reach proxy for the named KR 180 cell.

Implemented:

- YAML-driven launch, Xacro/SRDF/MoveIt configuration, Isaac camera and `/clock`.
- Reusable RGB-D geometry pipeline and live candidate selection.
- Explicit accepted-bag, cycle, gripper command/result, and terminal contracts.
- One motion owner using MoveIt IK, OMPL for local joins, Cartesian bag descent
  and lift, and a deterministic lateral two-anchor transfer.
- Mock joint trajectory playback with correlated completion IDs.
- Contact-gated bag attachment and continuous carried/delivered RViz markers.

Boundaries:

- Isaac supplies camera data but does not execute the arm trajectory.
- The active arm backend is `joint_state_sim`, not `ros2_control`.
- Bag attachment is a marker transform, not an Isaac physical joint or MoveIt
  attached collision object.
- MoveIt checks robot, pallet, box, and sweep geometry, but stateful sacks are
  excluded from its fixed model to prevent stale delivered-bag collisions.
- Perception candidates are geometric regions, not semantic detections.

## Authoritative configuration

`src/foundation/pickcell_config/config/application.yaml` is the single runtime
configuration. `pickcell_bringup/launch/system.launch.py` loads it and starts
the graph. The shared `two_anchor_task` group contains the cyclic task geometry.

Exactly two persistent anchors are active, in `cell` and XYZABC units of metres
and degrees:

```yaml
pick_anchor: [1.40, 0.00, 1.20, 0.0, 180.0, 0.0]  # A_PICK
drop_anchor: [-1.40, 0.00, 0.72, 0.0, 180.0, 0.0] # B_DROP
```

The initial joint vector is startup state only, not a third task anchor.
Bag-specific hover, contact, and lift poses are derived per cycle and are not
persistent anchors.

## Active data flow

```text
Isaac RGB/depth/camera info + /clock
  -> live_instance_segmentation
  -> perception/selected_bag_point (proposal with real sensor frame/stamp)
  -> motion_cycle (single acceptance owner)
  -> planning/active_bag + planning/pick_place_cycle
  -> trajectory_executor (single arm command owner)
  -> planning/joint_trajectory
  -> joint_state_sim
  -> joint_states + planning/trajectory_complete_id
  -> robot_state_publisher / MoveIt FK and planning / RViz
```

`bag_transfer_visualizer` consumes the accepted bag and correlated gripper
commands, verifies executed geometry from TF/joint freshness, publishes
gripper results, and renders the exact selected bag.

The important topics are:

| Topic | Type | Meaning |
| --- | --- | --- |
| `perception/selected_bag_point` | `geometry_msgs/PointStamped` | Current proposal, retaining sensor frame/stamp |
| `planning/active_bag` | `pickcell_interfaces/ActiveBag` | Accepted selection, explicit cycle and source identity |
| `planning/pick_place_cycle` | `pickcell_interfaces/PickPlaceCycle` | Named two-anchor and temporary-pose contract |
| `planning/active_motion_path` | `nav_msgs/Path` | Optional diagnostic display, off by default |
| `planning/task_anchors` | `visualization_msgs/MarkerArray` | Exactly A_PICK and B_DROP |
| `planning/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Sole arm command stream |
| `planning/trajectory_complete_id` | `std_msgs/UInt64` | Correlated mock-controller completion |
| `gripper/command` | `pickcell_interfaces/GripperCommand` | Explicit close/open cycle and command |
| `gripper/grasp_result` | `pickcell_interfaces/GripperResult` | Contact-gated close result |
| `gripper/release_result` | `pickcell_interfaces/GripperResult` | Box-gated release result |
| `planning/cycle_result` | `pickcell_interfaces/CycleResult` | Phase, success, delivered and held facts |

The old `active_bag_point`, path-as-command, `cycle_accepted_id`, signed
`cycle_result_id`, Boolean `gripper/closed`, and signed gripper confirmation
protocols are retired.

## Cycle and motion

The required sequence is:

1. Establish fresh executed state at exact q_A/A_PICK.
2. Accept one new source frame/stamp and assign a non-time-based session cycle ID.
3. Plan from A_PICK to the selected bag's temporary hover.
4. Execute a complete collision-aware Cartesian descent to contact.
5. Close only after matching fresh TCP/contact checks; attach that exact bag.
6. Execute Cartesian lift and return to exact q_A.
7. Execute the canonical loaded lateral sweep to q_B/B_DROP.
8. Release only when actual TCP and preserved bag center lie in the box target.
9. Generate new timing/derivatives on the exact reverse geometry to q_A.
10. Publish success, clear all owners, and accept a new snapshot.

`joint_1` is the model's Z-axis base yaw. The A_PICK-to-B_DROP route uses 81
minimum-jerk joint samples. It requires at least 150 degrees of configured base
yaw, limits joints 2-5 to 75 degrees of endpoint excursion, permits up to 190
degrees of axial joint_6 compensation, and limits TCP height to 1.35 m. Every
sample is checked through MoveIt state validity and FK before publication.

The reverse route is generated afresh from q_B to q_A. Geometry equality is
checked against normalized path progress so different legal time sampling does
not change the route.

## Selection and bag state

Perception stores one immutable accepted overlay, selected cloud, source
frame/stamp, and pick point while a cycle is active. It binds only an
`ActiveBag` carrying the exact source identity. A terminal message cannot claim
an unbound proposal by numerical recency.

The coordinator rejects the same retained source snapshot after completion or
recoverable failure until perception publishes a new stamp. The visualizer maps
the accepted surface to a bounded nearest still-pallet marker once; it never
reselects during an active cycle. Mapping beyond 0.16 m is rejected.

Attachment requires:

- matching cycle and gripper command IDs;
- an empty gripper and available mapped bag;
- joint state no older than 0.5 s and a valid executed TCP transform;
- command pose identical to the accepted grasp pose;
- TCP error <= 25 mm and suction-axis error <= 8 degrees;
- contact within 100 mm XY and 30 mm Z of the accepted surface.

The tool-to-bag transform is captured at contact, preserving world pose. Release
checks B_DROP and a box-frame bag center of `(0,0,0.63)` within 40 mm XY/Z.
Successful delivery changes the same marker to box state and exact-point
exclusion prevents the static camera image from selecting it logically.

## Failure policy and clocks

Known-empty failures before grasp recover to A_PICK and release the cycle lock.
If that recovery plan fails, the executor safe-stops instead of retrying in a
tight loop. Held or unknown-payload failures retain the lock and safe-stop; the
mock system's supported recovery is inspection followed by launch restart.
Successful release is recorded independently, so a later return failure does
not resurrect the bag or claim it is held.

ROS headers use simulated ROS time for data. Cycle identity does not. Playback,
freshness, and watchdogs consistently use monotonic time, allowing initial arm
display before Isaac's clock and avoiding epoch comparisons. The mock arm can
advance during a paused Isaac clock; production behavior would need a deliberate
pause interlock.

## Visualization

Default RViz shows the robot, independent pallet stand, receiving box, live
perception, stateful bags, and exactly two task-anchor markers. The legacy red
target and active diagnostic path are off by default. Hover/contact/lift poses,
sample axes, IK alternatives, and historical paths are not persistent displays.

Isaac receives the complete scene with nine bags. MoveIt receives the same
robot/pallet/box description with `include_bags=false`; treating moving bags as
fixed robot links caused stale collision geometry after delivery.

## Build, run, and verify

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to pickcell_bringup
source install/setup.bash
ros2 launch pickcell_bringup system.launch.py
```

Focused regression:

```bash
colcon test --packages-select pickcell_motion_server \
  pickcell_instance_segmentation pickcell_mock_nodes pickcell_bringup
colcon test-result --verbose
```

The 2026-10-03 host trace completed bags 9, 7, and 8 in consecutive cycles.
Each used real hover/descent/lift motion, 81 collision/FK checked samples in
each transfer direction, release inside the box, and full return to A_PICK.
Peak TCP height was 1.200 m. See `docs/deep-debug-report.md` for cycle IDs,
measured q_A/q_B, test totals, artifact paths, known limitations, and rollback.

## Repository map

- `src/foundation/pickcell_interfaces`: explicit ROS contracts.
- `src/foundation/pickcell_description`: robot/cell Xacro, meshes, RViz.
- `src/foundation/pickcell_config`: authoritative application YAML.
- `src/foundation/pickcell_bringup`: graph construction and description split.
- `src/perception/pickcell_instance_segmentation`: RGB-D pipeline and lock.
- `src/planning/pickcell_motion_server`: coordinator, executor, route, mock gripper.
- `src/planning/pickcell_moveit_config`: SRDF, KDL, OMPL, joint limits.
- `src/backends/pickcell_mock_nodes`: trajectory player and deterministic fixture.
- `src/backends/pickcell_isaac_sim`: Isaac scene/camera runner.
- `docs/deep-debug-report.md`: authoritative corrective report.

Do not reintroduce P1/side-entry/side-pass anchors, timestamp IDs, path-index
phase inference, a second trajectory publisher, or unconditional bag snapping.
