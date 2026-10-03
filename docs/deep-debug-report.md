# PickCell Corrective Deep-Debug Report

Date: 2026-10-03

## Baseline and active system

- Workspace: `/home/mahdi/pickcell_ws`
- Branch/HEAD: `master`, `e042c83d8e12ada841cfc1eac9f235439e282e86`
  (`Attach and deliver simulated bag payloads`)
- The pre-existing dirty working tree was preserved and treated as the baseline.
- ROS 2 Humble and MoveIt 2 2.5.9 are loaded from `/opt/ros/humble`; the
  rebuilt overlay resolves from this workspace's `install` directory.
- Authoritative launch/config: `pickcell_bringup/launch/system.launch.py` and
  `pickcell_config/config/application.yaml`.
- Isaac owns `/clock` and camera topics. `live_instance_segmentation` proposes
  one target. `motion_cycle` owns acceptance. `trajectory_executor` is the only
  publisher of `planning/joint_trajectory`. `joint_state_sim` publishes the
  executed joint states and correlated completion. `bag_transfer_visualizer`
  owns simulated contact, attachment, release confirmation, and bag markers.

The host graph was inspected during a real Isaac/RViz launch. No second arm
trajectory publisher or competing arm TF owner was present.

## Verified causes

The previous pass retained the old P1/side-entry/side-pass/P2 route, inferred
motion phases from path indexes, encoded cycle identity in timestamps, and let
the visual gripper attach whichever bag was nearest when a Boolean close
arrived. Its missing-acceptance workaround could associate a terminal ID by
recency rather than ownership. Those choices explain why it did not establish
the requested route, grasp timing, or identity.

The current-code investigation also found two sustained-run defects:

1. After a terminal result, `motion_cycle` could accept the same retained
   `PointStamped` once more before perception processed that result. The new
   cycle gate remembers the completed `(source_frame, source_stamp)` and waits
   for a genuinely new snapshot.
2. The nine bags were fixed URDF links in MoveIt's model while their markers
   moved between pallet and box. A delivered bag therefore remained as stale
   collision geometry. Isaac still receives all nine physical scene links;
   MoveIt now receives the robot, pallet, and box without the stateful bags.
   Payload and remaining-bag collision attachment are still a declared
   limitation, not claimed as validated.

Perception's geometric candidate grouping can still propose pallet/background
regions. The bounded visual-bag mapping rejects these without attaching a bag,
and the executor returns empty to A_PICK. This limitation is not described as
semantic bag detection.

## Two-anchor contract

Only these persistent task anchors exist, both in `cell`:

| Anchor | XYZABC | Purpose |
| --- | --- | --- |
| A_PICK | `[1.40, 0.00, 1.20, 0, 180, 0]` | Repeatable joint branch above the pallet |
| B_DROP | `[-1.40, 0.00, 0.72, 0, 180, 0]` | Release above the box |

Hover, contact, and lift are temporary targets derived from the accepted bag
surface. They are not persistent anchors and are not displayed by default.
RViz subscribes to one stable `task_anchors` MarkerArray containing IDs 0 and 1.
The diagnostic active `nav_msgs/Path` display and legacy red target are off by
default. Marker IDs are reused, so completed paths do not accumulate.

| State/event | Guard and action | Next state | Timeout/failure cleanup |
| --- | --- | --- | --- |
| Startup | Fresh joints; FK/plan reaches exact A_PICK branch | idle at A_PICK | Planning failure enters safe stop |
| Selection | No active cycle; new source frame/stamp | publish explicit contract and ActiveBag | pick approach | Malformed/stale duplicate rejected |
| Hover arrival | Correlated trajectory completion | Cartesian descent to bag surface | at pick | Planning/watchdog failure returns empty to A_PICK |
| Grasp result | Fresh joints/TF, matching cycle/command/pose, contact region | attach exact mapped bag; lift | grasp confirmed | Negative result returns empty; unknown timeout safe-stops held |
| Lift/A_PICK | Matching completions and exact cached q_A | generate/validate canonical loaded sweep | loaded transfer | Failure with known payload safe-stops held |
| B_DROP | Collision/FK validated sweep completed | correlated release command | released | Rejected/unknown release safe-stops held |
| Empty return | Payload confirmed released; reverse geometry | exact q_A | idle | Failure preserves delivered/excluded bag, then reports fault/recovery |
| Terminal | Exact q_A, empty gripper, delivered true | correlated CycleResult; clear owners | next selection | Duplicate terminal events ignored |

Before-grasp failures automatically plan back to A_PICK and unlock only after a
known-empty result. A failed recovery plan now enters an explicit executor safe
stop instead of retrying continuously. A failure with a held or unknown payload
does not unlock; supported mock recovery is to stop the launch, inspect the bag
marker/attachment state, and restart the simulation. There is no fabricated
success or automatic opening. A release followed by return failure retains the
bag as delivered and excludes its exact accepted point.

## Identity, contact, and time

`ActiveBag`, `PickPlaceCycle`, `GripperCommand`, `GripperResult`, and
`CycleResult` carry explicit positive session-scoped IDs. Cycle IDs use a
random process namespace plus a counter, not ROS time. Sensor frame/stamp are
separate fields and remain the TF/freshness identity. Trajectory command IDs
remain explicit in `pickcell_trajectory/<id>` and completions must match.

Perception binds an acceptance only when source frame and stamp exactly match
its locked snapshot. It no longer adopts a newer terminal result by recency.
The accepted overlay, cloud, point, grasp, white marker, attached bag, and
delivered exclusion therefore retain one source identity across reordered/new
camera frames.

The mock close succeeds only for the owning cycle and command when joints are
fresh (0.5 s), actual `cell -> gripper_tcp` exists, TCP position is within
25 mm, suction-axis error is within 8 degrees, and contact is within 100 mm XY
and 30 mm Z of that selected surface. A close at A_PICK is about 300 mm high
and is rejected. Attachment stores the actual tool-to-bag transform, so the
world pose is continuous and the marker follows executed TCP without snapping.

Release checks actual TCP against B_DROP and the preserved bag center against
`drop_box`: target `(0,0,0.63)`, 40 mm XY and 40 mm Z. The 40 mm XY bound is
inside the tighter geometric clearance of 50 mm between the 0.35 m box opening
half-width and 0.30 m bag half-width.

ROS message stamps use simulated ROS time. Joint interpolation, freshness, and
watchdogs use one monotonic clock, so epochs are never compared and cycle IDs
survive simulated-time reset. The mock arm can continue during a paused Isaac
clock by design; a watchdog fault is explicit. Pause/backward-clock behavior
was not exercised in the host run.

## Canonical lateral transfer

The KR QUANTEC model confirms `joint_1` is the base Z-yaw. The executor derives
collision-aware q_A/q_B once, requires a positive joint_1 sweep of at least
150 degrees, bounds joints 2-5 to 75 degrees, permits joint_6 axial compensation
up to 190 degrees, and rejects TCP height above 1.35 m. It creates an 81-sample
minimum-jerk joint trajectory using configured velocity, acceleration, and jerk
limits. Every sample is checked with MoveIt state validity and FK. The empty
route is freshly timed on the exact reverse geometry; comparison resamples
normalized path progress rather than assuming equal indexes.

Measured host values from the accepted branch were approximately:

- q_A: `[0.00195, -1.66543, 1.96589, 0.00090, 1.27108, 0.00093]` rad
- q_B: `[3.14159, -1.50603, 2.24038, 0, 0.83644, 3.14159]` rad
- joint deltas: `[179.89, 9.13, 15.73, -0.05, -24.90, 179.95]` degrees
- loaded and empty peak TCP height: 1.200 m
- collision/FK samples: 81 in each direction

This is a primarily base-yaw lateral sweep with bounded shoulder/elbow motion
and required axial-wrist compensation. It does not invoke an unconstrained
planner between A_PICK and B_DROP and has no over-the-top fallback.

## Runtime evidence

Launch command:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/pickcell_final_ros_logs \
  ros2 launch pickcell_bringup system.launch.py
```

The final rebuilt 150-second host trace completed two distinct visual bags
without node restart and delivered a third before the bounded timeout:

| Cycle suffix | Visual bag | Mapping error | Box center | Outcome |
| --- | --- | --- | --- | --- |
| `...8450` | 9 | 0.008 m | `(0.008, 0.003, 0.641)` | approach, grasp, deliver, reverse return complete |
| `...8451` | 7 | 0.032 m | `(-0.002, 0.031, 0.642)` | approach, grasp, deliver, reverse return complete |
| `...8452` | 8 | 0.033 m | `(0.000, -0.033, 0.641)` | delivered; reverse validated and started, then test timeout |

The two completed picks each included a 21/22-point Cartesian descent, a
32-point lift, the 81-sample loaded sweep, release, the independently timed
81-sample reverse, and full return to A_PICK. All three attachments logged
0.0000 m TCP error and 0.00 degree suction-axis error in the mock execution.
The final rebuilt run transcript is stored at
`/tmp/pickcell-final-runtime.typescript`; ROS logs are under
`/tmp/pickcell_final_ros_logs`.

The configured scene has nine bags, so no claim of a ten-target sustained run
is made. The source-identity gate has a ten-cycle deterministic regression, but
that is not presented as movement evidence. A ten-target Isaac fixture and
pause/reset runtime test remain incomplete.

## Tests

The affected package build succeeded. The final focused ROS suite reported
251 tests, 0 errors, 0 failures, 7 skipped. New or
updated regressions cover exactly two reusable anchors, distinct hover/contact,
explicit non-time identities, stale-source rejection, legal phase transitions,
lateral sweep direction/dynamics/resampled repeatability, no attachment at
A_PICK, and suction-axis contact behavior. Run:

```bash
colcon build --symlink-install --packages-select \
  pickcell_interfaces pickcell_description pickcell_moveit_config \
  pickcell_motion_server pickcell_instance_segmentation \
  pickcell_mock_nodes pickcell_bringup
colcon test --packages-select pickcell_motion_server \
  pickcell_instance_segmentation pickcell_mock_nodes pickcell_bringup
colcon test-result --verbose
```

## Git and consolidation findings

History is additive and contains no verified old revision with the requested
behavior: `f720df1` introduced live selection, `0013a20` introduced the surface
point/drop cycle, `3f10704` added repeating planned motion, `a9614ea` added
cycle locking/gripper commands, and `e042c83` added visual attachment. No bisect
was run because there is no known-good two-anchor revision.

Removed from the active contract: `camera_clear_home`, `transfer_start_anchor`,
`transfer_side_entry`, `transfer_side_pass`, separate box descent/ascent anchors,
path-index phase inference, timestamp cycle IDs, signed Int64 terminal IDs,
Boolean gripper commands, and obsolete RViz subscriptions. `runtime_node.py`
was already dead source; current entry points are `motion_cycle`,
`trajectory_executor`, and `bag_transfer_visualizer`. Generated install wrappers
are not source and were refreshed by scoped builds rather than deleting user
trees.

## Remaining limitations and rollback

- Bag attachment is a geometrically gated RViz marker mock, not an Isaac
  physical joint and not a MoveIt attached collision object.
- MoveIt validates robot/pallet/box geometry and the full sweep but does not
  include the carried payload or remaining sack pile. Do not treat this as
  physical payload certification.
- Isaac camera pixels remain static after marker-only delivery; perception's
  exact point exclusion provides logical consistency, not scene synchronization.
- Geometric segmentation is not a semantic classifier and rejected false
  proposals can add recovery motion.
- Ten consecutive physical-looking cycles and clock pause/reset remain
  unverified.

Rollback only this corrective work with a file-level reverse patch covering
the new interface messages, motion/perception nodes and tests, YAML, launch,
URDF/SRDF, RViz, and these documents. Do not use `git reset --hard`: substantial
uncommitted user work predated this pass.
