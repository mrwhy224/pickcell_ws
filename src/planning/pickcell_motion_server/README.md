# Motion solver

`InverseKinematicsSolver` accepts a target TCP pose and returns every unique,
finite configuration produced by a robot-specific inverse-kinematics backend
that satisfies the configured joint limits.

`JointDistanceOptimizer` currently ranks collision-free candidates by weighted
joint displacement from the simulated starting state. The weights are kept in
`application.yaml`, ready for additional safety or quality terms later.

`MoveItIKBackend` implements that boundary through MoveIt's `compute_ik`
service. It requests collision-aware IK from several seed configurations so
different solution branches can be discovered; the solver deduplicates and
validates the responses. The active KDL plugin is configured in
`pickcell_moveit_config/config/kinematics.yaml` and can later be replaced by
TRAC-IK, IKFast, or another MoveIt kinematics plugin.

At startup, `MotionSolverNode` reads the target through TF, asks MoveIt for
candidates, publishes the selected configuration on
`planning/selected_joint_configuration`, which is consumed by a separate
simulation state applier. The solver does not publish TF, plan a path, modify
the target, or command a controller.

## Pallet-to-box cycle

`PickPlaceCyclePlanner` defines the higher-level industrial motion contract.
The pallet is centred at `cell x=+1.4 m`; the open receiving box is mirrored
behind the robot at `cell x=-1.4 m`. The configured safe points are:

```text
camera-clear home:  [-1.40, 0.00, 1.05,   0, 180, 0]
transfer waypoint:  [-0.90, 0.00, 1.20,   0, 180, 0]
box approach:       [-1.40, 0.00, 1.05,   0, 180, 0]
box drop:           [-1.40, 0.00, 0.72,   0, 180, 0]
```

The home pose is directly above the rear box and is also the box-approach pose.
The robot waits there while perception identifies the next bag; motion begins
only after a selected top-surface point arrives. These values are TCP XYZABC
poses in the `cell` frame. They are commissioning
defaults, not certified taught points. They must be checked with the real
gripper/TCP, payload, safety zones, and collision model before execution.

The cycle intentionally separates variable and repeatable motion:

```text
fixed camera-clear home
  -> collision-planned bag approach
  -> linear pick and lift
  -> collision-planned transfer waypoint
  -> fixed transfer to box
  -> fixed drop and retreat
  -> fixed return through transfer waypoint to home
```

`MotionCycleNode` subscribes to the robust top-surface pick point from
perception, transforms it into `cell`, builds this ordered contract, and
publishes a `nav_msgs/Path` on
`planning/pick_place_cycle` for RViz and the future execution layer. It does not
command the robot. In production, the steps marked `FIXED` should be taught,
validated, and stored in the robot controller (or as validated joint
trajectories); only the steps marked `PLANNED` should invoke online planning.
