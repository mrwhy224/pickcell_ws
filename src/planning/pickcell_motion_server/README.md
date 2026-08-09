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
