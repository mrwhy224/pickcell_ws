# PickCell implementation roadmap

This tracker translates `deep-research-report.md` into implementation tasks. A checked item means corresponding source exists and was verified during the foundation audit on 2026-08-05. Placeholder or empty files are not counted as done.

## Phase 1 — Contract foundation

Outcome: stable public API, configuration semantics, and frame conventions.

### Workspace

- [x] Use a monorepo with organizational group directories below `src/`.
- [x] Keep ROS packages under `src/foundation/` and avoid a `package.xml` in the group directory.
- [x] Add repository-wide ignores for colcon output, Python caches, and local configuration.
- [x] Add a top-level README with supported OS/ROS versions and build/test instructions.
- [ ] Add `third_party.repos`, `docker/`, `scripts/`, `datasets/`, and `docs/` when their first real content is needed.

### `pickcell_interfaces`

- [x] Create the standalone ROS package with no implementation-package dependencies.
- [x] Define the recommended messages: `ObjectReference`, `GraspCandidate`, `GraspCandidateArray`, `MotionPlanMetadata`, `SafetyState`, `ModuleStatus`, `SystemStatus`, and `TaskState`.
- [x] Define public actions: `DetectObjects`, `PlanGrasps`, `PlanMotion`, `ExecuteTrajectory`, and `ExecuteTask`.
- [x] Define supporting services: `InjectDetections`, `GetCapabilities`, `ValidateTrajectory`, `ClearScene`, and `GetSystemProfile`.
- [x] Add machine-readable shared error codes and document their semantics.
- [x] Generate and test ROS type support with `rosidl_default_generators`.

### `pickcell_config`

- [x] Create an installable `ament_python` package.
- [x] Implement mode selection for `mock`, `sim`, and `real` without coupling mode to an algorithm.
- [x] Implement recursive configuration merging and the documented default/mode/profile/local/manifest precedence.
- [x] Install configuration, calibration, and schema resources into the package share directory.
- [x] Add resolver tests covering merge precedence, explicit overrides, environment mode selection, invalid input, and installed resources.
- [ ] Replace empty module configuration templates with valid ROS parameter defaults.
- [ ] Replace empty system manifests with declarative module/provider/profile selections.
- [ ] Replace `{}` schema placeholders with restrictive JSON Schemas and validate manifests/configuration during loading.
- [ ] Populate calibration templates with documented units, frames, and safe example values.
- [ ] Export an effective configuration hash for reproducible runs.
- [ ] Remove obsolete/empty Python modules and the stray `setup.py.before_config_generator` backup, or implement their intended APIs.
- [ ] Enable and pass copyright/style tests instead of skipping or leaving empty test modules.

### `pickcell_description`

- [x] Create the `ament_cmake` description package.
- [x] Establish the canonical frame hierarchy: `world -> cell`, robot base/tool/TCP, camera optical frame, and bin frame.
- [x] Provide a Xacro model with visual, collision, and inertial data for the temporary robot base.
- [x] Provide a display launch file using robot-state publisher, joint-state publisher GUI, and RViz.
- [ ] Replace the temporary fixed base-to-tool model with the selected robot's complete URDF/Xacro.
- [ ] Add meshes, joint limits, sensor/end-effector definitions, and `ros2_control` tags.
- [ ] Add an RViz configuration and pass it explicitly from the display launch file.
- [ ] Add automated Xacro/URDF and frame-tree tests.
- [ ] Replace TODO package metadata and generated lint suppressions.

### `pickcell_test_support`

- [x] Create the package.
- [x] Add initial synthetic pose, detection, and point-cloud generators plus frame/error contract assertions.
- [ ] Add ROS launch helpers and asynchronous action-client fixtures when the first server package is introduced.
- [ ] Add reusable contract tests for actions, cancellation, error codes, frames, namespaces, QoS, lifecycle, and clean shutdown.

### Foundation acceptance gate

- [ ] Build all four foundation packages on Ubuntu 24.04 / ROS 2 Jazzy.
- [ ] Run all unit, lint, interface-generation, Xacro/URDF, schema, and configuration tests.
- [ ] Document and freeze the initial public API and frame conventions before server packages proliferate.

## Phase 2 — Mock vertical slice

- [ ] Create contract-compatible mock action servers and sensor publishers.
- [ ] Create task, grasp, and motion servers behind the public interfaces.
- [ ] Create a mock trajectory executor with cancellation and normalized results.
- [ ] Run a complete pick-and-place pipeline without sensors or robot hardware.

## Phase 3 — Planning foundation

- [ ] Create `pickcell_moveit_config` with SRDF, kinematics, pipelines, limits, controllers, and planning-scene configuration.
- [ ] Create the scene model and deterministic pose/detection injector.
- [ ] Implement the motion-server facade and plan against injected collision objects.

## Phase 4 — Control integration

- [ ] Create controller configuration and controller-manager bringup.
- [ ] Implement the stable trajectory-execution action and safety gate.
- [ ] Execute through simulated `ros2_control` hardware.

## Phase 5 — Perception baseline

- [ ] Create the ROS-independent perception core/plugin API.
- [ ] Implement the lifecycle perception server and PCL baseline plugin.
- [ ] Add repeatable rosbag fixtures, contract tests, and accuracy/latency regression tests.

## Phase 6 — Isaac integration

- [ ] Add Isaac Sim assets, bridge configuration, sensors, controllers, and launch files.
- [ ] Run the full pipeline in simulation while preserving the canonical contracts and frames.

## Phase 7 — Hardware integration

- [ ] Add real sensor launch/calibration wrappers.
- [ ] Add robot and gripper hardware/vendor adapters behind `ros2_control`.
- [ ] Validate hardware-in-loop tracking, timing, cancellation, and recovery.

## Phase 8 — Algorithm expansion

- [ ] Add classical/vision/ML perception profiles without changing the public perception API.
- [ ] Add geometric and learned grasp strategies without changing the public grasp API.
- [ ] Add task variants and kinematics plugins without coupling subsystem implementations.

## Phase 9 — Production hardening

- [ ] Put safety authorization in every execution path and add stop/interlock handling.
- [ ] Add lifecycle supervision, diagnostics, heartbeats, status/profile reporting, and configuration hashes.
- [ ] Add package-subset CI, full mock CI, simulation CI, benchmark gates, and HIL tests.
- [ ] Establish interface versioning, release policy, quality criteria, and operational documentation.
