# Research-Grade ROS 2 Architecture for a Modular Industrial Pick-and-Place System

## Architectural recommendation

The strongest fit for this system is a **contract-oriented, layered ROS 2 architecture** built around stable ROS interfaces and replaceable subsystem servers. Each major capability—perception, grasp planning, motion planning, execution, task orchestration, and safety supervision—should expose a small public ROS API while keeping its algorithms and technology-specific dependencies private.

For the current toolchain, the safest compatibility baseline is **Ubuntu 24.04 with ROS 2 Jazzy**. Jazzy is supported through May 2029, and NVIDIA officially recommends and tests Jazzy and Humble with Isaac Sim; other ROS distributions may work through Isaac Sim’s experimental native-distro support, but are not the primary validated targets. citeturn0search0turn8view3turn0search14

The central design should be:

```text
                       ┌──────────────────────────────┐
                       │        Task Server           │
                       │ ExecuteTask action facade    │
                       └──────────────┬───────────────┘
                                      │
                    ┌─────────────────┴─────────────────┐
                    │                                   │
          ┌─────────▼──────────┐             ┌──────────▼─────────┐
          │ Perception Server  │             │ Scene Model Server │
          │ DetectObjects      │             │ collision world    │
          └─────────┬──────────┘             └──────────┬─────────┘
                    │                                   │
          canonical detections                  planning scene
                    │                                   │
          ┌─────────▼──────────┐                        │
          │   Grasp Server     │                        │
          │   PlanGrasps       │                        │
          └─────────┬──────────┘                        │
                    │                                   │
                    └─────────────────┬─────────────────┘
                                      │
                          ┌───────────▼───────────┐
                          │    Motion Server      │
                          │     PlanMotion        │
                          └───────────┬───────────┘
                                      │ RobotTrajectory
                          ┌───────────▼───────────┐
                          │   Safety Supervisor   │
                          │ validation + interlock│
                          └───────────┬───────────┘
                                      │ authorized trajectory
                          ┌───────────▼───────────┐
                          │ Trajectory Executor   │
                          │ ExecuteTrajectory     │
                          └───────────┬───────────┘
                                      │
                             ros2_control / driver
                                      │
                         mock / Isaac Sim / hardware
```

This architecture separates five concerns that are often incorrectly mixed together:

| Concern | Selected by | Example values |
|---|---|---|
| External environment | System profile | `mock`, `sim`, `real` |
| Input or hardware backend | Backend configuration | `rosbag`, `isaac_rgbd`, `realsense`, `vendor_robot` |
| Algorithm | Algorithm profile | `pcl_clusters`, `rgb_detector`, `learned_pose`, `ompl`, `pilz` |
| Task policy | Task profile | `generic_pick`, `bin_pick`, `known_object_pick` |
| Deployment topology | Deployment profile | separate processes, composed perception, GPU container |

These dimensions must remain independent. In particular, `mode: sim` should not imply a particular perception algorithm, planner, grasp generator, or task policy.

The architecture should use ROS communication according to the semantics of the operation: topics for continuous data, services for bounded request-response operations, and actions for long-running, cancellable operations with progress feedback. ROS 2 explicitly defines actions as long-running remote procedures that support feedback, cancellation, and preemption. citeturn1search5turn1search12turn1search19turn1search32

Lifecycle nodes are appropriate for sensor adapters, perception servers, scene management, planners, executors, and the safety supervisor because their configure/activate/deactivate transitions let bringup establish dependencies before data starts flowing. ROS 2 managed nodes are specifically intended to give supervisors greater control over system state. citeturn0search12turn0search27turn3search27

Run high-level modules in **separate processes by default**. This maximizes crash isolation, makes independent development straightforward, and proves that the interface boundary is real. Selective composition can later be enabled for high-bandwidth perception components where serialization or copying becomes measurable. ROS 2 component containers support loading multiple components into one process, while retaining the same ROS-facing API. citeturn9search0turn9search4turn9search7

The most important dependency rule is:

```text
High-level subsystem packages must never depend on one another.

Allowed:
  grasp_server       -> pickcell_interfaces
  perception_server  -> pickcell_interfaces
  motion_server      -> pickcell_interfaces
  task_server        -> pickcell_interfaces
  all servers        -> standard ROS interfaces

Forbidden:
  grasp_server       -> perception_server
  motion_server      -> grasp_server
  perception_server  -> motion_server
```

The task server may communicate with those servers at runtime through their actions, but it does not import their implementation code.

Pure algorithm SDK packages are an intentional exception to the phrase “no direct imports”: a perception plugin may depend on a low-level `pickcell_perception_core` API, but neither package communicates with grasping, motion, execution, or task packages through in-process calls.

## Workspace and package design

A monorepo is reasonable initially, provided the source tree is organized as if packages could later move into separate repositories. Group directories should not themselves contain `package.xml`; they are only organizational folders.

```text
pickcell_ws/
├── src/
│   ├── foundation/
│   │   ├── pickcell_interfaces/
│   │   ├── pickcell_description/
│   │   ├── pickcell_moveit_config/
│   │   ├── pickcell_config/
│   │   ├── pickcell_bringup/
│   │   └── pickcell_test_support/
│   │
│   ├── perception/
│   │   ├── pickcell_sensor_bringup/
│   │   ├── pickcell_perception_core/
│   │   ├── pickcell_perception_server/
│   │   ├── pickcell_perception_pcl_plugins/
│   │   ├── pickcell_perception_vision_plugins/
│   │   ├── pickcell_perception_ml_plugins/
│   │   ├── pickcell_scene_model/
│   │   └── pickcell_pose_injector/
│   │
│   ├── grasping/
│   │   ├── pickcell_grasp_core/
│   │   ├── pickcell_grasp_server/
│   │   ├── pickcell_grasp_geometric_plugins/
│   │   └── pickcell_grasp_learned_plugins/
│   │
│   ├── planning/
│   │   ├── pickcell_motion_server/
│   │   ├── pickcell_task_core/
│   │   ├── pickcell_task_server/
│   │   ├── pickcell_task_mtc_plugins/
│   │   └── pickcell_kinematics_plugins/
│   │
│   ├── execution/
│   │   ├── pickcell_control/
│   │   ├── pickcell_trajectory_executor/
│   │   ├── pickcell_robot_hardware/
│   │   └── pickcell_gripper_hardware/
│   │
│   ├── backends/
│   │   ├── pickcell_mock_nodes/
│   │   ├── pickcell_isaac_sim/
│   │   └── pickcell_sensor_hardware/
│   │
│   └── operations/
│       ├── pickcell_safety_supervisor/
│       ├── pickcell_system_monitor/
│       └── pickcell_benchmarks/
│
├── third_party.repos
├── docker/
├── scripts/
├── datasets/
├── docs/
└── README.md
```

ROS 2 package names conventionally use lowercase letters and underscore separators, which the proposed package names follow. citeturn3search6turn9search13

The package responsibilities should be as follows.

| Package | Responsibility | Runtime dependencies on other project modules |
|---|---|---|
| `pickcell_interfaces` | Custom messages, services, actions, constants, and interface documentation only | None |
| `pickcell_description` | URDF/Xacro, meshes, joint limits, sensors, end effectors, ros2_control tags, TF frame definitions | None |
| `pickcell_moveit_config` | SRDF, planning pipelines, kinematics, controller mappings, planning-scene configuration | Description only |
| `pickcell_config` | System manifests, module parameter profiles, calibration references, schemas | None |
| `pickcell_bringup` | Top-level launch assembly and profile resolution | May depend on all launchable packages |
| `pickcell_test_support` | ROS test fixtures, synthetic data generators, launch helpers, assertion utilities | Interfaces only |
| `pickcell_sensor_bringup` | Canonical launch wrappers around third-party camera/depth/stereo drivers | External drivers |
| `pickcell_perception_core` | Pure perception plugin API, common internal types, validation utilities | No high-level packages |
| `pickcell_perception_server` | Public `DetectObjects` action and detection topics; loads one configured pipeline | Core and interfaces |
| `pickcell_perception_pcl_plugins` | Filtering, plane removal, clustering, geometric fitting, template matching | Perception core, PCL |
| `pickcell_perception_vision_plugins` | RGB, grayscale, stereo, feature-based and classical image pipelines | Perception core, OpenCV |
| `pickcell_perception_ml_plugins` | Learned detection, segmentation and pose models | Perception core, inference runtime |
| `pickcell_scene_model` | Converts perceived or injected objects into a canonical world model and MoveIt collision objects | Interfaces, MoveIt messages |
| `pickcell_pose_injector` | Manual, scripted, CSV or RViz-based object-pose injection | Interfaces only |
| `pickcell_grasp_core` | Grasp-strategy API and candidate-scoring abstractions | Interfaces only |
| `pickcell_grasp_server` | Public `PlanGrasps` action; validation, ranking and algorithm loading | Grasp core and interfaces |
| `pickcell_grasp_geometric_plugins` | Parallel-jaw heuristics, surface-normal, antipodal and known-shape methods | Grasp core |
| `pickcell_grasp_learned_plugins` | Neural grasp proposal or scoring methods | Grasp core |
| `pickcell_motion_server` | Stable planning facade; translates domain goals into MoveIt requests and returns trajectories | Interfaces, MoveIt |
| `pickcell_task_core` | Task-strategy API and task-state abstractions | Interfaces only |
| `pickcell_task_server` | Public `ExecuteTask` action and task state machine | Task core and interfaces |
| `pickcell_task_mtc_plugins` | Generic pick, bin-picking and known-object task graphs using MoveIt Task Constructor | Task core, MTC |
| `pickcell_kinematics_plugins` | Robot-specific IK or service-backed IK plugins | MoveIt plugin APIs |
| `pickcell_control` | Controller YAML, controller spawning, controller-manager launch and mappings | ros2_control |
| `pickcell_trajectory_executor` | Stable execution action, safety gating, controller communication, cancellation and result normalization | Interfaces, control messages |
| `pickcell_robot_hardware` | Real robot ros2_control hardware plugin or vendor-driver adapter | Vendor SDK, ros2_control |
| `pickcell_gripper_hardware` | Gripper hardware component and controller configuration | Vendor SDK, ros2_control |
| `pickcell_mock_nodes` | Contract-compatible mock action servers, sensor publishers and mock executor | Interfaces only |
| `pickcell_isaac_sim` | Isaac USD assets, Action Graph setup, ROS bridge configuration and simulation launch | Isaac Sim bridge |
| `pickcell_sensor_hardware` | Cell-specific calibration and launch wrappers for actual sensors | External drivers |
| `pickcell_safety_supervisor` | Software-level constraints, health monitoring, execution authorization and stop requests | Interfaces only |
| `pickcell_system_monitor` | Aggregated lifecycle, heartbeat, latency, diagnostics and selected-profile reporting | Interfaces and diagnostics |
| `pickcell_benchmarks` | Offline evaluation, dataset manifests, regression metrics and report generation | Not part of production runtime |

`ros2_control` is the correct abstraction boundary for robot and gripper hardware. Its Controller Manager mediates between controllers and hardware components, while its Resource Manager dynamically loads hardware plugins and manages their state and command interfaces. citeturn8view1turn3search3turn3search14turn3search29

MoveIt should remain behind `pickcell_motion_server`; neither perception nor grasping should call `move_group` directly. MoveIt supports interchangeable motion-planning plugins and planning pipelines, so the motion server can present one stable domain action while selecting OMPL, Pilz, CHOMP, STOMP, a custom planner, or multiple fallback pipelines internally. citeturn0search10turn0search22turn0search31turn0search28

To avoid premature package proliferation, the initial implementation can combine plugin families:

```text
Initial minimum:
  pickcell_perception_plugins
  pickcell_grasp_plugins
  pickcell_task_plugins

Split later when:
  dependencies conflict,
  GPU deployment differs,
  ownership differs,
  release cadence differs,
  or build/test times become excessive.
```

The package boundary should follow dependency and ownership boundaries—not create one package for every individual class.

## Interface contracts, naming, and data flow

Use existing standard ROS messages whenever their semantics fit. Camera and depth data should use `sensor_msgs/Image`, `sensor_msgs/CameraInfo`, and `sensor_msgs/PointCloud2`. `PointCloud2` supports N-dimensional points plus fields such as normals and intensity. Visual detections should preferentially use `vision_msgs/Detection2DArray` and `vision_msgs/Detection3DArray`; `Detection3D` already supports object hypotheses, poses, a 3D bounding box, and an ID. Poses crossing a subsystem boundary must carry a frame and timestamp, normally through `PoseStamped` or `PoseWithCovarianceStamped`. citeturn2search0turn2search4turn2search5turn2search9turn2search12turn2search14

Custom interfaces belong in their own package rather than being defined beside an implementation. The official ROS 2 custom-interface tutorial likewise separates interface definitions from packages that consume them. citeturn1search0turn1search11

Recommended custom messages include:

```text
pickcell_interfaces/msg/
├── ObjectReference.msg
├── GraspCandidate.msg
├── GraspCandidateArray.msg
├── MotionPlanMetadata.msg
├── SafetyState.msg
├── ModuleStatus.msg
├── SystemStatus.msg
└── TaskState.msg
```

A compact `ObjectReference.msg` could be:

```text
# Select an object without copying a full scene.
string detection_id
string class_id

# Optional pose override, used when use_pose_override is true.
bool use_pose_override
geometry_msgs/PoseWithCovarianceStamped pose_override
```

A grasp candidate should describe all information that motion planning needs, without exposing the grasp algorithm:

```text
unique_identifier_msgs/UUID candidate_id
string object_id
string strategy_id

geometry_msgs/PoseStamped pregrasp_pose
geometry_msgs/PoseStamped grasp_pose
geometry_msgs/PoseStamped retreat_pose

trajectory_msgs/JointTrajectory pregrasp_posture
trajectory_msgs/JointTrajectory grasp_posture

float32 score
float32 estimated_width
float32 required_clearance
string[] tags
```

The recommended public actions are:

| Action | Goal | Result | Feedback |
|---|---|---|---|
| `DetectObjects` | class filters, ROI, requested representation, timeout | detection arrays and snapshot ID | acquisition, preprocessing, inference stage |
| `PlanGrasps` | object reference, gripper, grasp constraints, maximum count | ranked candidates | candidates evaluated and current best score |
| `PlanMotion` | target or candidate, planning group, constraints, scene version | `moveit_msgs/RobotTrajectory` and metadata | planner attempt, current stage and elapsed time |
| `ExecuteTrajectory` | trajectory, expected scene version, execution policy | final state and normalized result | progress, tracking error and controller state |
| `ExecuteTask` | task type, source/target specifications, policy | task outcome and selected solutions | current task stage and retry count |

Recommended supporting services are:

| Service | Purpose |
|---|---|
| `InjectDetections` | Deterministically inject a scene for development and tests |
| `GetCapabilities` | Report algorithms, representations, task types and modes supported by a server |
| `ValidateTrajectory` | Request software-level safety validation before execution |
| `ClearScene` | Clear dynamically perceived objects, with explicit semantics |
| `GetSystemProfile` | Report effective mode, algorithms, versions and configuration hash |

Expose IK through `moveit_msgs/srv/GetPositionIK` or a thin facade with equivalent semantics. MoveIt provides `GetPositionIK`, and its kinematics layer is explicitly plugin-based, which allows KDL, TRAC-IK, IKFast, pick_ik, custom plugins, or even service-backed solvers to be swapped without changing clients. citeturn2search39turn8view5turn5search6turn5search14turn5search22turn5search26

The canonical topics should be hierarchical and relative inside nodes so namespace and remapping rules remain effective:

```text
/sensors/front_camera/image_raw
/sensors/front_camera/camera_info
/sensors/front_camera/depth/image_raw
/sensors/front_camera/points

/perception/detections_2d
/perception/detections_3d
/perception/debug/segmentation
/perception/debug/filtered_points

/scene/objects
/scene/collision_objects
/scene/version

/robot/joint_states
/tf
/tf_static

/safety/state
/system/status
/diagnostics
```

ROS 2 names can be relative or absolute; relative names are expanded under the node namespace, while remapping allows the same implementation to be reused under different system layouts. Therefore, implementation code should create `detections_3d`, not hard-code `/pickcell/perception/detections_3d`. The namespace is supplied at bringup. citeturn3search1turn3search9turn0search9

Use the following naming conventions:

| Element | Convention | Example |
|---|---|---|
| Packages | lowercase snake case with product prefix | `pickcell_grasp_server` |
| Nodes | role, not algorithm name | `grasp_server` |
| Topics | lowercase hierarchical nouns | `perception/detections_3d` |
| Services | imperative or query phrase | `scene/clear`, `system/get_profile` |
| Actions | verb-object phrase | `perception/detect_objects` |
| Parameters | lowercase dotted hierarchy | `algorithm.cluster.tolerance` |
| Message files | UpperCamelCase | `GraspCandidate.msg` |
| Message fields | lowercase snake case | `estimated_width` |
| Plugin IDs | qualified implementation identifier | `pickcell_perception/PclClusterPose` |
| Frames | stable physical names, no mode or algorithm | `world`, `cell`, `base_link`, `tool0`, `camera_optical_frame` |

ROS interface fields are required to follow lowercase alphanumeric and underscore conventions. citeturn3search10turn3search13

The canonical frame hierarchy should remain identical in mock, simulation and hardware:

```text
world
└── cell
    ├── robot_base
    │   └── base_link
    │       └── ...
    │           └── tool0
    │               └── gripper_tcp
    ├── front_camera_link
    │   └── front_camera_optical_frame
    └── bin_frame
```

Do not publish perceived objects in camera coordinates as the public perception result unless the contract explicitly says so. The perception server should transform detections into a configured canonical frame such as `cell` before returning them, while retaining the original acquisition timestamp. This prevents every downstream module from independently implementing transform and timestamp policy.

The main data flow should be:

```text
Sensor backend
    │
    │ sensor_msgs/Image, CameraInfo, PointCloud2
    ▼
Canonical sensor topics
    │
    ▼
Perception server
    │ vision_msgs/Detection3DArray
    ├──────────────────────────────┐
    ▼                              ▼
Scene model                   Grasp server
    │ collision objects            │ GraspCandidateArray
    │                              ▼
    └──────────────────────► Motion server
                                   │ RobotTrajectory
                                   ▼
                           Safety validation
                                   │
                                   ▼
                          Trajectory executor
                                   │ FollowJointTrajectory
                                   ▼
                     ros2_control / vendor system
```

The scene model—not perception—should own conversion into MoveIt collision geometry. MoveIt’s planning scene represents robot state and world collision objects and is used for collision and constraint checking. Keeping that conversion separate lets a perception implementation output image detections, point clusters, meshes, boxes or known-object IDs without importing MoveIt. citeturn5search3turn5search7turn5search15turn5search35

For QoS, use explicit profiles rather than defaults:

| Data | Recommended QoS |
|---|---|
| Images and point clouds | sensor-data QoS: best effort, volatile, shallow history |
| Detections and scene updates | reliable, volatile, `keep_last` |
| Safety and system state | reliable, transient local, depth one |
| Debug visualization | best effort unless loss is unacceptable |
| Commands and actions | reliable through action/service transport |

The standard ROS 2 `SensorDataQoS` profile uses keep-last depth five, best-effort reliability and volatile durability. ROS 2 QoS is configurable specifically to accommodate different reliability, latency and storage requirements. citeturn9search2turn0search5

Every action result should include machine-readable error codes, not only text:

```text
SUCCESS
INVALID_REQUEST
NO_INPUT_DATA
NO_OBJECTS_FOUND
NO_GRASP_FOUND
PLANNING_FAILED
SCENE_CHANGED
SAFETY_REJECTED
CONTROLLER_REJECTED
CANCELED
TIMEOUT
INTERNAL_ERROR
```

Text fields remain useful for diagnostics, but downstream logic must branch on enums.

## Mode switching and algorithm-level variability

The core architectural mechanism is a **two-level substitution model**.

At the subsystem boundary, whole ROS nodes can be replaced because they implement the same topics, services and actions. Inside an implementation, algorithms can be loaded as plugins when they share a meaningful internal contract.

```text
Subsystem substitution:
  mock perception server
  real perception server
  experimental perception server
       all implement DetectObjects

Internal algorithm substitution:
  PclClusterPose
  StereoTriangulation
  RgbDetectorDepthLift
  LearnedSixDoFPose
       all implement PerceptionPipeline
```

The system-level mode should select external adapters and execution backends as follows:

| Module | Mock | Simulation | Real |
|---|---|---|---|
| Sensors | generated frames, static files, rosbag playback | Isaac ROS 2 publishers | vendor camera/depth drivers |
| Perception | injected detections or algorithm over recorded data | same algorithms over simulated sensors | same algorithms over physical sensors |
| Grasp planning | fixed candidates, deterministic test strategy | real algorithm against simulated objects | real algorithm against detected objects |
| Motion planning | canned success/failure or MoveIt fake state | MoveIt against simulated robot/scene | MoveIt against real robot state |
| Execution | validate and simulate progress | simulated `FollowJointTrajectory` | real controller or vendor driver |
| Safety | deterministic policy simulator | software constraints plus simulated interlocks | software monitor plus safety-rated external system |
| Task layer | scripted outcomes or real orchestration | real orchestration | real orchestration |

Isaac Sim’s ROS 2 bridge supports bidirectional communication with ROS topics and services through OmniGraph and Action Graph nodes, making the simulator an adapter at the ROS boundary rather than something that must be imported into perception or planning code. citeturn0search11turn8view3

The configuration should not cause core code to contain repeated logic such as:

```python
if mode == "mock":
    ...
elif mode == "sim":
    ...
elif mode == "real":
    ...
```

Instead, launch selects one provider for each contract:

```text
mode=mock:
  pickcell_mock_nodes/mock_camera
  pickcell_mock_nodes/mock_trajectory_executor

mode=sim:
  pickcell_isaac_sim/bridge_launch
  pickcell_trajectory_executor using simulated controller

mode=real:
  vendor camera driver
  pickcell_robot_hardware hardware plugin
  pickcell_trajectory_executor using real controller
```

Algorithm switching should use one of three mechanisms.

**In-process plugins** are appropriate when implementations share the same input/output semantics, can run in the same language/runtime, and have compatible dependencies. C++ implementations should normally use `pluginlib`, whose model consists of a base interface and dynamically loaded subclasses. MoveIt and ros2_control both use this pattern extensively for planners, kinematics solvers, controllers and hardware components. citeturn1search3turn0search25turn8view1turn8view5

A perception plugin API could be:

```cpp
class PerceptionPipeline
{
public:
  virtual ~PerceptionPipeline() = default;

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const PipelineConfiguration& configuration) = 0;

  virtual PipelineCapabilities capabilities() const = 0;

  virtual DetectionResult detect(
    const SensorBundle& input,
    const DetectionRequest& request) = 0;

  virtual void reset() = 0;
};
```

Do not force every algorithm to accept only a point cloud. Define a tagged `SensorBundle` internally:

```text
SensorBundle
├── optional RGB image
├── optional grayscale image
├── optional depth image
├── optional stereo pair
├── optional organized point cloud
├── camera models
├── timestamps
└── transforms
```

Each plugin declares capabilities:

```yaml
required_inputs:
  - rgb
  - depth
outputs:
  - detections_3d
supports:
  - known_classes
  - instance_ids
  - pose_covariance
```

The server validates that the selected backend can supply those inputs before becoming active.

**Separate ROS nodes** are better than plugins when an algorithm requires a different Python environment, GPU runtime, process isolation, distributed execution, a proprietary SDK, or fundamentally different resource scheduling. For example, a TensorRT detector should not necessarily be loaded into the same process as a lightweight PCL pipeline. Both nodes can implement `DetectObjects`; bringup selects the desired server executable.

**External framework plugins** should be used when the subsystem already has a mature plugin abstraction. MoveIt motion planners and IK solvers should be configured through MoveIt’s native plugin and pipeline mechanisms instead of adding another wrapper plugin layer. MoveIt planning pipelines support planner plugins and pre/post-processing adapters, and its kinematics layer is designed for interchangeable IK implementations. citeturn0search22turn0search31turn8view5

A useful decision table is:

| Variation | Preferred mechanism |
|---|---|
| Different PCL clustering methods | Perception plugin |
| PCL pipeline versus RGB neural network | Separate plugin packages; separate process if runtime differs |
| CPU inference versus GPU inference | Separate deployment or process |
| OMPL versus Pilz | MoveIt planning-pipeline configuration |
| KDL versus TRAC-IK versus IKFast | MoveIt kinematics plugin |
| Geometric versus learned grasping | Grasp plugin or separate server |
| Generic pick versus bin pick | Task strategy plugin |
| Simulated versus real robot | ros2_control hardware/backend substitution |
| Manual object pose versus perception | Canonical detection publisher substitution |

Task variability belongs above grasp and motion planning. A task strategy should orchestrate stable actions rather than contain the perception, grasp or motion algorithms itself.

```cpp
class TaskStrategy
{
public:
  virtual TaskCapabilities capabilities() const = 0;

  virtual TaskPlan build(
    const ExecuteTask::Goal& goal,
    TaskServices& services) = 0;
};
```

Suggested task strategies are:

```text
GenericPickTask
KnownObjectPickTask
BinPickingTask
SingleObjectSceneTask
SceneUnderstandingTask
PickAndPlaceTask
InspectAndPickTask
```

MoveIt Task Constructor is appropriate for manipulation task implementations because it decomposes complex manipulation into stages, passes state between stages, and supports generator, propagator and connector stages. It should live inside task plugins, behind the stable `ExecuteTask` action, rather than becoming the public interface of the whole project. citeturn8view4turn5search4turn5search8turn5search16turn5search24

A bin-picking task plugin might assemble:

```text
CurrentState
→ acquire or select object
→ generate grasp candidates
→ compute IK
→ connect to pregrasp
→ approach
→ close gripper
→ attach collision object
→ lift
→ connect to place
→ lower
→ open gripper
→ detach collision object
→ retreat
```

A known-object task could use the same public action while using a fixed CAD model, template-based perception and object-specific grasp library. Nothing outside the task, perception and grasp implementation packages changes.

Algorithm selection should normally happen at configure time:

```yaml
perception_server:
  ros__parameters:
    algorithm_plugin: pickcell_perception/PclClusterPose
```

For runtime switching, use this sequence:

```text
deactivate
→ unload or destroy current algorithm
→ set validated algorithm parameters
→ instantiate replacement
→ configure replacement
→ run self-test
→ activate
```

Do not hot-swap algorithms during an active task unless the task and state semantics explicitly support it. A task should record the selected algorithm IDs and configuration hash so results are reproducible.

## Configuration and bringup

Configuration should have three layers:

```text
System manifest:
  Which modules run?
  Which mode, algorithm and task profile are selected?

Module configuration:
  What parameters does each selected node or plugin use?

Calibration and cell data:
  What are the robot, camera, gripper and cell-specific physical values?
```

ROS 2 parameters belong to individual nodes and can be supplied at startup or changed at runtime; YAML parameter files address parameters under node names. The top-level system manifest is therefore not itself a ROS parameter file. It is a bringup input that resolves into launch actions and node-specific parameter files. citeturn1search6turn1search27turn1search9

Recommended configuration structure:

```text
pickcell_config/
├── config/
│   ├── systems/
│   │   ├── mock_ci.yaml
│   │   ├── isaac_bin_pick.yaml
│   │   ├── real_known_object.yaml
│   │   └── real_generic_pick.yaml
│   │
│   ├── perception/
│   │   ├── pcl_cluster_pose.yaml
│   │   ├── stereo_pose.yaml
│   │   ├── rgb_depth_lift.yaml
│   │   └── learned_pose.yaml
│   │
│   ├── grasp/
│   │   ├── geometric_antipodal.yaml
│   │   ├── known_object.yaml
│   │   └── learned_grasp.yaml
│   │
│   ├── motion/
│   │   ├── ompl.yaml
│   │   ├── pilz.yaml
│   │   └── multi_pipeline.yaml
│   │
│   ├── tasks/
│   │   ├── generic_pick.yaml
│   │   ├── bin_pick.yaml
│   │   └── known_object_pick.yaml
│   │
│   ├── execution/
│   │   ├── mock.yaml
│   │   ├── isaac.yaml
│   │   └── real.yaml
│   │
│   ├── safety/
│   │   ├── development.yaml
│   │   └── production.yaml
│   │
│   └── qos/
│       └── default_qos.yaml
│
├── calibration/
│   ├── cell_a/
│   │   ├── camera_intrinsics.yaml
│   │   ├── hand_eye.yaml
│   │   ├── tool_frames.yaml
│   │   └── workspace_limits.yaml
│   └── cell_b/
│
├── schema/
│   ├── system.schema.json
│   ├── perception.schema.json
│   └── task.schema.json
└── package.xml
```

A system manifest should be declarative:

```yaml
system:
  namespace: pickcell
  mode: sim
  deployment: development
  use_sim_time: true

modules:
  perception:
    enabled: true
    provider: pickcell_perception_server
    backend: isaac_rgbd
    algorithm: pcl_cluster_pose
    parameters: perception/pcl_cluster_pose.yaml

  scene_model:
    enabled: true
    parameters: scene/default.yaml

  grasp:
    enabled: true
    provider: pickcell_grasp_server
    algorithm: geometric_antipodal
    parameters: grasp/geometric_antipodal.yaml

  motion:
    enabled: true
    provider: pickcell_motion_server
    strategy: moveit
    planning_pipeline: ompl
    parameters: motion/ompl.yaml

  execution:
    enabled: true
    provider: pickcell_trajectory_executor
    backend: ros2_control
    parameters: execution/isaac.yaml

  task:
    enabled: true
    algorithm: bin_pick
    parameters: tasks/bin_pick.yaml

  safety:
    enabled: true
    parameters: safety/development.yaml
```

A module parameter file remains normal ROS 2 YAML:

```yaml
perception_server:
  ros__parameters:
    canonical_frame: cell
    algorithm_plugin: pickcell_perception/PclClusterPose
    input_timeout: 0.5
    publish_debug: false

    algorithm:
      voxel_leaf_size: 0.005
      remove_plane: true
      plane_distance_threshold: 0.008
      cluster_tolerance: 0.015
      min_cluster_points: 100
      max_cluster_points: 50000
```

Use typed parameter declarations and reject invalid configurations during lifecycle configuration. `generate_parameter_library` can generate C++ or Python parameter declarations, accessors, validation and documentation from declarative YAML, which is useful for maintaining a large, research-oriented parameter surface. citeturn7search0turn7search1

The separation of responsibilities should be strict:

```text
pickcell_config:
  Contains values and schemas.
  Does not instantiate nodes.

pickcell_bringup:
  Reads the system manifest.
  Resolves packages, launch files and parameter files.
  Does not implement robotics algorithms.

Runtime packages:
  Declare and validate their parameters.
  Do not inspect the top-level system manifest.
  Do not launch other high-level modules.
```

Bringup should use Python launch files because conditional assembly, manifest parsing and lifecycle sequencing are easier to express programmatically. ROS 2 launch supports Python, XML and YAML, and is intended to describe programs, arguments, configuration, namespaces and process composition. citeturn1search13turn3search24turn9search6turn9search11

Recommended launch files:

```text
pickcell_bringup/launch/
├── system.launch.py
├── foundation.launch.py
├── perception.launch.py
├── manipulation.launch.py
├── execution.launch.py
├── safety.launch.py
├── visualization.launch.py
└── development_tools.launch.py
```

`system.launch.py` should:

```text
read system manifest
→ validate schema
→ resolve absolute paths
→ set namespace and use_sim_time
→ include robot description and TF
→ launch controller or simulation backend
→ launch safety supervisor
→ launch sensor providers
→ launch requested subsystem servers
→ configure lifecycle nodes
→ verify dependencies and capabilities
→ activate data producers
→ activate consumers
→ publish effective system profile
```

A suitable activation order is:

```text
Robot description and static transforms
→ robot state and controllers
→ safety supervisor
→ sensor adapters
→ scene model
→ perception
→ grasp server
→ motion server
→ trajectory executor
→ task server
```

The exact order can vary, but the task server should not accept work until all capabilities required by the selected task profile are active.

Each module launch file should be reusable independently:

```bash
ros2 launch pickcell_perception_server perception.launch.py \
  input_profile:=rosbag \
  algorithm:=pcl_cluster_pose

ros2 launch pickcell_motion_server motion.launch.py \
  scene_source:=synthetic \
  planning_pipeline:=ompl

ros2 launch pickcell_trajectory_executor executor.launch.py \
  backend:=mock
```

Simulation time must be applied consistently to every participating node. Do not set `use_sim_time` in individual algorithm files; inject it as a system-wide launch parameter. ROS 2 nodes expose `use_sim_time`, while simulation environments publish `/clock`. citeturn7search2turn7search10turn7search25

Calibration data should not be mixed with algorithm tuning. Camera intrinsics, hand-eye transforms, tool-center points and joint offsets describe the physical cell. Voxel sizes, confidence thresholds and planner timeouts describe software behavior. Keeping them separate prevents an algorithm experiment from silently modifying hardware calibration.

Configuration precedence should be explicit:

```text
package defaults
< selected module profile
< cell calibration
< system manifest overrides
< command-line overrides
```

The effective merged configuration should be written to the log or exported with a configuration hash. That is essential for reproducible research and for diagnosing differences between simulation and hardware.

## Internal package structure and testing strategy

Every runtime package should separate ROS transport from domain logic.

A Python package should look like:

```text
pickcell_grasp_server/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── pickcell_grasp_server
├── pickcell_grasp_server/
│   ├── __init__.py
│   ├── node.py
│   ├── lifecycle.py
│   ├── action_server.py
│   ├── conversions.py
│   ├── validation.py
│   ├── errors.py
│   └── main.py
├── config/
│   └── defaults.yaml
├── launch/
│   └── grasp.launch.py
└── test/
    ├── test_validation.py
    ├── test_conversions.py
    ├── test_action_contract.py
    └── test_launch.py
```

The node class should be thin:

```python
class GraspServerNode(LifecycleNode):
    """ROS lifecycle, parameters, action handling, diagnostics."""

    def execute_callback(self, goal_handle):
        request = self._converter.from_ros(goal_handle.request)
        result = self._strategy.plan(request)
        return self._converter.to_ros(result)
```

The actual algorithm should be ROS-independent where practical:

```python
class GraspStrategy(Protocol):
    def configure(self, config: GraspConfiguration) -> None: ...
    def plan(self, request: GraspRequest) -> GraspResult: ...
```

A C++ package should similarly separate component/node code from libraries:

```text
pickcell_perception_server/
├── include/pickcell_perception_server/
│   ├── perception_server.hpp
│   ├── sensor_synchronizer.hpp
│   └── conversions.hpp
├── src/
│   ├── perception_server.cpp
│   ├── sensor_synchronizer.cpp
│   ├── conversions.cpp
│   └── main.cpp
├── launch/
├── config/
└── test/
```

Use C++ for real-time-sensitive execution, ros2_control integration, high-throughput point-cloud processing and mature MoveIt/pluginlib integration. Python remains appropriate for orchestration, research algorithms, learned-model wrappers, evaluation tools and many perception prototypes. A mixed-language system is not a modularity problem when the ROS contracts remain stable.

Testing should operate at several layers.

| Test layer | Scope | Examples |
|---|---|---|
| Pure unit tests | Algorithm and validation without ROS | cluster filtering, candidate scoring, constraint construction |
| Interface contract tests | One server against a generic client | action cancellation, error codes, frame validation |
| Component integration | Related nodes launched together | sensor adapter to perception |
| Subsystem tests | One module with mocked neighbors | motion server with synthetic scene |
| Full pipeline tests | Mock or simulation system | complete pick-and-place outcome |
| Hardware-in-loop | Real controller and selected sensors | tracking, timing and recovery |
| Benchmark tests | Accuracy and performance regression | pose error, success rate, latency |

ROS 2 provides `launch_testing` and `launch_pytest` for tests involving multiple launched processes. These should be used to validate action contracts, lifecycle transitions, namespace behavior, remapping, QoS compatibility and clean shutdown. citeturn4search0turn4search8turn4search12turn4search27

Use rosbag datasets as first-class test fixtures. ROS 2 supports recording and replaying topic data, and Jazzy also supports recording service data. This enables perception development against repeatable sensor sequences and end-to-end regression without physical hardware. citeturn4search2turn4search6turn4search10turn4search20

Required development workflows become straightforward:

```text
Develop IK without perception:
  pose injector
  → grasp fixture or manual target
  → motion server
  → mock executor

Develop motion planning:
  synthetic collision scene
  → PlanMotion action
  → trajectory visualizer or mock executor

Develop perception:
  rosbag or image dataset
  → perception server
  → detection evaluator
  no robot, grasp or motion packages

Develop grasp planning:
  InjectDetections service
  → grasp server
  → candidate visualization

Develop execution:
  canned RobotTrajectory
  → safety supervisor
  → simulated or hardware executor

Run partial pipeline:
  launch only the desired server and its explicit providers
```

Each public contract should have a reusable conformance test in `pickcell_test_support`. Any replacement perception server should pass the same test suite:

```text
accepts a valid goal
rejects unsupported input representation
reports capabilities
returns frame-stamped detections
supports cancellation
times out predictably
uses documented error codes
publishes diagnostics
does not require other high-level packages
```

CI should build and test package subsets as well as the complete workspace:

```text
interface CI:
  build pickcell_interfaces and verify compatibility

perception CI:
  build perception packages and mocks only

planning CI:
  build planning, description, MoveIt config and mocks

control CI:
  build ros2_control hardware and executor packages

system CI:
  launch complete mock pipeline

simulation CI:
  run selected Isaac Sim tests on GPU-capable workers
```

Track package maturity using criteria inspired by REP-2004: version policy, change control, documentation, testing, dependency stability, platform support and security practices. REP-2004 defines quality categories intended to communicate ROS package maturity. citeturn3search0

Interface compatibility deserves special governance:

```text
Patch release:
  bug fix, no contract change

Minor release:
  backward-compatible fields or capabilities

Major release:
  breaking action, service, message or semantic change
```

Adding a field to a ROS message may still affect recorded data, bridges and downstream language bindings. Treat message evolution as API evolution, and prefer adding optional semantics or a new interface version over repeatedly redesigning the same message.

Performance optimizations should come after profiling. A sensible progression is:

```text
separate nodes and standard messages
→ measure latency and CPU
→ enable intra-process composition for selected C++ components
→ reduce unnecessary representation conversions
→ add GPU or zero-copy paths only where justified
```

High-bandwidth sensor nodes may be composed, while task orchestration, safety and execution remain isolated. Composition is a deployment choice; it must not become an architectural dependency.

## Safety, reliability, and production blueprint

The safety supervisor should be treated as a **software operational-safety layer**, not as the safety-rated control system for an industrial cell.

Industrial robot safety is governed by risk assessment and applicable machinery standards. ISO 10218-1:2025 addresses safety requirements for industrial robots, while ISO 10218-2:2025 covers robot applications and complete robot cells. ISO 13849-1:2023 covers the design and integration of safety-related control-system parts, including software. A normal ROS 2 node should not be assumed to satisfy a required performance level merely because it monitors limits or publishes stop commands. citeturn8view7turn6search5turn6search1

The production safety architecture should therefore have two layers:

```text
Safety-rated layer outside ordinary ROS:
  emergency stop
  guard doors
  safety scanner
  enabling device
  safe torque off
  safe speed / safe position where supported
  certified safety PLC or robot safety controller

ROS operational-safety layer:
  workspace and joint constraints
  trajectory validation
  scene freshness
  speed-scaling requests
  health and heartbeat monitoring
  stale-data detection
  controller-state monitoring
  task cancellation
```

The software execution path should enforce:

```text
PlanMotion result
→ trajectory structural validation
→ joint, velocity, acceleration and timing checks
→ collision validation against current scene
→ scene-version consistency check
→ safety authorization
→ controller readiness check
→ execution
→ continuous monitoring
```

The executor, not the task server, must be the final software gate. No package should be able to bypass that gate by directly commanding the real controller under normal operation. Administrative test tools that intentionally bypass it should be separately packaged, visibly named, disabled in production manifests and protected by deployment policy.

`ros2_control` supports controller and hardware lifecycle management, command/state interfaces and controller-manager supervision. Current Jazzy controller-manager documentation also exposes command-limit enforcement and fallback-controller concepts, but those features supplement rather than replace a cell-level safety design. citeturn3search3turn6search4turn8view1

The safety supervisor should monitor at least:

| Condition | Response |
|---|---|
| Stale joint state | Reject new execution; request stop |
| Stale scene or transform | Reject planning or execution |
| Joint limit violation | Stop and fault |
| Excess tracking error | Cancel controller and fault |
| Unexpected controller change | Stop task |
| Sensor heartbeat loss | Degrade or stop according to task policy |
| Human-presence interlock | Forward to safety-rated layer; stop task |
| Scene changed after planning | Revalidate or replan |
| Invalid timestamp ordering | Reject data |
| Repeated planner/executor failures | Enter recoverable or latched fault |

`SafetyState` should be transient-local so late-joining nodes immediately receive the current state:

```text
uint8 UNKNOWN=0
uint8 SAFE=1
uint8 DEGRADED=2
uint8 STOP_REQUESTED=3
uint8 FAULT=4
uint8 EMERGENCY_STOP=5

uint8 state
string[] active_reasons
float32 requested_speed_scaling
builtin_interfaces/Time last_transition
bool execution_authorized
```

Diagnostics and lifecycle status should be aggregated separately from safety authorization. ROS diagnostic aggregation supports grouping robot diagnostics by subsystem, which is useful for operations dashboards, but diagnostic severity alone should not implicitly authorize motion. citeturn4search18

The final dependency architecture should be:

```text
Layer: Interfaces
  pickcell_interfaces
  standard ROS messages

Layer: Models and plugin APIs
  perception_core
  grasp_core
  task_core
  description

Layer: Implementations
  perception plugins
  grasp plugins
  task plugins
  hardware plugins

Layer: ROS facade servers
  perception_server
  grasp_server
  motion_server
  task_server
  scene_model
  trajectory_executor
  safety_supervisor

Layer: Deployment
  config
  bringup
  Isaac Sim
  hardware bringup
  mocks

Dependency direction:
  deployment → servers → core APIs → interfaces

Never:
  interfaces → implementation
  core API → server
  subsystem server → another subsystem implementation
```

A pragmatic implementation sequence is:

| Phase | Packages | Outcome |
|---|---|---|
| Contract foundation | interfaces, config, description, test support | Stable API and frame conventions |
| Mock vertical slice | mock nodes, task server, grasp server, motion server, mock executor | Full pipeline without sensors or robot |
| Planning foundation | MoveIt config, scene model, motion server | Planning against injected objects |
| Control integration | control, trajectory executor, simulated hardware | Trajectory execution through ros2_control |
| Perception baseline | perception core/server, PCL plugin, rosbag tests | Repeatable geometric perception |
| Isaac integration | Isaac package and simulated controllers/sensors | Simulation-based full pipeline |
| Hardware integration | sensor and robot hardware packages | Real-cell deployment |
| Algorithm expansion | ML perception, learned grasping, task variants | Research variability without API changes |
| Production hardening | safety supervision, diagnostics, HIL tests, release policy | Operationally maintainable system |

The minimum research-grade architecture does not require implementing every plugin package immediately. It does require establishing the contracts, dependency direction, mock providers, lifecycle behavior and configuration semantics before subsystem algorithms proliferate.

The defining characteristics of the completed architecture are:

```text
Perception can be removed and replaced by pose injection.

Grasp planning can consume detections from any conforming source.

Motion planning can be tested using synthetic targets and scenes.

Execution can switch between mock, Isaac Sim and real controllers.

Algorithms can change within a subsystem without changing its public API.

Task types can change without embedding task policy into perception or motion.

Bringup chooses implementations; runtime packages do not choose the system.

Safety authorization remains in the execution path.

Every subsystem can be built, launched and tested with mocked neighbors.
```

This is the appropriate balance between a prototype and an over-engineered framework: stable external contracts, small facade servers, plugin APIs only where algorithm families genuinely share semantics, separate processes where dependencies or failure domains differ, and a declarative bringup layer that assembles the system without leaking deployment choices into algorithm code.