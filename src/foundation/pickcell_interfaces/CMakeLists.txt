cmake_minimum_required(VERSION 3.8)
project(pickcell_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

set(message_files
  "msg/ActiveBag.msg"
  "msg/CycleResult.msg"
  "msg/GripperCommand.msg"
  "msg/GripperResult.msg"
  "msg/PickPlaceCycle.msg"
  "msg/ErrorCode.msg"
  "msg/GraspCandidate.msg"
  "msg/GraspCandidateArray.msg"
  "msg/ModuleStatus.msg"
  "msg/MotionPlanMetadata.msg"
  "msg/ObjectReference.msg"
  "msg/RuntimeMode.msg"
  "msg/SafetyState.msg"
  "msg/SystemStatus.msg"
  "msg/TaskState.msg"
)

set(service_files
  "srv/ClearScene.srv"
  "srv/GetCapabilities.srv"
  "srv/GetModuleStatus.srv"
  "srv/GetSystemProfile.srv"
  "srv/InjectDetections.srv"
  "srv/ValidateTrajectory.srv"
)

set(action_files
  "action/DetectObjects.action"
  "action/ExecuteTask.action"
  "action/ExecuteTrajectory.action"
  "action/PlanGrasps.action"
  "action/PlanMotion.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${message_files}
  ${service_files}
  ${action_files}
  DEPENDENCIES
    builtin_interfaces
    diagnostic_msgs
    geometry_msgs
    moveit_msgs
    sensor_msgs
    std_msgs
    trajectory_msgs
    unique_identifier_msgs
    vision_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
