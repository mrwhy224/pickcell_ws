# PickCell

PickCell is a contract-oriented ROS 2 workspace for a modular industrial pick-and-place system. Subsystems communicate through stable ROS interfaces so perception, grasping, planning, execution, simulation, and hardware implementations can evolve independently.

The architectural roadmap is in [`deep-research-report.md`](deep-research-report.md), and implementation progress is tracked in [`TASKS.md`](TASKS.md).

## Platform

The project target is Ubuntu 24.04 with ROS 2 Jazzy. The initial foundation is also kept source-compatible with ROS 2 Humble where the required message dependencies are installed.

## Foundation packages

- `pickcell_interfaces`: public messages, services, actions, and normalized errors.
- `pickcell_description`: canonical frames and the current placeholder cell model.
- `pickcell_config`: placeholder profiles, manifests, schemas, and configuration resolution.
- `pickcell_bringup`: centralized mode/profile resolution and top-level launch assembly.
- `pickcell_test_support`: deterministic ROS fixtures and reusable contract assertions.
- `pickcell_mock_nodes`: deterministic point-cloud, final-point, and target-point publishers for path-planning development.

The configuration files intentionally remain placeholders during the contract-foundation milestone. They will be populated and schema-validated in a later pass.

## Build and test

Install dependencies with `rosdep`, then build with `colcon`:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

For a focused foundation build:

```bash
colcon build --symlink-install --packages-up-to pickcell_test_support
```
