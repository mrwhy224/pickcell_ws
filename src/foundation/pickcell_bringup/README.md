# PickCell bringup

`pickcell_bringup` is the only package that selects the complete deployed system. It creates one `ConfigResolver`, resolves every selected module profile, calculates a reproducibility hash, and supplies each node with its final ROS parameter dictionary.

A live `ConfigResolver` object is intentionally not passed to runtime packages: nodes normally run in separate processes, where Python object sharing is impossible and would violate the ROS interface boundary. Runtime packages receive ordinary typed ROS parameters instead and never read the system manifest themselves.

## Launch

```bash
ros2 launch pickcell_bringup system.launch.py \
  mode:=sim \
  perception_profile:=pcl_cluster_pose \
  grasp_profile:=geometric_antipodal \
  motion_profile:=ompl \
  tasks_profile:=bin_pick \
  execution_profile:=isaac \
  safety_profile:=simulation
```

`system.launch.py` currently starts the foundation description. As runtime packages are added, their launch assembly should use `configured_node`:

```python
node = configured_node(
    resolved,
    section="perception",
    package="pickcell_perception_server",
    executable="perception_server",
    node_name="perception_server",
)
```

The node receives its selected section parameters plus these system values:

- `system_mode`
- `use_sim_time`
- `configuration_hash`

Mode, profile, backend, and algorithm remain separate selections.
