# PickCell motion server

The package has three active executables:

- `motion_cycle` accepts one source snapshot and publishes explicit
  `ActiveBag` and `PickPlaceCycle` contracts.
- `trajectory_executor` owns all arm commands and the guarded phase machine.
- `bag_transfer_visualizer` is both the stateful bag display and geometrically
  honest mock-gripper confirmation producer.

## Motion contract

The only persistent task anchors are A_PICK above the pallet and B_DROP above
the receiving box. Their values come from the shared `two_anchor_task` group in
`application.yaml`. A cycle executes:

```text
A_PICK -> temporary bag hover -> Cartesian contact -> grasp
       -> temporary lift -> exact A_PICK joint branch
       -> fixed lateral sweep -> B_DROP -> release
       -> freshly timed reverse sweep -> A_PICK
```

Temporary poses are derived from the accepted surface point. They are not
operator-managed anchors. The fixed transfer requires monotonic configured
`joint_1` base yaw, bounded joints 2-5, bounded axial `joint_6` compensation,
full sample collision/FK validation, and a TCP height envelope. There is no
planner fallback for the fixed sweep.

## Correlation and recovery

Cycle, trajectory, and gripper command IDs are separate. Sensor stamps remain
sensor stamps. Every result must match its owning cycle and command. The cycle
gate also rejects a retained source frame/stamp after terminal cleanup, closing
the cross-topic race before perception publishes a newer snapshot.

Before-grasp known-empty failures return to A_PICK and release the selection
lock. A held or unknown payload enters safe stop. A failed A_PICK recovery also
safe-stops rather than retrying without bound. Restarting the mock launch after
inspection is the supported held-payload recovery; no success is fabricated.

## Simulation boundary

The mock gripper checks fresh executed joints, actual TCP transform, 25 mm
position error, 8 degree suction-axis error, contact geometry, exact selected
bag mapping, and box-relative release geometry. It preserves the bag's
tool-relative pose while carried. Bags are not MoveIt attached collision
objects and are not physically attached in Isaac. See
`docs/deep-debug-report.md` for measured execution evidence and limitations.
