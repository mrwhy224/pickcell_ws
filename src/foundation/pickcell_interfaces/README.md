# PickCell interfaces

This package is the implementation-independent API boundary for PickCell. It must not depend on any PickCell server or algorithm package.

All long-running operations use actions and expose progress plus cancellation through the ROS action protocol. Every action and service result carries `ErrorCode`; clients must branch on its numeric `code`, using `message` only for diagnostics.

Poses crossing a subsystem boundary must retain their acquisition timestamp and use a documented canonical frame, normally `cell`. Scene-sensitive requests carry an expected scene version so servers can return `SCENE_CHANGED` instead of acting on stale state.

Changes to these definitions are API changes. Patch releases must not change contracts, minor releases may add backward-compatible capability, and breaking changes require a major version or a new versioned interface.

## Runtime mode declaration

Every runtime module must publish its `ModuleStatus` on the relative `status` topic and serve `GetModuleStatus` on the relative `get_status` service. Bringup supplies the selected `RuntimeMode` (`MOCK`, `SIM`, or `REAL`); a package must not infer it from its backend or algorithm.

`ModuleStatus` reports these independent dimensions explicitly:

- `mode`: external environment (`MOCK`, `SIM`, or `REAL`)
- `deployment`: deployment profile or topology
- `implementation`: selected provider/plugin implementation
- `backend`: input, simulator, controller, or hardware backend
- `algorithm`: selected algorithm or task policy
- `use_sim_time`: effective clock policy
- `configuration_hash`: reproducibility identifier

Library-only packages do not run a node and therefore do not publish status. The runtime node that loads them reports their selected implementation in its own status.
