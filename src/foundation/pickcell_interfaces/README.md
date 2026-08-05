# PickCell interfaces

This package is the implementation-independent API boundary for PickCell. It must not depend on any PickCell server or algorithm package.

All long-running operations use actions and expose progress plus cancellation through the ROS action protocol. Every action and service result carries `ErrorCode`; clients must branch on its numeric `code`, using `message` only for diagnostics.

Poses crossing a subsystem boundary must retain their acquisition timestamp and use a documented canonical frame, normally `cell`. Scene-sensitive requests carry an expected scene version so servers can return `SCENE_CHANGED` instead of acting on stale state.

Changes to these definitions are API changes. Patch releases must not change contracts, minor releases may add backward-compatible capability, and breaking changes require a major version or a new versioned interface.
