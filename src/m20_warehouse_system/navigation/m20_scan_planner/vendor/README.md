# Vendored SCAN planning dependencies

This directory contains the source snapshots currently required by the M20
planner: `bspline_opt`, `plan_env`, `path_searching`, `traj_utils`, and
`scan_planner_msgs`. They are kept under `COLCON_IGNORE` while the M20 build is
being migrated to an internal implementation, so they cannot create duplicate
ROS packages during normal workspace discovery.

The files are retained as an intermediate, auditable compatibility layer. The
M20 planner must not depend on the external checkout once the vendor build
transition is complete. Preserve each package's upstream license and notices
when distributing this source tree.
