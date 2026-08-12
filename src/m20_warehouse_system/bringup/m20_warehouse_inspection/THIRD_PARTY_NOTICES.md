# Third-party dependency record

This file records dependencies selected for the independently buildable M20 warehouse
inspection project. It does not replace the license or NOTICE files shipped by each
dependency.

## Deep Robotics M20 model

- Source: `https://github.com/DeepRoboticsLab/deep_robotics_model.git`
- Pinned revision: `6113c62da96295e8d53abbc079af5296bf4649f8`
- License: BSD 3-Clause
- Actual phase-2 use: byte-identical vendor URDF/mesh archive plus a traceable ROS 2
  URI/sensor wrapper
- Integration rule: ROS, sensor, and Gazebo additions are maintained in separate wrapper
  files so the official source asset remains identifiable.

## SCAN-Planner ROS 2 community branch

- Source: `https://github.com/wuyi2121/SCAN-Planner.git`
- Pinned revision: `d0b921c9b05a6d291d144d60882b2e0e88d2c0e0`
- Repository root license: Apache License 2.0
- Selected planner package manifests (`plan_env`, `path_searching`, `bspline_opt`,
  `traj_utils`, `scan_planner_msgs`): Apache-2.0
- Selected `local_sensing_node` package manifest: GPL-3.0-only
- Actual phase-2 use: GridMap, Dynamic A*, B-spline optimization, trajectory messages,
  local sensing simulation, and project-owned M20 ground-control adaptations
- Integration rule: project changes must retain Apache notices and carry an explicit
  modification record.

The repository-level license and the `local_sensing_node` package declaration are not
identical. Source and binary redistribution must preserve both upstream notices and
review the GPL-3.0-only obligations for the local-sensing component. Replacing the
simulation local-sensing backend with a project-owned or real-lidar backend is also an
explicit real-robot migration boundary; this record does not provide legal advice.

## Deep Robotics M20 sdk_deploy

- Source: `https://github.com/DeepRoboticsLab/sdk_deploy.git`
- Audited local base revision: `ee289d475f2dedf0332b7542f2b173fa9e8d1456`
- License: BSD 3-Clause
- Audited policy SHA-256:
  `e63169b7727d197abc952c626d02458f31859a627f811660c14a0048fa4f5302`
- Integration rule: the policy, joint calibration, state machine, hardware interface, and
  bundled runtime remain a separately pinned vendor dependency. Project navigation and
  Gazebo changes live in `m20_locomotion_control` and a future Gazebo joint bridge.

The audited vendor checkout contains local changes and is not yet the clean SDK profile
lock. A release profile must record a clean upstream snapshot and an explicit patch set.

## Optional real-robot dependencies

The following repositories are intentionally excluded from the RViz baseline dependency
lock. They will be documented in profile-specific manifests when the corresponding phase
starts:

- Deep Robotics M20 Lightning-LM adaptation
- RoboSense `rslidar_sdk` and `rslidar_msg`

This separation prevents real-robot and low-level control dependencies from becoming
mandatory for flat-region RViz simulation.
