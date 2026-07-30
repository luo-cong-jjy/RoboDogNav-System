# Changelog

## Unreleased - 2026-07-30

- Added an M20-specific heading-alignment controller profile without changing
  upstream SCAN planning or visualization: translation now slows from 0.15 rad
  of tangent error, freezes above 0.55 rad, and resumes below 0.20 rad after a
  0.40 s minimum hold.
- Strengthened rolling-path tracking with a 1.20 lateral-to-yaw gain, a
  0.02 rad/s cruise deadband, 0.05 m/s high-curvature crawl, and turn
  hysteresis. Pure-yaw requests always suppress translation.
- Aligned the conservative SCAN hard radius and independent guard at 0.30 m
  while preserving the 0.45 m nominal per-side planning envelope and the
  0.90 m minimum scene spacing. Larger 0.32/0.35 m raster radii were rejected
  because they close the validated passage at 0.05 m resolution.
- Added collision-checked yaw-only recovery for predicted-footprint stops.
  Current-footprint occupancy and all other hard safety holds still produce an
  unconditional zero command.
- Extended the navigation probe with controller heading error/alignment,
  locomotion-mode transitions, recovery events, and explicit MuJoCo obstacle
  contact metrics. The tuned 0.90 m passage completed in 31.21 s with 75.9 mm
  minimum guard clearance and no guard event or physical obstacle contact.
- Promoted the validated `0.90 m + conservative` bundle to the production
  defaults: the complete warehouse launch selects that bundle, and the
  raw/native collision guard fallback now uses the tested 0.25 m radius,
  0.05 m margin, and 0.70 s lookahead. Isolated benchmark selectors and the
  optional robust grid-route envelope remain independent.
- Completed official-M20 MuJoCo dynamic clearance trials: 0.75 m remains
  safely rejected, 0.80 m has only 2.3 mm measured guard clearance, and
  0.90 m passed three cold starts plus the tuned regression with about
  52.3 mm minimum guard clearance and no obstacle contact.
- Added deterministic single-obstacle double-bypass/direct-bypass profiles,
  an installed dynamic-clearance probe, continuous double-circle clearance
  metrics, and latched MuJoCo obstacle-contact event/force diagnostics.
- Tuned only the production conservative collision lookahead from 1.00 s to
  0.70 s. The direct obstacle bypass changed from a 110 s predicted-stop
  timeout to a 28.24 s clean pass, while the 0.75 m negative test remained
  stopped before contact. Footprint radius, margin, and 0.45 m optional route
  inflation were not reduced.
- Moved autonomous rolling adaptation ahead of collision prediction and
  safety gating, introduced `/m20/navigation/cmd_vel_candidate`, and disabled
  duplicate adaptation in the final SDK gate.
- Added a latched execution-hold contract that freezes both closed-loop and
  SCAN local-trajectory time; recovery replans now start at measured odometry
  with zero velocity/acceleration instead of stale B-spline derivatives.
- Replaced the collision guard's centre-circle check with SCAN-aligned
  front/rear circles and added first-blocking-sample diagnostics.
- Repeated the previously failing F1 lower-left leg in RViz and official-SDK
  MuJoCo: final errors were 0.164 m and 0.060 m, with zero local A-star
  failures, zero occupied-first-control-point warnings, and zero runtime
  collision-stop entries.
- Added a navigation-only rolling adapter between the safe SCAN command and
  the official M20 ONNX policy. It converts autonomous lateral tracking into
  yaw correction, adds turn hysteresis and cruise-yaw smoothing, preserves
  explicit manual lateral motion, and resets immediately on safety holds.
- Repeated the same six-goal MuJoCo A/B route: 6/6 goals passed with no
  collision stop, automatic `LATERAL_MANEUVER` fell from 63.81% to 0%, total
  time fell from 175.59 s to 101.59 s, mean final error fell from 0.162 m to
  0.092 m, and straight-line leg-motion RMS fell from 1.274 to 0.319 rad/s.
- Added a repeatable six-goal MuJoCo navigation-motion probe that records the
  planner, three Twist stages, body pose, joint velocities, locomotion mode,
  contacts, and collision stops. The baseline confirms simultaneous wheel-leg
  actuation and identifies excessive lateral-intent classification as the main
  adjustable source of visibly busy motion.
- Raised the formal `dense_four_corner` default obstacle-body separation from
  0.70 to 0.90 m, regenerated both deterministic floor assets without reducing
  the 246-obstacle count, and re-froze their hashes. The realized minimum
  boundary separation is 0.900236 m on both floors.
- Added explicit vendor/tight/balanced/conservative clearance profiles shared
  by native SCAN, optional grid A*, and the independent collision guard.
- Added six no-bypass doorway benchmarks from 0.60 to 0.90 m, deterministic
  0.05 m PCD/occupancy assets, a MuJoCo selector launch, and a clearance report.
- Separated physical, continuous planner, occupancy-raster, and dynamic
  passage limits; the normal 0.10 m-map system now defaults to the 0.90 m
  conservative profile.
- Recorded 0.75 m as the first static connected 0.05 m doorway and kept the
  dynamic MuJoCo threshold explicitly pending instead of treating connectivity
  as a physical pass.

- Added isolated `0.70 / 0.80 / 0.90 / 1.00 m` obstacle-spacing sweep
  profiles, deterministic assets, one selector launch, and regeneration tool.
- Added full static `0.30 m` inflated-connectivity regression across all 11
  mission legs and recorded native-SCAN first-leg dynamic comparisons.
- Added an isolated `route_challenge` profile with 240 deterministic random
  obstacles and 12 varied fixed obstacles that intersect all seven nominal
  mission-leg classes while retaining 0.30 m footprint-inflated connectivity.
- Kept the challenge profile outside the frozen stable-profile hash set so
  planner experiments cannot silently modify the accepted RViz baseline.

## 1.3.0 - 2026-07-28

- Froze the RViz v1 stable configuration, PCD/PGM assets, official M20 URDF,
  dependency release, and public control contracts in a checked SHA-256
  manifest.
- Added an isolated `dense_four_corner` profile and launch entry with 246 obstacles
  per scene: 240 deterministic random obstacles plus six configured 1 m aisle
  obstacles, all covered by a configurable 0.70 m minimum-separation rule.
- Made the navigation gateway wait for its own reliable floor-switch hold-release
  sample before evaluating a newly committed-floor goal, preventing a DDS
  subscriber-delivery race at the first target on the new floor.
- Compacted native SCAN PointCloud2 samples to standard 16-byte XYZI records,
  added optional reliable simulation delivery, and stopped forcing the WSL
  loopback interface that silently dropped matched local-cloud samples.
- Locked a second SCAN integration patch so isolated workspaces reproduce the
  current planner, floor-reset, map-reload, and local-sensing behavior.
- Added four corner inspection points per scene in lower-left, lower-right,
  upper-right, upper-left order.
- Extended terminal locations to support a configured floor initial pose, allowing
  the dense mission to switch F2→F1 and return to the entire workflow start.
- Parameterized the complete SCAN/mission launch graph by system profile while
  retaining the original verified profile as the default.
- Joined F1/F2 at the shared origin, opened matching physical PCD/PGM gateways, and
  published both scenes from the initial RViz snapshot.
- Replaced the default simulated pose transfer with an in-place, zero-jump active-map
  transaction at `(0, 0)`.
- Kept F2 as an interior replica of F1 while allowing the two opposite gateway boundaries
  to have different complete-file hashes.
- Expanded the default mission to inspect both F2_A and F2_B, return through a no-dwell
  transit waypoint, and finish at the shared-origin F2 start point.
- Made phase-4 and phase-5 mission-step acceptance derive the expected count from the
  configured sequence instead of assuming five steps.
- Regenerated the maps after the route change and confirmed all PCD/PGM hashes and both
  180-obstacle layouts are unchanged.

## 0.9.0 - 2026-07-27

- Unified the operator workflow around one complete-system launch: RViz manual
  `2D Goal Pose` remains immediately available, and a new `m20_start_inspection`
  client triggers the waiting mission executor without launching a second graph.
- Kept native SCAN as the default for manual and diagnostic entry points; the complete
  automatic mission retains its grid-A* long-range guide after pure-native runtime trials
  safely stopped at a static-obstacle topology that the local planner did not bypass.
- Added a native controller profile whose closed-loop parameters match the pinned upstream
  SCAN-Planner controller before the independent M20 safety/backend limits.
- Extended the native parameter contract to compare every upstream key except documented M20
  geometry/frame overrides, including the original `optimization.dist0`.
- Restored the upstream RViz styles for the SCAN global map, input sensor cloud, occupancy,
  inflated occupancy, and executed path while retaining separate warehouse overlays.
- Split the independent collision guard by navigation profile: native SCAN uses its 0.25 m
  lateral body cylinder plus 0.05 m margin, while grid-route mode retains the 0.50 m envelope.
- Clipped collision prediction to the supervised backend speed limits so lookahead evaluates
  the command that can actually reach the robot.
- Added a source/parameter/interface/visualization consistency audit and regression report.

## 0.8.0 - 2026-07-27

- Changed the fifth default mission step from `F2_B` to the valid F2 elevator-lobby terminal
  at `(8, 0, pi)`; the isolated inter-region gap remains non-navigable.
- Added a typed `terminal` mission step that navigates without inspection dwell.
- Made the automatic warehouse mission use the grid-A* global guide by default, while retaining
  native SCAN as the default manual and algorithm-diagnosis profile.
- Added an inactive-floor-only visualization cloud so RViz never renders the same active PCD
  through two overlapping color layers.
- Standardized the default RViz semantics to yellow for the active floor and blue-grey for
  inactive floors; color no longer changes with camera motion.
- Added regression checks proving all four PGM perimeter edges are occupied collision geometry.
- Clamped the terminal mission-state display to `5/5` while preserving the internal completed
  step count.

## 0.7.0 - 2026-07-27

- Replaced the sensor-augmented M20 xacro with a direct, byte-traceable copy of the official
  Deep Robotics M20 URDF and all 17 meshes.
- Removed the added lidar and IMU visual links from the RViz robot model.
- Made `m20_scan_planner` the single compiled SCAN FSM/controller implementation and removed
  duplicate planner/controller targets from `m20_scan_navigation`.
- Restored native SCAN goal handling as the default; retained inflated-grid A* routing as the
  explicit `use_grid_route:=true` compatibility profile.
- Restored the upstream 0.05 m / 7.5 m / 0.75 m/s SCAN trajectory parameters in native mode,
  while retaining M20 geometry overrides and the independent controller/safety limits.
- Removed the unbuilt duplicate SCAN C++ headers and sources from the Python adapter package.
- Restored the RViz SCAN panels, occupancy/inflation/sliding-map layers, search/trajectory
  markers, TF tree, and a dedicated live sensor-cloud display.
- Added model-tree equality, navigation ownership, launch-mode, RViz-topic, and isolated
  18-package closure tests.

## 0.6.0 - 2026-07-27

- Added a locked seven-package source boundary and 13-package Humble build closure.
- Added a non-destructive isolated-workspace preparation tool with online and local-mirror modes.
- Added pre-apply revision, patch-applicability, and SHA-256 verification for pinned dependencies.
- Added a CPU-only SCAN local-sensing patch that avoids the GPU-only GLM dependency path.
- Added switch, full-mission, and fault-injection system regression modes.
- Added exact generation, sensing freshness, confirmed safe-zero windows, release-pose, and bounded RSS checks.
- Added automatic phase-5 launch shutdown when the regression process completes.
- Added an isolated-workspace guide and an explicit Humble/Foxy compatibility matrix.
- Corrected the component-level SCAN/local-sensing license record.
- Verified 20 alternating floor switches and the cancel/stop/FAULT_HOLD retry fault matrix.
- Verified 10/10 complete five-step missions without restarting the node graph.

## 0.5.0 - 2026-07-27

- Added typed `NavigateFloor` and `RunMission` Actions plus navigation/mission state messages.
- Added floor/generation-aware navigation gateway with cancel, timeout, reset, and final-distance results.
- Added an explicit A* route reset service and ordered route-event handling.
- Added configured five-step F1→F2 inspection mission composition.
- Added pause, resume, stop, and retry-current mission control with an independent fail-closed hold.
- Added committed-target floor-switch recovery for safe mission retries.
- Added mission state text visualization and the phase-4 composed launch profile.
- Added in-graph quick and full runtime acceptance nodes for isolated DDS test environments.
- Verified pause/resume safety and the full five-step mission at F2 generation 2 with 0.001 m final error.

## 0.4.0 - 2026-07-27

- Added isolated typed floor, sensing, navigation-reset, pose-transfer, and floor-switch interfaces.
- Added generation-aware active PCD local sensing with explicit cache invalidation and diagnostics.
- Added compare-and-swap active PCD/occupancy switching and atomic typed floor state.
- Added SCAN GridMap, target, route, progress, and trajectory reset support.
- Added a confirmed simulation pose-transfer service that clears velocity and path state.
- Added a fail-closed bidirectional F1↔F2 floor-switch Action transaction.
- Made the route adapter and collision guard floor-switch aware.
- Added the complete phase-3 multi-floor SCAN/RViz bringup and contract tests.
- Verified F1→F2→F1 switching with no cross-floor point-cloud residue.

## 0.3.0 - 2026-07-27

- Added a byte-traceable ROS 2 wrapper for the official Deep Robotics M20 model.
- Added an RViz planar kinematic backend that only consumes the supervised velocity.
- Added the isolated M20 SCAN planner/controller port and F1 local-sensing bringup.
- Added inflated-grid A* routing with sequential short SCAN subgoals.
- Added fail-closed command arbitration and independent static-map collision lookahead.
- Added the complete F1 map/model/sensing/navigation/safety/RViz launch profile.
- Fixed the SCAN near-goal replan loop with an explicit hold trajectory and wait state.
- Added phase-2 unit, contract, runtime navigation, emergency-stop, and GUI records.

## 0.2.0 - 2026-07-27

- Added strict system configuration and cross-reference validation.
- Added deterministic F1/F2 PCD, occupancy-grid, and metadata generation.
- Added committed 40 m × 40 m map assets with 180 obstacles per floor.
- Added the static all-floor and initial active-floor ROS 2 map server.
- Added the phase-1 RViz two-region overview launch and display configuration.
- Added configuration, reproducibility, geometry, and asset-integrity tests.

## 0.1.0 - 2026-07-27

- Established the independent `m20_warehouse_inspection` integration package.
- Recorded the flat, side-by-side F1/F2 simulation decision.
- Added the initial floor, elevator, mission, map-switch, navigation, and safety contract.
- Pinned the SCAN-Planner and official M20 model source revisions.
