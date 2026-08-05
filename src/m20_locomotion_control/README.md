# M20 locomotion control

This package owns both sides of the autonomous motion boundary: rolling
adaptation before safety prediction and the final backend gate after safety.

```text
/m20/navigation/cmd_vel_raw
  -> m20_navigation_adapter
       <- /m20/sim/body_pose.twist (base_link, optional bounded PI)
  -> /m20/navigation/cmd_vel_candidate
  -> collision guard + safety supervisor
/m20/control/cmd_vel_safe
  -> m20_locomotion_manager
  -> /m20/locomotion/cmd_vel_sdk
  -> m20_sdk_deploy/rl_deploy_cmdvel
  -> /JOINTS_CMD
```

For autonomous navigation, the pre-safety adapter converts SCAN's body-frame lateral
tracking term into yaw correction. It uses turn-mode hysteresis, limits
translation during large heading changes, filters small cruise yaw
corrections, and publishes zero on input timeout. The collision guard, safety
supervisor, RViz backend, and SDK therefore all use the same adapted candidate
motion model. This avoids
treating routine path alignment as a prolonged crab-like
`LATERAL_MANEUVER`. Manual commands retain lateral motion for commissioning
and recovery by entering the safety supervisor on their separate manual topic.

The adapter also contains an optional measured body-velocity inner loop for
forward speed and yaw rate. It reads `/m20/sim/body_pose.twist`, verifies that
`child_frame_id` is `base_link`, then applies deadbanded PI correction with
separate integral, correction, and SDK-envelope limits. It never reverses the
requested motion. Stale/non-finite/wrong-frame odometry disables correction
and preserves the open-loop candidate. `/m20/control/execution_hold` clears
all integrators during collision recovery, mission/floor holds, manual
override, or emergency stops. Corrected candidates remain upstream of the
collision guard.

The gains remain staged with `velocity_feedback_enabled: false`. When enabled,
the current experimental gate permits correction only in `WHEEL_CRUISE` with
`|wz_reference| <= 0.08 rad/s`; turn, lateral and hold phases reset the PI and
publish `MANEUVER_GATED`. A stable bidirectional six-goal A/B improved total
time and final error, but two single-obstacle cold starts produced materially
different minimum clearances. The loop is therefore not approved as the
production default. Launches may opt in only for isolated diagnostics with
`velocity_feedback_enabled:=true`. Diagnostic JSON is published on
`/m20/navigation/velocity_feedback_state`.

`m20_locomotion_manager` no longer repeats rolling adaptation. It consumes
only `/m20/control/cmd_vel_safe`, applies the final SDK envelope and
backend-ready/fault gate, and publishes the diagnostic motion intent.

The versioned
`config/m20_policy_v1_capabilities.yaml` is the single source of truth for
body dimensions, command limits, the measured stable rolling interval,
tracking calibration, drift envelope, and bounded recovery. Launch files
validate it before starting nodes and inject the same limits into the
navigation adapter, collision guard, final backend gate, and vendor SDK
bridge. The derived 0.538 m minimum centreline turn radius and approximately
0.893 m outer-corner sweep are diagnostics/layout constraints; collision
checking still uses the more accurate oriented double-circle trajectory.

The same capability profile enables bidirectional B-spline execution only in
the complete M20 integration. Direction is selected once per new trajectory:
reverse is allowed only when sampled tangents over the initial B-spline are
approximately straight and aligned with the rear body axis. Reverse tracking
locks the entry body yaw so a degenerate endpoint tangent cannot trigger a
100 Hz direction or heading oscillation. Standalone SCAN launches keep this
feature disabled by default. Runtime direction is exposed as
`/planning/tracking_direction`.

The adapter labels commands as `STOPPED`, `WHEEL_CRUISE`,
`COORDINATED_TURN`, or `LATERAL_MANEUVER`. These labels describe motion intent
only. The current official ONNX policy accepts forward, lateral, and yaw
velocity; it does not expose a discrete wheel/leg gait input. The policy
therefore remains responsible for coordinating the twelve leg joints and four
wheel joints, which can participate simultaneously.

The vendor package remains an external, pinned dependency. Do not copy or edit
its ONNX model, joint calibration, hardware interface, or bundled runtime in
this package. A clean standalone workspace can import the audited SDK profile
from `m20_warehouse_inspection/dependencies_sdk.repos`; it is intentionally
separate from the RViz baseline dependency lock.

The SDK executable must not be started until exactly one state backend provides
`/JOINTS_DATA` and `/IMU_DATA` and exactly one actuator backend consumes
`/JOINTS_CMD`. The current RViz kinematic backend is not such a backend.

Build and test the isolated adapter:

```bash
colcon build --symlink-install --packages-select m20_locomotion_control
colcon test --packages-select m20_locomotion_control
```

Run only the safe command adapter:

```bash
ros2 launch m20_locomotion_control sdk_locomotion.launch.py
```

Enable the official controller only after a compatible simulator or robot
backend is running:

```bash
ros2 launch m20_locomotion_control sdk_locomotion.launch.py start_sdk:=true
```

Select another validated platform/backend profile explicitly:

```bash
ros2 launch m20_locomotion_control sdk_locomotion.launch.py \
  locomotion_capability_config:=/absolute/path/to/profile.yaml
```
