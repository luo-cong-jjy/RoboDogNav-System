# M20 locomotion control

This package owns both sides of the autonomous motion boundary: rolling
adaptation before safety prediction and the final backend gate after safety.

```text
/m20/navigation/cmd_vel_raw
  -> m20_navigation_adapter
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

`m20_locomotion_manager` no longer repeats rolling adaptation. It consumes
only `/m20/control/cmd_vel_safe`, applies the final SDK envelope and
backend-ready/fault gate, and publishes the diagnostic motion intent.

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
