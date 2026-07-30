# M20 MuJoCo backend

This isolated ROS 2 package makes the official 16-actuator M20 MJCF model the
execution backend for the warehouse system. It does not implement navigation:
the existing SCAN planner still produces safe body velocity commands, the
official `m20_sdk_deploy/rl_deploy_cmdvel` policy converts them into joint
commands, and this package integrates those commands with MuJoCo contacts.

The physics world is generated from the same `metadata_file` JSON files used
by each configured PCD. Both flat warehouse regions are always present as
collision geometry. A logical floor switch changes the active PCD and planner
state only; it never teleports the robot.

Runtime contract:

- input: `/JOINTS_CMD`
- motion intent: `/m20/locomotion/mode`
- SDK feedback: `/JOINTS_DATA`, `/IMU_DATA`
- system pose: `/m20/sim/body_pose`, `world -> base_link`
- visualization: `/joint_states`, `/quad_0/path`
- health: `/m20/sim/backend_ready`, `/m20/sim/backend_fault`,
  `/m20/sim/dynamics_state`

`/m20/sim/dynamics_state` separates normal ground contacts from generated
warehouse obstacles. `obstacle_contact_count` is instantaneous, while
`obstacle_contact_event_count`, `obstacle_contact_peak_force_n`, and
`obstacle_contact_pairs` are latched for the complete backend run so brief
physics contacts cannot be missed by the lower-rate diagnostic subscriber.

The backend begins with a PD hold at the official folded initial joint pose.
SDK reset/control-word messages containing zero motor gains cannot release
that hold; the first real stand-up command does. A lost joint command stops
wheel targets while preserving leg stabilization. System odometry and TF stay
gated until the SDK has held the body above 0.55 m with a stable attitude for
0.30 s, so SCAN never initializes from the folded pose.

The native viewer opens in a `base_link` tracking camera by default. Its
distance is exposed by the complete launch as `mujoco_viewer_distance`
(default `4.0` metres); mouse control remains available in the window.

The official policy continues to run at zero body command so its twelve leg
joints can balance the robot. Its four wheel velocity outputs are physically
braked only while the locomotion intent is `STOPPED`, `BACKEND_HOLD`, or
`FAULT_HOLD`. Moving intents immediately release the brake. This prevents
long-term zero-command rolling without freezing the floating base or writing
the robot pose.

MuJoCo's Python wheel is not a ROS Humble rosdep key. Install the version
validated by this project before building an isolated workspace:

```bash
python3 -m pip install --user "mujoco==3.10.0"
```
