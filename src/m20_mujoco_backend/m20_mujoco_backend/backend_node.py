# Copyright 2026 Virdyn Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Real-time MuJoCo execution backend for the official M20 RL controller."""

from __future__ import annotations

import json
import math
from pathlib import Path as FilePath
import time
from typing import Optional

from drdds.msg import (
    ImuData,
    ImuDataValue,
    JointData,
    JointsData,
    JointsDataCmd,
    JointsDataValue,
    MetaType,
)
from geometry_msgs.msg import PoseStamped, TransformStamped
import mujoco
import numpy as np
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

from .dynamics import (
    JOINT_INITIAL_POSITION,
    JOINT_NAMES,
    JOINT_VELOCITY_LIMIT,
    LEG_INDICES,
    WHEEL_INDICES,
    apply_wheel_brake,
    normalized_quaternion,
    pd_torque,
    quaternion_to_rpy,
    raw_to_sdk,
    sdk_to_raw,
    yaw_quaternion,
)


def _is_warehouse_collision_geom(name: Optional[str]) -> bool:
    """Return whether a MuJoCo geom is a generated obstacle or boundary."""
    if not name:
        return False
    return (
        name.startswith('warehouse_obstacle_')
        or name.startswith('warehouse_wall_')
    )


def warehouse_contact_summary(model, data) -> dict:
    """Summarise instantaneous robot contacts with warehouse geometry."""
    pairs = set()
    peak_normal_force = 0.0
    count = 0
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        left = mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            int(contact.geom1),
        )
        right = mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            int(contact.geom2),
        )
        if not (
            _is_warehouse_collision_geom(left)
            or _is_warehouse_collision_geom(right)
        ):
            continue
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, force)
        count += 1
        peak_normal_force = max(peak_normal_force, abs(float(force[0])))
        pairs.add(f'{left or "<unnamed>"}|{right or "<unnamed>"}')
    return {
        'count': count,
        'peak_normal_force_n': peak_normal_force,
        'pairs': sorted(pairs),
    }


class M20MujocoBackend(Node):
    """Execute official low-level commands against the full M20 MJCF model."""

    def __init__(self) -> None:
        super().__init__('m20_mujoco_backend')
        self.declare_parameter('model_xml_path', '')
        self.declare_parameter('use_viewer', True)
        self.declare_parameter('viewer_follow_robot', True)
        self.declare_parameter('viewer_follow_body', 'base_link')
        self.declare_parameter('viewer_distance', 4.0)
        self.declare_parameter('viewer_azimuth', 135.0)
        self.declare_parameter('viewer_elevation', -25.0)
        self.declare_parameter('timestep', 0.001)
        self.declare_parameter('real_time_factor', 1.0)
        self.declare_parameter('max_catchup_steps', 8)
        self.declare_parameter('state_publish_hz', 200.0)
        self.declare_parameter('diagnostics_publish_hz', 10.0)
        self.declare_parameter('path_publish_hz', 10.0)
        self.declare_parameter('path_max_poses', 5000)
        self.declare_parameter('render_interval', 20)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.20)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('command_timeout_sec', 0.50)
        self.declare_parameter('startup_hold_kp', 80.0)
        self.declare_parameter('startup_hold_kd', 2.0)
        self.declare_parameter('wheel_hold_kd', 0.60)
        self.declare_parameter('parking_brake_enabled', True)
        self.declare_parameter('parking_brake_kd', 4.0)
        self.declare_parameter(
            'locomotion_mode_topic',
            '/m20/locomotion/mode',
        )
        self.declare_parameter('min_base_height', 0.05)
        self.declare_parameter('max_tilt_rad', 1.20)
        self.declare_parameter('standing_ready_height', 0.55)
        self.declare_parameter('standing_ready_stable_sec', 0.30)
        self.declare_parameter('map_frame', 'world')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter(
            'body_pose_topic',
            '/m20/sim/body_pose',
        )
        self.declare_parameter('path_topic', '/quad_0/path')
        self.declare_parameter(
            'ready_topic',
            '/m20/sim/backend_ready',
        )
        self.declare_parameter(
            'fault_topic',
            '/m20/sim/backend_fault',
        )
        self.declare_parameter(
            'dynamics_state_topic',
            '/m20/sim/dynamics_state',
        )

        model_path = FilePath(
            str(self.get_parameter('model_xml_path').value)
        ).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(
                f'MuJoCo world does not exist: {model_path}'
            )
        self._model = mujoco.MjModel.from_xml_path(str(model_path))
        if self._model.nu != 16:
            raise RuntimeError(
                f'official M20 backend requires 16 actuators, '
                f'got {self._model.nu}'
            )
        self._data = mujoco.MjData(self._model)
        self._dt = max(
            1.0e-4,
            float(self.get_parameter('timestep').value),
        )
        self._model.opt.timestep = self._dt
        self._real_time_factor = max(
            0.01,
            float(self.get_parameter('real_time_factor').value),
        )
        self._max_catchup_steps = max(
            1,
            int(self.get_parameter('max_catchup_steps').value),
        )
        self._use_viewer = bool(self.get_parameter('use_viewer').value)
        self._viewer_follow_robot = bool(
            self.get_parameter('viewer_follow_robot').value
        )
        self._viewer_distance = max(
            0.5,
            float(self.get_parameter('viewer_distance').value),
        )
        self._viewer_azimuth = float(
            self.get_parameter('viewer_azimuth').value
        )
        self._viewer_elevation = max(
            -89.0,
            min(
                89.0,
                float(self.get_parameter('viewer_elevation').value),
            ),
        )
        self._render_interval = max(
            1,
            int(self.get_parameter('render_interval').value),
        )
        self._command_timeout = max(
            0.05,
            float(self.get_parameter('command_timeout_sec').value),
        )
        self._min_base_height = float(
            self.get_parameter('min_base_height').value
        )
        self._max_tilt = float(
            self.get_parameter('max_tilt_rad').value
        )
        self._standing_ready_height = float(
            self.get_parameter('standing_ready_height').value
        )
        self._standing_ready_stable_sec = max(
            0.0,
            float(
                self.get_parameter(
                    'standing_ready_stable_sec'
                ).value
            ),
        )
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)

        self._set_initial_state()
        self._ctrl_low = self._model.actuator_ctrlrange[:, 0].copy()
        self._ctrl_high = self._model.actuator_ctrlrange[:, 1].copy()
        self._kp = np.zeros(16)
        self._kd = np.zeros(16)
        self._desired_position = JOINT_INITIAL_POSITION.copy()
        self._desired_velocity = np.zeros(16)
        self._feedforward = np.zeros(16)
        startup_kp = float(self.get_parameter('startup_hold_kp').value)
        startup_kd = float(self.get_parameter('startup_hold_kd').value)
        self._kp[LEG_INDICES] = startup_kp
        self._kd[LEG_INDICES] = startup_kd
        self._kd[WHEEL_INDICES] = float(
            self.get_parameter('wheel_hold_kd').value
        )
        self._last_torque = np.zeros(16)
        self._last_command_monotonic: Optional[float] = None
        self._startup_hold = True
        self._command_timed_out = False
        self._parking_brake_enabled = bool(
            self.get_parameter('parking_brake_enabled').value
        )
        self._parking_brake_kd = max(
            0.0,
            float(self.get_parameter('parking_brake_kd').value),
        )
        self._locomotion_mode = 'BACKEND_HOLD'
        self._parking_brake_active = self._parking_brake_enabled
        self._fault = ''
        self._ready = False
        self._standing_since: Optional[float] = None
        self._steps = 0
        self._obstacle_contact_count = 0
        self._obstacle_contact_event_count = 0
        self._obstacle_contact_active = False
        self._obstacle_contact_peak_force = 0.0
        self._obstacle_contact_pairs = set()
        self._wall_start = time.monotonic()
        self._sim_start = float(self._data.time)

        state_hz = max(
            1.0,
            float(self.get_parameter('state_publish_hz').value),
        )
        diagnostics_hz = max(
            1.0,
            float(self.get_parameter('diagnostics_publish_hz').value),
        )
        path_hz = max(
            0.1,
            float(self.get_parameter('path_publish_hz').value),
        )
        self._state_every = max(1, round(1.0 / (self._dt * state_hz)))
        self._diagnostics_every = max(
            1,
            round(1.0 / (self._dt * diagnostics_hz)),
        )
        self._path_every = max(1, round(1.0 / (self._dt * path_hz)))
        self._path_max_poses = max(
            2,
            int(self.get_parameter('path_max_poses').value),
        )

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._ready_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('ready_topic').value),
            latched,
        )
        self._fault_pub = self.create_publisher(
            String,
            str(self.get_parameter('fault_topic').value),
            latched,
        )
        self._dynamics_pub = self.create_publisher(
            String,
            str(self.get_parameter('dynamics_state_topic').value),
            10,
        )
        self._body_pose_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter('body_pose_topic').value),
            50,
        )
        self._path_pub = self.create_publisher(
            Path,
            str(self.get_parameter('path_topic').value),
            10,
        )
        self._joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            50,
        )
        self._joints_pub = self.create_publisher(
            JointsData,
            '/JOINTS_DATA',
            200,
        )
        self._imu_pub = self.create_publisher(
            ImuData,
            '/IMU_DATA',
            200,
        )
        self._command_sub = self.create_subscription(
            JointsDataCmd,
            '/JOINTS_CMD',
            self._command_callback,
            50,
        )
        self._mode_sub = self.create_subscription(
            String,
            str(self.get_parameter('locomotion_mode_topic').value),
            self._locomotion_mode_callback,
            latched,
        )
        self._tf = TransformBroadcaster(self)
        self._path = Path()
        self._path.header.frame_id = self._map_frame
        self._viewer = None
        if self._use_viewer:
            from mujoco import viewer as mujoco_viewer
            self._viewer = mujoco_viewer.launch_passive(
                self._model,
                self._data,
            )
            self._configure_viewer_camera()

        self._publish_status()
        self._publish_robot_state()
        self.get_logger().info(
            'MuJoCo M20 physics loaded: '
            f'model={model_path}, dt={self._dt:.4f}s, '
            f'rtf={self._real_time_factor:.2f}, '
            f'pose=({self._data.qpos[0]:.2f}, '
            f'{self._data.qpos[1]:.2f}, {self._data.qpos[2]:.2f}); '
            'system pose remains gated until SDK stand-up is stable'
        )

    def _set_initial_state(self) -> None:
        self._data.qpos[:] = 0.0
        self._data.qvel[:] = 0.0
        self._data.qpos[0] = float(
            self.get_parameter('initial_x').value
        )
        self._data.qpos[1] = float(
            self.get_parameter('initial_y').value
        )
        self._data.qpos[2] = float(
            self.get_parameter('initial_z').value
        )
        self._data.qpos[3:7] = yaw_quaternion(
            float(self.get_parameter('initial_yaw').value)
        )
        self._data.qpos[7:23] = JOINT_INITIAL_POSITION
        mujoco.mj_forward(self._model, self._data)

    def _configure_viewer_camera(self) -> None:
        """Set a bounded tracking view centered on the M20 base."""
        if self._viewer is None or not self._viewer_follow_robot:
            return
        body_name = str(
            self.get_parameter('viewer_follow_body').value
        )
        body_id = mujoco.mj_name2id(
            self._model,
            mujoco.mjtObj.mjOBJ_BODY,
            body_name,
        )
        if body_id < 0:
            self.get_logger().warn(
                f'viewer follow body does not exist: {body_name}; '
                'keeping the free camera'
            )
            return
        with self._viewer.lock():
            self._viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self._viewer.cam.trackbodyid = body_id
            self._viewer.cam.distance = self._viewer_distance
            self._viewer.cam.azimuth = self._viewer_azimuth
            self._viewer.cam.elevation = self._viewer_elevation
            self._viewer.cam.lookat[:] = self._data.xpos[body_id]
        self.get_logger().info(
            f'MuJoCo viewer tracking {body_name}: '
            f'distance={self._viewer_distance:.2f}m, '
            f'azimuth={self._viewer_azimuth:.1f}deg, '
            f'elevation={self._viewer_elevation:.1f}deg'
        )

    @staticmethod
    def _is_active_motor_command(message: JointsDataCmd) -> bool:
        """Distinguish stand-up/RL commands from SDK reset control words."""
        for command in message.data.joints_data:
            if any(
                abs(float(value)) > 1.0e-7
                for value in (
                    command.kp,
                    command.kd,
                    command.position,
                    command.velocity,
                    command.torque,
                )
            ):
                return True
        return False

    def _command_callback(self, message: JointsDataCmd) -> None:
        if len(message.data.joints_data) != 16:
            self.get_logger().warn(
                'ignored /JOINTS_CMD with a non-16 joint payload'
            )
            return
        active = self._is_active_motor_command(message)
        if self._startup_hold and not active:
            return
        if self._startup_hold:
            self._startup_hold = False
            self.get_logger().info(
                'official SDK motor command detected; startup hold released'
            )

        positions = np.asarray(
            [float(item.position) for item in message.data.joints_data]
        )
        velocities = np.asarray(
            [float(item.velocity) for item in message.data.joints_data]
        )
        feedforward = np.asarray(
            [float(item.torque) for item in message.data.joints_data]
        )
        raw_position, raw_velocity, raw_feedforward = sdk_to_raw(
            positions,
            velocities,
            feedforward,
        )
        self._kp = np.asarray(
            [float(item.kp) for item in message.data.joints_data]
        )
        self._kd = np.asarray(
            [float(item.kd) for item in message.data.joints_data]
        )
        self._desired_position = raw_position
        self._desired_velocity = raw_velocity
        self._feedforward = raw_feedforward
        self._last_command_monotonic = time.monotonic()
        self._command_timed_out = False

    def _locomotion_mode_callback(self, message: String) -> None:
        """Engage wheel braking whenever no motion intent is active."""
        mode = str(message.data)
        moving_modes = {
            'WHEEL_CRUISE',
            'COORDINATED_TURN',
            'LATERAL_MANEUVER',
        }
        brake_active = (
            self._parking_brake_enabled and mode not in moving_modes
        )
        if (
            mode == self._locomotion_mode
            and brake_active == self._parking_brake_active
        ):
            return
        self._locomotion_mode = mode
        self._parking_brake_active = brake_active
        state = 'engaged' if brake_active else 'released'
        self.get_logger().info(
            f'wheel parking brake {state}; locomotion_mode={mode}'
        )

    def _enforce_command_timeout(self, now: float) -> None:
        if self._startup_hold or self._last_command_monotonic is None:
            return
        age = now - self._last_command_monotonic
        if age <= self._command_timeout or self._command_timed_out:
            return
        self._desired_velocity[WHEEL_INDICES] = 0.0
        self._feedforward[WHEEL_INDICES] = 0.0
        self._kd[WHEEL_INDICES] = np.maximum(
            self._kd[WHEEL_INDICES],
            float(self.get_parameter('wheel_hold_kd').value),
        )
        self._command_timed_out = True
        self.get_logger().warn(
            f'/JOINTS_CMD timed out after {age:.3f}s; wheel targets stopped'
        )

    def _step(self, now: float) -> None:
        self._enforce_command_timeout(now)
        q = self._data.qpos[7:23]
        dq = self._data.qvel[6:22]
        desired_velocity = self._desired_velocity
        kd = self._kd
        feedforward = self._feedforward
        if self._parking_brake_active:
            desired_velocity, kd, feedforward = apply_wheel_brake(
                desired_velocity,
                kd,
                feedforward,
                self._parking_brake_kd,
            )
        self._last_torque = pd_torque(
            position=q,
            velocity=dq,
            desired_position=self._desired_position,
            desired_velocity=desired_velocity,
            kp=self._kp,
            kd=kd,
            feedforward=feedforward,
            low=self._ctrl_low,
            high=self._ctrl_high,
        )
        self._data.ctrl[:] = self._last_torque
        mujoco.mj_step(self._model, self._data)
        self._update_obstacle_contacts()
        self._data.qvel[6:22] = np.clip(
            self._data.qvel[6:22],
            -JOINT_VELOCITY_LIMIT,
            JOINT_VELOCITY_LIMIT,
        )
        self._steps += 1
        self._update_fault()
        self._update_readiness(now)
        if self._steps % self._state_every == 0:
            self._publish_robot_state()
        if self._steps % self._path_every == 0:
            self._publish_path()
        if self._steps % self._diagnostics_every == 0:
            self._publish_diagnostics()
        if self._viewer and self._steps % self._render_interval == 0:
            self._viewer.sync()

    def _update_obstacle_contacts(self) -> None:
        """Latch brief obstacle contacts that a 10 Hz diagnostic may miss."""
        summary = warehouse_contact_summary(self._model, self._data)
        active = summary['count'] > 0
        if active and not self._obstacle_contact_active:
            self._obstacle_contact_event_count += 1
        self._obstacle_contact_active = active
        self._obstacle_contact_count = int(summary['count'])
        self._obstacle_contact_peak_force = max(
            self._obstacle_contact_peak_force,
            float(summary['peak_normal_force_n']),
        )
        self._obstacle_contact_pairs.update(summary['pairs'])

    def _update_fault(self) -> None:
        reason = ''
        if not (
            np.all(np.isfinite(self._data.qpos))
            and np.all(np.isfinite(self._data.qvel))
        ):
            reason = 'NONFINITE_STATE'
        else:
            roll, pitch, _ = quaternion_to_rpy(self._data.qpos[3:7])
            if float(self._data.qpos[2]) < self._min_base_height:
                reason = 'BASE_HEIGHT_LOW'
            elif max(abs(roll), abs(pitch)) > self._max_tilt:
                reason = 'EXCESSIVE_TILT'
        if reason == self._fault:
            return
        self._fault = reason
        if reason:
            self._ready = False
            self._standing_since = None
            self._publish_status()
            self.get_logger().error(f'MuJoCo backend fault: {reason}')
        else:
            self._publish_status()
            self.get_logger().info('MuJoCo backend fault cleared')

    def _update_readiness(self, now: float) -> None:
        """Open the system pose contract only after a stable SDK stand-up."""
        if self._ready or self._fault or self._startup_hold:
            self._standing_since = None
            return
        roll, pitch, _ = quaternion_to_rpy(self._data.qpos[3:7])
        standing = (
            float(self._data.qpos[2]) >= self._standing_ready_height
            and max(abs(roll), abs(pitch)) < 0.35
        )
        if not standing:
            self._standing_since = None
            return
        if self._standing_since is None:
            self._standing_since = now
            return
        if now - self._standing_since < self._standing_ready_stable_sec:
            return
        self._ready = True
        self._publish_status()
        self.get_logger().info(
            'MuJoCo M20 backend ready: official SDK stand-up is stable; '
            'system odometry, TF and navigation commands are now enabled'
        )

    def _publish_status(self) -> None:
        self._ready_pub.publish(Bool(data=self._ready))
        self._fault_pub.publish(String(data=self._fault))

    def _publish_robot_state(self) -> None:
        stamp = self.get_clock().now().to_msg()
        position = np.nan_to_num(self._data.qpos[:3])
        quaternion = normalized_quaternion(self._data.qpos[3:7])
        velocity = np.nan_to_num(self._data.qvel[:6])

        raw_position = self._data.qpos[7:23].copy()
        raw_velocity = self._data.qvel[6:22].copy()
        sdk_position, sdk_velocity, sdk_torque = raw_to_sdk(
            raw_position,
            raw_velocity,
            self._last_torque,
        )
        joints = JointsData()
        joints.header = MetaType()
        joints.header.frame_id = 0
        joints.header.stamp = stamp
        joints.data = JointsDataValue()
        joints.data.joints_data = [JointData() for _ in range(16)]
        for index, joint in enumerate(joints.data.joints_data):
            joint.name = [32, 32, 32, 32]
            joint.data_id = 0
            joint.status_word = 1
            joint.position = float(sdk_position[index])
            joint.velocity = float(sdk_velocity[index])
            joint.torque = float(sdk_torque[index])
            joint.motion_temp = 40.0
            joint.driver_temp = 45.0
        self._joints_pub.publish(joints)

        quaternion_sensor = normalized_quaternion(self._data.sensordata[:4])
        roll, pitch, yaw = quaternion_to_rpy(quaternion_sensor)
        acceleration = np.nan_to_num(self._data.sensordata[4:7])
        angular_velocity = np.nan_to_num(self._data.sensordata[7:10])
        imu = ImuData()
        imu.header = MetaType()
        imu.header.frame_id = 0
        imu.header.stamp = stamp
        imu.data = ImuDataValue()
        imu.data.roll = math.degrees(roll)
        imu.data.pitch = math.degrees(pitch)
        imu.data.yaw = math.degrees(yaw)
        imu.data.omega_x = float(angular_velocity[0])
        imu.data.omega_y = float(angular_velocity[1])
        imu.data.omega_z = float(angular_velocity[2])
        imu.data.acc_x = float(acceleration[0])
        imu.data.acc_y = float(acceleration[1])
        imu.data.acc_z = float(acceleration[2])
        self._imu_pub.publish(imu)

        if not self._ready:
            return

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self._map_frame
        odometry.child_frame_id = self._base_frame
        odometry.pose.pose.position.x = float(position[0])
        odometry.pose.pose.position.y = float(position[1])
        odometry.pose.pose.position.z = float(position[2])
        odometry.pose.pose.orientation.w = float(quaternion[0])
        odometry.pose.pose.orientation.x = float(quaternion[1])
        odometry.pose.pose.orientation.y = float(quaternion[2])
        odometry.pose.pose.orientation.z = float(quaternion[3])
        odometry.twist.twist.linear.x = float(velocity[0])
        odometry.twist.twist.linear.y = float(velocity[1])
        odometry.twist.twist.linear.z = float(velocity[2])
        odometry.twist.twist.angular.x = float(velocity[3])
        odometry.twist.twist.angular.y = float(velocity[4])
        odometry.twist.twist.angular.z = float(velocity[5])
        self._body_pose_pub.publish(odometry)

        transform = TransformStamped()
        transform.header = odometry.header
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation = odometry.pose.pose.orientation
        self._tf.sendTransform(transform)

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = list(JOINT_NAMES)
        joint_state.position = raw_position.tolist()
        joint_state.velocity = raw_velocity.tolist()
        joint_state.effort = self._last_torque.tolist()
        self._joint_state_pub.publish(joint_state)

    def _publish_path(self) -> None:
        if not self._ready:
            return
        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._map_frame
        pose.pose.position.x = float(self._data.qpos[0])
        pose.pose.position.y = float(self._data.qpos[1])
        pose.pose.position.z = float(self._data.qpos[2])
        quaternion = normalized_quaternion(self._data.qpos[3:7])
        pose.pose.orientation.w = float(quaternion[0])
        pose.pose.orientation.x = float(quaternion[1])
        pose.pose.orientation.y = float(quaternion[2])
        pose.pose.orientation.z = float(quaternion[3])
        self._path.header.stamp = stamp
        self._path.poses.append(pose)
        if len(self._path.poses) > self._path_max_poses:
            self._path.poses = self._path.poses[-self._path_max_poses:]
        self._path_pub.publish(self._path)

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        wall_elapsed = max(1.0e-9, now - self._wall_start)
        simulation_elapsed = float(self._data.time) - self._sim_start
        command_age = (
            None
            if self._last_command_monotonic is None
            else now - self._last_command_monotonic
        )
        roll, pitch, yaw = quaternion_to_rpy(self._data.qpos[3:7])
        state = {
            'ready': self._ready,
            'fault': self._fault,
            'startup_hold': self._startup_hold,
            'locomotion_mode': self._locomotion_mode,
            'parking_brake_active': self._parking_brake_active,
            'command_timed_out': self._command_timed_out,
            'command_age_sec': command_age,
            'sim_time_sec': float(self._data.time),
            'measured_real_time_factor': simulation_elapsed / wall_elapsed,
            'base_position': self._data.qpos[:3].tolist(),
            'base_rpy': [roll, pitch, yaw],
            'base_velocity': self._data.qvel[:6].tolist(),
            'contact_count': int(self._data.ncon),
            'obstacle_contact_count': self._obstacle_contact_count,
            'obstacle_contact_event_count': (
                self._obstacle_contact_event_count
            ),
            'obstacle_contact_peak_force_n': (
                self._obstacle_contact_peak_force
            ),
            'obstacle_contact_pairs': sorted(
                self._obstacle_contact_pairs
            ),
            'max_abs_torque': float(np.max(np.abs(self._last_torque))),
        }
        self._dynamics_pub.publish(
            String(data=json.dumps(state, separators=(',', ':')))
        )

    def run(self) -> None:
        """Advance physics against a monotonic real-time schedule."""
        period = self._dt / self._real_time_factor
        next_step = time.monotonic()
        while rclpy.ok():
            now = time.monotonic()
            completed = 0
            while now >= next_step and completed < self._max_catchup_steps:
                self._step(now)
                next_step += period
                completed += 1
            if now - next_step > period * self._max_catchup_steps:
                next_step = now
            try:
                rclpy.spin_once(
                    self,
                    timeout_sec=max(0.0, min(0.002, next_step - now)),
                )
            except Exception:
                if not rclpy.ok():
                    break
                raise
            if self._viewer and not self._viewer.is_running():
                break

    def close(self) -> None:
        """Publish an unavailable state before releasing the viewer."""
        self._ready = False
        if rclpy.ok():
            try:
                self._publish_status()
            except Exception:
                pass
        if self._viewer is not None:
            self._viewer.close()
            # launch_passive owns a GUI thread which still references mjModel
            # and mjData after close() raises its exit request. Give the native
            # render loop one event cycle before interpreter teardown. Do not
            # poll the native Simulate object while it is being deleted.
            time.sleep(1.0)
            self._viewer = None


def main(args=None) -> None:
    """Run the M20 MuJoCo backend."""
    # Keep Python's normal SIGINT -> KeyboardInterrupt path. The default
    # rclpy handler tears down its context asynchronously while MuJoCo's
    # GLFW render thread is still live, which can race native viewer cleanup.
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = M20MujocoBackend()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.close()
        except KeyboardInterrupt:
            pass
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()
