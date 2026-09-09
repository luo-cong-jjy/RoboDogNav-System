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

# ============================================================
# 文件：backend_node.py
# 用途：M20 官方 RL 控制器的实时 MuJoCo 执行后端（节点名 m20_mujoco_backend）。
#       - 加载由 world_generator 生成的 MJCF 世界（含仓库障碍物），
#         以 1kHz（默认 dt=0.001s）推进物理仿真；
#       - 订阅官方低层指令 /JOINTS_CMD（JointsDataCmd，16 关节），
#         按 SDK PD 律计算力矩并写入执行器，含启动保持（startup hold）、
#         指令超时保护与驻车制动（仅轮子）；
#       - 发布 /JOINTS_DATA、/IMU_DATA（SDK 风格）、/joint_states、
#         /m20/sim/body_pose（Odometry + TF）、/quad_0/path 轨迹、
#         /m20/sim/backend_ready 就绪信号、/m20/sim/backend_fault 故障、
#         /m20/sim/dynamics_state 诊断 JSON；
#       - 系统位姿契约（odometry/TF/导航指令）在官方 SDK 站立稳定后
#         才开放（readiness 门控），并带故障检测（非有限状态/过低/
#         过度倾斜）与仓库障碍接触统计。
# ============================================================

"""Real-time MuJoCo execution backend for the official M20 RL controller."""

from __future__ import annotations

import json
import math
from pathlib import Path as FilePath
import time
from typing import Optional

# drdds：厂商 SDK 自定义消息（IMU/关节数据/指令）。
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

# 从 dynamics.py 引入坐标换算与 PD 律等纯数值工具。
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
    world_vector_to_body,
    yaw_quaternion,
)


def _is_warehouse_collision_geom(name: Optional[str]) -> bool:
    """Return whether a MuJoCo geom is a generated obstacle or boundary."""
    # 判断 geom 是否属于 world_generator 生成的仓库障碍物/围墙
    # （名称前缀 warehouse_obstacle_ / warehouse_wall_）。
    if not name:
        return False
    return (
        name.startswith('warehouse_obstacle_')
        or name.startswith('warehouse_wall_')
    )


def warehouse_contact_summary(model, data) -> dict:
    """Summarise instantaneous robot contacts with warehouse geometry."""
    # 遍历当前所有接触对，只统计机器人（或地面）与仓库障碍物/围墙的接触：
    #   - pairs：接触对名称集合（去重）
    #   - peak_normal_force_n：峰值法向接触力（mj_contactForce 的 force[0]）
    #   - count：仓库接触数量
    # 用于 10Hz 诊断中锁存短暂接触事件（_update_obstacle_contacts）。
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
        # 只有接触对中至少一方是仓库几何才统计（忽略地面与机器人内部接触）。
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
        # ---------- 模型/物理参数 ----------
        self.declare_parameter('model_xml_path', '')
        # Retained only for direct legacy runs.  Standard launches keep this
        # physics process headless and start m20_mujoco_viewer separately.
        # 仅遗留直跑模式使用内嵌 viewer；标准 launch 保持本进程无头，
        # 单独启动 m20_mujoco_viewer 进程。
        self.declare_parameter('use_viewer', False)
        self.declare_parameter('viewer_follow_robot', True)
        self.declare_parameter('viewer_follow_body', 'base_link')
        self.declare_parameter('viewer_distance', 4.0)
        self.declare_parameter('viewer_azimuth', 90.0)
        self.declare_parameter('viewer_elevation', -89.0)
        self.declare_parameter('viewer_max_fps', 30.0)
        # timestep：物理步长（默认 1ms）；real_time_factor：实时倍率
        # max_catchup_steps：单次循环最多补跑的物理步数
        self.declare_parameter('timestep', 0.001)
        self.declare_parameter('real_time_factor', 1.0)
        self.declare_parameter('max_catchup_steps', 8)
        # ---------- 发布频率参数 ----------
        # state_publish_hz：机器人状态（关节/IMU/里程计）发布频率
        # diagnostics_publish_hz：诊断 JSON 频率
        # path_publish_hz / path_max_poses：轨迹发布频率与最大点数
        self.declare_parameter('state_publish_hz', 200.0)
        self.declare_parameter('diagnostics_publish_hz', 10.0)
        self.declare_parameter('path_publish_hz', 10.0)
        self.declare_parameter('path_max_poses', 5000)
        # ---------- 冷启动位姿 ----------
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.20)
        self.declare_parameter('initial_yaw', 0.0)
        # ---------- 控制与安全参数 ----------
        # command_timeout_sec：指令超时（超时后轮子目标清零）
        # startup_hold_kp/kd：启动保持阶段的腿部 PD 增益
        # wheel_hold_kd：轮子保持阻尼；parking_brake_*：驻车制动
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
        # ---------- 状态判定与坐标系参数 ----------
        # min_base_height：最低机身高度（过低判故障）
        # max_tilt_rad：最大倾斜角（过度倾斜判故障）
        # latch_faults / pause_on_fault：锁存故障并冻结故障现场
        # standing_ready_height / standing_ready_stable_sec：站立就绪判据
        self.declare_parameter('min_base_height', 0.05)
        self.declare_parameter('max_tilt_rad', 1.20)
        self.declare_parameter('latch_faults', True)
        self.declare_parameter('pause_on_fault', True)
        self.declare_parameter('standing_ready_height', 0.55)
        self.declare_parameter('standing_ready_stable_sec', 0.30)
        # map_frame / base_frame：世界系与机体系名称
        self.declare_parameter('map_frame', 'world')
        self.declare_parameter('base_frame', 'base_link')
        # ---------- 话题参数 ----------
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

        # ---------- 加载模型 ----------
        model_path = FilePath(
            str(self.get_parameter('model_xml_path').value)
        ).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(
                f'MuJoCo world does not exist: {model_path}'
            )
        self._model = mujoco.MjModel.from_xml_path(str(model_path))
        # 官方后端要求恰好 16 个执行器（12 腿 + 4 轮）。
        if self._model.nu != 16:
            raise RuntimeError(
                f'official M20 backend requires 16 actuators, '
                f'got {self._model.nu}'
            )
        self._data = mujoco.MjData(self._model)
        # 物理步长（下限 0.1ms），并写回模型 opt.timestep。
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
        # ---------- 内嵌 viewer 参数（仅遗留模式） ----------
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
        self._viewer_max_fps = max(
            1.0,
            float(self.get_parameter('viewer_max_fps').value),
        )
        # 每多少物理步同步一次 viewer（由 max_fps 与 dt 计算）。
        self._viewer_step_interval = max(
            1,
            int(math.ceil(1.0 / (self._viewer_max_fps * self._dt))),
        )
        self._last_viewer_sync_step = -self._viewer_step_interval
        self._viewer_follow_body_id: Optional[int] = None
        # ---------- 控制与安全状态 ----------
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
        self._latch_faults = bool(
            self.get_parameter('latch_faults').value
        )
        self._pause_on_fault = bool(
            self.get_parameter('pause_on_fault').value
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

        # ---------- 初始化控制向量 ----------
        self._set_initial_state()
        # 执行器力矩上下限（来自 MJCF actuator ctrlrange）。
        self._ctrl_low = self._model.actuator_ctrlrange[:, 0].copy()
        self._ctrl_high = self._model.actuator_ctrlrange[:, 1].copy()
        # 期望位置/速度/前馈与 PD 增益（16 维）。
        self._kp = np.zeros(16)
        self._kd = np.zeros(16)
        self._desired_position = JOINT_INITIAL_POSITION.copy()
        self._desired_velocity = np.zeros(16)
        self._feedforward = np.zeros(16)
        # 启动保持：腿部用较大 kp 站稳，轮子仅加阻尼。
        startup_kp = float(self.get_parameter('startup_hold_kp').value)
        startup_kd = float(self.get_parameter('startup_hold_kd').value)
        self._kp[LEG_INDICES] = startup_kp
        self._kd[LEG_INDICES] = startup_kd
        self._kd[WHEEL_INDICES] = float(
            self.get_parameter('wheel_hold_kd').value
        )
        self._last_torque = np.zeros(16)
        self._last_command_monotonic: Optional[float] = None
        # 启动保持：在收到官方 SDK 激活指令前保持站立。
        self._startup_hold = True
        self._command_timed_out = False
        # 驻车制动状态。
        self._parking_brake_enabled = bool(
            self.get_parameter('parking_brake_enabled').value
        )
        self._parking_brake_kd = max(
            0.0,
            float(self.get_parameter('parking_brake_kd').value),
        )
        self._locomotion_mode = 'BACKEND_HOLD'
        self._parking_brake_active = self._parking_brake_enabled
        # 故障/就绪状态。
        self._fault = ''
        self._ready = False
        self._standing_since: Optional[float] = None
        self._steps = 0
        # 仓库障碍接触统计（供诊断锁存）。
        self._obstacle_contact_count = 0
        self._obstacle_contact_event_count = 0
        self._obstacle_contact_active = False
        self._obstacle_contact_peak_force = 0.0
        self._obstacle_contact_pairs = set()
        # 墙钟与仿真时钟起点（用于实时倍率测量）。
        self._wall_start = time.monotonic()
        self._sim_start = float(self._data.time)

        # ---------- 计算各发布周期的步数间隔 ----------
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

        # ---------- 话题/服务声明 ----------
        # 就绪与故障话题用 TRANSIENT_LOCAL 锁存，迟到订阅者也能拿到状态。
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
        # 机身位姿（Odometry）、历史轨迹、标准关节状态。
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
        # SDK 风格关节数据与 IMU 数据（drdds 自定义消息）。
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
        # 订阅官方低层指令 /JOINTS_CMD 与运动模式话题。
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
        # TF 广播（world → base_link）与轨迹容器。
        self._tf_broadcaster = TransformBroadcaster(self)
        self._path = Path()
        self._path.header.frame_id = self._map_frame
        # ---------- 内嵌 viewer（仅遗留直跑模式） ----------
        self._viewer = None
        if self._use_viewer:
            from mujoco import viewer as mujoco_viewer
            self._viewer = mujoco_viewer.launch_passive(
                self._model,
                self._data,
            )
            self._configure_viewer_camera()

        # 启动即发布一次状态（未就绪），并打印启动日志。
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
        # 设置初始位姿：自由关节 qpos = [x, y, z, quat(wxyz), 关节角×16]，
        # 速度清零，再 mj_forward 初始化运动学。
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
        """Set a direct-follow free camera centered on the M20 base."""
        # 内嵌 viewer 相机配置：不用 TRACKING 模式（目标平滑会滞后），
        # 改用 FREE 相机，只按渲染节奏更新 lookat 点，保留鼠标旋转/缩放。
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
        self._viewer_follow_body_id = body_id
        with self._viewer.lock():
            # MuJoCo's TRACKING camera deliberately smooths the target and
            # therefore appears to lag behind a moving robot.  Keep a FREE
            # camera and update only its look-at point at the render cadence.
            # Mouse rotation and zoom remain available without target drift.
            self._viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self._viewer.cam.trackbodyid = -1
            self._viewer.cam.distance = self._viewer_distance
            self._viewer.cam.azimuth = self._viewer_azimuth
            self._viewer.cam.elevation = self._viewer_elevation
            self._viewer.cam.lookat[:] = self._data.xpos[body_id]
        self.get_logger().info(
            f'MuJoCo viewer directly following {body_name}: '
            f'distance={self._viewer_distance:.2f}m, '
            f'azimuth={self._viewer_azimuth:.1f}deg, '
            f'elevation={self._viewer_elevation:.1f}deg, '
            f'max_fps={self._viewer_max_fps:.1f}'
        )

    def _sync_viewer(self) -> None:
        """Copy physics to the GUI without native tracking-camera lag."""
        # 按 viewer_step_interval 间隔同步物理到 GUI；
        # 每次外层循环只 sync 一次，防止补跑时渲染突发，物理优先。
        if self._viewer is None or (
            self._steps - self._last_viewer_sync_step
            < self._viewer_step_interval
        ):
            return
        if self._viewer_follow_body_id is not None:
            with self._viewer.lock():
                self._viewer.cam.lookat[:] = self._data.xpos[
                    self._viewer_follow_body_id
                ]
        self._viewer.sync()
        # One sync per completed block of simulation steps makes rendering
        # yield to physics when the host cannot maintain real time.  Calling
        # this once per outer loop also prevents catch-up render bursts.
        self._last_viewer_sync_step = self._steps

    @staticmethod
    def _is_active_motor_command(message: JointsDataCmd) -> bool:
        """Distinguish stand-up/RL commands from SDK reset control words."""
        # 判断指令是否为「激活的电机指令」：只要 16 关节中有任一
        # kp/kd/position/velocity/torque 分量非零，就视为激活，
        # 用于区分真实的站立/RL 指令与 SDK 复位控制字。
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
        # 官方低层指令回调：
        # 1) 校验 16 关节载荷；
        # 2) 启动保持阶段忽略非激活指令，收到首个激活指令后解除保持；
        # 3) 把 SDK 坐标（位置/速度/前馈）经 sdk_to_raw 换算为 MuJoCo 原始坐标，
        #    并保存 kp/kd 与指令时间戳（用于超时保护）。
        if self._fault and self._latch_faults:
            # Do not accept late policy output after a terminal posture fault.
            return
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
        # 运动模式回调：只有明确的行进模式（轮巡航/协同转向/横移）才松开
        # 驻车制动；其余模式（含默认 BACKEND_HOLD）都保持轮子制动。
        mode = str(message.data)
        moving_modes = {
            'WHEEL_CRUISE',
            'COORDINATED_TURN',
            'LATERAL_MANEUVER',
        }
        brake_active = (
            self._parking_brake_enabled
            and (
                (bool(self._fault) and self._latch_faults)
                or mode not in moving_modes
            )
        )
        # 模式与制动状态都没变化时直接返回，避免刷屏。
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
        # 指令超时保护：启动保持或从未收到指令时跳过；
        # 超时后把轮子目标速度与前馈清零、轮子阻尼提升到 wheel_hold_kd，
        # 保证断链时机器人安全停轮。
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
        # 单步物理推进：
        # 1) 超时保护；2) 需要时施加轮子驻车制动；
        # 3) 计算 PD 力矩并写入 ctrl；4) mj_step 推进；
        # 5) 统计障碍接触；6) 关节速度限幅；
        # 7) 故障/就绪更新；8) 按周期发布状态/轨迹/诊断。
        self._enforce_command_timeout(now)
        if self._fault and self._latch_faults and self._pause_on_fault:
            # A physical posture fault is terminal for this validation run.
            # Freeze the first-fault state instead of letting the policy keep
            # driving a fallen model, while ROS status/diagnostics stay alive.
            self._last_torque.fill(0.0)
            self._data.ctrl.fill(0.0)
            self._steps += 1
            if self._steps % self._state_every == 0:
                self._publish_robot_state()
            if self._steps % self._path_every == 0:
                self._publish_path()
            if self._steps % self._diagnostics_every == 0:
                self._publish_diagnostics()
            return
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
        # 关节速度限幅（避免数值发散导致关节超速）。
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

    def _update_obstacle_contacts(self) -> None:
        """Latch brief obstacle contacts that a 10 Hz diagnostic may miss."""
        # 每物理步统计仓库接触；把「是否接触」的边沿锁存为事件计数，
        # 峰值力与接触对持续累计，供低频诊断读取（短暂擦碰不丢）。
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
        # 故障检测：
        #   - qpos/qvel 出现非有限值 → NONFINITE_STATE
        #   - 机身高度低于 min_base_height → BASE_HEIGHT_LOW
        #   - 横滚/俯仰超过 max_tilt → EXCESSIVE_TILT
        # 默认锁存首个物理故障；仿真重启才解除。这样既保留首因，也避免
        # 临界倾角附近反复“清除/重触发”而让上游在 HOLD 与运动间振荡。
        if self._fault and self._latch_faults:
            return
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
        # 就绪门控：只有「机身高度达标 + 姿态接近水平」持续
        # standing_ready_stable_sec 秒才置 ready=True，
        # 之后才发布 odometry/TF/导航相关话题（见 _publish_robot_state）。
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
        # 发布就绪（Bool）与故障（String）锁存状态。
        self._ready_pub.publish(Bool(data=self._ready))
        self._fault_pub.publish(String(data=self._fault))

    def _publish_robot_state(self) -> None:
        # 机器人状态发布（200Hz）：
        # 1) /JOINTS_DATA：SDK 坐标系下的关节位置/速度/力矩 + 温度字段；
        # 2) /IMU_DATA：SDK IMU 消息（RPY 角、角速度、加速度）；
        # 3) 就绪后：/m20/sim/body_pose（Odometry）、world→base_link TF、
        #    /joint_states（标准 ROS 消息）。
        stamp = self.get_clock().now().to_msg()
        position = np.nan_to_num(self._data.qpos[:3])
        quaternion = normalized_quaternion(self._data.qpos[3:7])
        generalized_velocity = np.nan_to_num(self._data.qvel[:6])
        # MuJoCo free-joint translation is world-frame while rotation is
        # body-frame. ROS Odometry requires the complete twist to use
        # child_frame_id, so rotate only the linear component to base_link.
        # MuJoCo 自由关节：平移是世界系、旋转是机体系；ROS Odometry
        # 的 twist 要求整体以 child_frame_id（base_link）为参考系，
        # 因此只把线速度旋转到机体系，角速度保持机体系。
        linear_velocity_world = generalized_velocity[:3]
        linear_velocity_body = world_vector_to_body(
            linear_velocity_world,
            quaternion,
        )
        angular_velocity_body = generalized_velocity[3:6]

        # ---------- 1) SDK 关节数据 /JOINTS_DATA ----------
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
            # SDK 兼容字段：name 固定为 4 个空格字节、data_id=0、
            # status_word=1（在线）、温度取固定值。
            joint.name = [32, 32, 32, 32]
            joint.data_id = 0
            joint.status_word = 1
            joint.position = float(sdk_position[index])
            joint.velocity = float(sdk_velocity[index])
            joint.torque = float(sdk_torque[index])
            joint.motion_temp = 40.0
            joint.driver_temp = 45.0
        self._joints_pub.publish(joints)

        # ---------- 2) SDK IMU 数据 /IMU_DATA ----------
        # sensordata 布局（由 m20_robot.xml 的 sensor 段决定）：
        #   [0:4]=机体系四元数, [4:7]=加速度, [7:10]=角速度。
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

        # Publish joint states even while the backend is standing up so RViz
        # can render the complete URDF immediately.  Odometry/TF remain gated
        # until the physical body is stable.
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = list(JOINT_NAMES)
        joint_state.position = raw_position.tolist()
        joint_state.velocity = raw_velocity.tolist()
        joint_state.effort = self._last_torque.tolist()
        self._joint_state_pub.publish(joint_state)

        # ---------- 3) 就绪后：里程计 / TF ----------
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
        odometry.twist.twist.linear.x = float(linear_velocity_body[0])
        odometry.twist.twist.linear.y = float(linear_velocity_body[1])
        odometry.twist.twist.linear.z = float(linear_velocity_body[2])
        odometry.twist.twist.angular.x = float(angular_velocity_body[0])
        odometry.twist.twist.angular.y = float(angular_velocity_body[1])
        odometry.twist.twist.angular.z = float(angular_velocity_body[2])
        self._body_pose_pub.publish(odometry)

        # 广播与 Odometry 同一时刻的动态 TF（world/map -> base_link），
        # 使 robot_state_publisher 发布的关节树能够连接到 RViz 固定坐标系。
        transform = TransformStamped()
        transform.header = odometry.header
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = odometry.pose.pose.position.x
        transform.transform.translation.y = odometry.pose.pose.position.y
        transform.transform.translation.z = odometry.pose.pose.position.z
        transform.transform.rotation = odometry.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

    def _publish_path(self) -> None:
        # 轨迹发布（10Hz）：把机身位置追加到 /quad_0/path，
        # 超过 path_max_poses 时只保留尾部，供 Rviz/导航查看历史轨迹。
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
        # 诊断发布（10Hz）：聚合就绪/故障/模式/制动/超时/时间/位姿/速度/
        # 接触统计/最大力矩等状态为 JSON 字符串，发到 /m20/sim/dynamics_state，
        # 供监控面板与离线分析使用（紧凑分隔符减小带宽）。
        now = time.monotonic()
        wall_elapsed = max(1.0e-9, now - self._wall_start)
        simulation_elapsed = float(self._data.time) - self._sim_start
        command_age = (
            None
            if self._last_command_monotonic is None
            else now - self._last_command_monotonic
        )
        quaternion = normalized_quaternion(self._data.qpos[3:7])
        roll, pitch, yaw = quaternion_to_rpy(quaternion)
        generalized_velocity = np.nan_to_num(self._data.qvel[:6])
        linear_velocity_world = generalized_velocity[:3]
        linear_velocity_body = world_vector_to_body(
            linear_velocity_world,
            quaternion,
        )
        angular_velocity_body = generalized_velocity[3:6]
        state = {
            'ready': self._ready,
            'fault': self._fault,
            'fault_latched': bool(self._fault and self._latch_faults),
            'physics_paused_on_fault': bool(
                self._fault and self._latch_faults and self._pause_on_fault
            ),
            'startup_hold': self._startup_hold,
            'locomotion_mode': self._locomotion_mode,
            'parking_brake_active': self._parking_brake_active,
            'command_timed_out': self._command_timed_out,
            'command_age_sec': command_age,
            'sim_time_sec': float(self._data.time),
            # 实测实时倍率 = 仿真时间增量 / 墙钟时间增量。
            'measured_real_time_factor': simulation_elapsed / wall_elapsed,
            'base_position': self._data.qpos[:3].tolist(),
            'base_rpy': [roll, pitch, yaw],
            # Compatibility field: MuJoCo free-joint qvel stores world-frame
            # translation followed by body-frame angular velocity.
            # 兼容字段：qvel 前半段是世界系线速度、后半段是机体系角速度。
            'base_velocity': generalized_velocity.tolist(),
            'base_linear_velocity_world': (
                linear_velocity_world.tolist()
            ),
            'base_linear_velocity_body': (
                linear_velocity_body.tolist()
            ),
            'base_angular_velocity_body': (
                angular_velocity_body.tolist()
            ),
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
        # 主循环：按 period = dt / rtf 的单调节拍推进物理，
        # 落后时最多补跑 max_catchup_steps 步，其余时间交给 spin_once
        # 处理回调；每轮同步一次 viewer，viewer 退出或 ROS 关闭即停止。
        period = self._dt / self._real_time_factor
        next_step = time.monotonic()
        while rclpy.ok():
            now = time.monotonic()
            completed = 0
            while now >= next_step and completed < self._max_catchup_steps:
                self._step(now)
                next_step += period
                completed += 1
            # 落后过多时丢弃积压节拍，避免“追时间”追到天荒地老。
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
            self._sync_viewer()
            if self._viewer and not self._viewer.is_running():
                break

    def close(self) -> None:
        """Publish an unavailable state before releasing the viewer."""
        # 退出前发布「未就绪」状态，随后关闭内嵌 viewer；
        # launch_passive 拥有 GUI 线程，close() 只发退出请求，
        # 睡 1s 让原生渲染循环收尾后再置空引用，避免解释器退出竞态。
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
    # 禁用 rclpy 默认异步信号处理器，保留 Python 的 SIGINT→KeyboardInterrupt
    # 路径，避免 MuJoCo 原生渲染线程与 ROS 上下文回收产生竞态。
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
