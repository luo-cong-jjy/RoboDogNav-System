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
# 文件：viewer_node.py
# 用途：M20 MuJoCo「只读显示进程」（节点名 m20_mujoco_viewer）。
#       - 独立于物理后端进程运行：订阅 /m20/sim/body_pose（Odometry）
#         与 /joint_states（JointState），把 ROS 状态镜像到一个
#         「不推进物理」的 MuJoCo 模型中用于 3D 显示；
#       - 自带 GLFW 窗口与鼠标旋转/缩放/滚轮控制，按 max_fps 限帧，
#         GUI 卡顿不会阻塞 1kHz 物理循环（见 run() 的独立节拍）；
#       - 支持 low_cost_render（关闭阴影/反射/抗锯齿），
#         仅影响显示副本，不影响物理模型；
#       - 由 launch 以 nice -n 10 低优先级启动，仅供诊断观察。
# ============================================================

"""Display a read-only M20 MuJoCo replica outside the physics process."""

from __future__ import annotations

import math
from pathlib import Path
import signal
import threading
import time

import glfw
from nav_msgs.msg import Odometry
import mujoco
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray

from .dynamics import JOINT_INITIAL_POSITION, JOINT_NAMES, yaw_quaternion


class M20MujocoViewer(Node):
    """Mirror ROS state into an isolated, non-physical MuJoCo viewer."""

    def __init__(self) -> None:
        super().__init__('m20_mujoco_viewer')
        # ---------- 参数声明 ----------
        # model_xml_path：world_generator 生成的世界文件路径（必填）
        # pose_topic / joint_states_topic：订阅的位姿与关节话题
        # follow_body：跟随的机体（默认 base_link）
        # distance/azimuth/elevation：跟随相机初始参数
        # max_fps：显示帧率上限；low_cost_render：低开销渲染开关
        # initial_x/y/z/yaw：冷启动位姿
        self.declare_parameter('model_xml_path', '')
        self.declare_parameter('pose_topic', '/m20/sim/body_pose')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter(
            'dynamic_obstacles_topic',
            '/m20/factory/dynamic_obstacles',
        )
        self.declare_parameter('follow_body', 'base_link')
        # Keep the viewer camera free by default.  A continuously forced
        # top-down follow camera is useful for demos but makes diagnosing
        # GLFW/HiDPI rendering issues difficult.
        self.declare_parameter('follow_camera', False)
        self.declare_parameter('distance', 4.0)
        self.declare_parameter('azimuth', 90.0)
        self.declare_parameter('elevation', -89.0)
        self.declare_parameter('max_fps', 30.0)
        self.declare_parameter('low_cost_render', True)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.20)
        self.declare_parameter('initial_yaw', 0.0)

        # 加载世界模型（由 launch 生成的 MJCF 文件）。
        model_path = Path(
            str(self.get_parameter('model_xml_path').value)
        ).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(
                f'MuJoCo viewer world does not exist: {model_path}'
            )
        self._model = mujoco.MjModel.from_xml_path(str(model_path))
        self._low_cost_render = bool(
            self.get_parameter('low_cost_render').value
        )
        self._follow_camera = bool(
            self.get_parameter('follow_camera').value
        )
        if self._low_cost_render:
            # These settings affect only the read-only display replica.  The
            # physical model keeps its original contacts and materials.
            # 低开销渲染：只修改显示副本，关闭阴影、反射、抗锯齿，
            # 物理模型保持原有接触与材质不变。
            self._model.light_castshadow[:] = 0
            self._model.mat_reflectance[:] = 0.0
            # Zero-sized shadow buffers trigger incomplete/black regions on
            # some WSLg OpenGL implementations; keep the smallest valid size.
            self._model.vis.quality.shadowsize = 1
            self._model.vis.quality.offsamples = 0
        self._data = mujoco.MjData(self._model)
        # 冷启动位姿：自由关节 qpos 布局为 [x, y, z, quat(wxyz), 关节角×16]。
        self._data.qpos[:] = 0.0
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

        # 解析跟随机体 id（用于相机 lookat 跟踪）。
        body_name = str(self.get_parameter('follow_body').value)
        self._body_id = mujoco.mj_name2id(
            self._model,
            mujoco.mjtObj.mjOBJ_BODY,
            body_name,
        )
        if self._body_id < 0:
            raise RuntimeError(
                f'MuJoCo viewer follow body does not exist: {body_name}'
            )
        # 建立「关节名 → qpos 地址」映射，供关节回调直接写入。
        self._joint_qpos = {}
        for name in JOINT_NAMES:
            joint_id = mujoco.mj_name2id(
                self._model,
                mujoco.mjtObj.mjOBJ_JOINT,
                name,
            )
            if joint_id >= 0:
                self._joint_qpos[name] = int(
                    self._model.jnt_qposadr[joint_id]
                )
        if len(self._joint_qpos) != len(JOINT_NAMES):
            raise RuntimeError(
                'MuJoCo viewer could not resolve all official M20 joints'
            )
        self._dynamic_mocap = {}
        dynamic_index = 0
        while True:
            body_id = mujoco.mj_name2id(
                self._model,
                mujoco.mjtObj.mjOBJ_BODY,
                f'm20_dynamic_obstacle_{dynamic_index}',
            )
            if body_id < 0:
                break
            mocap_id = int(self._model.body_mocapid[body_id])
            if mocap_id >= 0:
                self._dynamic_mocap[dynamic_index] = mocap_id
            dynamic_index += 1

        # 位姿与关节话题都用 BEST_EFFORT + depth=1 的 QOS，
        # 显示进程允许丢帧，不需要可靠传输。
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('pose_topic').value),
            self._pose_callback,
            qos,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_states_topic').value),
            self._joint_callback,
            qos,
        )
        self.create_subscription(
            MarkerArray,
            str(self.get_parameter('dynamic_obstacles_topic').value),
            self._dynamic_obstacles_callback,
            qos,
        )
        # 帧周期：1 / max_fps（最小 1fps）。
        self._frame_period = 1.0 / max(
            1.0,
            float(self.get_parameter('max_fps').value),
        )
        self._dirty = True
        mujoco.mj_forward(self._model, self._data)
        self._open_window()
        self.get_logger().info(
            'isolated lightweight MuJoCo viewer ready: '
            f'follow={body_name}, '
            f'distance={self._camera.distance:.2f}m, '
            f'azimuth={self._camera.azimuth:.1f}deg, '
            f'elevation={self._camera.elevation:.1f}deg, '
            f'max_fps={1.0 / self._frame_period:.1f}; '
            f'low_cost_render={self._low_cost_render}; '
            'GUI timing cannot block physics'
        )

    def _open_window(self) -> None:
        """Create a frame-capped MuJoCo OpenGL window on this thread."""
        # 初始化 GLFW 并创建 960×720 窗口（不开 MSAA）。
        if not glfw.init():
            raise RuntimeError('GLFW initialization failed')
        glfw.window_hint(glfw.SAMPLES, 0)
        glfw.window_hint(glfw.DOUBLEBUFFER, glfw.TRUE)
        self._window = glfw.create_window(
            960,
            720,
            'M20 MuJoCo - free camera',
            None,
            None,
        )
        if self._window is None:
            glfw.terminate()
            raise RuntimeError('MuJoCo viewer window creation failed')
        glfw.make_context_current(self._window)
        # Avoid an unconstrained native render thread.  The ROS render loop
        # below is the only owner of swap_buffers and enforces max_fps.
        # 垂直同步 1：渲染循环是 swap_buffers 的唯一所有者并按 max_fps 限帧。
        glfw.swap_interval(1)

        # 初始化 MuJoCo 相机/渲染选项/场景/渲染上下文。
        self._camera = mujoco.MjvCamera()
        self._option = mujoco.MjvOption()
        mujoco.mjv_defaultCamera(self._camera)
        mujoco.mjv_defaultOption(self._option)
        # 关闭 geomgroup[1]（碰撞体组），显示时只画外观组。
        self._option.geomgroup[1] = 0
        self._scene = mujoco.MjvScene(self._model, maxgeom=10000)
        self._render_context = mujoco.MjrContext(
            self._model,
            mujoco.mjtFontScale.mjFONTSCALE_150.value,
        )
        # 自由相机 + 手动 lookat 跟踪（不用 TRACKING 模式，避免平滑滞后）。
        self._camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self._camera.trackbodyid = -1
        self._camera.distance = max(
            0.5,
            float(self.get_parameter('distance').value),
        )
        self._camera.azimuth = float(
            self.get_parameter('azimuth').value
        )
        self._camera.elevation = max(
            -89.0,
            min(
                89.0,
                float(self.get_parameter('elevation').value),
            ),
        )
        self._camera.lookat[:] = self._data.xpos[self._body_id]
        # 注册鼠标/滚轮/键盘回调，支持原生旋转缩放。
        self._last_cursor = glfw.get_cursor_pos(self._window)
        glfw.set_cursor_pos_callback(self._window, self._cursor_callback)
        glfw.set_scroll_callback(self._window, self._scroll_callback)
        glfw.set_key_callback(self._window, self._key_callback)

    def _cursor_callback(self, window, x_position, y_position) -> None:
        """Keep native rotate and zoom controls without viewer side panels."""
        # 鼠标拖拽控制：右键=平移（Shift 切换水平/垂直）、
        # 左键=旋转、中键=缩放；无按键按下时直接返回。
        last_x, last_y = self._last_cursor
        self._last_cursor = (x_position, y_position)
        left = glfw.get_mouse_button(
            window,
            glfw.MOUSE_BUTTON_LEFT,
        ) == glfw.PRESS
        middle = glfw.get_mouse_button(
            window,
            glfw.MOUSE_BUTTON_MIDDLE,
        ) == glfw.PRESS
        right = glfw.get_mouse_button(
            window,
            glfw.MOUSE_BUTTON_RIGHT,
        ) == glfw.PRESS
        if not (left or middle or right):
            return
        shift = (
            glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
            or glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
        )
        if right:
            action = (
                mujoco.mjtMouse.mjMOUSE_MOVE_H
                if shift
                else mujoco.mjtMouse.mjMOUSE_MOVE_V
            )
        elif left:
            action = (
                mujoco.mjtMouse.mjMOUSE_ROTATE_H
                if shift
                else mujoco.mjtMouse.mjMOUSE_ROTATE_V
            )
        else:
            action = mujoco.mjtMouse.mjMOUSE_ZOOM
        # 位移量除以窗口高度归一化，再交给 mjv_moveCamera 处理。
        _, height = glfw.get_window_size(window)
        scale = max(1, height)
        mujoco.mjv_moveCamera(
            self._model,
            action,
            (x_position - last_x) / scale,
            (y_position - last_y) / scale,
            self._scene,
            self._camera,
        )

    def _scroll_callback(self, _window, _x_offset, y_offset) -> None:
        """Zoom the direct-follow camera with the mouse wheel."""
        # 滚轮缩放：每格 y_offset 缩放 -5%。
        mujoco.mjv_moveCamera(
            self._model,
            mujoco.mjtMouse.mjMOUSE_ZOOM,
            0.0,
            -0.05 * y_offset,
            self._scene,
            self._camera,
        )

    def _key_callback(self, window, key, _scan, action, _mods) -> None:
        """Close the diagnostic viewer with Escape."""
        # 按 Esc 关闭窗口。
        if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
            glfw.set_window_should_close(window, True)

    def _pose_callback(self, message: Odometry) -> None:
        # 位姿回调：把 Odometry 的位姿写入 qpos。
        # 校验所有分量有限后：
        #   qpos[0:3]  = 位置
        #   ROS 四元数是 xyzw，MuJoCo 自由关节是 wxyz，需交换顺序。
        pose = message.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return
        self._data.qpos[0:3] = values[0:3]
        # ROS uses xyzw while MuJoCo free joints use wxyz.
        self._data.qpos[3:7] = (
            values[6],
            values[3],
            values[4],
            values[5],
        )
        self._dirty = True

    def _joint_callback(self, message: JointState) -> None:
        # 关节回调：按名称把 JointState 位置写入对应 qpos 地址（若存在且有限）。
        for name, position in zip(message.name, message.position):
            address = self._joint_qpos.get(name)
            if address is not None and math.isfinite(float(position)):
                self._data.qpos[address] = float(position)
                self._dirty = True

    def _dynamic_obstacles_callback(self, message: MarkerArray) -> None:
        """Mirror the RViz/lidar dynamic obstacle poses into mocap bodies."""
        for marker in message.markers:
            mocap_id = self._dynamic_mocap.get(int(marker.id))
            if mocap_id is None:
                continue
            pose = marker.pose
            values = (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            if not all(math.isfinite(float(value)) for value in values):
                continue
            self._data.mocap_pos[mocap_id] = values[0:3]
            self._data.mocap_quat[mocap_id] = (
                values[6], values[3], values[4], values[5],
            )
            self._dirty = True

    def _render(self) -> None:
        # 有更新时先 mj_forward 计算运动学（显示副本不推进物理时间）；
        # 相机 lookat 跟随指定机体，然后更新场景并渲染、交换缓冲、轮询事件。
        if self._dirty:
            mujoco.mj_forward(self._model, self._data)
            self._dirty = False
        if self._follow_camera:
            self._camera.lookat[:] = self._data.xpos[self._body_id]
        width, height = glfw.get_framebuffer_size(self._window)
        viewport = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, self._render_context)
        # Explicitly clear the complete window framebuffer.  On WSLg and
        # other HiDPI GLFW backends, relying on mjr_render's implicit clear
        # can leave stale/black strips when the framebuffer is resized.
        mujoco.mjr_rectangle(viewport, 0.58, 0.58, 0.58, 1.0)
        mujoco.mjv_updateScene(
            self._model,
            self._data,
            self._option,
            None,
            self._camera,
            mujoco.mjtCatBit.mjCAT_ALL.value,
            self._scene,
        )
        mujoco.mjr_render(viewport, self._scene, self._render_context)
        glfw.swap_buffers(self._window)
        glfw.poll_events()

    def run(self, stop_requested: threading.Event) -> None:
        """Service ROS state and render without driving simulation time."""
        # 主循环：以 max_fps 为节拍，每帧先 spin_once（超时不超过下一帧），
        # 到点才渲染；显示进程不推进任何物理时间。
        next_frame = time.monotonic()
        while (
            not stop_requested.is_set()
            and rclpy.ok()
            and not glfw.window_should_close(self._window)
        ):
            now = time.monotonic()
            rclpy.spin_once(
                self,
                timeout_sec=max(0.0, min(0.01, next_frame - now)),
            )
            now = time.monotonic()
            if now >= next_frame:
                self._render()
                next_frame = time.monotonic() + self._frame_period

    def close(self) -> None:
        """Request closure of the isolated native window."""
        # 销毁 GLFW 窗口并终止 GLFW，退出前清理原生资源。
        if getattr(self, '_window', None) is not None:
            glfw.destroy_window(self._window)
            self._window = None
        glfw.terminate()


def main(args=None) -> None:
    """Run the read-only MuJoCo display process."""
    stop_requested = threading.Event()

    def request_stop(_signum, _frame) -> None:
        stop_requested.set()

    # Let the render loop close GLFW before ROS tears down its context.  The
    # default asynchronous rclpy signal handler can otherwise leave MuJoCo's
    # native viewer thread alive during interpreter shutdown.
    # 禁用 rclpy 默认异步信号处理器，改为自己捕获 SIGINT/SIGTERM，
    # 先让渲染循环关闭 GLFW，再执行 ROS 收尾，避免原生线程残留。
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = M20MujocoViewer()
    try:
        node.run(stop_requested)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
