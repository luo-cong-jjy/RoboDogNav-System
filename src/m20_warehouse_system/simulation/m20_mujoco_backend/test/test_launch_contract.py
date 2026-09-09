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
# 文件：test_launch_contract.py
# 用途：对「全系统 MuJoCo 接口契约」的静态检查（pytest）。
#       通过字符串断言锁定后端/launch/查看器源码中的关键话题名、
#       实现模式与参数，防止后续重构意外破坏接口约定
#       （例如把 viewer 改成可推进物理、改掉话题、去掉独立 viewer）。
#       注意：这些断言只针对包内源码文本，不启动任何 ROS 节点。
# ============================================================

"""Static checks for the full-system MuJoCo interface contract."""

from pathlib import Path

# 包根目录（test/ 的上一级）。
ROOT = Path(__file__).parents[1]


def test_backend_owns_current_scan_pose_and_sdk_topics():
    # 后端节点源码契约：
    #   - 必须拥有 /m20/sim/body_pose、/quad_0/path 与 SDK 话题
    #     /JOINTS_CMD、/JOINTS_DATA、/IMU_DATA、/m20/locomotion/mode；
    #   - 必须使用 FREE 相机同步（mjCAMERA_FREE + _sync_viewer），
    #     并按 viewer_step_interval 限频同步（_last_viewer_sync_step）；
    #   - 必须实现轮子制动（apply_wheel_brake）与世界系→机体系
    #     速度换算（world_vector_to_body），且诊断含三组速度字段。
    source = (
        ROOT / 'm20_mujoco_backend' / 'backend_node.py'
    ).read_text(encoding='utf-8')
    assert "'/m20/sim/body_pose'" in source
    assert "'/quad_0/path'" in source
    assert "'/JOINTS_CMD'" in source
    assert "'/JOINTS_DATA'" in source
    assert "'/IMU_DATA'" in source
    assert 'mjtCamera.mjCAMERA_FREE' in source
    assert 'def _sync_viewer(' in source
    assert 'self._viewer_step_interval = max(' in source
    assert 'self._last_viewer_sync_step = self._steps' in source
    assert 'self._viewer.cam.lookat[:]' in source
    assert "'/m20/locomotion/mode'" in source
    assert 'apply_wheel_brake(' in source
    assert 'world_vector_to_body(' in source
    assert 'TransformStamped' in source
    assert 'TransformBroadcaster' in source
    assert 'self._tf_broadcaster.sendTransform(transform)' in source
    assert "self.declare_parameter('latch_faults', True)" in source
    assert "self.declare_parameter('pause_on_fault', True)" in source
    assert 'if self._fault and self._latch_faults:' in source
    assert 'Do not accept late policy output' in source
    assert "'fault_latched'" in source
    assert "'physics_paused_on_fault'" in source
    assert "'base_linear_velocity_world'" in source
    assert "'base_linear_velocity_body'" in source
    assert "'base_angular_velocity_body'" in source


def test_world_launch_uses_selected_system_configuration():
    # launch 文件契约：
    #   - 必须通过 LaunchConfiguration('system_config') 选择系统配置；
    #   - 必须调用 generate_world 生成世界；
    #   - 必须把 viewer_max_fps 透传给独立 viewer 节点
    #     （executable='m20_mujoco_viewer'），物理节点 use_viewer=False；
    #   - viewer 必须以 nice -n 10 低优先级运行。
    source = (
        ROOT / 'launch' / 'mujoco_backend.launch.py'
    ).read_text(encoding='utf-8')
    assert "LaunchConfiguration('system_config')" in source
    assert 'generate_world(' in source
    assert "'viewer_max_fps': LaunchConfiguration(" in source
    assert "'viewer_max_fps'" in source
    assert "executable='m20_mujoco_viewer'" in source
    assert "'use_viewer': False" in source
    assert "prefix='nice -n 10'" in source


def test_viewer_is_a_read_only_ros_state_replica():
    # viewer 节点契约：
    #   - 订阅 /m20/sim/body_pose 与 /joint_states；
    #   - 是只读显示副本：允许 mj_forward（运动学），但绝不出现
    #     mujoco.mj_step(（不推进物理），也不创建 launch_passive 被动查看器；
    #   - 使用 FREE 相机、支持 low_cost_render（关阴影）、
    #     GLFW 自绘渲染（swap_buffers / mjv_updateScene / mjr_render）；
    #   - 日志必须声明 GUI 时序不阻塞物理。
    source = (
        ROOT / 'm20_mujoco_backend' / 'viewer_node.py'
    ).read_text(encoding='utf-8')
    assert "'/m20/sim/body_pose'" in source
    assert "'/joint_states'" in source
    assert 'mujoco.mj_step(' not in source
    assert 'mujoco.mj_forward(' in source
    assert 'mjtCamera.mjCAMERA_FREE' in source
    assert "self.declare_parameter('low_cost_render', True)" in source
    assert 'light_castshadow[:]' in source
    assert 'glfw.swap_buffers(' in source
    assert 'mujoco.mjv_updateScene(' in source
    assert 'mujoco.mjr_render(' in source
    assert 'launch_passive(' not in source
    assert 'GUI timing cannot block physics' in source
