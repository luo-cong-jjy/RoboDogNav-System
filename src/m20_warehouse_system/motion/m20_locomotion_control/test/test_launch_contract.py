# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：test_launch_contract.py
# 所属：m20_locomotion_control —— M20 运动控制包的静态契约测试
# 核心职责：对 SDK 启动边界（launch 文件与配置文件）做静态文本契约检查。
#   - 校验 sdk_locomotion.launch.py 的保守默认值（start_sdk=false 等）与参数注入；
#   - 校验 sdk_locomotion.yaml 中导航适配器/进度跟踪器/运动管理器的关键参数；
#   - 校验工厂后端（basic_server/direct_ros）配置与 common Twist 契约、
#     DrDDS 话题与 QoS 契约、优雅停机行为、速度反馈来源支持。
# 这些测试不启动任何节点，只做文本断言。
# ============================================================================
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

"""Static contract checks for the SDK launch boundary."""
# 【中文注释】模块说明：SDK 启动边界的静态契约检查。

from pathlib import Path  # 跨平台路径对象（定位包根目录）


ROOT = Path(__file__).parents[1]  # 【中文注释】包根目录（本文件位于 test/ 下，父级即包根）


def test_sdk_is_opt_in_until_a_joint_backend_exists():
    # 【中文注释】校验：SDK 控制器默认"选择性启动"（start_sdk=false），
    # 直到有联动机器人后端（MuJoCo/仿真/实机）在线。
    launch_text = (
        ROOT / 'launch' / 'sdk_locomotion.launch.py'
    ).read_text(encoding='utf-8')
    assert "default_value='false'" in launch_text       # 默认不启动
    assert "executable='rl_deploy_cmdvel'" in launch_text  # 引用了厂商控制器
    assert "'locomotion_capability_config'" in launch_text  # 能力配置参数存在
    assert 'load_capability_profile' in launch_text     # 使用能力配置文件加载
    assert 'profile.intent_parameters()' in launch_text # 注入意图参数
    assert 'profile.sdk_parameters()' in launch_text    # 注入 SDK 限幅参数
    assert "'max_forward': profile.max_forward" in launch_text  # 数值来自配置
    assert "'max_side': profile.max_side" in launch_text
    assert "'max_yaw': profile.max_yaw" in launch_text
    assert "'max_forward': 0.75" not in launch_text     # 不得出现硬编码旧值


def test_navigation_adapter_precedes_safety_and_backend_gate():
    # 【中文注释】校验：导航适配器位于安全预测与后端门控之前
    # （原始指令 → 候选指令 → 安全指令 → SDK 指令的链路顺序）。
    config_text = (
        ROOT / 'config' / 'sdk_locomotion.yaml'
    ).read_text(encoding='utf-8')
    assert 'input_topic: /m20/navigation/cmd_vel_raw' in config_text  # 输入原始指令
    assert (
        'output_topic: /m20/navigation/cmd_vel_candidate'  # 输出候选指令
        in config_text
    )
    assert 'input_topic: /m20/control/cmd_vel_safe' in config_text  # 管理器输入为安全指令
    assert 'output_topic: /m20/locomotion/cmd_vel_sdk' in config_text  # 管理器输出为 SDK 指令
    assert 'max_forward:' not in config_text  # 数值参数不得硬编码在 yaml 中（由配置注入）
    assert 'turn_min_forward:' not in config_text


def test_measured_progress_tracker_is_an_explicit_execution_profile():
    # 【中文注释】校验：实测进度跟踪器是显式的执行方式（m20_progress），
    # 且对应脚本存在且可执行。
    config_text = (
        ROOT / 'config' / 'sdk_locomotion.yaml'
    ).read_text(encoding='utf-8')
    sdk_launch = (
        ROOT / 'launch' / 'sdk_locomotion.launch.py'
    ).read_text(encoding='utf-8')
    script = ROOT / 'scripts' / 'm20_trajectory_progress_tracker'

    assert 'm20_trajectory_progress_tracker:' in config_text  # 节点配置存在
    assert 'trajectory_topic: /planning/bspline' in config_text  # B 样条话题
    assert 'lookahead_m: 0.60' in config_text              # 前视距离
    assert "'m20_progress'" in sdk_launch                  # 执行方式选项存在
    assert script.exists()                                 # 脚本存在
    assert script.stat().st_mode & 0o111                   # 脚本有可执行权限


def test_autonomous_rolling_does_not_remove_manual_lateral_control():
    # 【中文注释】校验：自主滚动适配不剥夺手动横向控制
    # （NAVIGATION 模式下用滚动适配，MANUAL 模式仍允许横向）。
    config_text = (
        ROOT / 'config' / 'sdk_locomotion.yaml'
    ).read_text(encoding='utf-8')
    assert 'safety_state_topic: /m20/control/safety_state' in config_text  # 安全状态话题
    assert 'rolling_navigation_enabled: false' in config_text  # 管理器层不重复适配
    assert 'allow_manual_lateral: true' in config_text     # 允许手动横向


def test_velocity_feedback_is_staged_before_safety_with_fail_open_timeout():
    # 【中文注释】校验：速度反馈位于安全预测之前且带"故障开放"超时
    # （反馈异常时直通原始指令，而不是卡在旧输出）。
    config_text = (
        ROOT / 'config' / 'sdk_locomotion.yaml'
    ).read_text(encoding='utf-8')
    node_text = (
        ROOT
        / 'm20_locomotion_control'
        / 'navigation_adapter_node.py'
    ).read_text(encoding='utf-8')

    assert 'velocity_feedback_enabled: false' in config_text  # 默认关闭
    assert (
        'velocity_feedback_odometry_topic: /m20/sim/body_pose'
        in config_text
    )
    assert (
        'velocity_feedback_execution_hold_topic: '
        '/m20/control/execution_hold'
        in config_text
    )
    assert 'velocity_feedback_expected_child_frame: base_link' in config_text
    assert 'velocity_feedback_cruise_only: true' in config_text  # 仅巡航反馈
    assert (
        'velocity_feedback_max_abs_yaw_reference: 0.08'
        in config_text
    )
    assert "reason = 'MANEUVER_GATED'" in node_text  # 机动时旁路反馈
    assert 'ODOMETRY_STALE' in node_text             # 测量过期处理
    assert 'ODOMETRY_FRAME_MISMATCH' in node_text    # 帧不匹配处理
    assert 'EXECUTION_HOLD' in node_text             # 执行保持处理
    assert 'feedback_result.output' in node_text     # 使用反馈输出


def test_factory_backends_are_guarded_behind_the_common_twist_contract():
    # 【中文注释】校验：工厂后端（basic_server/direct_ros）都受公共 Twist 契约保护
    # （输入均为 /m20/locomotion/cmd_vel_sdk，默认不自动启用运动）。
    config_text = (
        ROOT / 'config' / 'm20_factory_basic_server.yaml'
    ).read_text(encoding='utf-8')
    launch_text = (
        ROOT / 'launch' / 'official_locomotion.launch.py'
    ).read_text(encoding='utf-8')
    direct_text = (
        ROOT / 'config' / 'm20_factory_direct_ros.yaml'
    ).read_text(encoding='utf-8')
    direct_node_text = (
        ROOT
        / 'm20_locomotion_control'
        / 'direct_ros_backend_node.py'
    ).read_text(encoding='utf-8')

    assert 'input_topic: /m20/control/cmd_vel_safe' in config_text  # 公共安全指令入口
    assert 'output_topic: /m20/locomotion/cmd_vel_sdk' in config_text  # 公共 SDK 出口
    assert 'input_topic: /m20/locomotion/cmd_vel_sdk' in config_text   # 后端输入
    assert 'command_rate_hz: 20.0' in config_text      # 指令频率
    assert 'command_timeout_sec: 0.30' in config_text  # 指令超时
    assert 'auto_enable_motion: false' in config_text  # 不自动启用
    assert 'command_ownership_confirmed: false' in config_text  # 所有权未确认
    assert "default_value='false'" in launch_text      # 启动参数默认 false
    assert 'm20_factory_agile_flat_capabilities.yaml' in launch_text  # 默认能力配置
    assert "{'basic_server', 'direct_ros'}" in launch_text  # 传输方式选项
    assert "else 'm20_direct_ros_backend'" in launch_text  # 后端选择逻辑
    assert 'nav_cmd_topic: /NAV_CMD' in direct_text   # DrDDS 话题
    assert 'motion_info_topic: /MOTION_INFO' in direct_text
    assert 'motion_state_topic: /MOTION_STATE' in direct_text
    assert 'gait_topic: /GAIT' in direct_text
    assert 'command_rate_hz: 20.0' in direct_text
    assert 'command_timeout_sec: 0.30' in direct_text
    assert 'auto_enable_motion: false' in direct_text
    assert 'command_ownership_confirmed: false' in direct_text
    assert 'reliability=ReliabilityPolicy.RELIABLE' in direct_node_text  # QoS 契约
    assert 'durability=DurabilityPolicy.VOLATILE' in direct_node_text
    assert 'self._hard_estop_callback,\n            latched_qos' in (  # HES 用锁存 QoS
        direct_node_text
    )


def test_direct_ros_shutdown_sends_zero_before_destroying_ros_context():
    # 【中文注释】校验：direct_ros 后端在销毁 ROS 上下文前先发送零指令
    # （SIGINT/SIGTERM 信号处理 + 手动自旋循环）。
    node_text = (
        ROOT
        / 'm20_locomotion_control'
        / 'direct_ros_backend_node.py'
    ).read_text(encoding='utf-8')

    assert 'signal.signal(signal.SIGINT, request_stop)' in node_text   # SIGINT 处理
    assert 'signal.signal(signal.SIGTERM, request_stop)' in node_text  # SIGTERM 处理
    assert 'while rclpy.ok() and not stop_requested:' in node_text     # 自旋循环
    assert 'if rclpy.ok():\n            node.stop()' in node_text      # 先停再销毁


def test_feedback_can_use_factory_motion_status_without_fake_odometry():
    # 【中文注释】校验：速度反馈可使用工厂运动状态（twist 源），
    # 无需伪造里程计（支持 odometry 与 twist 两种来源）。
    node_text = (
        ROOT
        / 'm20_locomotion_control'
        / 'navigation_adapter_node.py'
    ).read_text(encoding='utf-8')

    assert "'velocity_feedback_source', 'odometry'" in node_text  # 默认来源
    assert 'velocity_feedback_twist_topic' in node_text           # twist 话题参数
    assert "{'odometry', 'twist'}" in node_text                   # 来源取值校验
    assert 'def _twist_callback' in node_text                     # twist 回调存在
