# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：test_direct_ros_policy.py
# 所属：m20_locomotion_control —— M20 运动控制包的单元测试
# 核心职责：测试显式的工厂 ROS 2 运动转移契约（direct_ros_policy.py）。
#   - 解码当前 deep-robotics MotionInfo 嵌套 ABI（state/gait 在 value 消息内）；
#   - 状态机转移：STAND → RL_CONTROL、步态切换需静止、就绪判定；
#   - fail-closed：所有权未确认 / 急停触发均输出故障。
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

"""Tests for the explicit factory ROS 2 motion transition contract."""
# 【中文注释】模块说明：显式工厂 ROS 2 运动转移契约的测试。

from types import SimpleNamespace  # 简易命名空间（构造模拟消息对象）

from m20_locomotion_control.direct_ros_policy import (  # 被测模块
    decode_motion_info,             #   解码 MotionInfo
    DirectMotionStatus,             #   运动状态数据类
    select_direct_transition,       #   状态机转移决策
)


def test_current_deep_robotics_motion_info_nested_abi_is_decoded():
    # 【中文注释】校验：当前 deep-robotics MotionInfo 嵌套 ABI 能被正确解码
    # （state 与 gait 分别嵌套在 motion_state/gait_state 的 value 消息里）。
    data = SimpleNamespace(
        vel_x=0.21,
        vel_y=-0.04,
        vel_yaw=0.33,
        motion_state=SimpleNamespace(state=17),
        gait_state=SimpleNamespace(gait=0x3002),
    )

    status = decode_motion_info(data)

    assert status.state == 17          # RL 控制状态
    assert status.gait == 0x3002       # 敏捷平地步态
    assert status.linear_x == 0.21     # 实测速度
    assert status.linear_y == -0.04
    assert status.angular_z == 0.33


def _status(state, gait=0x1001, velocity=0.0):
    # 【中文注释】构造一个运动状态对象（默认静止、默认步态 0x1001）。
    return DirectMotionStatus(
        state=state,
        gait=gait,
        linear_x=velocity,
        linear_y=0.0,
        angular_z=0.0,
    )


def test_enable_explicitly_advances_stand_then_rl_control():
    # 【中文注释】校验：启用时显式推进状态：卧倒(4) → 站立(1) → RL 控制(17)。
    stand = select_direct_transition(
        motion_requested=True,
        ownership_confirmed=True,
        e_stop=False,
        status=_status(4),          # 卧倒状态
        requested_gait=0x3002,
    )
    rl = select_direct_transition(
        motion_requested=True,
        ownership_confirmed=True,
        e_stop=False,
        status=_status(1),          # 站立状态
        requested_gait=0x3002,
    )
    assert stand.state_command == 1    # 卧倒 → 站立
    assert rl.state_command == 17      # 站立 → RL 控制


def test_gait_change_waits_until_motion_info_reports_stationary():
    # 【中文注释】校验：步态切换必须等待 MotionInfo 报告静止（运动时拒绝切换）。
    moving = select_direct_transition(
        motion_requested=True,
        ownership_confirmed=True,
        e_stop=False,
        status=_status(17, velocity=0.10),  # RL 控制但仍在运动
        requested_gait=0x3002,
    )
    stopped = select_direct_transition(
        motion_requested=True,
        ownership_confirmed=True,
        e_stop=False,
        status=_status(17),                # RL 控制且静止
        requested_gait=0x3002,
    )
    assert moving.gait_command is None     # 运动时不切步态
    assert not moving.ready
    assert stopped.gait_command == 0x3002  # 静止时发送步态指令


def test_ready_requires_rl_control_and_requested_gait():
    # 【中文注释】校验：就绪要求状态为 RL 控制且步态已与请求一致。
    result = select_direct_transition(
        motion_requested=True,
        ownership_confirmed=True,
        e_stop=False,
        status=_status(17, gait=0x3002),
        requested_gait=0x3002,
    )
    assert result.ready
    assert not result.fault


def test_estop_and_ownership_are_fail_closed():
    # 【中文注释】校验：急停与所有权未确认均为 fail-closed（输出故障，不输出指令）。
    ownership = select_direct_transition(
        motion_requested=True,
        ownership_confirmed=False,   # 所有权未确认
        e_stop=False,
        status=_status(17, gait=0x3002),
        requested_gait=0x3002,
    )
    estop = select_direct_transition(
        motion_requested=True,
        ownership_confirmed=True,
        e_stop=True,                 # 急停触发
        status=_status(17, gait=0x3002),
        requested_gait=0x3002,
    )
    assert ownership.fault == 'COMMAND_OWNERSHIP_NOT_CONFIRMED'
    assert estop.fault == 'E_STOP_ASSERTED'
