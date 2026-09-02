# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：direct_ros_policy.py
# 所属：m20_nav2_locomotion —— M20 运动控制（仓库导航 → Deep Robotics M20 RL 控制器）
# 核心职责：M20 工厂 ROS 2 运动状态机的"纯转移策略"。
#   - 定义运动状态常量（IDLE/STAND/SOFT_ESTOP/BOOT_DAMPING/LIE_DOWN/RL_CONTROL）；
#   - decode_motion_info：解码 deep-robotics-msg 1.1 的 MotionInfoValue ABI
#     （把 DrDDS 布局隔离在传输边界，避免泄漏到公共运动状态机）；
#   - select_direct_transition：根据运动请求、所有权确认、急停、当前状态/步态，
#     确定性地选择下一个显式状态或步态指令（fail-closed）。
# 本模块不依赖 ROS，可直接单元测试。
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

"""Pure transition policy for the M20 factory ROS 2 motion state machine."""
# 【中文注释】模块说明：M20 工厂 ROS 2 运动状态机的纯转移策略（无 ROS 依赖）。

from __future__ import annotations  # 延迟求值类型注解

from dataclasses import dataclass   # 数据类装饰器
from typing import Any, Optional    # 类型提示：任意类型、可选类型


IDLE = 0            # 【中文注释】空闲状态
STAND = 1           # 【中文注释】站立状态（basic_server 会自动推进到 RL 控制 17）
SOFT_ESTOP = 2      # 【中文注释】软急停状态（被锁存后需要复位）
BOOT_DAMPING = 3    # 【中文注释】开机阻尼状态
LIE_DOWN = 4        # 【中文注释】卧倒状态
RL_CONTROL = 17     # 【中文注释】RL 控制状态（可接收速度指令的工作状态）


@dataclass(frozen=True)  # 【中文注释】冻结数据类
class DirectMotionStatus:
    """Motion fields reported by ``drdds/msg/MotionInfo``."""
    # 【中文注释】由 drdds/msg/MotionInfo 上报的运动字段。

    state: int          # 运动状态（如 17 = RL_CONTROL）
    gait: int           # 当前步态（如 0x3002 = 敏捷平地步态）
    linear_x: float     # 实测前向速度（m/s）
    linear_y: float     # 实测横向速度（m/s）
    angular_z: float    # 实测偏航角速度（rad/s）

    @property
    def stationary(self) -> bool:
        """Return whether gait changes are safe under the vendor contract."""
        # 【中文注释】是否静止：速度各分量都小于阈值时，厂商合同下允许切换步态。
        return (
            abs(self.linear_x) < 0.03
            and abs(self.linear_y) < 0.03
            and abs(self.angular_z) < 0.05
        )


def decode_motion_info(data: Any) -> DirectMotionStatus:
    """
    Decode the public deep-robotics-msg 1.1 MotionInfoValue ABI.

    The current vendor interface nests state and gait in their corresponding
    value messages. Keeping this conversion in the transport boundary avoids
    leaking DrDDS layout details into the common locomotion state machine.
    """
    # 【中文注释】解码公开的 deep-robotics-msg 1.1 MotionInfoValue ABI。
    # 当前厂商接口把 state 和 gait 嵌套在各自的 value 消息中；把该转换放在传输边界，
    # 可避免把 DrDDS 布局细节泄漏到公共运动状态机中。
    return DirectMotionStatus(
        state=int(data.motion_state.state),  # 运动状态（嵌套取值）
        gait=int(data.gait_state.gait),      # 步态（嵌套取值）
        linear_x=float(data.vel_x),
        linear_y=float(data.vel_y),
        angular_z=float(data.vel_yaw),
    )


@dataclass(frozen=True)  # 【中文注释】冻结数据类
class DirectTransition:
    """One deterministic state-machine decision."""
    # 【中文注释】一次确定性的状态机决策。

    ready: bool = False        # 是否已就绪（可发送速度指令）
    fault: str = ''            # 故障原因（非空 = 存在故障，fail-closed）
    state_command: Optional[int] = None  # 需要发送的状态指令（如 STAND=1）
    gait_command: Optional[int] = None   # 需要发送的步态指令（如 0x3002）


def select_direct_transition(
    *,
    motion_requested: bool,
    ownership_confirmed: bool,
    e_stop: bool,
    status: Optional[DirectMotionStatus],
    requested_gait: int,
) -> DirectTransition:
    """Choose the next explicit ROS 2 state or gait command."""
    # 【中文注释】选择下一个显式 ROS 2 状态或步态指令。
    # 参数（关键字参数）：motion_requested 是否请求运动；ownership_confirmed 命令所有权是否确认；
    #   e_stop 软急停是否触发；status 当前运动状态（可能为 None）；requested_gait 请求的步态。
    # 返回：DirectTransition 决策（含 ready/fault/state_command/gait_command）。
    if e_stop:  # 急停优先：故障闭锁
        return DirectTransition(fault='E_STOP_ASSERTED')
    if not motion_requested:  # 未请求运动：空决策（保持现状）
        return DirectTransition()
    if not ownership_confirmed:  # 所有权未确认：故障闭锁
        return DirectTransition(fault='COMMAND_OWNERSHIP_NOT_CONFIRMED')
    if status is None:  # 无状态反馈：保守等待
        return DirectTransition()
    if status.state == SOFT_ESTOP:  # 处于软急停锁存：故障闭锁
        return DirectTransition(fault='M20_SOFT_ESTOP_LATCHED')
    if status.state in {IDLE, BOOT_DAMPING, LIE_DOWN}:  # 从空闲/开机阻尼/卧倒 → 站立
        return DirectTransition(state_command=STAND)
    if status.state == STAND:  # 站立 → RL 控制
        return DirectTransition(state_command=RL_CONTROL)
    if status.state != RL_CONTROL:  # 其他未知状态：故障闭锁
        return DirectTransition(
            fault='UNSUPPORTED_MOTION_STATE_{0}'.format(status.state)
        )
    if status.gait != requested_gait:  # 步态不符：仅静止时才发送步态切换
        if status.stationary:
            return DirectTransition(gait_command=requested_gait)
        return DirectTransition()
    return DirectTransition(ready=True)  # 一切就绪：允许速度指令
