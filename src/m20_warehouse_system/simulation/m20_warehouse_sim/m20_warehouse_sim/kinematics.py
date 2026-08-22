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

# ======================================================================
# kinematics.py —— 平面运动学纯函数库（中文注释版）
# 作用：为 RViz 平面运动学后端及其单元测试提供共享的纯函数：
#   1) PlanarState：模拟机器人机身状态（位姿 + 速度）
#   2) normalize_angle：角度归一化到 [-pi, pi]
#   3) clamp_planar_command：把机身坐标系速度指令限制在安全上限内
#   4) integrate_planar：把机身坐标系 Twist 积分成地图坐标系位姿与速度
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""Pure planar kinematics shared by the RViz backend and tests."""

# 数据类装饰器：自动生成 __init__/__repr__ 等
from dataclasses import dataclass
# 数学库：三角函数、pi
import math
# 类型标注：Tuple（元组）
from typing import Tuple


# 机器人机身状态：位姿（x/y/z/yaw）与速度（机身系 + 世界系）
@dataclass
class PlanarState:
    """Pose and world-frame velocity of the simulated robot body."""

    # 世界坐标系 x 位置（米）
    x: float
    # 世界坐标系 y 位置（米）
    y: float
    # 世界坐标系 z 位置（米，高度）
    z: float
    # 航向角 yaw（绕 z 轴，弧度）
    yaw: float
    # 机身坐标系前向速度（米/秒）
    vx_body: float = 0.0
    # 机身坐标系侧向速度（米/秒）
    vy_body: float = 0.0
    # 世界坐标系 x 方向速度（米/秒）
    vx_world: float = 0.0
    # 世界坐标系 y 方向速度（米/秒）
    vy_world: float = 0.0
    # 绕 z 轴角速度（弧度/秒）
    wz: float = 0.0


# 把任意角度归一化到 [-pi, pi] 区间
def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    # 用 atan2(sin, cos) 数学上保证结果落在 [-pi, pi]
    return math.atan2(math.sin(angle), math.cos(angle))


# 把机身坐标系平面速度指令限幅到后端防御性上限内
def clamp_planar_command(
    vx: float,
    vy: float,
    wz: float,
    limits: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Clamp body-frame planar velocity to backend defense limits."""
    # 取各轴上限的绝对值（防御性：即使配置为负数也按正值处理）
    max_vx, max_vy, max_wz = (abs(value) for value in limits)
    return (
        # x 方向限幅到 [-max_vx, max_vx]
        max(-max_vx, min(max_vx, vx)),
        # y 方向限幅到 [-max_vy, max_vy]
        max(-max_vy, min(max_vy, vy)),
        # 角速度限幅到 [-max_wz, max_wz]
        max(-max_wz, min(max_wz, wz)),
    )


# 把机身坐标系 Twist 指令积分 dt 秒，得到新的地图坐标系位姿与速度
def integrate_planar(
    state: PlanarState,
    body_command: Tuple[float, float, float],
    dt: float,
) -> PlanarState:
    """Integrate a body-frame Twist into map-frame pose and velocity."""
    # 时间步长非正：不积分，直接返回原状态
    if dt <= 0.0:
        return state
    # 解包机身坐标系速度指令 (vx, vy, wz)
    vx, vy, wz = body_command
    # 由当前航向角计算旋转矩阵元素
    cosine = math.cos(state.yaw)
    sine = math.sin(state.yaw)
    # 机身系速度旋转到世界系（2D 旋转）
    vx_world = cosine * vx - sine * vy
    vy_world = sine * vx + cosine * vy
    # 返回积分后的新状态（位置按世界系速度累加，航向按角速度累加并归一化）
    return PlanarState(
        x=state.x + vx_world * dt,
        y=state.y + vy_world * dt,
        z=state.z,
        yaw=normalize_angle(state.yaw + wz * dt),
        vx_body=vx,
        vy_body=vy,
        vx_world=vx_world,
        vy_world=vy_world,
        wz=wz,
    )
