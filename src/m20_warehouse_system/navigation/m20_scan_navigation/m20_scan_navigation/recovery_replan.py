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

# =============================================================================
# recovery_replan.py —— 碰撞触发的 SCAN 重规划策略（纯策略模块，无 ROS 依赖）
# 所属模块：m20_scan_navigation（导航网关的配套决策逻辑）
# 职责：根据在线点云碰撞守卫（m20_collision_guard）发布的诊断字符串，
#       判定是否需要停止当前轨迹并让原生 SCAN 重新规划，同时用硬性次数上限
#       约束"单次目标内重规划"的次数，防止重规划变成无界的兜底循环。
# 判定来源：diagnostic 字符串来自碰撞守卫话题 /m20/control/collision_guard_diagnostic，
#       前缀约定见下方两个常量（RECOVERY_BUDGET_EXHAUSTED / RECOVERY_UNAVAILABLE）。
# =============================================================================

"""Pure policy for bounded collision-triggered SCAN replanning."""


# 诊断前缀：碰撞恢复预算已耗尽（时间/距离硬预算花光，守卫已安全停车）
RECOVERY_EXHAUSTED_PREFIX = 'RECOVERY_BUDGET_EXHAUSTED;'
# 诊断前缀：不存在经碰撞检查的可滚动逃生路径，恢复不可用
RECOVERY_UNAVAILABLE_PREFIX = 'RECOVERY_UNAVAILABLE;'


def recovery_budget_exhausted(diagnostic: str) -> bool:
    """返回守卫是否已安全停止一段"预算耗尽"的恢复过程。"""
    # 去掉首尾空白后检查是否以"预算耗尽"前缀开头
    return str(diagnostic).strip().startswith(RECOVERY_EXHAUSTED_PREFIX)


def recovery_replan_required(diagnostic: str) -> bool:
    """返回一段已停止的轨迹是否需要生成一份全新的 SCAN 规划。"""
    text = str(diagnostic).strip()
    # 只要诊断是"预算耗尽"或"恢复不可用"两者之一，就要求重规划
    return text.startswith(
        (RECOVERY_EXHAUSTED_PREFIX, RECOVERY_UNAVAILABLE_PREFIX)
    )


def collision_replan_available(attempts: int, maximum: int) -> bool:
    """返回当前目标内是否还能再发起一次 SCAN 重规划（硬性上限内）。"""
    # 已用次数严格小于上限才允许下一次；负数按 0 处理
    return max(0, int(attempts)) < max(0, int(maximum))
