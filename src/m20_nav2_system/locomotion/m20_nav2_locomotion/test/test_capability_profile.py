# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：test_capability_profile.py
# 所属：m20_nav2_locomotion —— M20 运动控制包的单元测试
# 核心职责：测试版本化 M20 平台能力边界（capability_profile.py）。
#   - 默认配置（m20_policy_v1）的实测滚动包络与派生半径；
#   - 同一配置向所有消费方注入一致的限幅；
#   - 工厂配置（m20_factory_agile_flat_v1）与 MuJoCo 策略测量隔离；
#   - 非法配置（转弯区间/布防时间/倒车角度/对齐角）在 launch 之前被拒绝。
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

"""Tests for the versioned M20 platform capability boundary."""
# 【中文注释】模块说明：版本化 M20 平台能力边界的测试。

import math  # 数学库（isclose 等）
from pathlib import Path  # 跨平台路径对象

import pytest  # pytest 测试框架

from m20_nav2_locomotion.capability_profile import (  # 被测模块
    load_capability_profile,         #   配置文件加载/校验
)


ROOT = Path(__file__).parents[1]  # 【中文注释】包根目录
PROFILE = ROOT / 'config' / 'm20_policy_v1_capabilities.yaml'  # 官方策略配置
FACTORY_PROFILE = (  # 工厂配置
    ROOT / 'config' / 'm20_factory_agile_flat_capabilities.yaml'
)


def test_default_profile_has_measured_rolling_envelope() -> None:
    # 【中文注释】校验：默认配置具有实测滚动包络与正确的派生转弯半径。
    profile = load_capability_profile(PROFILE)

    assert profile.profile_id == 'm20_policy_v1'
    assert not profile.supports_zero_radius_yaw     # 不支持原地偏航
    assert not profile.supports_autonomous_lateral  # 不支持自主横移
    assert math.isclose(  # 最小中心线转弯半径 = 0.35 / 0.65
        profile.minimum_centerline_turn_radius,
        0.35 / 0.65,
    )
    assert math.isclose(profile.turn_swept_radius, 0.893, abs_tol=0.002)  # 扫掠半径


def test_one_profile_injects_identical_limits_into_all_consumers() -> None:
    # 【中文注释】校验：同一配置向所有消费方（意图/碰撞防护/控制器/安全/SDK）注入一致限幅。
    profile = load_capability_profile(PROFILE)
    intent = profile.intent_parameters()
    guard = profile.collision_guard_parameters()
    controller = profile.controller_parameters()
    safety = profile.safety_parameters()
    sdk = profile.sdk_parameters()

    assert intent['max_forward'] == guard['max_linear_x']  # 各消费方限幅一致
    assert intent['max_forward'] == sdk['max_forward']
    assert intent['max_forward'] == safety['max_linear_x']
    assert intent['max_side'] == guard['max_linear_y']
    assert intent['max_yaw'] == guard['max_angular_z']
    assert intent['max_yaw'] == safety['max_angular_z']
    assert intent['turn_min_forward'] == guard['recovery_forward_speed']
    assert guard['recovery_rearm_clear_sec'] == 1.50
    assert controller == {  # 控制器参数（双向跟踪）
        'bidirectional_tracking_enabled': True,
        'reverse_tracking_enter_angle': 2.10,
        'reverse_tracking_exit_angle': 1.75,
        'reverse_tracking_min_hold_sec': 0.80,
        'reverse_tracking_entry_alignment': 0.20,
        'reverse_tracking_exit_alignment': 0.35,
    }
    assert intent['capability_profile_id'] == 'm20_policy_v1'


def test_factory_profile_is_separate_from_mujoco_policy_measurements() -> None:
    # 【中文注释】校验：工厂配置与 MuJoCo 策略测量相互隔离
    # （不把仿真漂移值冒充为物理控制器实测值）。
    profile = load_capability_profile(FACTORY_PROFILE)

    assert profile.profile_id == 'm20_factory_agile_flat_v1'
    assert profile.supports_zero_radius_yaw      # 工厂接口支持原地偏航
    assert not profile.supports_reverse_tracking # 不支持倒车跟踪
    assert profile.minimum_centerline_turn_radius == 0.0  # 允许原地转向 → 半径为 0
    assert profile.measurement_source == 'factory_motion_status'  # 测量来源
    assert profile.positive_yaw_lateral_drift == 0.0  # 漂移未知 → 置零


def test_invalid_turn_interval_fails_before_launch(tmp_path: Path) -> None:
    # 【中文注释】校验：非法转弯区间（min > max）在 launch 前被拒绝。
    invalid = tmp_path / 'invalid.yaml'
    text = PROFILE.read_text(encoding='utf-8').replace(
        'stable_min_forward_mps: 0.35',
        'stable_min_forward_mps: 0.55',  # 超过 max_forward 0.45
    )
    invalid.write_text(text, encoding='utf-8')

    with pytest.raises(ValueError, match='min <= max <= command max'):
        load_capability_profile(invalid)


def test_recovery_rearm_must_not_be_shorter_than_release(
    tmp_path: Path,
) -> None:
    # 【中文注释】校验：恢复"重新布防"清除时间不得短于首次释放确认时间。
    invalid = tmp_path / 'invalid_rearm.yaml'
    text = PROFILE.read_text(encoding='utf-8').replace(
        'rearm_clear_sec: 1.50',
        'rearm_clear_sec: 0.20',  # 短于 clear_confirm_sec 0.30
    )
    invalid.write_text(text, encoding='utf-8')

    with pytest.raises(ValueError, match='at least the release time'):
        load_capability_profile(invalid)


def test_reverse_tracking_hysteresis_must_be_ordered(
    tmp_path: Path,
) -> None:
    # 【中文注释】校验：倒车跟踪滞回角必须有序（0 <= exit < enter <= pi）。
    invalid = tmp_path / 'invalid_reverse_tracking.yaml'
    text = PROFILE.read_text(encoding='utf-8').replace(
        'reverse_tracking_exit_angle_rad: 1.75',
        'reverse_tracking_exit_angle_rad: 2.20',  # 退出角 > 进入角 2.10
    )
    invalid.write_text(text, encoding='utf-8')

    with pytest.raises(ValueError, match='0 <= exit < enter <= pi'):
        load_capability_profile(invalid)


def test_reverse_tracking_alignment_must_have_hysteresis(
    tmp_path: Path,
) -> None:
    # 【中文注释】校验：倒车跟踪对齐角必须具有滞回（0 < entry < exit < pi/2）。
    invalid = tmp_path / 'invalid_reverse_alignment.yaml'
    text = PROFILE.read_text(encoding='utf-8').replace(
        'reverse_tracking_exit_alignment_rad: 0.35',
        'reverse_tracking_exit_alignment_rad: 0.15',  # 退出对齐 < 进入对齐 0.20
    )
    invalid.write_text(text, encoding='utf-8')

    with pytest.raises(ValueError, match='0 < entry < exit < pi/2'):
        load_capability_profile(invalid)
