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
# test_navigation_contract.py —— 导航契约回归测试
# 所属模块：m20_scan_navigation / test
# 职责：用字符串/内容断言保护 SCAN 的话题与安全边界，防止 launch 回归：
#   - 守卫就绪策略（guard_allows_motion）行为正确；
#   - 规划器/控制器核心代码与上游 SCAN-Planner 保持一致（厂商等价）；
#   - launch 中的话题重映射、参数覆盖接缝、速度包络未被破坏；
#   - 网关（gateway）与重规划策略（recovery_replan）的有界行为。
# 运行：colcon test / pytest（由 CMakeLists 注册为 ament pytest 测试）。
# =============================================================================

"""Protect the SCAN topic and safety boundaries from launch regressions."""

from pathlib import Path      # 跨平台路径处理
import re                     # 正则（剥离 C++ 注释做一致性对比）

import yaml                   # YAML 解析（读取各配置档）

# 导入被测模块：守卫判定与碰撞重规划策略
from m20_scan_navigation.navigation_gateway_node import guard_allows_motion
from m20_scan_navigation.recovery_replan import (
    collision_replan_available,
    recovery_budget_exhausted,
    recovery_replan_required,
)


# ---- 工程目录常量（供各测试定位源文件）----
PACKAGE_ROOT = Path(__file__).resolve().parents[1]          # 本包根目录
SYSTEM_ROOT = PACKAGE_ROOT.parents[1]                       # m20_warehouse_system 目录
SOURCE_ROOT = SYSTEM_ROOT.parent                            # src 目录
PLANNER_ROOT = SYSTEM_ROOT / 'navigation' / 'm20_scan_planner'   # SCAN 规划器包
SAFETY_ROOT = SYSTEM_ROOT / 'safety_mission' / 'm20_inspection_core'  # 安全模块包
UPSTREAM_SCAN_ROOT = SOURCE_ROOT / 'third_party' / 'SCAN-Planner'     # 上游 SCAN 源码


def test_guard_readiness_follows_execution_profile() -> None:
    """守卫就绪判定必须遵循执行档位契约。"""
    # 不要求守卫：无论停车标志/诊断如何都放行
    assert guard_allows_motion(False, True, '')
    assert guard_allows_motion(False, True, 'STALE_CLOUD')
    # 要求守卫：停车标志为真（守卫要求停车）或诊断非 CLEAR 时必须禁行
    assert not guard_allows_motion(True, True, 'CLEAR')
    assert not guard_allows_motion(True, False, 'STALE_CLOUD')
    # 要求守卫且诊断 CLEAR、无停车标志：放行
    assert guard_allows_motion(True, False, 'CLEAR')


def _canonical_cpp(source: str) -> str:
    """去掉 C++ 注释与空白，用于"复制核心"的一致性对比。"""
    source = re.sub(r'/\*.*?\*/', '', source, flags=re.DOTALL)   # 去掉块注释
    source = re.sub(r'//[^\n]*', '', source)                     # 去掉行注释
    return re.sub(r'\s+', '', source)                            # 去掉所有空白


def test_planner_manager_logic_matches_the_local_upstream_baseline() -> None:
    """本地规划器 manager 逻辑必须与上游基线逐字节一致（去注释后）。"""
    upstream = (
        UPSTREAM_SCAN_ROOT
        / 'src'
        / 'planner'
        / 'plan_manage'
        / 'src'
        / 'planner_manager.cpp'
    ).read_text(encoding='utf-8')            # 上游原始实现
    adapted = (
        PLANNER_ROOT
        / 'src'
        / 'planner_manager.cpp'
    ).read_text(encoding='utf-8')            # 本地移植实现
    assert _canonical_cpp(adapted) == _canonical_cpp(upstream)   # 去注释后必须相等


def test_scan_core_has_no_m20_time_scaling_delta() -> None:
    """SCAN 核心不得混入 M20 时间缩放修改（保持上游轨迹时钟语义）。"""
    adapted = (
        PLANNER_ROOT
        / 'src'
        / 'planner_manager.cpp'
    ).read_text(encoding='utf-8')
    assert 'M20_INTEGRATION_SHORT_ROUTE_TIMING' not in adapted   # 无集成宏
    assert 'scaled_interval' not in adapted                      # 无缩放间隔


def test_controller_only_targets_raw_velocity_interface() -> None:
    """控制器输出只能指向原始速度接口，不得直连安全话题。"""
    launch_text = (
        PACKAGE_ROOT / 'launch' / 'f1_scan.launch.py'
    ).read_text(encoding='utf-8')
    # 输出重映射到原始速度话题
    assert "'cmd_vel', '/m20/navigation/cmd_vel_raw'" in launch_text
    # 不得绕过安全层直发 /cmd_vel
    assert "'cmd_vel', '/cmd_vel'" not in launch_text
    # 不得直接发布到安全过滤后的速度话题
    assert "'cmd_vel', '/m20/control/cmd_vel_safe'" not in launch_text


def test_f1_planner_is_ground_limited() -> None:
    """F1 规划器档必须是地面受限（低速、点云世界系、机身高度正确）。"""
    with (PACKAGE_ROOT / 'config' / 'f1_planner.yaml').open(
        'r', encoding='utf-8'
    ) as stream:
        parameters = yaml.safe_load(stream)['/**']['ros__parameters']   # 通配节点参数
    assert parameters['fsm.navi_mode'] == 1                     # 目标点导航模式
    assert parameters['grid_map.sensor_type'] == 'lidar'        # 激光雷达
    assert parameters['grid_map.cloud_is_world'] is True        # 点云为世界系
    assert parameters['manager.max_vel'] <= 0.40                # 最大速度不超过 0.40
    assert parameters['grid_map.body_height'] == 0.59           # 机身高度


def test_native_profile_keeps_upstream_scan_trajectory_parameters() -> None:
    """原生档（scan_vendor_planner）必须与上游 planner.yaml 参数完全一致。"""
    with (PACKAGE_ROOT / 'config' / 'scan_vendor_planner.yaml').open(
        'r', encoding='utf-8'
    ) as stream:
        native = yaml.safe_load(stream)['/**']['ros__parameters']
    upstream_path = (
        UPSTREAM_SCAN_ROOT
        / 'src'
        / 'planner'
        / 'plan_manage'
        / 'config'
        / 'planner.yaml'
    )
    with upstream_path.open('r', encoding='utf-8') as stream:
        upstream = yaml.safe_load(stream)[
            'scan_planner_node'
        ]['ros__parameters']
    # 每个上游参数必须存在且相等
    for key in upstream:
        assert native[key] == upstream[key]
    # 键集合必须完全相同（不增不减）
    assert set(native) == set(upstream)


def test_native_controller_matches_upstream_before_safety_limits() -> None:
    """原生控制器在安全限速之前必须与上游 controllers.yaml 一致。"""
    with (PACKAGE_ROOT / 'config' / 'scan_vendor_controller.yaml').open(
        'r', encoding='utf-8'
    ) as stream:
        native = yaml.safe_load(stream)['/**']['ros__parameters']
    upstream_path = (
        UPSTREAM_SCAN_ROOT
        / 'src'
        / 'planner'
        / 'plan_manage'
        / 'config'
        / 'controllers.yaml'
    )
    with upstream_path.open('r', encoding='utf-8') as stream:
        upstream = yaml.safe_load(stream)[
            'closed_loop_controller'
        ]['ros__parameters']
    # 上游每个参数必须与本地一致
    for key, value in upstream.items():
        assert native[key] == value


def test_m20_controller_keeps_vendor_heading_behavior() -> None:
    """M20 控制器源码必须保持厂商航向行为，仅允许特定集成扩展。"""
    controller = (
        PLANNER_ROOT
        / 'src'
        / 'closed_loop_controller.cpp'
    ).read_text(encoding='utf-8')
    # 厂商源码不得包含 M20 专用航向参数（它们只存在于 yaml 覆盖档）
    assert 'heading_error_resume_threshold' not in controller
    assert 'heading_slowdown_threshold' not in controller
    assert 'heading_alignment_min_hold_sec' not in controller
    assert 'headingTranslationScale' not in controller
    # 必须保留上游的航向误差判定
    assert (
        'if (std::abs(yaw_error) > heading_error_threshold_)'
        in controller
    )
    # This is the only controller extension needed by atomic floor switching
    # and fail-closed collision supervision.
    # （以下是原子楼层切换与 fail-closed 碰撞监督所需的全部控制器扩展。）
    assert 'execution_hold_topic' in controller                # 外部执行保持
    assert 'if (!external_execution_hold_)' in controller      # 保持生效时暂停
    assert 'bidirectional_tracking_enabled' in controller      # 双向跟踪开关
    assert 'reverse_tracking_enter_angle' in controller        # 倒车进入角
    assert 'reverse_tracking_exit_angle' in controller         # 倒车退出角
    assert 'reverse_tracking_entry_alignment' in controller    # 进入对齐阈值
    assert 'reverse_tracking_exit_alignment' in controller     # 退出对齐阈值
    assert 'reversePathIsStraight' in controller               # 路径是否笔直
    assert 'updateTrackingDirection' in controller             # 更新跟踪方向
    assert 'return reverse_tracking_ ? reverse_tracking_yaw_' in controller  # 倒车航向返回
    assert 'planning/tracking_direction' in controller         # 跟踪方向话题

    launch = (
        PACKAGE_ROOT / 'launch' / 'f1_scan.launch.py'
    ).read_text(encoding='utf-8')
    # launch 必须保留控制器参数覆盖接缝，且不引入航向状态话题
    assert 'controller_config = LaunchConfiguration' in launch
    assert (
        "str(share / 'config' / 'scan_vendor_controller.yaml'),\n"
        "            controller_config,"
    ) in launch
    assert "('heading_error'," not in launch
    assert "('heading_aligning'," not in launch
    # 反向跟踪相关参数必须存在
    assert "default_value='false'" in launch
    assert 'bidirectional_tracking_enabled' in launch
    assert 'reverse_tracking_entry_alignment' in launch
    assert 'reverse_tracking_exit_alignment' in launch


def test_scan_launch_keeps_vendor_default_with_an_explicit_override_seam() -> None:
    """SCAN launch 必须保持厂商默认值，并留有显式覆盖接缝。"""
    launch = (
        PACKAGE_ROOT / 'launch' / 'f1_scan.launch.py'
    ).read_text(encoding='utf-8')
    # 规划器覆盖参数接缝：参数名 -> LaunchConfiguration
    assert "planner_config = LaunchConfiguration('planner_config')" in launch
    # 规划器参数加载顺序：厂商基准 + 覆盖 + 净空
    assert (
        "str(share / 'config' / 'scan_vendor_planner.yaml'),\n"
        "            planner_config,\n"
        "            clearance_config,"
    ) in launch
    # launch 参数默认值指向厂商基准文件
    assert (
        "'planner_config',\n"
        "                default_value=str(\n"
        "                    share / 'config' / 'scan_vendor_planner.yaml'"
        in launch
    )


def test_m20_velocity_overrides_match_the_capability_envelope() -> None:
    """M20 速度覆盖必须精确等于能力包络（规划器与控制器两侧）。"""
    planner = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'scan_m20_velocity_planner.yaml')
        .read_text(encoding='utf-8')
    )['scan_planner_node']['ros__parameters']
    controller = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'scan_m20_velocity_controller.yaml')
        .read_text(encoding='utf-8')
    )['closed_loop_controller']['ros__parameters']

    # 规划器覆盖只能包含这两个速度键
    assert planner == {
        'manager.max_vel': 0.45,
        'optimization.max_vel': 0.45,
    }
    # 控制器覆盖必须是完整的 M20 包络
    assert controller == {
        'max_vx': 0.45,
        'max_vy': 0.20,
        'max_vyaw': 0.65,
        'trajectory_progress_sync': True,
        'projection_samples': 60,
        'max_time_ahead': 0.20,
    }


def test_conservative_profile_keeps_vendor_scan_clearance() -> None:
    """保守档的 SCAN 规划参数必须与厂商净空一致。"""
    with (
        PACKAGE_ROOT / 'config' / 'clearance_conservative.yaml'
    ).open('r', encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)[
            'scan_planner_node'
        ]['ros__parameters']
    assert parameters == {
        'grid_map.double_cylinder_radius': 0.25,
        'grid_map.double_cylinder_offset': 0.18,
        'optimization.dist0': 0.20,
    }


def test_local_sensing_consumes_only_active_map() -> None:
    """局部感知只能消费当前激活地图，不得全楼层点云。"""
    launch_text = (
        PACKAGE_ROOT / 'launch' / 'f1_scan.launch.py'
    ).read_text(encoding='utf-8')
    # 全局地图输入来自 map_generator
    assert '/map_generator/global_cloud' in launch_text
    # SCAN 输入点云话题
    assert "'/quad_0/cloud'" in launch_text
    # 使用厂商感知参数文件
    assert 'scan_vendor_local_sensing.yaml' in launch_text
    # 不得消费"全部楼层点云"
    assert '/m20/visualization/all_floors_cloud' not in launch_text


def test_native_scan_is_the_only_runtime_route() -> None:
    """原生 SCAN 必须是运行时唯一路径（无网格路线规划器）。"""
    launch_text = (
        PACKAGE_ROOT / 'launch' / 'f1_scan.launch.py'
    ).read_text(encoding='utf-8')
    assert 'm20_grid_route_planner' not in launch_text      # 无网格路线规划器
    assert 'use_grid_route' not in launch_text              # 无网格路线开关
    assert "package='m20_scan_planner'" in launch_text      # 规划器包正确
    assert "executable='scan_planner_node'" in launch_text  # 规划器可执行正确
    assert "'scan_vendor_planner.yaml'" in launch_text      # 使用厂商规划参数
    assert "'scan_vendor_controller.yaml'" in launch_text   # 使用厂商控制器参数
    assert "('move_base_simple/goal', '/move_base_simple/goal')" in launch_text  # 目标话题保持公共名
    assert "'direct_goal_topic': '/move_base_simple/goal'" in launch_text         # 网关直发话题


def test_adapter_does_not_build_a_duplicate_scan_core() -> None:
    """适配包不得重复编译 SCAN 核心（算法在 m20_scan_planner 包中）。"""
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    # 本包不编译规划器/控制器可执行
    assert 'add_executable(m20_scan_planner_node' not in cmake
    assert 'add_executable(m20_scan_controller' not in cmake
    # 依赖声明了 SCAN 规划器包
    assert '<exec_depend>m20_scan_planner</exec_depend>' in (
        PACKAGE_ROOT / 'package.xml'
    ).read_text(encoding='utf-8')
    # 本包不含任何 C++ 源码/头文件
    assert not list((PACKAGE_ROOT / 'src').glob('*.cpp'))
    assert not list((PACKAGE_ROOT / 'include').rglob('*.h'))
    assert not list((PACKAGE_ROOT / 'include').rglob('*.hpp'))


def test_replan_state_keeps_upstream_goal_handling() -> None:
    """重规划状态机必须保持上游的目标处理逻辑。"""
    source = (
        PLANNER_ROOT
        / 'src'
        / 'scan_replan_fsm.cpp'
    ).read_text(encoding='utf-8')
    # 上游没有"目标到达容差"（由网关层处理）
    assert 'target_reached_tolerance_' not in source
    assert 'changeFSMExecState(WAIT_TARGET, "GOAL_REACHED")' not in source


def test_typed_gateway_resets_native_scan_on_cancel() -> None:
    """类型化网关在取消时必须复位原生 SCAN。"""
    source = (
        PACKAGE_ROOT
        / 'm20_scan_navigation'
        / 'navigation_gateway_node.py'
    ).read_text(encoding='utf-8')
    # action 名
    assert "'/m20/navigation/navigate'" in source
    # 只复位 SCAN（无路线复位客户端）
    assert 'self._scan_reset_client' in source
    assert 'self._route_reset_client' not in source
    # 楼层代次匹配逻辑存在
    assert 'floor.generation == goal.map_generation' in source
    assert 'self._floor_matches(goal)' in source


def test_bounded_collision_replan_policy() -> None:
    """碰撞重规划策略必须有界（预算耗尽/不可用才要求重规划）。"""
    # "预算耗尽"前缀命中
    assert recovery_budget_exhausted(
        'RECOVERY_BUDGET_EXHAUSTED; reason=TIME_LIMIT'
    )
    # 其他诊断不命中
    assert not recovery_budget_exhausted('RECOVERY_EPISODE_REARM')
    assert not recovery_budget_exhausted('CLEAR')
    # 重规划要求：预算耗尽或恢复不可用
    assert recovery_replan_required(
        'RECOVERY_BUDGET_EXHAUSTED; reason=TIME_LIMIT'
    )
    assert recovery_replan_required(
        'RECOVERY_UNAVAILABLE; trigger=PREDICTED_FOOTPRINT'
    )
    # 普通诊断不要求重规划
    assert not recovery_replan_required('PREDICTED_FOOTPRINT')
    assert not recovery_replan_required('CURRENT_FOOTPRINT')
    # 重规划可用性受硬上限约束
    assert collision_replan_available(0, 2)
    assert collision_replan_available(1, 2)
    assert not collision_replan_available(2, 2)   # 达到上限
    assert not collision_replan_available(0, 0)   # 上限为 0


def test_gateway_replans_only_after_guard_rearms() -> None:
    """网关只能在守卫重新就绪后重规划。"""
    source = (
        PACKAGE_ROOT
        / 'm20_scan_navigation'
        / 'navigation_gateway_node.py'
    ).read_text(encoding='utf-8')
    assert 'COLLISION_REPLAN_EXHAUSTED' in source      # 预算耗尽分支存在
    assert "diagnostic == 'CLEAR'" in source           # 守卫 CLEAR 判定
    assert "'WAITING_FOR_RECOVERY_REARM'" in source    # 等待守卫重就绪阶段
    assert 'collision_replan_max_attempts' in source   # 重规划上限参数
    assert 'self._publish_goal(target)' in source      # 重发目标
    assert 'goal republished to native SCAN' in source # 重发日志
    # 无路线模式残留
    assert 'route_mode_active' not in source
    assert '_collision_route_publisher' not in source
    # 守卫参数与判定函数存在
    assert "self.declare_parameter('collision_guard_required', True)" in source
    assert 'guard_allows_motion(' in source


def test_managed_rviz_goal_uses_same_bounded_fallback() -> None:
    """受管 RViz 目标使用与类型化 action 相同的有界兜底。"""
    source = (
        PACKAGE_ROOT
        / 'm20_scan_navigation'
        / 'navigation_gateway_node.py'
    ).read_text(encoding='utf-8')
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'navigation_gateway.yaml').read_text(
            encoding='utf-8'
        )
    )['m20_navigation_gateway']['ros__parameters']

    # 手点监控相关代码存在
    assert "'manual_goal_monitor_enabled'" in source
    assert 'self._manual_goal_callback' in source
    assert 'self._manual_tick' in source
    assert "'TRACKING_NATIVE'" in source
    assert "'TRACKING_ROUTE'" not in source
    assert 'self._publish_goal(target)' in source
    assert "'COLLISION_REPLAN_EXHAUSTED'" in source
    assert 'self._reset_navigation(' in source
    # 配置文件里的关键参数与代码一致
    assert config['manual_goal_monitor_enabled'] is False
    assert config['manual_goal_topic'] == '/move_base_simple/goal'
    assert config['manual_timeout_sec'] == 300.0
    assert config['navigation_reset_timeout_sec'] == 10.0
    assert config['collision_guard_required'] is True
    assert config['terminal_orientation_mode'] == 'position_only'
    assert 'self._diagnostic_requires_replan()' in source


def test_integrated_scan_keeps_upstream_visualization_rate() -> None:
    """集成后的 SCAN 必须保留上游可视化频率参数（在源码中，不在 yaml 覆盖）。"""
    planner_config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'scan_vendor_planner.yaml').read_text(
            encoding='utf-8'
        )
    )['/**']['ros__parameters']
    grid_map_source = (
        UPSTREAM_SCAN_ROOT
        / 'src'
        / 'planner'
        / 'plan_env'
        / 'src'
        / 'grid_map.cpp'
    ).read_text(encoding='utf-8')

    # 厂商 yaml 中不得覆盖可视化频率
    assert 'grid_map.visualization_rate_hz' not in planner_config
    # 频率参数存在于上游源码中
    assert 'grid_map.visualization_rate_hz' in grid_map_source
    assert 'visualization_rate_hz, 20.0' in grid_map_source


def test_guard_exposes_missing_safe_recovery_to_gateway() -> None:
    """碰撞守卫必须把"缺少安全恢复"暴露给网关。"""
    source = (
        SAFETY_ROOT
        / 'm20_inspection_core'
        / 'collision_guard_node.py'
    ).read_text(encoding='utf-8')
    # 恢复不可用诊断前缀
    assert "'RECOVERY_UNAVAILABLE; '" in source
    assert 'elif not self._recovery_exhausted_reason:' in source


def test_collision_guard_consumes_scan_online_occupancy() -> None:
    """碰撞守卫必须消费 SCAN 的在线占据点云。"""
    source = (
        SAFETY_ROOT
        / 'm20_inspection_core'
        / 'collision_guard_node.py'
    ).read_text(encoding='utf-8')
    # 订阅 SCAN 在线占据点云话题
    assert "'occupancy_cloud_topic', '/grid_map/occupancy'" in source
    assert 'PointCloud2' in source                       # 点云消息类型
    assert 'rasterize_online_occupancy' in source        # 在线占据栅格化
    assert 'OccupancyGrid' not in source                 # 不使用占据栅格消息
