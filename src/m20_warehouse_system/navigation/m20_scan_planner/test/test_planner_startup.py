"""Smoke-test that the migrated planner accepts its ROS 2 parameter file."""
# ============================================================
# 文件职责：冒烟测试（smoke test）——验证迁移后的 scan_planner_node
#   能够正常加载其 ROS 2 参数文件（config/planner.yaml）并成功启动。
#   若参数文件有语法或类型错误，进程将无法启动，测试即失败。
# ============================================================

import os

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    # 函数：生成 launch 测试描述（launch_testing 固定接口）
    config = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "planner.yaml")
    )
    # 定位规划器参数文件的绝对路径
    planner = launch_ros.actions.Node(
        package="m20_scan_planner",      # 目标 ROS 2 包名
        executable="scan_planner_node",  # 待测试的可执行文件
        name="scan_planner_node",
        parameters=[config], # 传入参数文件
        output="screen",
    )
    return (
        launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()]),
        # 启动规划器 + ReadyToTest 动作
        {"planner": planner},
    )


class TestPlannerStartup:
    def test_process_starts(self, proc_info, planner):
        # 测试用例：规划器进程应在 15 秒内成功启动
        proc_info.assertWaitForStartup(process=planner, timeout=15)
